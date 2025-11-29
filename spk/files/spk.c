// audio_udp_rx.c — Zybo Z7-10: UDP(AUD0) 수신 → 큐 → DMA(MM2S) → I2S TX (프리버퍼 추가)
// build: gcc -O2 -Wall -Wextra -o audio_udp_rx audio_udp_rx.c -lpthread -lm

#define _GNU_SOURCE
#include <unistd.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <sys/mman.h>
#include <errno.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <pthread.h>

#include <arpa/inet.h>
#include <fcntl.h>
#include <signal.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <time.h>
#include <stdbool.h>

/* ===== I2S regs ===== */
#define I2S_RESET_REG               0x00
#define I2S_TRANSFER_CONTROL_REG    0x04
#define I2S_FIFO_CONTROL_REG        0x08
#define I2S_DATA_IN_REG             0x0C
#define I2S_DATA_OUT_REG            0x10
#define I2S_STATUS_REG              0x14
#define I2S_CLOCK_CONTROL_REG       0x18
#define I2S_PERIOD_COUNT_REG        0x1C
#define I2S_STREAM_CONTROL_REG      0x20

/* ===== AXI DMA regs ===== */
#define DMA_MM2S_DMACR   0x00
#define DMA_MM2S_DMASR   0x04
#define DMA_MM2S_SA      0x18
#define DMA_MM2S_LENGTH  0x28
#define DMA_S2MM_DMACR   0x30
#define DMA_S2MM_DMASR   0x34
#define DMA_S2MM_DA      0x48
#define DMA_S2MM_LENGTH  0x58
#define DMACR_RS         (1u<<0)
#define DMACR_RESET      (1u<<2)
#define DMASR_IDLE       (1u<<1)
#define DMASR_ERR_Irq    (1u<<14)

#define AUD_MAGIC 0x41554430u
#define DMA_DMASR_IOC (1u<<12)
#define DMA_DMASR_DLY (1u<<13)
#define DMA_DMASR_ERR (1u<<14)

#define MAX_SLOTS 128
#define LOG(fmt, ...) fprintf(stderr, fmt, ##__VA_ARGS__)

typedef struct { unsigned fs, ch, bits; } audio_fmt_t;

typedef struct {
    uint8_t*    buf;
    size_t      slot_size;
    int         num_slots;
    int         head;
    int         tail;
    int         count;
    pthread_mutex_t mutex;
    pthread_cond_t  not_empty;
    pthread_cond_t  not_full;
} slot_queue_t;

typedef struct {
    volatile void* dma;
    volatile void* i2s;
    void*          ub;
    uint32_t       ub_pa;
    size_t         ub_sz;
    audio_fmt_t    fmt;
    uint32_t       period_bytes;
    uint32_t       period_frames;
    int            port;
    slot_queue_t   q;
} app_t;

static volatile int stop_flag = 0;
static int i2c_fd=-1;

/* ===== helpers ===== */
static void on_sig(int s){ (void)s; stop_flag = 1; }
static inline uint32_t rd32(volatile void* b, off_t o){ return *(volatile uint32_t*)((uintptr_t)b+o); }
static inline void     wr32(volatile void* b, off_t o, uint32_t v){ *(volatile uint32_t*)((uintptr_t)b+o)=v; asm volatile("":::"memory"); }
static void msleep(unsigned ms){ usleep(ms*1000); }

/* hexdump */
static void hexdump(const void* data, size_t len, size_t max_show, const char* tag){
    size_t n = len < max_show ? len : max_show;
    fprintf(stderr, "[hex %s] len=%zu show=%zu\n", tag, len, n);
    const uint8_t* p = (const uint8_t*)data;
    for(size_t i=0;i<n;i++){
        if((i%16)==0) fprintf(stderr, "  %04zu: ", i);
        fprintf(stderr, "%02X ", p[i]);
        if((i%16)==15 || i+1==n) fprintf(stderr, "\n");
    }
}

#pragma pack(push,1)
typedef struct {
    uint32_t magic;  uint8_t ver; uint8_t ch; uint16_t bits;
    uint32_t fs;     uint32_t seq; uint32_t payload_len; uint64_t pts_ns;
} aud_hdr_t;
#pragma pack(pop)

static bool is_buffer_silent(const uint8_t* buf, size_t len) {
    for (size_t i = 0; i < len; i++) if (buf[i]) return false;
    return true;
}

/* ===== queue ===== */
void sq_init(slot_queue_t* q, uint8_t* buf, size_t slot_size, int num_slots) {
    q->buf = buf; q->slot_size = slot_size; q->num_slots = num_slots;
    q->head = 0; q->tail = 0; q->count = 0;
    pthread_mutex_init(&q->mutex, NULL);
    pthread_cond_init(&q->not_empty, NULL);
    pthread_cond_init(&q->not_full, NULL);
    printf("[queue] init slots=%d slot_size=%zu total=%zu\n",
           q->num_slots, q->slot_size, (size_t)q->num_slots*q->slot_size);
}
uint8_t* sq_get_write_buf(slot_queue_t* q) {
    pthread_mutex_lock(&q->mutex);
    while (q->count == q->num_slots) {
        printf("[queue] get_write_buf WAIT (full) head=%d tail=%d count=%d\n",
               q->head, q->tail, q->count);
        pthread_cond_wait(&q->not_full, &q->mutex);
    }
    uint8_t* buf = q->buf + q->head * q->slot_size;
    printf("[queue] get_write_buf OK  head=%d tail=%d count=%d\n",
           q->head, q->tail, q->count);
    pthread_mutex_unlock(&q->mutex);
    return buf;
}
void sq_push(slot_queue_t* q) {
    pthread_mutex_lock(&q->mutex);
    q->head = (q->head + 1) % q->num_slots;
    q->count++;
    printf("[queue] PUSH  -> head=%d tail=%d count=%d\n", q->head, q->tail, q->count);
    pthread_cond_signal(&q->not_empty);
    pthread_mutex_unlock(&q->mutex);
}
uint8_t* sq_get_read_buf(slot_queue_t* q) {
    pthread_mutex_lock(&q->mutex);
    while (q->count == 0) {
        if(stop_flag) { pthread_mutex_unlock(&q->mutex); return NULL; }
        printf("[queue] get_read_buf WAIT (empty) head=%d tail=%d count=%d\n",
               q->head, q->tail, q->count);
        pthread_cond_wait(&q->not_empty, &q->mutex);
    }
    uint8_t* buf = q->buf + q->tail * q->slot_size;
    printf("[queue] get_read_buf OK   head=%d tail=%d count=%d\n",
           q->head, q->tail, q->count);
    pthread_mutex_unlock(&q->mutex);
    return buf;
}
void sq_pop(slot_queue_t* q) {
    pthread_mutex_lock(&q->mutex);
    q->tail = (q->tail + 1) % q->num_slots;
    q->count--;
    printf("[queue] POP   -> head=%d tail=%d count=%d\n", q->head, q->tail, q->count);
    pthread_cond_signal(&q->not_full);
    pthread_mutex_unlock(&q->mutex);
}

/* ===== map helpers ===== */
static volatile void* map_uio(const char* path, size_t len, int* fd_out){
    int fd=open(path,O_RDWR|O_SYNC);
    if(fd<0){ perror(path); exit(1); }
    void* base=mmap(NULL,len,PROT_READ|PROT_WRITE,MAP_SHARED,fd,0);
    if(base==MAP_FAILED){ perror("mmap uio"); exit(1); }
    *fd_out=fd;
    printf("[map] %s -> base=%p size=0x%zx\n", path, base, len);
    return base;
}
static uint32_t be32_to_cpu(uint32_t x){
    unsigned char *p=(unsigned char*)&x;
    return ((uint32_t)p[0]<<24)|((uint32_t)p[1]<<16)|((uint32_t)p[2]<<8)|((uint32_t)p[3]);
}
static void* map_audio_buf(size_t* size_out, uint32_t* phys_out){
    const char* regpath = getenv("AUDIO_BUF_DT");
    if(!regpath) regpath="/proc/device-tree/reserved-memory/audio_buf@30000000/reg";
    FILE* freg=fopen(regpath,"rb");
    if(freg){
        unsigned char buf[8];
        size_t n=fread(buf,1,sizeof(buf),freg); fclose(freg);
        if(n==8){
            uint32_t be_base, be_size;
            memcpy(&be_base, buf, 4); memcpy(&be_size, buf+4, 4);
            uint32_t base=be32_to_cpu(be_base), sz=be32_to_cpu(be_size);
            int fd=open("/dev/mem",O_RDWR|O_SYNC);
            if(fd<0){ perror("/dev/mem"); exit(1); }
            void* p=mmap(NULL, sz, PROT_READ|PROT_WRITE, MAP_SHARED, fd, base);
            if(p==MAP_FAILED){ perror("mmap /dev/mem (DT-reg)"); exit(1); }
            close(fd);
            *size_out=sz; *phys_out=base;
            printf("[map] audio_buf pa=0x%08X sz=0x%zx va=%p\n", base, (size_t)sz, p);
            return p;
        }
    }
    fprintf(stderr,"No udmabuf; DT reg not found; AUDIO_BUF_PA/SZ not set.\n");
    exit(1);
}

/* ===== i2c/codec ===== */
static void i2c_open(const char* dev, int addr7){
    i2c_fd=open(dev,O_RDWR); if(i2c_fd<0){ perror("open i2c"); exit(1); }
    if(ioctl(i2c_fd, I2C_SLAVE, addr7)<0){ perror("I2C_SLAVE"); exit(1); }
    printf("[i2c] open %s addr=0x%02X\n", dev, addr7);
}
static void ssm_wr(uint8_t reg7, uint16_t val9){
    uint8_t b[2]; b[0]=(reg7<<1)|((val9>>8)&1); b[1]=(uint8_t)(val9&0xFF);
    int n=write(i2c_fd,b,2);
    if(n!=2){ perror("i2c write"); exit(1); }
}
static void codec_startup_config(void){
    printf("[codec] startup config...\n");
    ssm_wr(15, 0b000000000);  usleep(1000);
    ssm_wr( 6, 0b000110000);
    ssm_wr( 0, 0b000010111);
    ssm_wr( 1, 0b000010111);
    ssm_wr( 2, 0b101111001);
    ssm_wr( 3, 0b101111001);
    ssm_wr( 4, 0b000000000);
    ssm_wr( 5, 0b000000000);
    ssm_wr( 7, 0b000001010);
    ssm_wr( 8, 0b000000000);
    usleep(1000);
    ssm_wr( 9, 0b000000001);
    ssm_wr( 6, 0b000100000);
    printf("[codec] startup done\n");
}
static void codec_set_hp(void){ ssm_wr(4,0b000010110); ssm_wr(5,0); }
static void codec_apply_linux_fix_0x08_0x14(void){
    ssm_wr(4, 0x14); ssm_wr(5, 0x00); ssm_wr(6, 0x00); ssm_wr(9, 0x01);
    printf("[codec] fix: R4=0x14, R5=0x00, R6=0x00, R9=0x01\n");
}

/* ===== DMA/I2S ===== */
static void dma_reset_run(volatile void* dma, int mm2s){
    off_t CR = mm2s? DMA_MM2S_DMACR : DMA_S2MM_DMACR;
    wr32(dma, CR, DMACR_RESET);
    while ( (rd32(dma,CR) & DMACR_RESET) ) {}
    wr32(dma, CR, DMACR_RS);
    wr32(dma, DMA_MM2S_DMASR, DMA_DMASR_IOC | DMA_DMASR_DLY | DMA_DMASR_ERR);
    printf("[dma] %s reset+run\n", mm2s?"MM2S":"S2MM");
}

/* ===== play thread (MM2S → I2S TX) with warmup ===== */
static void* audio_play_thread(void* arg){
    app_t* a = (app_t*)arg;

    while(!stop_flag && a->period_bytes == 0) usleep(1000);
    if (stop_flag) { LOG("[play] stop before start\n"); return NULL; }

    /* 프리버퍼 4프레임 확보 후 시작 */
    const int warm = 4;
    pthread_mutex_lock(&a->q.mutex);
    while(!stop_flag && a->q.count < warm)
        pthread_cond_wait(&a->q.not_empty, &a->q.mutex);
    int cc = a->q.count;
    pthread_mutex_unlock(&a->q.mutex);
    printf("[play] warmup OK (count=%d)\n", cc);

    dma_reset_run(a->dma, 1);
    LOG("[play] 재생 루프 시작 (period_bytes=%u frames=%u)\n", a->period_bytes, a->period_frames);

    uint32_t play_count = 0;

    while(!stop_flag){
        uint8_t* buf = sq_get_read_buf(&a->q);
        if (buf == NULL) break;

        uint32_t buf_pa_offset = (uint32_t)(buf - (uint8_t*)a->ub);
        uint32_t dma_addr = a->ub_pa + buf_pa_offset;

        wr32(a->i2s, I2S_STREAM_CONTROL_REG, 0);
        wr32(a->i2s, I2S_TRANSFER_CONTROL_REG, 0);

        wr32(a->i2s, I2S_PERIOD_COUNT_REG, a->period_frames);
        wr32(a->dma, DMA_MM2S_SA, dma_addr);
        wr32(a->dma, DMA_MM2S_LENGTH, a->period_bytes);

        wr32(a->i2s, I2S_TRANSFER_CONTROL_REG, 1 << 0); // TX Enable
        wr32(a->i2s, I2S_STREAM_CONTROL_REG, 2);        // TX Master Stream Enable

        uint32_t st = rd32(a->dma, DMA_MM2S_DMASR);
        if (play_count < 3)
            printf("[play %u] start SA=0x%08X LEN=%u DMASR=0x%08X\n", play_count, dma_addr, a->period_bytes, st);

        while(!(rd32(a->dma, DMA_MM2S_DMASR) & DMASR_IDLE)){
            break; // 상세 로깅은 생략 (필요시 추가)
        }

        /* (선택) 완료 후 스티키 클리어 */
        wr32(a->dma, DMA_MM2S_DMASR, DMA_DMASR_IOC | DMA_DMASR_DLY | DMA_DMASR_ERR);

        uint32_t dma_status_after = rd32(a->dma, DMA_MM2S_DMASR);
        if (play_count < 3)
            printf("[play %u] done DMASR=0x%08X\n", play_count, dma_status_after);

        sq_pop(&a->q);
        play_count++;
    }

    wr32(a->i2s,I2S_STREAM_CONTROL_REG,0);
    wr32(a->i2s,I2S_TRANSFER_CONTROL_REG,0);
    LOG("[play] exit\n");
    return NULL;
}

/* ===== UDP RX thread (producer) ===== */
static void* udp_rx_thread(void* arg){
    app_t* a = (app_t*)arg;
    int s = socket(AF_INET, SOCK_DGRAM, 0);
    if (s < 0) { perror("socket UDP"); stop_flag=1; return NULL; }
    /* 수신 버퍼 크게 */
    int sz = 1<<20; setsockopt(s, SOL_SOCKET, SO_RCVBUF, &sz, sizeof(sz));
    int yes = 1; setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

    struct sockaddr_in sa = {0};
    sa.sin_family = AF_INET;
    sa.sin_port   = htons(a->port);
    sa.sin_addr.s_addr = INADDR_ANY;
    if (bind(s, (struct sockaddr*)&sa, sizeof(sa)) < 0){
        perror("bind UDP"); close(s); stop_flag=1; return NULL;
    }
    char ipbuf[64];
    printf("[udp] listening on %s:%d\n",
           inet_ntop(AF_INET, &sa.sin_addr, ipbuf, sizeof(ipbuf)) ? ipbuf : "0.0.0.0",
           (int)ntohs(sa.sin_port));

    uint8_t pkt[65536];
    struct sockaddr_in src; socklen_t sl = sizeof(src);
    aud_hdr_t hdr;
    uint32_t recv_count = 0, last_seq = (uint32_t)-1;

    while(!stop_flag){
        sl = sizeof(src);
        ssize_t n = recvfrom(s, pkt, sizeof(pkt), 0, (struct sockaddr*)&src, &sl);
        if (n < 0){
            if (errno == EINTR) continue;
            perror("[udp] recvfrom"); break;
        }

        if ((size_t)n < sizeof(aud_hdr_t)) { LOG("[udp] too small packet (%zd)\n", n); continue; }

        memcpy(&hdr, pkt, sizeof(hdr));
        uint32_t magic = ntohl(hdr.magic);
        uint32_t len   = ntohl(hdr.payload_len);
        uint32_t seq   = ntohl(hdr.seq);
        if (magic != AUD_MAGIC){ LOG("[udp] bad magic\n"); continue; }

        /* 시퀀스 갭 로깅 */
        if (last_seq != (uint32_t)-1 && seq != last_seq + 1)
            printf("[udp] SEQ GAP: last=%u now=%u (drop=%u)\n", last_seq, seq, seq - last_seq - 1);
        last_seq = seq;

        if (a->period_bytes == 0) {
            a->period_bytes  = len;
            a->period_frames = len / (a->fmt.ch * (a->fmt.bits / 8));
            int slots = (int)(a->ub_sz / a->period_bytes);
            if (slots > MAX_SLOTS) slots = MAX_SLOTS;
            if (slots < 2)         slots = 2;
            sq_init(&a->q, (uint8_t*)a->ub, a->period_bytes, slots);
            memset(a->ub, 0, a->ub_sz);
            printf("[udp] init period_bytes=%u frames=%u slots=%d\n",
                   a->period_bytes, a->period_frames, slots);
        }
        if ((size_t)n < sizeof(aud_hdr_t) + a->period_bytes){
            LOG("[udp] short payload (%zd)\n", n); continue;
        }

        uint8_t* buf = sq_get_write_buf(&a->q);
        memcpy(buf, pkt + sizeof(aud_hdr_t), a->period_bytes);
        sq_push(&a->q);
        recv_count++;
    }

    close(s);
    stop_flag = 1;
    pthread_cond_broadcast(&a->q.not_empty);
    LOG("[udp] done\n");
    return NULL;
}

/* ===== main ===== */
int main(void){
    setvbuf(stdout, NULL, _IOLBF, 0);
    setvbuf(stderr, NULL, _IOLBF, 0);

    const char* i2cdev="/dev/i2c-1"; int addr=0x1a;
    const char* uio_dma = "/dev/uio1";
    const char* uio_i2s = "/dev/uio3";
    audio_fmt_t fmt={.fs=48000,.ch=2,.bits=16};
    int port=5000, fd_dma=-1, fd_i2s=-1;

    signal(SIGINT, on_sig);
    signal(SIGTERM, on_sig);

    printf("[main] fs=%u ch=%u bits=%u port=%d\n", fmt.fs, fmt.ch, fmt.bits, port);

    i2c_open(i2cdev,addr);
    codec_startup_config();
    codec_set_hp();
    codec_apply_linux_fix_0x08_0x14();
    close(i2c_fd);
    puts("codec init done");

    volatile void* dma=map_uio(uio_dma,0x10000,&fd_dma);
    volatile void* i2s=map_uio(uio_i2s,0x10000,&fd_i2s);
    size_t ub_sz; uint32_t ub_pa; void* ub = map_audio_buf(&ub_sz,&ub_pa);

    printf("[main] UIO dma=%p i2s=%p ub_va=%p ub_pa=0x%08X ub_sz=%zu\n",
           dma, i2s, ub, ub_pa, ub_sz);

    app_t app = {
        .dma=dma, .i2s=i2s,
        .ub=ub, .ub_pa=ub_pa, .ub_sz=ub_sz,
        .fmt=fmt, .period_bytes=0, .period_frames=0,
        .port=port
    };

    pthread_t th_rx, th_play;
    pthread_create(&th_rx,   NULL, udp_rx_thread,   &app);
    pthread_create(&th_play, NULL, audio_play_thread, &app);

    pthread_join(th_play, NULL);
    stop_flag = 1;
    pthread_kill(th_rx, SIGINT);
    pthread_join(th_rx, NULL);

    wr32(i2s, I2S_STREAM_CONTROL_REG, 0);
    wr32(dma, DMA_MM2S_DMACR, DMACR_RESET);

    close(fd_dma);
    close(fd_i2s);

    puts("main exit");
    return 0;
}
