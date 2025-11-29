// zybo_audio_uio_udp_tx.c — Zybo Z7-10: I2S 캡처 → UDP(AUD0) 전송 (프레임 패이싱 추가)
// build: gcc -O2 -Wall -Wextra -o zybo_audio_uio_udp_tx zybo_audio_uio_udp_tx.c -lpthread -lm

#define _GNU_SOURCE
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <math.h>
#include <netinet/in.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>
#include <pthread.h>
#include <stdbool.h>

/* ===== d_axi_i2s_audio_0 register offsets ===== */
#define I2S_RESET_REG               0x00
#define I2S_TRANSFER_CONTROL_REG    0x04  /* bit0: TX_RS, bit1: RX_RS */
#define I2S_FIFO_CONTROL_REG        0x08
#define I2S_DATA_IN_REG             0x0C
#define I2S_DATA_OUT_REG            0x10
#define I2S_STATUS_REG              0x14
#define I2S_CLOCK_CONTROL_REG       0x18
#define I2S_PERIOD_COUNT_REG        0x1C
#define I2S_STREAM_CONTROL_REG      0x20

/* ===== AXI DMA registers ===== */
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

/* ===== AUD0 header ===== */
#define AUD_MAGIC 0x41554430u /* 'AUD0' */
#pragma pack(push,1)
typedef struct {
    uint32_t magic;  uint8_t ver; uint8_t ch; uint16_t bits;
    uint32_t fs;     uint32_t seq; uint32_t payload_len; uint64_t pts_ns;
} aud_hdr_t;
#pragma pack(pop)

/* ===== format ===== */
typedef struct { unsigned fs, ch, bits; } audio_fmt_t;

/* ===== 송신 컨텍스트 ===== */
typedef struct {
    int sock;
    struct sockaddr_in dst;
    const char* uio_dma;
    const char* uio_i2s;
    const audio_fmt_t* fmt;
    unsigned frame_ms;     // 헤더용 chunk 단위(예: 10ms)
    double   chunk_sec;    // 실제 캡처 파일 길이(예: 0.1s 권장)
    const char* pathA;     // /tmp/micA.raw
    const char* pathB;     // /tmp/micB.raw
    uint32_t seq;
    volatile int run;
    pthread_mutex_t lock;
    pthread_cond_t  cond;
    int buf_ready[2]; // 0:A, 1:B
} stream_ctx_t;

/* ===== globals/utility ===== */
static volatile int g_run = 1;
static void on_sigint(int signo){ (void)signo; g_run = 0; }
static inline void msleep(unsigned ms){ usleep(ms*1000); }

static inline uint32_t rd32(volatile void* b, off_t o){ return *(volatile uint32_t*)((uintptr_t)b+o); }
static inline void     wr32(volatile void* b, off_t o, uint32_t v){ *(volatile uint32_t*)((uintptr_t)b+o)=v; asm volatile("":::"memory"); }

/* ===== UIO map ===== */
static volatile void* map_uio(const char* path, size_t len, int* fd_out){
    int fd=open(path,O_RDWR|O_SYNC);
    if(fd<0){ perror(path); exit(1); }
    void* base=mmap(NULL,len?len:0x10000,PROT_READ|PROT_WRITE,MAP_SHARED,fd,0);
    if(base==MAP_FAILED){ perror("mmap uio"); exit(1); }
    *fd_out=fd; return base;
}

/* big-endian 32bit to cpu */
static uint32_t be32_to_cpu(uint32_t x){
    unsigned char *p=(unsigned char*)&x;
    return ((uint32_t)p[0]<<24)|((uint32_t)p[1]<<16)|((uint32_t)p[2]<<8)|((uint32_t)p[3]);
}

/* ===== reserved-memory audio buf 매핑 (/dev/mem fallback) ===== */
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
            *size_out=sz; *phys_out=base; return p;
        }
    }
    fprintf(stderr,"No udmabuf; DT reg not found; AUDIO_BUF_PA/SZ not set.\n");
    fprintf(stderr,"Set AUDIO_BUF_PA/SZ or AUDIO_BUF_DT, or add udmabuf.\n");
    exit(1);
}

/* ===== codec (SSM2603) over i2c ===== */
static int i2c_fd=-1;
static void i2c_open(const char* dev, int addr7){
    i2c_fd=open(dev,O_RDWR); if(i2c_fd<0){ perror("open i2c"); exit(1); }
    if(ioctl(i2c_fd, I2C_SLAVE, addr7)<0){ perror("I2C_SLAVE"); exit(1); }
}
static void ssm_wr(uint8_t reg7, uint16_t val9){
    uint8_t b[2]; b[0]=(reg7<<1)|((val9>>8)&1); b[1]=(uint8_t)(val9&0xFF);
    if(write(i2c_fd,b,2)!=2){ perror("i2c write"); exit(1); }
}
static void codec_startup_config(void){
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
}
static void codec_set_hp(void){ ssm_wr(4,0b000010110); ssm_wr(5,0); }
static void codec_apply_linux_fix_0x08_0x14(void){
    ssm_wr(4, 0x14); // R4(Analog path)=0x14 : DACSEL=1, BYPASS=0, MUTEMIC=1
    ssm_wr(5, 0x00); // R5 Digital path: SOFTMUTE=0
    ssm_wr(6, 0x00); // R6 Power-down: all on
    ssm_wr(9, 0x01); // R9 Active=1
}

/* ===== DMA/I2S ===== */
static void dma_reset_run(volatile void* dma, int mm2s){
    off_t CR = mm2s? DMA_MM2S_DMACR : DMA_S2MM_DMACR;
    wr32(dma, CR, DMACR_RESET);
    while ( (rd32(dma,CR) & DMACR_RESET) ) {}
    wr32(dma, CR, DMACR_RS);
}

/* ===== UDP helpers ===== */
static int udp_open_and_set_dst(const char* ip, uint16_t port, struct sockaddr_in* out_dst){
    int s = socket(AF_INET, SOCK_DGRAM, 0);
    if (s < 0){ perror("socket UDP"); return -1; }
    /* 송신 버퍼 크게 */
    int sz = 1<<20; setsockopt(s, SOL_SOCKET, SO_SNDBUF, &sz, sizeof(sz));
    memset(out_dst, 0, sizeof(*out_dst));
    out_dst->sin_family = AF_INET;
    out_dst->sin_port   = htons(port);
    if (inet_pton(AF_INET, ip, &out_dst->sin_addr) != 1){
        perror("inet_pton"); close(s); return -1;
    }
    return s;
}

/* ===== 타임 유틸 ===== */
static inline void ts_add_ns(struct timespec* t, long ns){
    t->tv_nsec += ns;
    while(t->tv_nsec >= 1000000000L){ t->tv_nsec -= 1000000000L; t->tv_sec++; }
}

/* ===== UDP: 한 datagram에 [aud_hdr][payload]를 붙여 전송 (프레임 패이싱) ===== */
static int send_file_over_udp(int sock, const struct sockaddr_in* dst,
                              unsigned fs, unsigned ch, unsigned bits,
                              unsigned frame_ms,
                              const char* path, uint32_t* io_seq)
{
    int fd=open(path,O_RDONLY); if(fd<0){ perror(path); return -1; }
    printf("[udp] open '%s' for send\n", path);

    size_t bps = bits/8;
    size_t bytes_per_frame = (size_t)((uint64_t)fs * frame_ms / 1000) * ch * bps;
    if(bytes_per_frame == 0) bytes_per_frame = 1;

    uint8_t* buf = (uint8_t*)malloc(bytes_per_frame);
    if(!buf){ fprintf(stderr,"oom\n"); close(fd); return -1; }

    uint32_t seq = (io_seq && *io_seq)? *io_seq : 0;
    unsigned pktc = 0;

    /* 패이싱: 첫 프레임은 즉시 보내고, 이후부터 매 frame_ms 간격으로 */
    struct timespec next; clock_gettime(CLOCK_MONOTONIC, &next);
    long frame_ns = (long)frame_ms * 1000000L;

    for(;;){
        ssize_t n = read(fd, buf, bytes_per_frame);
        if(n < 0){ if(errno==EINTR) continue; perror("read"); break; }
        if(n == 0) break; /* EOF */

        struct timespec ts; clock_gettime(CLOCK_MONOTONIC,&ts);
        uint64_t pts=(uint64_t)ts.tv_sec*1000000000ull+(uint64_t)ts.tv_nsec;

        aud_hdr_t h;
        h.magic = htonl(AUD_MAGIC);
        h.ver   = 1;
        h.ch    = (uint8_t)ch;
        h.bits  = htons((uint16_t)bits);
        h.fs    = htonl(fs);
        h.seq   = htonl(seq++);
        h.payload_len = htonl((uint32_t)n);
        h.pts_ns = ((uint64_t)htonl((uint32_t)(pts>>32))<<32) | htonl((uint32_t)pts);

        size_t pkt_sz = sizeof(h) + (size_t)n;
        uint8_t* pkt = (uint8_t*)malloc(pkt_sz);
        if(!pkt){ fprintf(stderr,"oom\n"); free(buf); close(fd); if(io_seq) *io_seq=seq; return -1; }
        memcpy(pkt, &h, sizeof(h));
        memcpy(pkt + sizeof(h), buf, (size_t)n);

        ssize_t m = sendto(sock, pkt, pkt_sz, 0, (const struct sockaddr*)dst, sizeof(*dst));
        free(pkt);
        if ((pktc++ % 50) == 0) {
            printf("[udp] sent datagram #%u size=%zd (payload=%zd)\n", seq-1, m, (ssize_t)n);
        }
        if(m < 0){
            if(errno==EINTR) continue;
            perror("sendto");
            free(buf); close(fd); if(io_seq) *io_seq=seq; return -1;
        }

        /* 다음 프레임 기한까지 대기 */
        ts_add_ns(&next, frame_ns);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
    }

    if(io_seq) *io_seq = seq;
    free(buf); close(fd);
    printf("[udp] done sending '%s'\n", path);
    return 0;
}

/* ===== 캡처 (UIO+DMA S2MM → reserved buf → 파일로 저장) ===== */
static void* map_audio_buf(size_t* size_out, uint32_t* phys_out);
static int do_record_no_send(const char* uio_dma,const char* uio_i2s,
                             const audio_fmt_t* fmt,double sec,const char* out_raw)
{
    int fd_dma=-1, fd_i2s=-1;
    volatile void* dma = map_uio(uio_dma,0x10000,&fd_dma);
    volatile void* i2s = map_uio(uio_i2s,0x10000,&fd_i2s);
    size_t ub_sz; uint32_t ub_pa; void* ub = map_audio_buf(&ub_sz,&ub_pa);

    size_t frames=(size_t)(sec*fmt->fs);
    size_t bytes = frames*fmt->ch*(fmt->bits/8);
    printf("[rec] fs=%u ch=%u bits=%u sec=%.3f -> frames=%zu bytes=%zu\n",
           fmt->fs, fmt->ch, fmt->bits, sec, frames, bytes);
    if(bytes>ub_sz){ fprintf(stderr,"audio buf too small\n"); return 1; }
    memset(ub,0,bytes);

    wr32(i2s, I2S_PERIOD_COUNT_REG, frames);
    wr32(i2s, I2S_TRANSFER_CONTROL_REG, 0);
    wr32(i2s, I2S_TRANSFER_CONTROL_REG, 1<<1);     /* RX_RS */

    /* S2MM */
    wr32(dma, DMA_S2MM_DMACR, DMACR_RESET);
    while (rd32(dma, DMA_S2MM_DMACR) & DMACR_RESET) {}
    wr32(dma, DMA_S2MM_DMACR, DMACR_RS);
    wr32(dma, DMA_S2MM_DA, ub_pa);
    wr32(dma, DMA_S2MM_LENGTH, (uint32_t)bytes);
    wr32(i2s, I2S_STREAM_CONTROL_REG, 0x00000001); /* RX Master Stream Enable */

    while(!(rd32(dma, DMA_S2MM_DMASR) & DMASR_IDLE)){
        if(rd32(dma, DMA_S2MM_DMASR) & DMASR_ERR_Irq){ fprintf(stderr,"S2MM error\n"); break; }
    }

    FILE* f=fopen(out_raw,"wb"); if(!f){ perror("fopen"); return 1; }
    fwrite(ub,1,bytes,f); fclose(f);

    wr32(i2s, I2S_STREAM_CONTROL_REG, 0);
    wr32(i2s, I2S_TRANSFER_CONTROL_REG, 0);

    munmap((void*)dma,0x10000); munmap((void*)i2s,0x10000);
    munmap(ub,ub_sz); close(fd_dma); close(fd_i2s);
    return 0;
}

/* ===== 레코드/송신 스레드 ===== */
static void* record_thread(void* arg){
    stream_ctx_t* ctx = (stream_ctx_t*)arg;
    int cur = 0;
    while(ctx->run){
        const char* path = (cur==0)? ctx->pathA : ctx->pathB;
        if (do_record_no_send(ctx->uio_dma, ctx->uio_i2s, ctx->fmt, ctx->chunk_sec, path) != 0){
            fprintf(stderr,"[record] error\n");
            msleep(20);
            continue;
        }
        pthread_mutex_lock(&ctx->lock);
        ctx->buf_ready[cur] = 1;
        pthread_cond_signal(&ctx->cond);
        pthread_mutex_unlock(&ctx->lock);
        cur = 1 - cur;
    }
    return NULL;
}
static void* send_thread(void* arg){
    stream_ctx_t* ctx = (stream_ctx_t*)arg;
    while(ctx->run){
        int target = -1;
        pthread_mutex_lock(&ctx->lock);
        while(ctx->buf_ready[0]==0 && ctx->buf_ready[1]==0 && ctx->run)
            pthread_cond_wait(&ctx->cond, &ctx->lock);
        if(!ctx->run){ pthread_mutex_unlock(&ctx->lock); break; }
        if(ctx->buf_ready[0]) target=0; else if(ctx->buf_ready[1]) target=1;
        ctx->buf_ready[target]=0;
        pthread_mutex_unlock(&ctx->lock);

        const char* path = (target==0)? ctx->pathA : ctx->pathB;
        (void)send_file_over_udp(ctx->sock, &ctx->dst,
                                 ctx->fmt->fs, ctx->fmt->ch, ctx->fmt->bits,
                                 ctx->frame_ms, path, &ctx->seq);
    }
    return NULL;
}

/* ===== 상위 루프 ===== */
static int record_send_threaded_loop_client(const char* ip, int port,
                                            const char* uio_dma, const char* uio_i2s,
                                            const audio_fmt_t* fmt,
                                            unsigned frame_ms, double chunk_sec,
                                            const char* tmpA, const char* tmpB)
{
    signal(SIGINT,on_sigint);
    signal(SIGTERM,on_sigint);
    signal(SIGPIPE,SIG_IGN);

    stream_ctx_t ctx = {0};
    ctx.sock = udp_open_and_set_dst(ip,(uint16_t)port,&ctx.dst);
    if(ctx.sock<0){ fprintf(stderr,"[init] udp open failed\n"); return -1; }

    ctx.uio_dma=uio_dma; ctx.uio_i2s=uio_i2s; ctx.fmt=fmt;
    ctx.frame_ms=frame_ms; ctx.chunk_sec=chunk_sec;
    ctx.pathA=tmpA; ctx.pathB=tmpB; ctx.seq=0; ctx.run=1;
    ctx.buf_ready[0]=ctx.buf_ready[1]=0;
    pthread_mutex_init(&ctx.lock,NULL);
    pthread_cond_init(&ctx.cond,NULL);

    pthread_t th_rec, th_send;
    pthread_create(&th_rec,NULL,record_thread,&ctx);
    pthread_create(&th_send,NULL,send_thread,&ctx);

    fprintf(stderr,"[threaded-loop] running (Ctrl+C to stop)\n");
    while(g_run) sleep(1);

    ctx.run=0;
    pthread_cond_broadcast(&ctx.cond);
    pthread_join(th_rec,NULL);
    pthread_join(th_send,NULL);

    close(ctx.sock);
    pthread_mutex_destroy(&ctx.lock);
    pthread_cond_destroy(&ctx.cond);
    fprintf(stderr,"[threaded-loop] stopped\n");
    return 0;
}

/* ===== main ===== */
int main(void){
    const char* ip = "192.168.1.102";
    int port = 5000;
    unsigned frame_ms = 10;   
    double   chunk_sec = 0.1;          

    const char* i2cdev="/dev/i2c-1"; int addr=0x1a;
    audio_fmt_t fmt={.fs=48000,.ch=2,.bits=16};

    const char* tmpA = "/tmp/micA.raw";
    const char* tmpB = "/tmp/micB.raw";
    unlink(tmpA); unlink(tmpB);

    const char* uio_dma = "/dev/uio1";
    const char* uio_i2s = "/dev/uio3";

    /* 코덱 초기화 */
    i2c_open(i2cdev,addr);
    codec_startup_config();
    codec_set_hp();
    codec_apply_linux_fix_0x08_0x14();
    close(i2c_fd);
    puts("codec init done");

    /* 캡처→UDP 전송 */
    return record_send_threaded_loop_client(ip, port, uio_dma, uio_i2s,
                                            &fmt, frame_ms, chunk_sec, tmpA, tmpB);
}
