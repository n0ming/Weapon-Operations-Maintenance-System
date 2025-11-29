#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#include <sys/un.h>
#include <sched.h>
#include <ctype.h>

// ===== 런타임 보드 선택 =====
typedef enum { BOARD_A=0, BOARD_B=1, BOARD_C=2 } board_t;
static board_t g_board = BOARD_A;                 /* 기본값: A */
static const char* g_log_prefix = "[?/RS422] ";   /* 결정 후 세팅 */
static const char* g_error_sock = "/run/error.C1.rs422.sock";
static uint8_t g_my_node = 1;                     /* A:1, B:2, C:3 */

/* 감시 링크(보드별 서로 다른 두 링크) */
static uint8_t g_w1_src=0, g_w1_dst=0, g_w2_src=0, g_w2_dst=0;
static inline int link_match(uint8_t s, uint8_t d, uint8_t a, uint8_t b){
    return ((s==a && d==b) || (s==b && d==a));
}

static void handle_rs422_alive(uint8_t sof, uint8_t type, uint8_t alive, uint8_t chk);
static void update_zybo_alive(uint8_t alive); 
static void recalc_err_wire_locked(void);

/* ===== 로깅 매크로(최상단 배치: 사용 전 정의 보장) ===== */
#define LOG_PREFIX (g_log_prefix)
#ifndef HEX_DUMP_MAX
#define HEX_DUMP_MAX 64
#endif
#define LOGF(fmt, ...)  fprintf(stderr, "%s" fmt, LOG_PREFIX, ##__VA_ARGS__)

/* MAC 읽기 */
static int read_mac_lower(const char* ifname, char* out, size_t outsz){
    char p[128];
    snprintf(p,sizeof(p),"/sys/class/net/%s/address", ifname?ifname:"eth0");
    int fd=open(p,O_RDONLY);
    if(fd<0) return -1;
    char buf[64]={0};
    ssize_t n=read(fd,buf,sizeof(buf)-1);
    close(fd);
    if(n<=0) return -1;
    size_t j=0;
    for(ssize_t i=0;i<n && j<outsz-1;i++){
        char c=buf[i];
        if(c=='\n' || c=='\r') break;
        out[j++]=(char)tolower((unsigned char)c);
    }
    out[j]=0;
    return 0;
}

/* 보드 결정 */
static void decide_board_by_mac(void){
    struct { const char* mac; board_t b; } map[] = {
        { "00:0a:35:00:1e:52", BOARD_A },
        { "00:0a:35:00:1e:53", BOARD_B },
        { "00:0a:35:00:1e:54", BOARD_C },
    };

    /* 강제 지정 우선 */
    const char* env_board = getenv("RS422_BOARD");
    if(env_board){
        if (env_board[0]=='A' || env_board[0]=='a') g_board=BOARD_A;
        else if (env_board[0]=='B' || env_board[0]=='b') g_board=BOARD_B;
        else if (env_board[0]=='C' || env_board[0]=='c') g_board=BOARD_C;
    }else{
        const char* ifn = getenv("RS422_IFACE"); if(!ifn) ifn = "eth0";
        char mac[64]={0};
        if(read_mac_lower(ifn, mac, sizeof(mac))==0){
            for(size_t i=0;i<sizeof(map)/sizeof(map[0]);++i){
                if(strcmp(mac, map[i].mac)==0){ g_board = map[i].b; break; }
            }
        }
    }

    switch(g_board){
        case BOARD_A:
            g_log_prefix="[A/RS422] ";
            g_error_sock="/run/error.C1.rs422.sock";
            g_my_node=1;
            /* A: link1=BB(B-C), link2=CC(A-C) */
            g_w1_src=2; g_w1_dst=3;
            g_w2_src=1; g_w2_dst=3;
            break;
        case BOARD_B:
            g_log_prefix="[B/RS422] ";
            g_error_sock="/run/error.C1.rs422.sock";
            g_my_node=2;
            /* B: link1=AA(A-B), link2=BB(B-C) */
            g_w1_src=1; g_w1_dst=2;
            g_w2_src=2; g_w2_dst=3;
            break;
        default: /* C */
            g_log_prefix="[C/RS422] ";
            g_error_sock="/run/error.C1.rs422.sock";
            g_my_node=3;
            /* C: link1=CC(A-C), link2=BB(B-C) */
            g_w1_src=1; g_w1_dst=3;
            g_w2_src=2; g_w2_dst=3;
            break;
    }
}

// ===== 경로/디바이스 =========================================================
#define SERIAL_DEV             "/dev/ttyUL1"
#define BAUDRATE               B115200
#define SOCK_RX_FROM_CAN       "/run/rs422.sock"           // CAN→RS422 (bind)
#define ALIVE_SOCKET_PATH      "/run/zybo_alive.sock"   // receive from tcp
#define TEMPSVC_SOCKET_PATH    "/run/tempsvc.rs422.sock"   // sendto
#define LED_SOCKET_PATH        "/run/led.rs422.sock"   // C sendto error led
#define LED_POWER_SOCKET_PATH  "/run/led.rs422.power.sock"   // C sendto
#define MIC_SOCKET_PATH        "/run/mic.rs422.sock"   // B sendto
#define SPEAKER_SOCKET_PATH    "/run/speaker.rs422.sock" // A sendto

#ifndef TX_GUARD_US
#define TX_GUARD_US 3000   // 2 ms부터 시작해서 3000~5000 까지 가며 튜닝
#endif

static volatile uint64_t g_quiet_until_ns = 0;
/* forward decl to avoid implicit declaration */
static inline uint64_t now_ns(void);
static int bind_dgram(const char* path);


static inline void quiet_for_us(unsigned us){
    uint64_t now = now_ns();
    uint64_t until = now + (uint64_t)us * 1000ULL;
    if (until > g_quiet_until_ns) g_quiet_until_ns = until;
}

static inline void wait_quiet_if_needed(void){
    for(;;){
        uint64_t now = now_ns();
        uint64_t until = g_quiet_until_ns;
        if (now >= until) break;
        uint64_t remain_us = (until - now) / 1000ULL;
        if (remain_us > 1000) usleep(1000);
        else if (remain_us)   usleep((useconds_t)remain_us);
        else sched_yield();
    }
}

// ===== zybo_alive(에러 존재 여부) - 추가 ===== 
typedef struct {
    uint8_t zybo_alive;   /* 1: 에러 존재, 0: 에러 없음 */
} zybo_state_t;

// ====== led로 보낼 구조체 =================
typedef struct __attribute__((packed)){
    uint8_t zybo1;
    uint8_t zybo2;
    uint8_t zybo3;
} err_wire_t;

static err_wire_t     g_err_now       = {0,0,0};      // 현재 값
static err_wire_t     g_err_last_sent = {0xFF,0xFF,0xFF}; // 마지막으로 소켓으로 보낸 값
static zybo_state_t   g_zybo_state    = {0};
static int            g_led_sock      = -1;
static pthread_mutex_t g_err_lock     = PTHREAD_MUTEX_INITIALIZER;

static pthread_mutex_t g_alive_lock = PTHREAD_MUTEX_INITIALIZER;
static uint8_t g_alive_local = 0;  // 0=dead, 1=alive

// TCP에서 받은 zybo_alive 상태
static uint8_t g_alive1 = 0;
static uint8_t g_alive2 = 0;
static uint8_t g_alive3 = 0;

// RS422 하트비트 타임아웃 판단 결과 (C가 직접 관측)
static uint8_t g_hberr1 = 0;   // A 보드와의 링크 (A-C)
static uint8_t g_hberr2 = 0;   // B 보드와의 링크 (B-C)
static uint8_t g_hberr3 = 0;   // 자기자신 (사용 안함, 0으로 둠)



// ===== HB 주기/타임아웃만 유지 =============================================
#define HB_TX_PERIOD_MS 1000
#define HB_TIMEOUT_SEC  6

// ===== V2 프레임 =============================================================
// 5B: E1 01 P0 P1 XOR --- P= control code
#define RS422_V2_SOF  0xE1
#define RS422_V2_TYP  0x01
static inline uint8_t v2_chk(uint8_t a,uint8_t b,uint8_t c,uint8_t d){ return (uint8_t)(a^b^c^d); }

// ===== HB 프레임 (Magic 분리: 0xE7 'H') =====================================
#define HB_MAGIC 0xE7
#define HB_TYPE  0x48   /* 'H' */
enum { N_A=1, N_B=2, N_C=3 };
#define MY_NODE (g_my_node)

static inline uint8_t hb_crc8_xor(const uint8_t* p,int n){ uint8_t c=0; for(int i=0;i<n;i++) c ^= p[i]; return c; }

/* HB 필드: [0]=E7, [1]=48, [2]=(src<<6)|(dst<<4)|(role<<3)|(seq_lo3), [3]=seq, [4]=crc(xor 0..3) */
static inline int hb_build(uint8_t* f, uint8_t src, uint8_t dst, uint8_t role, uint8_t seq){
    f[0]=HB_MAGIC; f[1]=HB_TYPE;
    f[2]= (uint8_t)(((src&3u)<<6) | ((dst&3u)<<4) | ((role&1u)<<3) | (seq&0x7u));
    f[3]= seq;
    f[4]= hb_crc8_xor(f,4);
   return 5;
}
static inline int hb_parse(const uint8_t* f, uint8_t* src,uint8_t* dst,uint8_t* role,uint8_t* seq){
    if(f[0]!=HB_MAGIC || f[1]!=HB_TYPE) return 0;
    if(hb_crc8_xor(f,4)!=f[4]) return 0;
    *src=(f[2]>>6)&3u; *dst=(f[2]>>4)&3u; *role=(f[2]>>3)&1u; *seq=f[3];
    return 1;
}

// ===== 에코 드롭 윈도우(자기 송신 → 곧바로 RX 유입 차단) ====================
// 115200bps 기준 5B(V2) ≈ 0.43 ms, 2B(HB) ≈ 0.17 ms. 지터/스케줄 여유로 5 ms.
#define ECHO_WIN_NS  (800ULL*1000*1000)

static inline uint64_t now_ns(void){
    struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec*1000000000ull + (uint64_t)ts.tv_nsec;
}

// 최근 송신 프레임 저장(원본 바이트 그대로) — V2(5B), HB/레거시(2B)
#define ECHO_KEEP  8
struct echo_entry { uint8_t b[5]; uint8_t len; uint64_t tns; };
static struct echo_entry g_echo[ECHO_KEEP];
static size_t g_echo_idx = 0;

static void echo_remember(const uint8_t* b, size_t len){
    if(len==0 || len>5) return;
    struct echo_entry *e = &g_echo[g_echo_idx++ % ECHO_KEEP];
    memcpy(e->b, b, len); e->len = (uint8_t)len; e->tns = now_ns();
}
static int echo_is_recent(const uint8_t* b, size_t len){
    uint64_t tnow = now_ns();
    for(size_t i=0;i<ECHO_KEEP;i++){
        if(g_echo[i].len==len && (tnow - g_echo[i].tns) < ECHO_WIN_NS){
            if(memcmp(g_echo[i].b, b, len)==0) return 1;
        }
    }
    return 0;
}

// LED 윈도우
static void alive_set(uint8_t v)
{
    pthread_mutex_lock(&g_alive_lock);
    g_alive_local = v ? 1u : 0u;
    pthread_mutex_unlock(&g_alive_lock);
}

static uint8_t alive_get(void)
{
    pthread_mutex_lock(&g_alive_lock);
    uint8_t v = g_alive_local;
    pthread_mutex_unlock(&g_alive_lock);
    return v;
}


// ===== RS-485 DE/RE GPIO =====================================================
// 보드에 맞춰 번호 수정(예: 1022=DE, 1023=/RE)
#define RS485_DE_GPIO "1022"
#define RS485_RE_GPIO "1023"     // /RE(active-low) → 0 이면 수신 Enable

static void gpio_export(const char* num){
    int fd=open("/sys/class/gpio/export",O_WRONLY);
    if(fd>=0){ (void)write(fd,num,strlen(num)); close(fd); }
}
static void gpio_write_path(const char* path,const char* s){
    int fd=open(path,O_WRONLY);
    if(fd>=0){ (void)write(fd,s,strlen(s)); close(fd); }
}
static void rs485_set_de(int on){
    char p[64]; snprintf(p,sizeof(p),"/sys/class/gpio/gpio%s/value", RS485_DE_GPIO);
    gpio_write_path(p, on ? "1":"0");
}
static void rs485_init_dir(void){
    char p[64];
    gpio_export(RS485_DE_GPIO);
    gpio_export(RS485_RE_GPIO);
    usleep(100*1000);
    snprintf(p,sizeof(p),"/sys/class/gpio/gpio%s/direction", RS485_DE_GPIO); gpio_write_path(p,"out");
    snprintf(p,sizeof(p),"/sys/class/gpio/gpio%s/direction", RS485_RE_GPIO); gpio_write_path(p,"out");
    // 초기값: DE=0(하이임피던스), /RE=0(수신 Enable)
    snprintf(p,sizeof(p),"/sys/class/gpio/gpio%s/value", RS485_DE_GPIO); gpio_write_path(p,"0");
    snprintf(p,sizeof(p),"/sys/class/gpio/gpio%s/value", RS485_RE_GPIO); gpio_write_path(p,"0");
    LOGF("GPIO init: DE=0, /RE=0 (RX enable)\n");
}

// ===== 로깅 유틸 =====
static void dump_hex(const char* tag, const uint8_t* p, size_t n){
    size_t m=(n>HEX_DUMP_MAX)?HEX_DUMP_MAX:n;
    LOGF("%s (%zu):", tag, n);
    for(size_t i=0;i<m;i++) fprintf(stderr," %02X", p[i]);
    if(n>m) fprintf(stderr," ...");
    fputc('\n', stderr);
}
static inline void dump2(const char* tag, const uint8_t f[2]){
    LOGF("%s %02X %02X\n", tag, f[0], f[1]);
}

// ===== 전역/링버퍼 ===========================================================
static volatile sig_atomic_t g_stop=0;
static int serial_fd=-1, sock_can_rx=-1;

#define RB_SZ 4096
struct ringbuf{ uint8_t buf[RB_SZ]; size_t head,tail; } rb;

static inline size_t rb_avail(const struct ringbuf* r){
    return (r->head>=r->tail)?(r->head-r->tail):(RB_SZ-(r->tail-r->head));
}
static inline void rb_consume(struct ringbuf* r,size_t n){ r->tail=(r->tail+n)%RB_SZ; }
static inline void rb_push(struct ringbuf* r,const uint8_t* s,size_t n){
    for(size_t i=0;i<n;i++){ r->buf[r->head]=s[i]; r->head=(r->head+1)%RB_SZ; if(r->head==r->tail) r->tail=(r->tail+1)%RB_SZ; }
}
static inline void rb_peek1(const struct ringbuf* r, uint8_t* b){
    *b = r->buf[r->tail];
}
static inline void rb_peek2(const struct ringbuf* r,uint8_t f[2]){ f[0]=r->buf[r->tail]; f[1]=r->buf[(r->tail+1)%RB_SZ]; }

/* RB pop (HB): 0xE7 스캔, 완전 5B일 때만 소비 */
static int rb_try_pop_hb_frame(struct ringbuf* r, uint8_t raw5[5]){
    size_t avail=rb_avail(r); if(avail<5) return 0;
    size_t t=r->tail;
    for(size_t i=0;i<avail;i++){
        uint8_t b0=r->buf[t];
        if(b0==HB_MAGIC){
            if(avail-i<5) return 0; // 아직 부족
            uint8_t f0=r->buf[t], f1=r->buf[(t+1)%RB_SZ], f2=r->buf[(t+2)%RB_SZ],
                    f3=r->buf[(t+3)%RB_SZ], f4=r->buf[(t+4)%RB_SZ];
            uint8_t tmp[5]={f0,f1,f2,f3,f4}, s,d,role,seq;
            if(hb_parse(tmp,&s,&d,&role,&seq)){
                if(i) rb_consume(r,i);
                rb_consume(r,5);
                if(raw5) memcpy(raw5,tmp,5);
                return 1;
            }
        }
        t=(t+1)%RB_SZ;
    }
    return 0;
}

// ===== V2 pop: SOF E1, TYPE 01, XOR =========================================
// V2 팝: 원본 5바이트와 페이로드 2바이트를 모두 돌려줌(에코 비교용)
static int rb_try_pop_v2_frame_ex(struct ringbuf* r, uint8_t raw5[5], uint8_t out[2]){
    size_t avail=rb_avail(r); if(avail<5) return 0;
    size_t t=r->tail;
    for(size_t i=0;i<avail;i++){
        uint8_t b0=r->buf[t];
        if(b0==RS422_V2_SOF){
            if(avail-i<5) return 0;
            uint8_t b1=r->buf[(t+1)%RB_SZ], p0=r->buf[(t+2)%RB_SZ], p1=r->buf[(t+3)%RB_SZ], ck=r->buf[(t+4)%RB_SZ];
            if(b1==RS422_V2_TYP && ck==v2_chk(b0,b1,p0,p1)){
                if(i) rb_consume(r,i);
                rb_consume(r,5);
                if(raw5){ raw5[0]=b0; raw5[1]=b1; raw5[2]=p0; raw5[3]=p1; raw5[4]=ck; }
                out[0]=p0; out[1]=p1;
                return 1;
            }
        }
        t=(t+1)%RB_SZ;
    }
    return 0;
}

// ===== tempsvc DGRAM =========================================================
static ssize_t send_unix_dgram_path(const char* path,const void* buf,size_t len){
    int fd=socket(AF_UNIX,SOCK_DGRAM|SOCK_CLOEXEC,0); if(fd<0) return -1;
    struct sockaddr_un to; memset(&to,0,sizeof(to)); to.sun_family=AF_UNIX;
    strncpy(to.sun_path,path,sizeof(to.sun_path)-1);
    ssize_t n = sendto(fd,buf,len,0,(struct sockaddr*)&to,sizeof(to));
    int save = errno; close(fd); errno = save;
    return n;
}
static inline void tempsvc_send(const uint8_t f[2]){
    dump2("→ tempsvc", f);
    (void)send_unix_dgram_path(TEMPSVC_SOCKET_PATH,f,2);
}

static inline void mic_send(const uint8_t f[2]){
    dump2("→ mic", f);
    (void)send_unix_dgram_path(MIC_SOCKET_PATH,f,2);
}

static inline void spk_send(const uint8_t f[2]){
    dump2("→ spk", f);
    ssize_t n = send_unix_dgram_path(SPEAKER_SOCKET_PATH,f,2);
    if (n < 0) {
        LOGF("spk_send sendto(%s) FAILED: %s\n",
             SPEAKER_SOCKET_PATH, strerror(errno));
    } else {
        LOGF("spk_send OK (%zd bytes) : %02X %02X\n",
             n, f[0], f[1]);
    }
}

static inline void led_send(const uint8_t f[2]){
    ssize_t n = send_unix_dgram_path(LED_POWER_SOCKET_PATH,f,2);
    if (n < 0) { 
        LOGF("led_send sendto(%s) FAILED: %s\n",
             LED_POWER_SOCKET_PATH, strerror(errno));
    } else {
        LOGF("led_send OK (%zd bytes) : %02X %02X\n",
             n, f[0], f[1]);
    }
    dump2("→ led", f);
}

//----------에러 구조체(런타임 보드 union)--------------------
typedef union __attribute__((packed)) {
    /* A: B의 AA(PONG) 미수신 - hb_C2_err, C의 CC(PING) 미수신 - hb_C3_err */
    struct __attribute__((packed)) { uint8_t socket_err; uint16_t hb_C2_err; uint16_t hb_C3_err; } A;
    /* B: C의 BB(PONG) 미수신 - hb_C3_err, A의 AA(PING) 미수신 - hb_C1_err */
    struct __attribute__((packed)) { uint8_t socket_err; uint16_t hb_C1_err; uint16_t hb_C3_err; } B;
    /* C: A의 CC(PONG) 미수신 - hb_C1_err, B의 BB(PING) 미수신 - hb_C2_err */
    struct __attribute__((packed)) { uint8_t socket_err; uint16_t hb_C1_err; uint16_t hb_C2_err; } C;
} rs422_error_u;

static rs422_error_u g_error;
static pthread_mutex_t error_lock = PTHREAD_MUTEX_INITIALIZER;

/* 링크 관측 최근시각(보드별 감시 2개 링크) */
static volatile uint64_t g_last_seen_link1_ns = 0;
static volatile uint64_t g_last_seen_link2_ns = 0;

/* MY_HB를 마지막으로 보낸 시각(에코 판단/지연 관찰용) */
static volatile uint64_t g_last_myhb_tx_ns = 0;

static void* thread_error_tx(void* arg){
    (void)arg;
    /* 부팅 직후 불필요한 오류 방지를 위해 초기 수신시각을 현재로 세팅 */
    uint64_t t0 = now_ns();
    g_last_seen_link1_ns = t0;
    g_last_seen_link2_ns = t0;

    const uint64_t TO_NS = (uint64_t)HB_TIMEOUT_SEC * 1000000000ull;
    for(;;){
        if(g_stop) break;
        uint64_t now = now_ns();

        pthread_mutex_lock(&error_lock);
        memset(&g_error, 0, sizeof(g_error));

        uint16_t e1 = (now - g_last_seen_link1_ns > TO_NS) ? 1 : 0;
        uint16_t e2 = (now - g_last_seen_link2_ns > TO_NS) ? 1 : 0;

        switch(g_board){
            case BOARD_A:
                g_error.A.socket_err = 0;
                g_error.A.hb_C2_err = e1; /* BB */
                g_error.A.hb_C3_err = e2; /* CC */
                break;
            case BOARD_B:
                g_error.B.socket_err = 0;
                g_error.B.hb_C1_err = e1; /* AA */
                g_error.B.hb_C3_err = e2; /* BB */
                break;
            default: /* C */
                g_error.C.socket_err = 0;
                g_error.C.hb_C1_err = e1; /* CC */
                g_error.C.hb_C2_err = e2; /* BB */
                break;
        }
        pthread_mutex_unlock(&error_lock);
        //C보드일 때: RS422 HB 에러를 LED용 상태에 반영
        if (g_board == BOARD_C) {
            pthread_mutex_lock(&g_err_lock);

            // link1 = A–C, link2 = B–C
            g_hberr1 = e1 ? 1u : 0u;
            g_hberr2 = e2 ? 1u : 0u;
            g_hberr3 = 0u;

            recalc_err_wire_locked();  // alive + HB 에러 OR → LED 송신

            pthread_mutex_unlock(&g_err_lock);
        }

        /* 사람이 읽기 쉬운 로그 */
        if(g_board==BOARD_A){
            LOGF("ERROR tx -> %s  socket_err=%u  hb_C2_err=%u  hb_C3_err=%u\n",
                g_error_sock, g_error.A.socket_err, g_error.A.hb_C2_err, g_error.A.hb_C3_err);
        }else if(g_board==BOARD_B){
            LOGF("ERROR tx -> %s  socket_err=%u  hb_C1_err=%u  hb_C3_err=%u\n",
                g_error_sock, g_error.B.socket_err, g_error.B.hb_C1_err, g_error.B.hb_C3_err);
        }else{
            LOGF("ERROR tx -> %s  socket_err=%u  hb_C1_err=%u  hb_C2_err=%u\n",
                g_error_sock, g_error.C.socket_err, g_error.C.hb_C1_err, g_error.C.hb_C2_err);
        }

        /* 실제 전송 payload 5바이트 선택 */
        const uint8_t* pay = (const uint8_t*)((g_board==BOARD_A)? (const void*)&g_error.A
                                  : (g_board==BOARD_B)? (const void*)&g_error.B
                                                       : (const void*)&g_error.C);
        LOGF("ERROR tx bytes(5): %02X %02X %02X %02X %02X\n",
                pay[0], pay[1], pay[2], pay[3], pay[4]);

        /* 1초마다 내 보드의 에러 소켓으로 송신 */
        ssize_t n = send_unix_dgram_path(g_error_sock, pay, 5);
        if(n == 5) {
            LOGF("ERROR tx OK (%zd bytes)\n", n);
        } else {
            int e = errno;
            LOGF("ERROR tx FAIL (ret=%zd, errno=%d: %s)\n",
                    n, e, strerror(e));
        }
        usleep(1000*1000);
    }
    return NULL;
}

// ===== 시리얼 ================================================================
static int open_serial(const char* dev){
    int fd=open(dev,O_RDWR|O_NOCTTY|O_NONBLOCK); if(fd<0) return -1;
    struct termios tio; if(tcgetattr(fd,&tio)<0){ close(fd); return -1; }
    cfmakeraw(&tio); cfsetispeed(&tio,BAUDRATE); cfsetospeed(&tio,BAUDRATE);
    tio.c_cflag|=(CLOCAL|CREAD); tio.c_cflag&=~CRTSCTS; tio.c_cc[VMIN]=0; tio.c_cc[VTIME]=1;
    if(tcsetattr(fd,TCSANOW,&tio)<0){ close(fd); return -1; }
    tcflush(fd,TCIOFLUSH);
    LOGF("UART ready: %s 115200-8N1\n", dev);
    // 블로킹으로
    int fl=fcntl(fd,F_GETFL); fcntl(fd,F_SETFL, fl&~O_NONBLOCK);
    return fd;
}
static pthread_mutex_t tx_lock=PTHREAD_MUTEX_INITIALIZER;
static ssize_t write_all_locked_de(int fd,const void* buf,size_t len,const char* why){
    // 하프/멀티드롭 안전: TX 동안에만 DE=1, 그리고 /RE=1(수신 Disable)로 에코 차단
    // (/RE GPIO 제어가 가능하다는 전제; init에서 /RE=0으로 두고 사용 중)
    char p_re[64]; snprintf(p_re,sizeof(p_re),"/sys/class/gpio/gpio%s/value", RS485_RE_GPIO);
    gpio_write_path(p_re,"1");     // /RE=1 -> RX 비활성(에코 물리 차단)
    rs485_set_de(1);               // DE=1  -> TX Enable

    // (115200bps 기준 1바이트 ≈ 87us) — 바로 쏴도 됨
    pthread_mutex_lock(&tx_lock);
    const uint8_t* p=buf; size_t off=0;
    while(off<len){
        ssize_t n=write(fd,p+off,len-off);
        if(n<0){
            if(errno==EINTR) continue;
            pthread_mutex_unlock(&tx_lock);
            rs485_set_de(0);
            gpio_write_path(p_re,"0");  // <- /RE=0 (RX 재개) 누락 복구
            return -1;
        }
        if(n==0) break; off+=(size_t)n;
    }
    pthread_mutex_unlock(&tx_lock);
    if(len==2) dump2(why,(const uint8_t*)buf); else dump_hex(why,(const uint8_t*)buf,len);
    // 에코 드롭용으로 원본 프레임 기억
    echo_remember((const uint8_t*)buf, len);

    // 프레임이 선로에서 완전히 빠져나가도록 대기
    tcdrain(fd);               // 실제 송신 끝까지 대기
    /* HB면 바로 RX를 켠다: PONG이 즉시 돌아올 수 있으므로 */
    int is_hb = (len==5 &&
                 ((const uint8_t*)buf)[0]==HB_MAGIC &&
                 ((const uint8_t*)buf)[1]==HB_TYPE);
    if(is_hb){
        rs485_set_de(0);           // 먼저 드라이버 끄고
        gpio_write_path(p_re,"0"); // 곧바로 /RE=0 → RX ON (지연 없음)
        /* HB에서는 TX_GUARD_US 지연을 생략(또는 아주 작게) */
        // usleep(100); // 필요하면 100us 정도만
    }else{
        usleep(TX_GUARD_US);       // 다른 프레임은 기존 가드 유지
        rs485_set_de(0);
        gpio_write_path(p_re,"0");
    }
    return (ssize_t)off;
}

// ===== 보드별 process_frame(수신→tempsvc) ===================================
static void process_frame(const uint8_t f[2]){
    switch(g_board){
        case BOARD_A: {
            uint8_t out[2]; out[0]=0x00; out[1]=(uint8_t)(f[1] & 0x07);
            dump2("process(A) temp raw", f); tempsvc_send(out);
            dump2("process(A) spk raw", f); spk_send(out);
        } break;
        case BOARD_B: {
            uint8_t out[2]; out[0]=0x00; out[1]=(uint8_t)((f[1] & 0x38)>>3);
            dump2("process(B) temp raw", f); tempsvc_send(out);
            dump2("process(B) mic raw", f); mic_send(out);
        } break;
        case BOARD_C: {
            uint16_t v = ((uint16_t)(f[0] & 0x01) << 2) | ((f[1] & 0xC0) >> 6);
            uint8_t out[2] = { (uint8_t)(v >> 8), (uint8_t)v };

            /*
            uint8_t led_bits = (uint8_t)v;   // v: [bit0: 외부?, bit1: 내부?, bit2: (예전 정의용)]


            
            // ★ RS422 전원 참여 로직:
            //    - 외부/내부 중 하나라도 1이면 -> 전체 전원 비트(bit2, 0x04)도 1
            //    - 둘 다 0이면 -> 전원 비트 0
            if (led_bits & 0x03) {        // bit0 또는 bit1이 세트면
                led_bits |= 0x04;         // bit2(0x04)를 켬 → "전원 ON"
            } else {
                led_bits &= ~0x04;        // bit2를 끔 → "전원 OFF"
            }
        

            uint8_t out[2] = { 0x00, led_bits };
            */

            // tempsvc 용도 그대로 유지 (필요하다면 여기도 out[1] 사용)
            dump2("process(C) temp raw", f);
            tempsvc_send(out);

            // ★ 이 out[1]이 /run/led.rs422.power.sock 으로 나가서
            //    led.c에서 (lo & 0x04) 검사 → RS422 전원 참여
            dump2("process(C) led raw", out);
            led_send(out);
        } break;
    }
}

// ===== 보드별 CAN->UART 패킹(네 쉬프팅/비트위치 유지) + V2 캡슐화 ===========
static void can_to_uart_v2_send(uint32_t id,uint8_t dlc,const uint8_t d[8]){
    if(dlc!=2) return;
    uint8_t p0=0, p1=0;
    switch(g_board){
        case BOARD_A: {
            if(id!=0x00084051u) return;
            uint16_t v=0;
            v|=((d[1]>>3)&1u)<<0; v|=((d[1]>>4)&1u)<<1; v|=((d[1]>>5)&1u)<<2;
            v|=((d[1]>>6)&1u)<<3; v|=((d[1]>>7)&1u)<<4; v|=((d[0]>>0)&1u)<<5;
            v <<= 3;  // ★ 유지
            p0=(uint8_t)(v>>8); p1=(uint8_t)v;
        } break;
        case BOARD_B: {
            if(id!=0x00084052u) return;
            uint16_t v=0;
            v|=((d[1]>>3)&1u)<<0; v|=((d[1]>>4)&1u)<<1; v|=((d[1]>>5)&1u)<<2;
            v|=((d[1]>>6)&1u)<<6; v|=((d[1]>>7)&1u)<<7; v|=((d[0]>>0)&1u)<<8;
            p0=(uint8_t)(v>>8); p1=(uint8_t)v;
        } break;
        default: { /* C */
            if(id!=0x00084053u) return;
            uint16_t v=0;
            v|=((d[1]>>3)&1u)<<0; v|=((d[1]>>4)&1u)<<1; v|=((d[1]>>5)&1u)<<2;
            v|=((d[1]>>6)&1u)<<3; v|=((d[1]>>7)&1u)<<4; v|=((d[0]>>0)&1u)<<5;
            p0=(uint8_t)(v>>8); p1=(uint8_t)v;
        } break;
    }
    uint8_t v2[5]={ RS422_V2_SOF, RS422_V2_TYP, p0, p1, v2_chk(RS422_V2_SOF,RS422_V2_TYP,p0,p1) };
    dump2("CAN→UART out", (uint8_t[]){p0,p1});
    dump_hex("V2 pkt", v2, sizeof(v2));

    echo_remember(v2, 5);        // V2 원본 5바이트 기억
    echo_remember(&v2[2], 2);    // V2 페이로드 2바이트도 따로 기억

    (void)write_all_locked_de(serial_fd, v2, sizeof(v2), "TX uart(V2)");
}

// ===== HB 송신(내 보드만 말함), 응답 필터 ====================================
// 지터 추가 완 버전
static void* thread_hb_tx(void* arg){
    (void)arg;
    uint8_t hb[5];
    uint8_t seq_ab=0, seq_ac=0, seq_ba=0, seq_bc=0, seq_ca=0, seq_cb=0;
    
    if(g_board==BOARD_A)      usleep(   0 * 1000);
    else if(g_board==BOARD_B) usleep( 300 * 1000);
    else                      usleep( 600 * 1000);

    while(!g_stop){
        int next_sleep_ms = HB_TX_PERIOD_MS;

        if(serial_fd>=0){
            uint8_t dsts[2];
            uint8_t* pseqs[2];
            int cnt = 0;

            switch(g_board){
                case BOARD_A:
                    dsts[0] = N_B; pseqs[0] = &seq_ab;
                    dsts[1] = N_C; pseqs[1] = &seq_ac;
                    cnt = 2;
                    break;
                case BOARD_B:
                    dsts[0] = N_A; pseqs[0] = &seq_ba;
                    dsts[1] = N_C; pseqs[1] = &seq_bc;
                    cnt = 2;
                    break;
                default: // BOARD_C
                    dsts[0] = N_A; pseqs[0] = &seq_ca;
                    dsts[1] = N_B; pseqs[1] = &seq_cb;
                    cnt = 2;
                    break;
            }

            for (int i = 0; i < cnt; i++) {
                hb_build(hb, MY_NODE, dsts[i], 0, (*pseqs[i])++);
                wait_quiet_if_needed();
                (void)write_all_locked_de(serial_fd, hb, 5, "TX HB PING");
            }
            g_last_myhb_tx_ns = now_ns();

            int jitter = (rand()%141) - 70; // -30~+30ms
            next_sleep_ms = HB_TX_PERIOD_MS + jitter;
            if(next_sleep_ms < 100) next_sleep_ms = 100;
        } else {
            next_sleep_ms = 200;
        }

        usleep((useconds_t)next_sleep_ms * 1000);
    }
    return NULL;
}
// ===== CAN DGRAM 스레드 ======================================================
static void* thread_can_ipc(void* arg){
    (void)arg;
    struct __attribute__((packed)){ uint32_t id; uint8_t dlc; uint8_t data[8]; } p;
    struct sockaddr_un peer; socklen_t sl=sizeof(peer);
    for(;;){
        if(g_stop) break;
        ssize_t n=recvfrom(sock_can_rx,&p,sizeof(p),0,(struct sockaddr*)&peer,&sl);
        if(n<0){ if(errno==EINTR) continue; usleep(10*1000); continue; }
        if(n==(ssize_t)sizeof(p)){
            LOGF("CAN RX id=0x%08X dlc=%u data=%02X %02X\n", p.id, p.dlc, p.data[0], p.data[1]);
            can_to_uart_v2_send(p.id,p.dlc,p.data);
        }
    }
    return NULL;
}

//======= Alive LED RS422 Tx =============================================
//A랑 B만 
static void* thread_led_alive(void* arg)
{
    (void)arg;
    LOGF("thread_led_alive start - real");

    int s = bind_dgram(ALIVE_SOCKET_PATH);  
    if (s < 0) {
        perror("bind_dgram(ALIVE_SOCKET_PATH)");
        return NULL;
    }

    while (!g_stop) {
        zybo_state_t msg;

        struct sockaddr_un peer;
        socklen_t sl = sizeof(peer);
        ssize_t n = recvfrom(s, &msg, sizeof(msg), 0,
                             (struct sockaddr*)&peer, &sl);
        if (n < 0) {
            if (errno == EINTR)
                continue;
            usleep(10 * 1000);
            continue;
        }
        if (n < 1)          // 최소 1바이트는 있어야 함
            continue;

        uint8_t alive = msg.zybo_alive ? 1u : 0u;

        if (g_board == BOARD_A || g_board == BOARD_B) {
            /* ===== A/B: RS422로 FF/EE ALIVE 프레임 보내기 ===== */
            uint8_t frame[4];

            if (g_board == BOARD_A)
                frame[0] = 0xFF;  // A → FF
            else                 // BOARD_B
                frame[0] = 0xEE;  // B → EE

            frame[1] = 0x01;             // type
            frame[2] = alive;            // 0 or 1
            frame[3] = frame[0] ^ frame[1] ^ frame[2];

            dump_hex("LED->RS422", frame, sizeof(frame));
            echo_remember(frame, sizeof(frame));
            (void)write_all_locked_de(serial_fd, frame, sizeof(frame),
                                      "TX uart(LED)");
        }
        else if (g_board == BOARD_C) {
            /* ===== C: 자기 alive(zybo3) 갱신 후 LED 에러 집계 ===== */
            update_zybo_alive(alive);
        }
    }
    close(s);
    LOGF("thread_led_alive end\n");
    return NULL;
}

// ===== RX 파서 스레드(V2→HB→레거시) =========================================
static inline bool is_valid_legacy(const uint8_t f[2]) {
    if (f[0] != 0x00)
        return false;
    // 상위 비트가 하나라도 켜져 있으면 쓰레기로 간주
    if (f[1] & ~0x07)
        return false;
    return true;
}

static void* thread_rx(void* arg){
    (void)arg;
    uint8_t chunk[512], pay[2], f[2], raw5[5];
    while(!g_stop){
        ssize_t n=read(serial_fd,chunk,sizeof(chunk));
        if(n<0){ if(errno==EINTR) continue; usleep(5*1000); continue; }
        if(n==0){ usleep(1*1000); continue; }
        dump_hex("RX-chunk", chunk, (size_t)n);
        rb_push(&rb,chunk,(size_t)n);

        /* 1) HB(0xE7) 먼저 처리 — V2와 완전히 분리 */
        uint8_t hb5[5];
        while(rb_try_pop_hb_frame(&rb, hb5)){
            uint8_t src,dst,role,seq;
            if(!hb_parse(hb5,&src,&dst,&role,&seq)) continue; /* 가드 */
            
            /* ★ 남의 링크(A↔C 등)를 본 순간 5ms 동안 송신 정지(Quiet Window) */
            if (dst != MY_NODE && src != MY_NODE) {
                quiet_for_us(5000);
                dump_hex("HB third-link seen → quiet 5ms", hb5, 5);
            }

            /* 감시 링크 관측 시각 갱신 (src/dst 양방향 매칭) */
            uint64_t t = now_ns();
            if (link_match(src,dst,g_w1_src,g_w1_dst)) g_last_seen_link1_ns = t;
            if (link_match(src,dst,g_w2_src,g_w2_dst)) g_last_seen_link2_ns = t;

            if(dst != MY_NODE){
                dump_hex("HB other-dst (ignored)", hb5, 5);
                continue;
            }
        
        }

        // 2) V2 프레임 (0xE1) — 기존 그대로
        // V2 프레임 우선  에코 드롭 먼저 체크
        while(rb_try_pop_v2_frame_ex(&rb,raw5,pay)){
            if(echo_is_recent(raw5, 5)){
                dump_hex("DROP self-echo(V2)", raw5, 5);
                continue;
            }
            dump2("V2→process", pay);
            process_frame(pay);
        }


        // 2.5) ALIVE 프레임 [FF/EE, 01, alive, chk] — C보드만 사용
        if (g_board == BOARD_C) {
            while (rb_avail(&rb) >= 4) {
                uint8_t b0;
                rb_peek1(&rb, &b0);

                // 시작 바이트가 FF/EE가 아니면 ALIVE 프레임 아님 → 다음 단계(레거시/드롭)로 넘김
                if (b0 != 0xFF && b0 != 0xEE)
                    break;

                size_t t = rb.tail;
                uint8_t sof   = rb.buf[t];
                uint8_t type  = rb.buf[(t+1) % RB_SZ];
                uint8_t alive = rb.buf[(t+2) % RB_SZ];
                uint8_t chk   = rb.buf[(t+3) % RB_SZ];

                LOGF("ALIVE candidate: %02X %02X %02X %02X\n", sof, type, alive, chk);

                // 형식/체크섬은 handle_rs422_alive 안에서 한 번 더 검사
                handle_rs422_alive(sof, type, alive, chk);

                // 무조건 4바이트 소비 (잘못된 프레임이면 그냥 버리는 셈)
                rb_consume(&rb, 4);
            }
        }

        // ★ V2/ALIVE 헤더가 맨 앞인데 프레임 길이만큼 아직 안 모였으면,
        //    밑의 레거시/드롭 단계로 내려가지 말고 다음 read를 기다린다.
        size_t avail = rb_avail(&rb);
        if (avail > 0) {
            uint8_t b0 = rb.buf[rb.tail];

            // V2 헤더 보호 (기존 코드)
            if (b0 == RS422_V2_SOF && avail < 5) {
                continue;
            }

            if (g_board == BOARD_C &&
                    (b0 == 0xFF || b0 == 0xEE) && avail < 4) {
                continue;
            }
        }

        // 3) (구) 2B HB/레거시 — HB 부분은 비활성, 레거시만 남기기
        while(rb_avail(&rb)>=2){
            rb_peek2(&rb,f);

            /*
            // ★ 레거시(선택) — V2 payload와 동일한 2바이트가 '에코'로 들어오면 버린다
            if(is_valid_legacy(f)){
                if(echo_is_recent(f, 2)){
                    dump2("DROP self-echo(LEGACY)", f);
                    rb_consume(&rb,2);
                    continue;
                }
                dump2("LEGACY→process", f);
                process_frame(f);
                rb_consume(&rb,2);
                continue;
            }
                */
            uint8_t b0 = f[0];

            // HB/V2/ALIVE 헤더인데, 아직 프레임 길이만큼 안 모였으면 기다린다
            size_t avail = rb_avail(&rb);
            size_t need = 0;
            if (b0 == HB_MAGIC || b0 == RS422_V2_SOF) {
                need = 5;
            } else if (g_board == BOARD_C && (b0 == 0xFF || b0 == 0xEE)) {
                need = 4;
            }

            if (need) {
                if (avail < need) break;   // 다음 read를 기다림
                // 5/4바이트 이상인데 여기로 왔다? 앞단 파서가 못 먹은 쓰레기 → 1바이트만 버림
                dump2("RESYNC-1B", f);
                rb_consume(&rb,1);
                continue;
            }

            // 여기까지 내려왔으면 진짜 쓰레기 1바이트
            dump2("DROP-1B", f);
            rb_consume(&rb,1);

        }
    }
    return NULL;
}

//alive frame chk check
static inline uint8_t rs422_chk(uint8_t b0, uint8_t b1, uint8_t b2)
{
    return (uint8_t)(b0 ^ b1 ^ b2);
}
//led socket이랑 연결
static int init_led_sock(void)
{
    int s = socket(AF_UNIX, SOCK_DGRAM, 0);
    if (s < 0) return -1;

    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, LED_SOCKET_PATH, sizeof(addr.sun_path)-1);

    // bind() 없음: 수신측이 이 경로로 bind하고 있고, 우리는 sendto만 할 것
    g_led_sock = s;
    return 0;
}
//led한테 값이 바뀔때만 보내도록 비교

static void send_err_if_changed_locked(void)
{
    if (g_led_sock < 0) return;

    // 변화 없으면 아무 것도 안 함
    if (memcmp(&g_err_now, &g_err_last_sent, sizeof(err_wire_t)) == 0) {
        LOGF("LED agg no-change: zybo1=%u zybo2=%u zybo3=%u\n",
            g_err_now.zybo1, g_err_now.zybo2, g_err_now.zybo3);
        return;
    }

    LOGF("LED agg SEND -> zybo1=%u zybo2=%u zybo3=%u\n",
        g_err_now.zybo1, g_err_now.zybo2, g_err_now.zybo3);

    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, LED_SOCKET_PATH, sizeof(addr.sun_path)-1);

    ssize_t n = sendto(g_led_sock,
                    &g_err_now, sizeof(g_err_now),
                    0,
                    (struct sockaddr*)&addr, sizeof(addr));

    if (n == (ssize_t)sizeof(g_err_now)) {
        g_err_last_sent = g_err_now;   // 성공한 경우에만 갱신
        LOGF("LED agg send OK (%zd bytes)\n", n);
    } else {
        int e = errno;
        LOGF("LED agg send FAIL (ret=%zd, errno=%d: %s)\n",
            n, e, strerror(e));
    }
}

//패킷 검증, 바뀌면 전송
static void handle_rs422_alive(uint8_t sof, uint8_t type, uint8_t alive, uint8_t chk)
{
    // 유효성 체크
    if ((sof != 0xFF && sof != 0xEE) || type != 0x01)
        return;

    if (chk != rs422_chk(sof, type, alive))
        return;     // 체크섬 불일치 -> 무시

    uint8_t norm_alive = alive ? 1u : 0u;

    LOGF("ALIVE rx: sof=%02X type=%02X alive(raw)=%u chk=%02X norm=%u\n",
        sof, type, alive, chk, norm_alive);

    pthread_mutex_lock(&g_err_lock);

    if (sof == 0xFF) {
        g_alive1 = norm_alive;   // A 보드 alive 상태
    } else if (sof == 0xEE) {
        g_alive2 = norm_alive;   // B 보드 alive 상태
    }

    LOGF("ALIVE update(C): zybo1=%u zybo2=%u zybo3=%u\n",
        g_err_now.zybo1, g_err_now.zybo2, g_err_now.zybo3);

    // zybo3는 zybo_state_t에서 따로 갱신

    recalc_err_wire_locked(); // alive + HB 에러 OR 반영 후 LED 송신

    pthread_mutex_unlock(&g_err_lock);
}

// aliveX 와 hberrX를 OR해서 LED용 err_wire_t로 갱신
static void recalc_err_wire_locked(void)
{
    // 1) 둘 다 끊긴 경우: 셋 다 빨강
    if (g_hberr1 && g_hberr2) {
        g_err_now.zybo1 = 1u;
        g_err_now.zybo2 = 1u;
        g_err_now.zybo3 = 1u;
    } else {
        // 2) 그 외: aliveX || hberrX
        g_err_now.zybo1 = (g_alive1 || g_hberr1) ? 1u : 0u;
        g_err_now.zybo2 = (g_alive2 || g_hberr2) ? 1u : 0u;
        g_err_now.zybo3 = (g_alive3 || g_hberr3) ? 1u : 0u;
    }

    send_err_if_changed_locked(); // LED로 송신
}


static void update_zybo_alive(uint8_t alive)
{
    uint8_t norm_alive = alive ? 1u : 0u;

    pthread_mutex_lock(&g_err_lock);

    g_alive3 = norm_alive;   // C 보드 alive 상태
    g_zybo_state.zybo_alive = norm_alive;

    LOGF("LOCAL alive update: zybo3=%u (zybo1=%u zybo2=%u)\n",
        g_err_now.zybo3, g_err_now.zybo1, g_err_now.zybo2);


    recalc_err_wire_locked();  // alive + HB 에러 OR 반영 후 LED 송신

    pthread_mutex_unlock(&g_err_lock);
}

// ===== 공용 ==================================================================
static int bind_dgram(const char* path){
    int fd=socket(AF_UNIX,SOCK_DGRAM,0); if(fd<0) return -1;
    struct sockaddr_un sa; memset(&sa,0,sizeof(sa)); sa.sun_family=AF_UNIX;
    strncpy(sa.sun_path,path,sizeof(sa.sun_path)-1);
    unlink(path);
    if(bind(fd,(struct sockaddr*)&sa,sizeof(sa))<0){ close(fd); return -1; }
    return fd;
}
static void on_sigint(int s){ (void)s; g_stop=1; }

// ===== main ==================================================================
int main(void){
    srand((unsigned)time(NULL) ^ (unsigned)getpid());
    setvbuf(stdout,NULL,_IONBF,0); setvbuf(stderr,NULL,_IONBF,0);
    signal(SIGINT,on_sigint); signal(SIGTERM,on_sigint);

    /* 보드 자동 판별 (MAC/환경변수) */
    decide_board_by_mac();
    LOGF("Board decided: %s (MY_NODE=%u, error_sock=%s)\n",
            (g_board==BOARD_A)?"A":(g_board==BOARD_B)?"B":"C",
            (unsigned)g_my_node, g_error_sock);

    rs485_init_dir();                    // DE=0, /RE=0
    serial_fd=open_serial(SERIAL_DEV);   // 시리얼 준비
    if(serial_fd<0){ perror("serial"); return 1; }

    sock_can_rx=bind_dgram(SOCK_RX_FROM_CAN);
    if(sock_can_rx<0) perror("bind(rs422.sock)");

    if (g_board == BOARD_C) {
        if (init_led_sock() < 0) {
            perror("init_led_sock");
        }
    }


    pthread_t th_hb, th_rx, th_can, th_err, th_alive;
    pthread_create(&th_hb ,NULL,thread_hb_tx ,NULL);
    pthread_create(&th_rx ,NULL,thread_rx    ,NULL);
    pthread_create(&th_can,NULL,thread_can_ipc,NULL);
    pthread_create(&th_err,NULL,thread_error_tx,NULL);
    pthread_create(&th_alive,NULL,thread_led_alive,NULL);

    pthread_join(th_hb ,NULL);
    pthread_join(th_rx ,NULL);
    pthread_join(th_can,NULL);
    pthread_join(th_err,NULL);
    pthread_join(th_alive,NULL);

    if(sock_can_rx>=0) close(sock_can_rx);
    if(serial_fd>=0)    close(serial_fd);
    return 0;
}
