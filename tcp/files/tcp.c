// boardd.c — NTP 동기화 + unix sock 기반 상태 수집(단일 poll 스레드) + TCP 응답
// (BASIC/DETAIL + Ethernet 캐시 + binary 수신 + zybo_alive 송신 + 5초 주기 ping)
// build (PC):  gcc -O2 -std=gnu99 -Wall -pthread boardd.c -o boardd
// build (ARM): arm-linux-gnueabihf-gcc -O2 -std=gnu99 -Wall -pthread boardd.c -o boardd

#define _GNU_SOURCE
#define _POSIX_C_SOURCE 200809L

#include "protocol.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/sysinfo.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/wait.h>
#include <signal.h>
#include <stddef.h>
#include <poll.h>
#include <fcntl.h>
#include <sys/stat.h>

#ifndef ETH_IFACE
#define ETH_IFACE "eth0"
#endif

/* 기본값: 자기 자신에게 ping (루프백). 필요하면 빌드 옵션으로 override 가능 */
#ifndef ETH_TARGET_IP
#define ETH_TARGET_IP "127.0.0.1"
#endif

#define ETH_ERR_LOSS_PCT  20
#define ETH_ERR_FAIL_CNT   3
#define ETH_PERIOD_SEC     5   /* 5초마다 ping */

#define SVC_MAX_UNITS 6

/* 실제 ping 타겟 IP (기본은 ETH_TARGET_IP) */
static const char* g_eth_target_ip = ETH_TARGET_IP;

/* ===== 전역 상태 ===== */
static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;
static T_BITM_RACK_DETAIL_INFO g_temp;
static T_BITM_RACK_DETAIL_INFO g_can;
static T_BITM_RACK_DETAIL_INFO g_rs422;
static T_BITM_RACK_DETAIL_INFO g_mic;
static T_BITM_RACK_DETAIL_INFO g_spk;

/* LED 상태(led_err.sock 수신용) */
typedef struct {
    uint8_t led_link_ok;
    uint8_t led_power_err;
    uint8_t led_comm_err;
} led_status_t;
static led_status_t g_led = { .led_link_ok = 1, .led_power_err = 0, .led_comm_err = 0 };

/* Ethernet 캐시 */
static uint8_t g_eth_link_up=1;
static uint8_t g_eth_loss_pct=0;
static uint8_t g_eth_ping_fail_cnt=0;
static time_t  g_eth_last_ts=0;

/* ===== zybo_alive(에러 존재 여부) ===== */
typedef struct {
    uint8_t zybo_alive;   /* 1: 에러 존재, 0: 에러 없음 */
} zybo_state_t;
static zybo_state_t g_zybo = { .zybo_alive = 0 };

/* zybo_alive 전송기 (/run/zybo_alive.sock) */
static int zybo_alive_send(uint8_t alive)
{
    static int fd = -1;
    static struct sockaddr_un addr;
    static socklen_t alen = 0;
    const char* PATH = "/run/zybo_alive.sock";

    /* 목적지 소켓 없으면 조용히 스킵 */
    struct stat st;
    if (stat(PATH, &st) != 0 || !S_ISSOCK(st.st_mode)) {
        return 0;
    }

    if (fd < 0) {
        fd = socket(AF_UNIX, SOCK_DGRAM, 0);
        if (fd < 0) { return -1; }
        memset(&addr, 0, sizeof(addr));
        addr.sun_family = AF_UNIX;
        strncpy(addr.sun_path, PATH, sizeof(addr.sun_path)-1);
        alen = sizeof(addr);
    }

    zybo_state_t stt = { .zybo_alive = alive };
    if (sendto(fd, &stt, sizeof(stt), 0, (struct sockaddr*)&addr, alen) < 0) {
        if (errno == ENOENT) return 0;
        int e = errno; close(fd); fd = -1; errno = e;
        return -1;
    }

    fprintf(stderr, "[TCP-> RS422 Zyboalive] alive :%u\n", (unsigned)alive);
    return 0;
}

/* ===== rs422 error_t ===== */
typedef struct __attribute__((packed)) {
    uint8_t  socket_err;
    uint16_t tx_err;
    uint16_t rx_err;
} rs422_error_t;

/* ===== statusd → boardd : status packet ===== */
typedef struct __attribute__((packed)) {
    uint8_t  app_ver[4];
    uint8_t  fpga_ver[4];
    uint8_t  cpu_usage;        /* 0~100 */
    uint8_t  mem_usage;        /* 0~100 */
    int16_t  board_temp_dC;    /* 0.1°C (signed) */
    uint8_t  svc_status[SVC_MAX_UNITS];
    uint32_t uptime_sec;       /* LE */
} status_pkt_t;

typedef struct {
    uint8_t  app_ver[4];
    uint8_t  fpga_ver[4];
    uint8_t  cpu_usage;
    uint8_t  mem_usage;
    int16_t  board_temp_dC;          /* 0.1°C */
    uint8_t  svc_status[SVC_MAX_UNITS];
    uint32_t uptime_sec;             /* host endian */
    uint8_t  valid;
} g_status_t;

typedef struct __attribute__((packed)) {
    uint8_t  lm75b_link_ok;
    uint8_t  lm75b_power_err;
    uint8_t  lm75b_comm_err;
    uint16_t lm75b_temp_centi_be;   // 0.01°C, BE
} lm75_min_pkt_t;

typedef struct __attribute__((packed)) {
    uint8_t  spk_power_err;
    uint8_t  spk_comm_err;
} spk_min_pkt_t;

/* LED 에러 패킷 (led.c와 동일 포맷) */
typedef struct __attribute__((packed)) {
    uint8_t led_link_ok;   // 1=OK, 0=LINK ERROR
    uint8_t led_comm_err;  // 1=COMM ERROR
} led_err_packet_t;

static g_status_t g_status = {0};

/* ===== NTP one-shot 플래그 ===== */
static volatile int g_ntp_sync_done = 0;

/* ===== NTP 동기화 ===== */
static void trigger_ntp_sync_async(void){
    pid_t pid=fork();
    if(pid==0){
        execl("/usr/local/bin/pc-ntp-sync.sh","pc-ntp-sync.sh",(char*)NULL);
        _exit(127);
    }
    if(pid>0){
        int st;
        (void)waitpid(pid,&st,WNOHANG);
        fprintf(stderr,"[time] NTP sync triggered (script exec).\n");
        fflush(stderr);
    }
    else {
        perror("[time] fork");
    }
}

static void* time_sync_probe_thread(void* arg){
    int delay_ms = (int)(intptr_t)arg;
    struct timespec ts={.tv_sec=delay_ms/1000,.tv_nsec=(delay_ms%1000)*1000000L};
    nanosleep(&ts,NULL);

    char buf[64]={0};
    FILE* fp=popen("timedatectl show -p NTPSynchronized --value 2>/dev/null","r");
    if(fp){
        if(!fgets(buf,sizeof(buf),fp)) buf[0]='\0';
        pclose(fp);
        for(int i=0;buf[i];++i){
            if(buf[i]=='\n'||buf[i]=='\r'){ buf[i]=0; break; }
        }
    } else {
        strcpy(buf,"unknown");
    }

    time_t now=time(NULL);
    struct tm g,k;
    gmtime_r(&now,&g);
    localtime_r(&now,&k);
    char utc_s[64],loc_s[64];
    strftime(utc_s,sizeof(utc_s),"%Y-%m-%d %H:%M:%S UTC",&g);
    strftime(loc_s,sizeof(loc_s),"%Y-%m-%d %H:%M:%S %Z",&k);

    fprintf(stderr,"[time] NTP sync state after %d ms: %s\n[time] now(UTC)=%s, now(local)=%s\n",
            delay_ms, buf[0]?buf:"unknown", utc_s, loc_s);
    fflush(stderr);
    return NULL;
}

static void log_time_sync_result_later(int delay_ms){
    pthread_t th; pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr,PTHREAD_CREATE_DETACHED);
    if(pthread_create(&th,&attr,time_sync_probe_thread,(void*)(intptr_t)delay_ms)!=0)
        perror("[time] pthread_create");
    pthread_attr_destroy(&attr);
}

/* 한 번만 NTP 동기화 트리거 */
static void trigger_ntp_sync_once(void)
{
    if (__sync_lock_test_and_set(&g_ntp_sync_done, 1))
        return;

    trigger_ntp_sync_async();
    log_time_sync_result_later(2500);  // 2.5초 뒤 상태 로그 찍기
}

/* ===== 시간 함수 ===== */
static void fill_utc_now(T_UtcTime* t){
    struct timespec ts; clock_gettime(CLOCK_REALTIME,&ts);
    time_t sec=ts.tv_sec; struct tm lt; localtime_r(&sec,&lt);
    t->year=(uint16_t)(lt.tm_year+1900);
    t->mon=(uint8_t)(lt.tm_mon+1);
    t->day=(uint8_t)lt.tm_mday;
    t->hour=(uint8_t)lt.tm_hour;
    t->min=(uint8_t)lt.tm_min;
    t->sec=(uint8_t)lt.tm_sec;
    t->sec100=(uint8_t)((ts.tv_nsec/10000000L)%100);
}

/* ===== Ethernet 유틸 ===== */
static int read_int_file(const char* path,int* out){
    FILE* f=fopen(path,"r"); if(!f)return -1;
    long v; int ok=(fscanf(f,"%ld",&v)==1); fclose(f);
    if (!ok) return -2;
    *out = (int)v; return 0;
}
static int is_link_up(const char* ifname){
    char p[128]; snprintf(p,sizeof(p),"/sys/class/net/%s/carrier",ifname);
    int v=0; if(read_int_file(p,&v)==0) return v?1:0;
    snprintf(p,sizeof(p),"/sys/class/net/%s/operstate",ifname);
    FILE* f=fopen(p,"r"); if(!f) return 0;
    char s[32]={0}; if(!fgets(s,sizeof(s),f)){fclose(f);return 0;}
    fclose(f); return (strncmp(s,"up",2)==0);
}

static int ping_loss_pct_if(const char* ifname, const char* target)
{
    (void)ifname;  /* Zybo에서는 인터페이스 강제하지 않음 */

    char cmd[256];
    snprintf(cmd, sizeof(cmd),
        "ping -c 3 -w 2 %s 2>/dev/null | "
        "grep -o '[0-9]\\+%% packet loss' | tr -d '%%' | awk '{print $1}'",
        target);

    FILE* fp = popen(cmd, "r");
    if (!fp) return 100;

    int loss = 100;
    if (fscanf(fp, "%d", &loss) != 1)
        loss = 100;
    pclose(fp);

    if (loss < 0)   loss = 0;
    if (loss > 100) loss = 100;
    return loss;
}


/* 실제 ping + 결과 캐시에 반영 + 로그 */
static void probe_eth_detail(const char* ifname,const char* target,
                             uint8_t* link_up,uint8_t* loss_pct,uint8_t* fail_cnt)
{
    uint8_t lu=is_link_up(ifname)?1:0;
    uint8_t lp = lu ? (uint8_t)ping_loss_pct_if(ifname, target) : 100;

    pthread_mutex_lock(&g_lock);
    if(lp>0){ if(g_eth_ping_fail_cnt<255) g_eth_ping_fail_cnt++; }
    else if(g_eth_ping_fail_cnt>0) g_eth_ping_fail_cnt--;
    g_eth_link_up=lu; g_eth_loss_pct=lp;
    if (link_up)  *link_up = g_eth_link_up;
    if (loss_pct) *loss_pct = g_eth_loss_pct;
    if (fail_cnt) *fail_cnt = g_eth_ping_fail_cnt;
    g_eth_last_ts = time(NULL);
    pthread_mutex_unlock(&g_lock);

    fprintf(stderr,
        "[ETH] probe: if=%s target=%s link_up=%u loss=%u%% fail_cnt=%u ts=%ld\n",
        ifname, target,
        (unsigned)g_eth_link_up,
        (unsigned)g_eth_loss_pct,
        (unsigned)g_eth_ping_fail_cnt,
        (long)g_eth_last_ts);
}

/* 5초 주기 ping 스레드 */
static void* eth_periodic_thread(void* arg)
{
    (void)arg;
    while (1) {
        uint8_t lu, lp, fc;
        probe_eth_detail(ETH_IFACE, g_eth_target_ip, &lu, &lp, &fc);
        sleep(ETH_PERIOD_SEC);
    }
    return NULL;
}

/* ===== 단일 poll 스레드로 모든 수신 소켓 통합 ===== */
typedef enum { S_LM75, S_CAN, S_MIC, S_SPK, S_RS422_ERR, S_STATUS, S_LED } SockType;

typedef struct {
    const char* path;
    int fd;
    SockType type;
} Sock;

static Sock g_socks[] = {
    { "/run/lm75.sock",             -1, S_LM75      },
    { "/run/can.sock",              -1, S_CAN       },
    { "/run/micerror.sock",         -1, S_MIC       },
    { "/run/speaker_error.sock",    -1, S_SPK       },
    { "/run/error.C1.rs422.sock",   -1, S_RS422_ERR },
    { "/run/status.sock",           -1, S_STATUS    },
    { "/run/led_err.sock",          -1, S_LED       },
};
static const int NSOCK = (int)(sizeof(g_socks)/sizeof(g_socks[0]));

static int bind_unix_nb(const char* path){
    int fd = socket(AF_UNIX, SOCK_DGRAM|SOCK_CLOEXEC, 0);
    if (fd<0) return -1;
    int rcvsz = 262144; setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &rcvsz, sizeof(rcvsz));
    int flags = fcntl(fd, F_GETFL, 0); fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    struct sockaddr_un a; memset(&a,0,sizeof(a));
    a.sun_family = AF_UNIX;
    if (path[0]=='@') { a.sun_path[0]='\0'; strncpy(a.sun_path+1,path+1,sizeof(a.sun_path)-2); }
    else { unlink(path); strncpy(a.sun_path, path, sizeof(a.sun_path)-1); }
    if (bind(fd,(struct sockaddr*)&a,sizeof(a))<0){ close(fd); return -1; }
    return fd;
}

/* ===== BASIC 마스크/ALIVE 관련 정의 ===== */
#define ALIVE_ERR_MASK ( (1u<<RS0_TEMP_ERR)   | (1u<<RS0_CAN_ERR)   | \
                         (1u<<RS0_SERIAL_ERR) | (1u<<RS0_ETH_ERR)   | \
                         (1u<<RS0_MIC_ERR)    | (1u<<RS0_SPK_ERR)   | \
                         (1u<<RS0_SVC_ERR)    | (1u<<RS0_LED_ERR) )

static uint32_t build_basic_mask_from_detail(void); /* forward */

static uint8_t g_last_alive = 0;
static time_t  g_last_alive_ts = 0;
#define ALIVE_SEND_PERIOD_SEC 2

static void zybo_alive_maybe_send_from_mask(uint32_t mask_host)
{
    uint8_t alive = ( (mask_host & ALIVE_ERR_MASK) != 0 ) ? 1 : 0;
    time_t now = time(NULL);
    if (alive != g_last_alive || now - g_last_alive_ts >= ALIVE_SEND_PERIOD_SEC) {
        g_last_alive    = alive;
        g_last_alive_ts = now;
        g_zybo.zybo_alive = alive;
        (void)zybo_alive_send(alive);
    }
}

static void zybo_alive_recalc_and_send(void)
{
    uint32_t mask_host = build_basic_mask_from_detail();
    zybo_alive_maybe_send_from_mask(mask_host);
}

static void* poll_loop(void* arg){
    (void)arg;
    for (int i=0;i<NSOCK;i++){
        g_socks[i].fd = bind_unix_nb(g_socks[i].path);
        if (g_socks[i].fd >= 0)
            printf("[boardd] listening (binary) %s\n", g_socks[i].path);
        else
            fprintf(stderr,"[boardd] bind fail: %s (%s)\n", g_socks[i].path, strerror(errno));
    }

    struct pollfd pfds[NSOCK];
    for (int i=0;i<NSOCK;i++){ pfds[i].fd = g_socks[i].fd; pfds[i].events = POLLIN; }

    const int MAX_DRAIN = 16;
    uint8_t buf[512];

    for(;;){
        int r = poll(pfds, NSOCK, 1000);
        if (r <= 0) continue;

        for (int i=0;i<NSOCK;i++){
            if (pfds[i].fd < 0) continue;
            if (!(pfds[i].revents & POLLIN)) continue;

            int drained = 0;
            for (;;){
                ssize_t n = recv(pfds[i].fd, buf, sizeof(buf), MSG_DONTWAIT);
                if (n <= 0) break;
                drained++;

                switch (g_socks[i].type){
                    case S_STATUS:
                        if (n == (ssize_t)sizeof(status_pkt_t)){
                            status_pkt_t *ps=(status_pkt_t*)buf;
                            pthread_mutex_lock(&g_lock);
                            memcpy(g_status.app_ver,  ps->app_ver,  4);
                            memcpy(g_status.fpga_ver, ps->fpga_ver, 4);
                            g_status.cpu_usage      = ps->cpu_usage;
                            g_status.mem_usage      = ps->mem_usage;
                            g_status.board_temp_dC  = ps->board_temp_dC;
                            memcpy(g_status.svc_status, ps->svc_status, SVC_MAX_UNITS);
                            g_status.uptime_sec     = ps->uptime_sec;
                            g_status.valid          = 1;
                            pthread_mutex_unlock(&g_lock);
                            printf("[recv][SYS ] cpu=%u%% mem=%u%% up=%us temp=%.1f°C\n",
                                   ps->cpu_usage, ps->mem_usage, ps->uptime_sec, ps->board_temp_dC/10.0);
                            printf("[recv][SVC ] ");
                            for (int si=0; si<SVC_MAX_UNITS; ++si) {
                                printf("%u%s", ps->svc_status[si], (si==SVC_MAX_UNITS-1)?"":" ");
                            }
                            printf("\n");
                            zybo_alive_recalc_and_send();
                        }
                        break;

                    case S_LM75:
                        if (n == (ssize_t)sizeof(lm75_min_pkt_t)){
                            lm75_min_pkt_t *q=(lm75_min_pkt_t*)buf;
                            pthread_mutex_lock(&g_lock);
                            g_temp.lm75b_link_ok    = q->lm75b_link_ok;
                            g_temp.lm75b_power_err  = q->lm75b_power_err;
                            g_temp.lm75b_comm_err   = q->lm75b_comm_err;
                            g_temp.lm75b_temp_centi = q->lm75b_temp_centi_be;
                            pthread_mutex_unlock(&g_lock);
                            printf("[recv][TEMP] (5B) pwr=%u comm=%u temp=%.2f°C\n",
                                   q->lm75b_power_err, q->lm75b_comm_err, ntohs(q->lm75b_temp_centi_be)/100.0);
                            zybo_alive_recalc_and_send();
                        }
                        break;

                    case S_CAN:
                        if (n == (ssize_t)sizeof(T_BITM_RACK_DETAIL_INFO)){
                            T_BITM_RACK_DETAIL_INFO *p=(T_BITM_RACK_DETAIL_INFO*)buf;
                            pthread_mutex_lock(&g_lock);
                            memcpy(&g_can, p, sizeof(g_can));
                            pthread_mutex_unlock(&g_lock);
                            printf("[recv][CAN ] state=%u tx=%u rx=%u\n",
                                   p->can_bus_state, p->can_tx_err_cnt, p->can_rx_err_cnt);
                            zybo_alive_recalc_and_send();
                        }
                        break;

                    case S_MIC:
                        if (n == (ssize_t)sizeof(T_BITM_RACK_DETAIL_INFO)){
                            T_BITM_RACK_DETAIL_INFO *p=(T_BITM_RACK_DETAIL_INFO*)buf;
                            pthread_mutex_lock(&g_lock);
                            memcpy(&g_mic, p, sizeof(g_mic));
                            pthread_mutex_unlock(&g_lock);
                            printf("[recv][MIC ] pwr_err=%u comm_err=%u\n",
                                   p->mic_power_err, p->mic_comm_err);
                            zybo_alive_recalc_and_send();
                        }
                        break;

                    case S_SPK:
                        if (n == (ssize_t)sizeof(spk_min_pkt_t)) {
                            spk_min_pkt_t *p = (spk_min_pkt_t*)buf;
                            pthread_mutex_lock(&g_lock);
                            g_spk.spk_power_err = p->spk_power_err;
                            g_spk.spk_comm_err  = p->spk_comm_err;
                            pthread_mutex_unlock(&g_lock);
                            printf("[recv][SPK ] pwr_err=%u comm_err=%u\n",
                                   p->spk_power_err, p->spk_comm_err);
                            zybo_alive_recalc_and_send();
                        }
                        break;

                    case S_RS422_ERR:
                        if (n == (ssize_t)sizeof(rs422_error_t)){
                            rs422_error_t *e=(rs422_error_t*)buf;
                            pthread_mutex_lock(&g_lock);
                            g_rs422.rs422_socket_err = e->socket_err;
                            g_rs422.rs422_tx_err     = htons(e->tx_err);
                            g_rs422.rs422_rx_err     = htons(e->rx_err);
                            pthread_mutex_unlock(&g_lock);
                            printf("[recv][RS422-ERR] sock=%u tx=%u rx=%u\n",
                                   e->socket_err, e->tx_err, e->rx_err);
                            zybo_alive_recalc_and_send();
                        }
                        break;

                    case S_LED:
                        if (n == (ssize_t)sizeof(led_err_packet_t)) {
                            led_err_packet_t *p = (led_err_packet_t*)buf;
                            pthread_mutex_lock(&g_lock);
                            g_led.led_link_ok   = p->led_link_ok;
                            g_led.led_comm_err  = p->led_comm_err;
                            pthread_mutex_unlock(&g_lock);
                            printf("[recv][LED ] link_ok=%u comm_err=%u\n",
                                   p->led_link_ok, p->led_comm_err);
                            zybo_alive_recalc_and_send();
                        }
                        break;
                }

                if (drained >= MAX_DRAIN) break;
            }
        }
    }
    return NULL;
}

/* ===== DETAIL 채우기 ===== */
static void fill_detail_common(T_BITM_RACK_DETAIL_INFO* d)
{
    memset(d,0,sizeof(*d));
    pthread_mutex_lock(&g_lock);

    memcpy(d,&g_temp,sizeof(*d)); // TEMP 기반 복사

    /* CAN */
    d->can_bus_state   = g_can.can_bus_state;
    d->can_tx_err_cnt  = g_can.can_tx_err_cnt;
    d->can_rx_err_cnt  = g_can.can_rx_err_cnt;

    /* RS422 */
    d->rs422_socket_err= g_rs422.rs422_socket_err;
    d->rs422_tx_err    = g_rs422.rs422_tx_err;
    d->rs422_rx_err    = g_rs422.rs422_rx_err;

    /* MIC/SPK */
    d->mic_power_err   = g_mic.mic_power_err;
    d->mic_comm_err    = g_mic.mic_comm_err;
    d->spk_power_err   = g_spk.spk_power_err;
    d->spk_comm_err    = g_spk.spk_comm_err;

    /* svcwatch-mini 값 */
    if (g_status.valid) {
        memcpy(d->app_ver,  g_status.app_ver,  4);
        memcpy(d->fpga_ver, g_status.fpga_ver, 4);
        d->cpu_usage         = g_status.cpu_usage;
        d->mem_usage         = g_status.mem_usage;
        d->temperature_board = htons((uint16_t)g_status.board_temp_dC);
        for (int i=0;i<6;i++) d->service_error[i] = g_status.svc_status[i];
        d->uptime_sec        = htonl(g_status.uptime_sec);
    }

    /* Ethernet (캐시) */
    d->eth_link_up        = g_eth_link_up;
    d->eth_loss_pct       = g_eth_loss_pct;
    d->eth_ping_fail_cnt  = g_eth_ping_fail_cnt;

    /* LED */
    d->led_link_ok   = g_led.led_link_ok;
    d->led_power_err = g_led.led_power_err;
    d->led_comm_err  = g_led.led_comm_err;

    pthread_mutex_unlock(&g_lock);
}

/* ===== BASIC 계산(캐시만 사용; ping 호출 없음) ===== */
static uint32_t build_basic_mask_from_detail(void)
{
    uint32_t mask=0;
    pthread_mutex_lock(&g_lock);

    int temp_centi=ntohs(g_temp.lm75b_temp_centi);
    if(!g_temp.lm75b_link_ok || g_temp.lm75b_power_err || g_temp.lm75b_comm_err ||
       temp_centi>8000 || temp_centi<-4000)
        mask|=(1u<<RS0_TEMP_ERR);

    if(g_can.can_bus_state!=1 || g_can.can_tx_err_cnt>0 || g_can.can_rx_err_cnt>0)
        mask|=(1u<<RS0_CAN_ERR);

    if(g_rs422.rs422_socket_err || (ntohs(g_rs422.rs422_tx_err)>0 && ntohs(g_rs422.rs422_rx_err)>0))
        mask|=(1u<<RS0_SERIAL_ERR);

    if(g_mic.mic_power_err || g_mic.mic_comm_err)
        mask|=(1u<<RS0_MIC_ERR);

    if(g_spk.spk_power_err || g_spk.spk_comm_err)
        mask|=(1u<<RS0_SPK_ERR);

    if (g_eth_link_up==0 ||
        g_eth_loss_pct >= ETH_ERR_LOSS_PCT ||
        g_eth_ping_fail_cnt >= ETH_ERR_FAIL_CNT)
        mask|=(1u<<RS0_ETH_ERR);

    if (g_led.led_link_ok == 0 || g_led.led_comm_err || g_led.led_power_err)
        mask|=(1u<<RS0_LED_ERR);

    /* ===== svc_status 기반 SVC_ERR 계산 =====
     * - 0 = OK
     * - 1 = 에러 (너 말대로라면 '1'이 에러)
     */
    uint8_t svc_err = 0;
    if (g_status.valid) {
        for (int i = 0; i < SVC_MAX_UNITS; i++) {
            if (g_status.svc_status[i] == 1) {
                svc_err = 1;
                break;
            }
        }
    }
    if (svc_err) {
        mask |= (1u << RS0_SVC_ERR);
    }

    /* 디버그 용: 실제로 어떤 값으로 계산됐는지 확인 */
    fprintf(stderr,
        "[BASIC] svc_status=%u %u %u %u %u %u -> svc_err=%u, mask=0x%08X\n",
        g_status.svc_status[0], g_status.svc_status[1],
        g_status.svc_status[2], g_status.svc_status[3],
        g_status.svc_status[4], g_status.svc_status[5],
        svc_err, mask);

    pthread_mutex_unlock(&g_lock);

    zybo_alive_maybe_send_from_mask(mask);
    return mask;
}


/* ===== TCP 요청 처리 ===== */
static void handle_request(int cfd,uint16_t ic_host){
    uint8_t mode=(ic_host>>12)&0x7;
    uint8_t subj=(ic_host>>8)&0xF;
    uint8_t act =(ic_host>>4)&0xF;
    printf("[boardd] REQ 0x%04X subj=%u act=%u mode=%u\n",ic_host,subj,act,mode);

    trigger_ntp_sync_once();

    T_TCP_HEADER h; memset(&h,0,sizeof(h));
    h.start   = htonl(TCP_MAGIC_START);
    h.sender  = 0x10;
    h.receiver= 0x01;
    h.infocode= htons(ic_host | IC_RESP);
    fill_utc_now(&h.time);

    T_BITM_BASIC_INFO b; memset(&b,0,sizeof(b));
    b.op_mode=1;
    fill_utc_now(&b.check_time);
    b.position=0; b.assigned_func=1;
    uint32_t mask=build_basic_mask_from_detail();
    b.raw_states.v[0]=htonl(mask);

    T_TCP_TAIL t; t.tail = htonl(TCP_MAGIC_TAIL);

    uint8_t buf[512]; size_t off=0;
    memcpy(buf+off, &h, sizeof(h)); off += sizeof(h);
    memcpy(buf+off, &b, sizeof(b)); off += sizeof(b);

    if (mode==IC_NRT && act==IC_ACT_DETAIL){
        T_BITM_RACK_DETAIL_INFO d; fill_detail_common(&d);
        memcpy(buf+off, &d, sizeof(d)); off += sizeof(d);
    }

    memcpy(buf+off, &t, sizeof(t)); off += sizeof(t);
    h.length = htonl((uint32_t)off);
    memcpy(buf, &h, sizeof(h));

    ssize_t w = write(cfd, buf, off); (void)w;

    printf("[boardd] SENT len=%zu (mask=0x%08X)  [ETH link:%u loss:%u%% fail:%u ts:%ld]  zybo_alive=%u\n",
           off, ntohl(b.raw_states.v[0]),
           g_eth_link_up, g_eth_loss_pct, g_eth_ping_fail_cnt, (long)g_eth_last_ts,
           g_zybo.zybo_alive);
}

/* ===== 메인 ===== */
int main(void)
{
    /* Ethernet 5초 주기 ping 스레드 시작 (자기 자신으로 ping) */
    pthread_t th_eth;
    pthread_create(&th_eth, NULL, eth_periodic_thread, NULL);

    /* 단일 poll 수신 스레드 시작 */
    pthread_t th_poll;
    pthread_create(&th_poll, NULL, poll_loop, NULL);

    /* TCP 서버 */
    int sfd=socket(AF_INET,SOCK_STREAM,0);
    if(sfd<0){perror("socket");return 1;}
    int yes=1; setsockopt(sfd,SOL_SOCKET,SO_REUSEADDR,&yes,sizeof(yes));
    struct sockaddr_in sa={0};
    sa.sin_family=AF_INET; sa.sin_port=htons(TCP_PORT_DEFAULT);
    sa.sin_addr.s_addr=htonl(INADDR_ANY);

    if(bind(sfd,(struct sockaddr*)&sa,sizeof(sa))<0){perror("bind");return 1;}
    if(listen(sfd,4)<0){perror("listen");return 1;}
    printf("[boardd] listening on port %d...\n",TCP_PORT_DEFAULT); fflush(stdout);

    uint8_t rx[256];
    for(;;){
        int cfd=accept(sfd,NULL,NULL);
        if(cfd<0){perror("accept");continue;}
        printf("[boardd] client connected\n");
        for(;;){
            ssize_t n=read(cfd,rx,sizeof(rx));
            if(n<=0){ if(n<0)perror("read"); close(cfd); break; }
            if((size_t)n<sizeof(T_TCP_HEADER)) continue;
            T_TCP_HEADER in; memcpy(&in,rx,sizeof(in));
            uint16_t ic_host=ntohs(in.infocode)&0x7FFF;
            handle_request(cfd,ic_host);
        }
    }
    close(sfd);
    return 0;
}
