// svcwatch-mini.c  (gcc -O2 -std=gnu99 -Wall -o svcwatch-mini svcwatch-mini.c)
// 각 서비스가 "active" 인지만 확인해서 .sock(UNIX DGRAM)으로 전송 + 콘솔 출력
// - /etc/status.conf 사용 안 함
// - MAC 주소로 보드 A/B/C 판별 → 보드별 서비스 유닛 목록 사용

#define _GNU_SOURCE
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <time.h>
#include <sys/wait.h>   // WIFEXITED, WEXITSTATUS
#include <errno.h>
#include <strings.h>    // strcasecmp

#define MAX_UNITS 6

/* ===== 설정 ===== */
#define STATUS_SOCK_PATH   "/run/status.sock"  /* 전송 대상 UNIX DGRAM 소켓 경로 */
#define STATUS_INTERVAL_SEC 2                  /* 전송 주기(초) */
#define ETH_IFACE          "eth0"             /* MAC 읽어올 인터페이스 */

/* ===== MAC 기반 보드 구분 ===== */
typedef enum {
    BOARD_UNKNOWN = 0,
    BOARD_A,
    BOARD_B,
    BOARD_C
} board_t;

/* 여기 MAC ↔ 보드 매핑은 네가 쓰는 값 그대로 */
static board_t detect_board_by_mac(void)
{
    char path[128];
    snprintf(path, sizeof(path), "/sys/class/net/%s/address", ETH_IFACE);

    FILE* f = fopen(path, "r");
    if (!f) {
        perror("[statusd] open mac");
        return BOARD_UNKNOWN;
    }

    char mac[32] = {0};
    if (!fgets(mac, sizeof(mac), f)) {
        fclose(f);
        return BOARD_UNKNOWN;
    }
    fclose(f);

    /* 개행 제거 */
    for (int i = 0; mac[i]; ++i) {
        if (mac[i] == '\n' || mac[i] == '\r') {
            mac[i] = 0;
            break;
        }
    }

    /* /sys 쪽은 보통 소문자 "00:0a:35:..." 형태라 대소문자 무시 비교 */
    if (!strcasecmp(mac, "00:0a:35:00:1e:52")) return BOARD_A;
    if (!strcasecmp(mac, "00:0a:35:00:1e:53")) return BOARD_B;
    if (!strcasecmp(mac, "00:0a:35:00:1e:54")) return BOARD_C;

    return BOARD_UNKNOWN;
}

/* ===== 패킷 구조 (boardd와 동일) ===== */
typedef struct __attribute__((packed)) {
    uint8_t  app_ver[4];
    uint8_t  fpga_ver[4];
    uint8_t  cpu_usage;        // 0~100
    uint8_t  mem_usage;        // 0~100
    int16_t  board_temp_dC;    // 0.1°C 단위
    uint8_t  svc_status[6];    // 0=정상, 1=에러
    uint32_t uptime_sec;       // LE
} status_pkt_t;

/* ===== 파일에서 정확히 N바이트 읽기 ===== */
static int read_exact(const char* path, void* buf, size_t n) {
    FILE* f = fopen(path, "rb");
    if (!f) return -1;
    size_t r = fread(buf, 1, n, f);
    fclose(f);
    return (r == n) ? 0 : -1;
}

/* ===== 버전 파일 읽기 ===== */
static void get_app_ver(uint8_t out[4]) {
    if (read_exact("/etc/app_ver.bin", out, 4) < 0) memset(out, 0, 4);
}
static void get_fpga_ver(uint8_t out[4]) {
    if (read_exact("/etc/fpga_ver.bin", out, 4) < 0) memset(out, 0, 4);
}

/* ===== CPU/MEM/uptime ===== */
typedef struct { unsigned long long u, n, s, i, iw, irq, sirq, st; } cpu_sample_t;

static int read_cpu_sample(cpu_sample_t* cs) {
    FILE* f = fopen("/proc/stat", "r");
    if (!f) return -1;
    int ok = fscanf(f, "cpu %llu %llu %llu %llu %llu %llu %llu %llu",
                    &cs->u, &cs->n, &cs->s, &cs->i, &cs->iw, &cs->irq, &cs->sirq, &cs->st);
    fclose(f);
    return (ok >= 4) ? 0 : -1;
}

static uint8_t get_cpu_usage_percent(void) {
    cpu_sample_t a, b;
    if (read_cpu_sample(&a) < 0) return 0;
    usleep(100 * 1000);  // 100ms 샘플링
    if (read_cpu_sample(&b) < 0) return 0;

    unsigned long long idle_a = a.i + a.iw;
    unsigned long long idle_b = b.i + b.iw;
    unsigned long long total_a = a.u + a.n + a.s + a.i + a.iw + a.irq + a.sirq + a.st;
    unsigned long long total_b = b.u + b.n + b.s + b.i + b.iw + b.irq + b.sirq + b.st;

    unsigned long long idle_d = (idle_b > idle_a) ? (idle_b - idle_a) : 0;
    unsigned long long total_d = (total_b > total_a) ? (total_b - total_a) : 1;
    unsigned long long busy_d = (total_d > idle_d) ? (total_d - idle_d) : 0;

    unsigned int pct = (unsigned int)((busy_d * 100ULL) / total_d);
    if (pct > 100) pct = 100;
    return (uint8_t)pct;
}

static uint8_t get_mem_usage_percent(void) {
    long total = 0, avail = 0;
    FILE* f = fopen("/proc/meminfo", "r");
    if (!f) return 0;
    char key[64]; long val;
    while (fscanf(f, "%63s %ld kB", key, &val) == 2) {
        if (!strcmp(key, "MemTotal:")) total = val;
        else if (!strcmp(key, "MemAvailable:")) avail = val;
    }
    fclose(f);
    if (total <= 0) return 0;
    long used = total - avail;
    if (used < 0) used = 0;
    unsigned int pct = (unsigned int)((used * 100L) / total);
    if (pct > 100) pct = 100;
    return (uint8_t)pct;
}

static uint32_t get_uptime_sec(void) {
    double up = 0.0;
    FILE* f = fopen("/proc/uptime", "r");
    if (!f) return 0;
    if (fscanf(f, "%lf", &up) != 1) { fclose(f); return 0; }
    fclose(f);
    if (up < 0) up = 0;
    if (up > 4294967295.0) up = 4294967295.0;
    return (uint32_t)(up + 0.5);
}

/* ===== 온도 읽기 (IIO) ===== */
static int read_value_from_file(const char* filepath) {
    FILE* fp = fopen(filepath, "r");
    if (fp == NULL) { return -errno; }
    int v = 0;
    if (fscanf(fp, "%d", &v) != 1) { fclose(fp); return -EIO; }
    fclose(fp);
    return v;
}
static double read_double_from_file(const char* filepath) {
    FILE* fp = fopen(filepath, "r");
    if (fp == NULL) { return -errno; }
    double v = 0.0;
    if (fscanf(fp, "%lf", &v) != 1) { fclose(fp); return -EIO; }
    fclose(fp);
    return v;
}

/* 내부 온도: raw/scale/offset → 0.1°C(int16) */
static int16_t get_board_temp_dC(void) {
    const char* raw_file    = "/sys/bus/iio/devices/iio:device0/in_temp0_raw";
    const char* offset_file = "/sys/bus/iio/devices/iio:device0/in_temp0_offset";
    const char* scale_file  = "/sys/bus/iio/devices/iio:device0/in_temp0_scale";

    int raw_val    = read_value_from_file(raw_file);
    int offset_val = read_value_from_file(offset_file);
    double scale_val = read_double_from_file(scale_file);
    if (raw_val < 0 || offset_val < 0 || scale_val < 0.0)
        return 432; // 실패 시 43.2°C로 대체

    double temp_celsius = (raw_val + offset_val) * scale_val / 1000.0;
    return (int16_t)(temp_celsius * 10.0);
}

/* ===== systemd unit 상태 ===== */
static int is_active(const char* unit) {
    char cmd[256];
    snprintf(cmd, sizeof(cmd), "systemctl is-active --quiet %s", unit);
    int rc = system(cmd);
    return (rc != -1 && WIFEXITED(rc) && WEXITSTATUS(rc) == 0) ? 1 : 0;
}

/* ===== 보기 좋은 출력 ===== */
static void print_status_plain(const status_pkt_t* pkt,
                               const char* units[], int ucnt) {
    time_t now = time(NULL);
    struct tm lt; localtime_r(&now, &lt);
    char ts[32]; strftime(ts, sizeof(ts), "%F %T", &lt);

    printf("[statusd] ts=%s  app=%u.%u.%u.%u  fpga=%u.%u.%u.%u  cpu=%u%%  mem=%u%%  up=%us  temp=%.1f°C\n",
           ts,
           pkt->app_ver[0],  pkt->app_ver[1],  pkt->app_ver[2],  pkt->app_ver[3],
           pkt->fpga_ver[0], pkt->fpga_ver[1], pkt->fpga_ver[2], pkt->fpga_ver[3],
           pkt->cpu_usage, pkt->mem_usage, pkt->uptime_sec, pkt->board_temp_dC/10.0);

    printf("[statusd] service_error: ");
    for (int i = 0; i < ucnt; i++) {
        printf("%s=%u%s", units[i], pkt->svc_status[i],
               (i==ucnt-1 && ucnt==MAX_UNITS) ? "" : "  ");
    }
    for (int i = ucnt; i < MAX_UNITS; i++) {
        printf("UNUSED_%d=%u%s", i+1, pkt->svc_status[i],
               (i==MAX_UNITS-1) ? "" : "  ");
    }
    printf("\n");
}

static void print_status_hex(const status_pkt_t* pkt) {
    const uint8_t* p = (const uint8_t*)pkt;
    printf("[statusd] packet(%zuB):", sizeof(*pkt));
    for (size_t i=0; i<sizeof(*pkt); ++i) printf(" %02X", p[i]);
    printf("\n");
}

static void print_status_json(const status_pkt_t* pkt,
                              const char* units[], int ucnt) {
    printf("{\"app\":\"%u.%u.%u.%u\",\"fpga\":\"%u.%u.%u.%u\",\"cpu\":%u,"
           "\"mem\":%u,\"uptime\":%u,\"temp_dC\":%d,\"svc\":[",
           pkt->app_ver[0],  pkt->app_ver[1],  pkt->app_ver[2],  pkt->app_ver[3],
           pkt->fpga_ver[0], pkt->fpga_ver[1], pkt->fpga_ver[2], pkt->fpga_ver[3],
           pkt->cpu_usage, pkt->mem_usage, pkt->uptime_sec, pkt->board_temp_dC);
    for (int i=0;i<ucnt;i++){
        printf("{\"name\":\"%s\",\"err\":%u}%s",
               units[i], pkt->svc_status[i], (i==ucnt-1)?"":",");
    }
    printf("]}\n");
}

/* ===== 보드별 유닛 이름 정의 ===== */
/* 여기 배열 안에 들어가는 문자열을 네가 원하는 유닛 이름으로 바꿔 쓰면 됨 */

static const char* BOARD_A_UNITS[] = {
    "errorcheck.service",
    "tempsvc.service",
    "tcp.service",
    "rs422.service",
    "can.service",
    "spk.service"
};

static const int BOARD_A_UNITS_CNT =
    sizeof(BOARD_A_UNITS)/sizeof(BOARD_A_UNITS[0]);

static const char* BOARD_B_UNITS[] = {
    "errorcheck.service",
    "tempsvc.service",
    "tcp.service",
    "rs422.service",
    "can.service",
    "mic.service"
};

static const int BOARD_B_UNITS_CNT =
    sizeof(BOARD_B_UNITS)/sizeof(BOARD_B_UNITS[0]);

static const char* BOARD_C_UNITS[] = {
    "errorcheck.service",
    "tempsvc.service",
    "tcp.service",
    "rs422.service",
    "can.service",
    "led.service"
};

static const int BOARD_C_UNITS_CNT =
    sizeof(BOARD_C_UNITS)/sizeof(BOARD_C_UNITS[0]);

/* 보드에 맞는 유닛 목록 채우기 */
static void get_units_for_board(board_t b,
                                const char* units_out[MAX_UNITS],
                                int* ucnt_out)
{
    *ucnt_out = 0;

    const char** src = NULL;
    int src_cnt = 0;

    switch (b) {
    case BOARD_A:
        src = BOARD_A_UNITS;
        src_cnt = BOARD_A_UNITS_CNT;
        break;
    case BOARD_B:
        src = BOARD_B_UNITS;
        src_cnt = BOARD_B_UNITS_CNT;
        break;
    case BOARD_C:
        src = BOARD_C_UNITS;
        src_cnt = BOARD_C_UNITS_CNT;
        break;
    default:
        /* UNKNOWN이면 일단 A와 같은 구성을 쓰거나, 전혀 안 쓸 수도 있음 */
        src = BOARD_A_UNITS;
        src_cnt = BOARD_A_UNITS_CNT;
        break;
    }

    if (src_cnt > MAX_UNITS) src_cnt = MAX_UNITS;
    for (int i = 0; i < src_cnt; i++) {
        units_out[i] = src[i];
    }
    *ucnt_out = src_cnt;

    /* 남는 슬롯용 이름은 따로 안 채움 (출력 시 UNUSED_x 로 표시) */
    for (int i = src_cnt; i < MAX_UNITS; i++) {
        units_out[i] = NULL;
    }
}

/* ===== 메인 ===== */
int main(void) {
    const char* units[MAX_UNITS];
    int ucnt = 0;

    board_t bd = detect_board_by_mac();
    printf("[statusd] detected board: %d\n", bd);

    get_units_for_board(bd, units, &ucnt);
    if (ucnt == 0) {
        fprintf(stderr, "[statusd] no units defined for this board\n");
        return 2;
    }

    printf("[statusd] units(%d): ", ucnt);
    for (int i = 0; i < ucnt; i++) {
        printf("%s%s", units[i], (i==ucnt-1)?"":"  ");
    }
    printf("\n");

    setvbuf(stdout, NULL, _IONBF, 0);

    /* 송신 소켓 */
    int s = socket(AF_UNIX, SOCK_DGRAM, 0);
    if (s < 0) {
        perror("socket");
        return 3;
    }
    struct sockaddr_un addr; memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, STATUS_SOCK_PATH, sizeof(addr.sun_path) - 1);

    printf("[statusd] send target: %s, interval=%ds\n",
           STATUS_SOCK_PATH, STATUS_INTERVAL_SEC);

    for (;;) {
        status_pkt_t pkt;
        memset(&pkt, 0, sizeof(pkt));

        /* 값 수집 */
        get_app_ver(pkt.app_ver);
        get_fpga_ver(pkt.fpga_ver);
        pkt.cpu_usage     = get_cpu_usage_percent();
        pkt.mem_usage     = get_mem_usage_percent();
        pkt.board_temp_dC = get_board_temp_dC();
        pkt.uptime_sec    = get_uptime_sec();

        /* 정상(active)=0, 에러(비활성/실패)=1 */
        for (int i = 0; i < ucnt && i < MAX_UNITS; i++) {
            pkt.svc_status[i] = is_active(units[i]) ? 0 : 1;
        }
        /* 남는 칸은 감시대상 아님 → 0(정상)으로 채움 */
        for (int i = ucnt; i < MAX_UNITS; i++) {
            pkt.svc_status[i] = 0;
        }

        /* 출력 */
        print_status_plain(&pkt, units, ucnt);
        if (getenv("STATUS_HEX"))  print_status_hex(&pkt);
        if (getenv("STATUS_JSON")) print_status_json(&pkt, units, ucnt);

        /* 전송 */
        ssize_t n = sendto(s, &pkt, sizeof(pkt), 0,
                           (struct sockaddr*)&addr, sizeof(addr));
        if (n < 0) {
            fprintf(stderr, "[statusd] sendto error: %s\n", strerror(errno));
        }

        sleep(STATUS_INTERVAL_SEC);
    }

    close(s);
    return 0;
}
