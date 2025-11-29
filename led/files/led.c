// led.c — WS2812 커스텀 IP(AXI-GPIO) 제어 + /run/led.rs422.sock 수신 + 상태 송신
// build: gcc -O2 -Wall -Wextra -o led led.c
#define _GNU_SOURCE
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <sys/mman.h>
#include <errno.h>
#include <signal.h>
#include <stddef.h>
#include <time.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/stat.h>
#include <poll.h>
#include <pthread.h>

/* ===== AXI-GPIO 레지스터 ===== */
#define GPIO_CH1_DATA  0x00
#define GPIO_CH1_TRI   0x04
#define GPIO_CH2_DATA  0x08
#define GPIO_CH2_TRI   0x0C

static int g_led_can_power = 0;      // CAN에서 온 전원상태
static int g_led_rs422_power = 0;    // RS422에서 온 전원상태
static volatile int g_led_power_enable = 1;   // 1=LED 동작, 0=강제 OFF

/* ===== rs422 — 수신 포맷(3바이트 0/1) ===== */
typedef struct __attribute__((packed)) {
    uint8_t zybo1;
    uint8_t zybo2;
    uint8_t zybo3;
} err_wire_t;

/* 내부 표현 */
typedef struct {
    bool zybo1;
    bool zybo2;
    bool zybo3;
} ERR;

/* ===== led_err 상태 전송 포맷 ===== */
typedef struct __attribute__((packed)) {
    uint8_t led_link_ok;   // 1=OK, 0=LINK ERROR (UIO/AXI 경로 문제)
    uint8_t led_comm_err;  // 1=COMM ERROR (/run/led.rs422.sock 수신 경로 문제)
} led_err_packet;

//========Socket power Rx Addr==========
#define LED_CAN_SOCK           "/run/led.sock"              // CAN에서 받는 전원제어
#define CAN_LED_SOCK           "/run/can.led.sock"          // CAN으로 보내는 소켓주소 (ZYBO > UI)
#define LED_RS422_POWER_SOCK   "/run/led.rs422.power.sock"  // RS422에서 받는 전원제어

/* ===== 전역 ===== */
static volatile uint32_t* g = NULL;
static int g_run = 1;

/* ===== 유틸 ===== */
static inline void mmio_write(uint32_t off, uint32_t v) { g[off / 4] = v; }
static inline uint32_t mmio_read(uint32_t off) { return g[off / 4]; }

static void on_sig(int s) { (void)s; g_run = 0; }

static void msleep(unsigned ms) {
    struct timespec ts = { .tv_sec = ms / 1000, .tv_nsec = (ms % 1000) * 1000000L };
    nanosleep(&ts, NULL);
}
static uint64_t now_ms(void) {
    struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ull + ts.tv_nsec / 1000000ull;
}

/* ===== AXI-GPIO 기본 ===== */
static inline void gpio_set_dir_output(void) {
    mmio_write(GPIO_CH1_TRI, 0x00000000);
    mmio_write(GPIO_CH2_TRI, 0x00000000);
}

static inline void set_strip_color(uint8_t strip_id, uint8_t r, uint8_t g8, uint8_t b) {
    // GRB 24b
    uint32_t grb = ((uint32_t)g8 << 16) | ((uint32_t)r << 8) | b;
    mmio_write(GPIO_CH1_DATA, grb);
    // stb=1, strip_id=[10:8]
    uint32_t cmd = (1u << 31) | (((uint32_t)strip_id & 0x7) << 8);
    mmio_write(GPIO_CH2_DATA, cmd);
    // stb=0
    mmio_write(GPIO_CH2_DATA, 0u);
}

static inline void start_all(void) {
    mmio_write(GPIO_CH2_DATA, 1u << 0);
    mmio_write(GPIO_CH2_DATA, 0u);
}

/* 간단 루프백 self-test(원치 않으면 true만 반환하도록 바꿔도 됨) */
static bool gpio_selftest(void) {
    uint32_t s1 = mmio_read(GPIO_CH1_DATA);
    uint32_t s2 = mmio_read(GPIO_CH2_DATA);
    uint32_t p1 = 0x00AA55u, p2 = 0x5500AAu;

    mmio_write(GPIO_CH1_DATA, p1);
    mmio_write(GPIO_CH2_DATA, p2);
    uint32_t r1 = mmio_read(GPIO_CH1_DATA);
    uint32_t r2 = mmio_read(GPIO_CH2_DATA);

    mmio_write(GPIO_CH1_DATA, s1);
    mmio_write(GPIO_CH2_DATA, s2);

    return (r1 == p1 && r2 == p2);
}

/* ===== CAN 송신 (LED 상태) ===== */
static int tx_led_and_power_to_can(uint8_t state_bits) { 
    int fd = socket(AF_UNIX, SOCK_DGRAM, 0);
    if (fd < 0) { perror("socket"); return -1; }

    struct sockaddr_un to;
    memset(&to, 0, sizeof(to));
    to.sun_family = AF_UNIX;
    strncpy(to.sun_path, CAN_LED_SOCK, sizeof(to.sun_path) - 1);

    ssize_t n = sendto(fd, &state_bits, sizeof(state_bits), 0,
        (struct sockaddr*)&to, sizeof(to));
    if (n < 0) {
        fprintf(stderr, "[IPC] sendto(%s) failed: %s\n", CAN_LED_SOCK, strerror(errno));
        close(fd);
        return -1;
    }
    close(fd);
    return 0;
}

// ================= led 전원제어 (from CAN, RS422) ============
//  - /run/led.sock              (CAN)
//  - /run/led.rs422.power.sock  (RS422)
static void* led_power_rx_thread(void* arg)
{
    (void)arg;

    const char* sock_paths[2] = {
        LED_CAN_SOCK,           // "/run/led.sock"             — CAN 제어
        LED_RS422_POWER_SOCK    // "/run/led.rs422.power.sock" — RS422 제어
    };
    int fds[2] = { -1, -1 };

    // 소켓 2개 생성 + bind
    for (int i = 0; i < 2; i++) {
        unlink(sock_paths[i]);

        int fd = socket(AF_UNIX, SOCK_DGRAM, 0);
        if (fd < 0) {
            perror("[led] socket");
            continue;
        }

        struct sockaddr_un me;
        memset(&me, 0, sizeof(me));
        me.sun_family = AF_UNIX;
        strncpy(me.sun_path, sock_paths[i], sizeof(me.sun_path) - 1);

        if (bind(fd, (struct sockaddr*)&me, sizeof(me)) < 0) {
            fprintf(stderr, "[led] bind(%s) failed: %s\n",
                sock_paths[i], strerror(errno));
            close(fd);
            continue;
        }

        chmod(sock_paths[i], 0660);
        fds[i] = fd;
        printf("[led] power rx listening on %s\n", sock_paths[i]);
        fflush(stdout);
    }

    struct pollfd pfds[2];
    for (int i = 0; i < 2; i++) {
        pfds[i].fd = fds[i];
        pfds[i].events = POLLIN;
        pfds[i].revents = 0;
    }

    // 그냥 전원 상태 로컬로 유지
    int can_power_on = 1;
    int rs422_power_on = 1;
    int final_on = 1;
    uint8_t prev_state = 0x04;   // 직전 CAN 전송 값 (초기값: 1)

    while (g_run) {
        int ret = poll(pfds, 2, 1000); // 1초 타임아웃
        if (ret < 0) {
            if (errno == EINTR)
                continue;
            perror("[led] poll");
            break;
        }
        if (ret == 0)
            continue; // timeout

        for (int i = 0; i < 2; i++) {
            if (pfds[i].fd < 0)
                continue;
            if (!(pfds[i].revents & POLLIN))
                continue;

            uint8_t cmd[8] = { 0 }; // 최대 8바이트까지 받되, 실제로는 앞 2바이트만 사용
            ssize_t n = recv(pfds[i].fd, cmd, sizeof(cmd), 0);
            if (n < 0) {
                if (errno == EINTR)
                    continue;
                perror("[led] recv");
                continue;
            }

            if (n >= 2) {
                uint8_t hi = cmd[0];
                uint8_t lo = cmd[1];

                // 여기서 hi/lo 비트로 의미 해석
                // 예) lo의 bit0 = 외부 LED, bit1 = 내부 LED, bit2(0x04) = 전원 비트
                int new_on = (lo & 0x04) ? 1 : 0;

                if (i == 0) {
                    // CAN 소스
                    can_power_on = new_on;
                    g_led_can_power = new_on;
                }
                else {
                    // RS422 소스
                    rs422_power_on = new_on;
                    g_led_rs422_power = new_on;
                }

                // LED 전원 제어
                final_on = new_on;

                // 전역 플래그에 반영
                g_led_power_enable = final_on ? 1 : 0;

                uint8_t state = (uint8_t)((g_led_power_enable ? 1 : 0) << 2);
                if (state != prev_state) {
                    tx_led_and_power_to_can(state); // CAN으로 상태 보고
                    prev_state = state;

                    printf("[led] TX CAN_LED_STATE=0x%02X (FINAL=%d)\n", state, final_on);
                    fflush(stdout);
                }

                // LED on/off 실제 적용
                if (final_on) {
                    printf("[led] ON!!\n");
                    // 여기서는 색상은 err 상태에 따라 메인 루프에서 갱신
                }
                else { // OFF: 모두 끄기
                    set_strip_color(0, 0, 0, 0);
                    set_strip_color(1, 0, 0, 0);
                    set_strip_color(2, 0, 0, 0);
                    start_all();
                }

                printf("[led] SRC=%s hi=0x%02X lo=0x%02X → CAN=%d RS422=%d FINAL=%d\n G_LED=%d\n",
                    sock_paths[i], hi, lo,
                    can_power_on, rs422_power_on, final_on, g_led_power_enable);
                fflush(stdout);
            }
            else {
                fprintf(stderr, "[led] power cmd too short from %s (n=%zd)\n",
                    sock_paths[i], n);
            }
        }
    }

    for (int i = 0; i < 2; i++) {
        if (fds[i] >= 0) {
            close(fds[i]);
            unlink(sock_paths[i]);
        }
    }

    return NULL;
}


/* ===== rs422한테 받는 error 데이터 수신 소켓 생성(bind, nonblock) ===== */
static int create_error_sock(const char* path) {
    int fd = socket(AF_UNIX, SOCK_DGRAM, 0);
    if (fd < 0) { perror("socket"); return -1; }

    struct sockaddr_un su; memset(&su, 0, sizeof(su));
    su.sun_family = AF_UNIX;
    strncpy(su.sun_path, path, sizeof(su.sun_path) - 1);

    unlink(path); // 우리가 수신자이므로 기존 파일 제거
    if (bind(fd, (struct sockaddr*)&su, sizeof(su)) < 0) {
        perror("bind");
        fprintf(stderr, "[led] another process may already bind %s\n", path);
        close(fd);
        return -1;
    }
    chmod(path, 0666);

    int fl = fcntl(fd, F_GETFL, 0);
    if (fl >= 0) fcntl(fd, F_SETFL, fl | O_NONBLOCK);

    printf("[led] listening on %s (UNIX DGRAM, 3-byte ERR)\n", path);
    fflush(stdout);
    return fd;
}

/* ===== /run/led_err.sock 송신 소켓(바인드 없음, sendto 전용) ===== */
static int open_sender_sock(void) {
    int s = socket(AF_UNIX, SOCK_DGRAM, 0);
    if (s < 0) perror("sender socket");
    return s;
}
static int send_led_err(int s, const char* dst, const led_err_packet* p) {
    struct sockaddr_un a; memset(&a, 0, sizeof(a));
    a.sun_family = AF_UNIX;
    strncpy(a.sun_path, dst, sizeof(a.sun_path) - 1);
    return sendto(s, p, sizeof(*p), 0, (struct sockaddr*)&a, sizeof(a));
}

/* ===== main ===== */
int main(void)
{
    const char* uio = "/dev/uio2";              // AXI-GPIO
    const char* err_path = "/run/led.rs422.sock";   // 수신(우리 bind) — C보드 RS422에서 err_wire_t 받는 소켓
    const char* lederr_path = "/run/led_err.sock";  // 송신(상대 bind) — boardd가 bind 하는 LED 상태 소켓

    /* UIO mmap (링크 판단의 1차 기준) */
    int ufd = open(uio, O_RDWR | O_SYNC);
    if (ufd < 0) { fprintf(stderr, "open %s: %s\n", uio, strerror(errno)); return 1; }

    size_t mapsz = 0x10000;
    void* base = mmap(NULL, mapsz, PROT_READ | PROT_WRITE, MAP_SHARED, ufd, 0);
    if (base == MAP_FAILED) {
        fprintf(stderr, "mmap: %s\n", strerror(errno));
        close(ufd);
        return 1;
    }
    g = (volatile uint32_t*)base;

    signal(SIGINT, on_sig);
    signal(SIGTERM, on_sig);

    gpio_set_dir_output();

    /* 초기 색 (보여주기용) */
    set_strip_color(0, 255, 0, 0);
    set_strip_color(1, 0, 255, 0);
    set_strip_color(2, 0, 0, 255);
    start_all();

    /* /run/led.rs422.sock 수신 */
    int efd = create_error_sock(err_path);   // 실패하면 통신 에러로 처리

    /* led_err 송신 */
    int sd = open_sender_sock();

    /* 링크 self-test 1회 (원하면 주기적으로 다시 호출) */
    bool uiomap_ok = true;          // mmap 성공 시 true
    bool self_ok = gpio_selftest();

    /* "/run/led.rs422.sock" 최근 수신 시각 (지금은 comm_err 판단에는 안 씀, 필요하면 유지) */
    uint64_t g_last_recv_ms = 0;

    /* 상태 보고 주기 */
    uint64_t next_report_ms = 0;

    /* 표시용 내부 상태 */
    ERR err = { false,false,false };

    pthread_t th_power;
    pthread_create(&th_power, NULL, led_power_rx_thread, NULL);

    while (g_run) {
        /* ---- "/run/led.rs422.sock" 수신(논블로킹) ---- */
        if (efd >= 0) {
            for (;;) {
                uint8_t buf[32];
                ssize_t n = recv(efd, buf, sizeof(buf), 0);
                if (n < 0) {
                    if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) break;
                    perror("recv"); break;
                }
                if (n >= (ssize_t)sizeof(err_wire_t)) {
                    const err_wire_t* w = (const err_wire_t*)buf;
                    err.zybo1 = (w->zybo1 != 0);
                    err.zybo2 = (w->zybo2 != 0);
                    err.zybo3 = (w->zybo3 != 0);
                    g_last_recv_ms = now_ms();

                    printf("[led] recv ERR: z1=%u z2=%u z3=%u\n",
                        w->zybo1, w->zybo2, w->zybo3);
                    fflush(stdout);
                }
            }
        }

        /* ---- LED 갱신(에러 true=빨강, false=초록) ---- */
        if (g_led_power_enable) {
            set_strip_color(0, err.zybo1 ? 255 : 0, err.zybo1 ? 0 : 255, 0);
            set_strip_color(1, err.zybo2 ? 255 : 0, err.zybo2 ? 0 : 255, 0);
            set_strip_color(2, err.zybo3 ? 255 : 0, err.zybo3 ? 0 : 255, 0);
            start_all();
        }

        /* ---- 링크/통신 상태 산출 & 전송(1s) ---- */
        uint64_t now = now_ms();

        // === 방법 1 적용 ===
        // "패킷이 10초 안 왔다" 같은 조건은 제거하고,
        // 소켓이 깨졌을 때(efd < 0)만 통신 에러로 본다.
        bool comm_err = (efd < 0);

        bool link_ok = uiomap_ok && self_ok; // 최소 기준

        if (now >= next_report_ms && sd >= 0) {
            led_err_packet lep = {
                .led_link_ok  = link_ok ? 1 : 0,
                .led_comm_err = comm_err ? 1 : 0,
            };
            int r = send_led_err(sd, lederr_path, &lep);
            if (r < 0) {
                // 수신자 미가동/경로 없음 등은 여기서만 경고
                fprintf(stderr, "[led] led_err send fail: %s\n", strerror(errno));
            }
            next_report_ms = now + 1000; // 1초 주기
        }

        msleep(100);
    }

    pthread_join(th_power, NULL);
    if (sd >= 0) close(sd);
    if (efd >= 0) { close(efd); unlink(err_path); }
    munmap((void*)base, mapsz);
    close(ufd);
    return 0;
}
