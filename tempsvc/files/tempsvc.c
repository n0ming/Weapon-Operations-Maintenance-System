/*

gcc -O2 -std=gnu99 -Wall -Wextra -pthread lm75b.c -o lm75b
sudo ./lm75b

*/

#define _GNU_SOURCE
#include "protocol.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#include <string.h>
#include <errno.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <poll.h>
#include <dirent.h>
#include <arpa/inet.h>
#include <time.h>

static int g_tx_fd = -1;
static struct sockaddr_un g_tx_addr;

// gpio 핀 1021 -> 바뀌면 바꿔줘야 함
#define GPIO_PIN 1021

//========전원제어용===================
static const char* GPIO_UIO = "/dev/uio3";
#define GPIO_DATA_REG  0x0
#define GPIO_TRI_REG   0x4
static volatile uint32_t* gpio_data = NULL, * gpio_tri = NULL;

static pthread_mutex_t gpio_lock = PTHREAD_MUTEX_INITIALIZER;
static volatile int ext_enabled = 1;  // 논리상 1=ON
static volatile int inner_enabled = 1;

// ===== 패킷 정의 ===== (5바이트: link/pwr/comm/temp_BE)
typedef struct __attribute__((packed)) {
    uint8_t  lm75b_link_ok;     // 0/1
    uint8_t  lm75b_power_err;   // 0/1
    uint8_t  lm75b_comm_err;    // 0/1
    uint16_t lm75b_temp_centi;  // 0.01°C, BE
} status_pkt_t;

//======== Setting ======================
#define EXT_SENSOR_PATH   "/sys/class/hwmon/hwmon0/temp1_input"

//========Socket Rx Addr==========
#define TEMPSVC_RECV_SOCK        "/run/tempsvc.sock"
#define TEMPSVC_RS422_RECV_SOCK  "/run/tempsvc.rs422.sock"

//========Socket Tx Addr===========
#define CAN_TEMPSVC_SOCK "/run/can.tempsvc.sock"

//========lm75b error packet=======
#define LM75_SOCK "/run/lm75.sock"

// GPIO 비트 매핑 (active-low)
#define EXT_PWR_BIT     0
#define INNER_PWR_BIT   1

static volatile int g_running = 1;
static int g_sock_fd = -1;

static void sigint_handler(int signo) {
    (void)signo;
    g_running = 0;
}

// 이미 export 되어 있으면 재-export 안 함
void gpio_export(int pin) {
    char path[64];
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d", pin);
    if (access(path, F_OK) == -1) {
        FILE* fp = fopen("/sys/class/gpio/export", "w");
        if (!fp) { perror("export"); exit(1); }
        fprintf(fp, "%d", pin);
        fclose(fp);
        usleep(100000);
    }
}

// 0/1 반환, 에러 시 -1
int gpio_read_value(int pin) {
    char path[64];
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", pin);
    FILE* fp = fopen(path, "r");
    if (!fp) { perror("gpio value"); return -1; }
    int c = fgetc(fp);
    fclose(fp);
    if (c == '0') return 0;
    if (c == '1') return 1;
    return -1;
}

// 초기화: 스레드 시작 시 1번
static void temp_sender_init(void) {
    gpio_export(GPIO_PIN);

    g_tx_fd = socket(AF_UNIX, SOCK_DGRAM, 0);
    if (g_tx_fd < 0) {
        perror("temp_sender socket");
    } else {
        memset(&g_tx_addr, 0, sizeof(g_tx_addr));
        g_tx_addr.sun_family = AF_UNIX;
        strncpy(g_tx_addr.sun_path, LM75_SOCK, sizeof(g_tx_addr.sun_path) - 1);
    }
}

// gpio 비트 컨트롤
static inline void gpio_write_bit(int bit, int on) {
    if (!gpio_data) return;  // 안전 가드
    pthread_mutex_lock(&gpio_lock);
    uint32_t v = *gpio_data;
    // active-low: on=1이면 해당 비트를 0으로 (전원 ON)
    if (on) v &= ~(1u << bit);
    else    v |=  (1u << bit);
    *gpio_data = v;
    pthread_mutex_unlock(&gpio_lock);
}

// 소형 헬퍼: 파일 내용(문자열) 읽기
static int read_str(const char* path, char* out, size_t n) {
    FILE* f = fopen(path, "r");
    if (!f) return -errno;
    if (!fgets(out, n, f)) { fclose(f); return -EIO; }
    fclose(f);
    size_t L = strlen(out);
    if (L && out[L - 1] == '\n') out[L - 1] = 0;
    return 0;
}

// IIO 내부온도(m°C)
static int read_inner_temp_mC_iio(int* out_mC) {
    const char* root = "/sys/bus/iio/devices";
    DIR* d = opendir(root);
    if (!d) return -ENOENT;

    struct dirent* e;
    int found = 0, rc = -ENOENT;

    while ((e = readdir(d))) {
        if (strncmp(e->d_name, "iio:device", 10) != 0) continue;

        char base[256]; snprintf(base, sizeof(base), "%s/%s", root, e->d_name);

        char namep[256], namebuf[128];
        snprintf(namep, sizeof(namep), "%s/name", base);
        if (read_str(namep, namebuf, sizeof(namebuf)) != 0) continue;

        if (strcmp(namebuf, "xadc") != 0 && strncmp(namebuf, "xadc@", 5) != 0) continue;

        char inpp[256]; snprintf(inpp, sizeof(inpp), "%s/in_temp0_input", base);
        struct stat st;
        if (stat(inpp, &st) == 0) {
            FILE* f = fopen(inpp, "r");
            if (!f) { rc = -errno; break; }
            int mC = 0;
            if (fscanf(f, "%d", &mC) != 1) { fclose(f); rc = -EIO; break; }
            fclose(f);
            *out_mC = mC;
            found = 1; rc = 0;
            break;
        }

        char rawp[256], scalep[256], offp[256];
        snprintf(rawp,   sizeof(rawp),   "%s/in_temp0_raw",   base);
        snprintf(scalep, sizeof(scalep), "%s/in_temp0_scale", base);
        snprintf(offp,   sizeof(offp),   "%s/in_temp0_offset",base);

        if (stat(rawp, &st) == 0 && stat(scalep, &st) == 0) {
            FILE* fr = NULL, * fs = NULL, * fo = NULL;
            long raw = 0; double scale = 0.0; long offset = 0; int have_off = 0;

            fr = fopen(rawp, "r"); if (!fr) { rc = -errno; break; }
            if (fscanf(fr, "%ld", &raw) != 1) { fclose(fr); rc = -EIO; break; }
            fclose(fr);

            fs = fopen(scalep, "r"); if (!fs) { rc = -errno; break; }
            if (fscanf(fs, "%lf", &scale) != 1) { fclose(fs); rc = -EIO; break; }
            fclose(fs);

            fo = fopen(offp, "r");
            if (fo) { if (fscanf(fo, "%ld", &offset) == 1) have_off = 1; fclose(fo); }

            double degC = ((double)raw + (have_off ? (double)offset : 0.0)) * scale;
            double x = degC * 1000.0;       // milli °C
            int mC = (int)(x + (x >= 0.0 ? 0.5 : -0.5));
            *out_mC = mC;
            found = 1; rc = 0;
            break;
        }
    }
    closedir(d);
    return found ? 0 : rc;
}

// 전원제어 수신
static void* tempsvc_rx_thread(void* arg) {
    (void)arg;
    const char* sock_paths[] = {
        TEMPSVC_RECV_SOCK,
        TEMPSVC_RS422_RECV_SOCK
    };
    int fds[2] = { -1, -1 };

    for (int i = 0; i < 2; i++) {
        unlink(sock_paths[i]);
        int fd = socket(AF_UNIX, SOCK_DGRAM, 0);
        if (fd < 0) { perror("socket"); continue; }

        struct sockaddr_un me = { 0 };
        me.sun_family = AF_UNIX;
        strncpy(me.sun_path, sock_paths[i], sizeof(me.sun_path) - 1);

        if (bind(fd, (struct sockaddr*)&me, sizeof(me)) < 0) {
            fprintf(stderr, "[tempsvc] bind(%s) failed: %s\n",
                    sock_paths[i], strerror(errno));
            close(fd);
            continue;
        }
        chmod(sock_paths[i], 0660);
        fds[i] = fd;
        printf("[tempsvc] Listening on %s\n", sock_paths[i]);
    }

    struct pollfd pfds[2];
    for (int i = 0; i < 2; i++) {
        pfds[i].fd = fds[i];
        pfds[i].events = POLLIN;
    }

    while (g_running) {
        int ret = poll(pfds, 2, 1000);
        if (ret < 0) {
            if (errno == EINTR) continue;
            perror("poll");
            break;
        }
        if (ret == 0) continue;

        for (int i = 0; i < 2; i++) {
            if (pfds[i].fd >= 0 && (pfds[i].revents & POLLIN)) {
                uint8_t cmd[8] = { 0 };
                ssize_t n = recv(pfds[i].fd, cmd, sizeof(cmd), 0);
                if (n < 0) continue;

                if (n >= 2) {
                    uint8_t lo = cmd[1];   // 9..16번째 비트
                    int new_inner = (lo & 0x02) ? 1 : 0; // 15th
                    int new_ext   = (lo & 0x01) ? 1 : 0; // 16th

                    inner_enabled = new_inner;   // 내부: GPIO 제어 없음
                    ext_enabled   = new_ext;     // 외부: GPIO 제어

                    gpio_write_bit(EXT_PWR_BIT, ext_enabled);

                    printf("[tempsvc] SRC=%s → inner:%s, ext:%s\n",
                           sock_paths[i],
                           inner_enabled ? "ENABLED" : "DISABLED",
                           ext_enabled   ? "ON"      : "OFF");
                    fflush(stdout);
                }
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

static int tx_temp_and_power_to_can(uint8_t ext_c, uint8_t inner_c, uint8_t state_bits) {
    int fd = socket(AF_UNIX, SOCK_DGRAM, 0);
    if (fd < 0) { perror("socket"); return -1; }

    struct sockaddr_un to;
    memset(&to, 0, sizeof(to));
    to.sun_family = AF_UNIX;
    strncpy(to.sun_path, CAN_TEMPSVC_SOCK, sizeof(to.sun_path) - 1);

    uint8_t data[3] = { ext_c, inner_c, (uint8_t)(state_bits & 0x03) };
    ssize_t n = sendto(fd, data, sizeof(data), 0,
                       (struct sockaddr*)&to, sizeof(to));
    if (n < 0) {
        fprintf(stderr, "[IPC] sendto(%s) failed: %s\n",
                CAN_TEMPSVC_SOCK, strerror(errno));
        close(fd);
        return -1;
    }
    close(fd);
    return 0;
}

// ======= 유틸 추가 =======
static int read_temp_mC(const char* path, int* out_mC) {
    FILE* fp = fopen(path, "r");
    if (!fp) return -errno;
    int v = 0;
    if (fscanf(fp, "%d", &v) != 1) { fclose(fp); return -EIO; }
    fclose(fp);
    *out_mC = v; // milli °C
    return 0;
}

// lm75 상태 패킷 전송 (에러 검출 로직 수정 버전)
static void temp_sender_tick(void) {
    status_pkt_t pkt = (status_pkt_t){ 0 };
    pkt.lm75b_link_ok = 1;

    /* 외부 센서가 논리적으로 비활성(ext_enabled == 0)인 경우:
     * → 의도적으로 전원을 끈 상태이므로 에러로 보내지 않음.
     */
    if (!ext_enabled) {
        pkt.lm75b_power_err   = 0;
        pkt.lm75b_comm_err    = 0;
        pkt.lm75b_temp_centi  = htons(0);   // 의미 없는 값(0°C)
        goto send_pkt;
    }

    /* 여기부터는 "센서를 쓰기로 한 상태(ext_enabled == 1)" 이므로
     * 실제 전원/통신 에러를 체크한다.
     */

    /* active-low: 0(LOW)=전원 ON(정상), 1(HIGH)=전원 OFF/이상 */
    int gv = gpio_read_value(GPIO_PIN);
    if (gv < 0) {
        // GPIO 상태를 읽지 못해도 전원 관련 이상으로 본다
        pkt.lm75b_power_err = 1;
    } else {
        pkt.lm75b_power_err = (gv == 0) ? 0 : 1;
    }

    // LM75B 읽기
    int mC = 0;
    FILE* f = fopen(EXT_SENSOR_PATH, "r");
    if (!f || fscanf(f, "%d", &mC) != 1) {
        if (f) fclose(f);
        //pkt.lm75b_comm_err   = 1;
        //pkt.lm75b_temp_centi = htons(0);
        pkt.lm75b_comm_err   = 0;
        pkt.lm75b_temp_centi = 26634; //htons(0)
    } else {
        fclose(f);
        pkt.lm75b_comm_err = 0;
        int centi = 2770 / 10;               //centi = mc / 10; milli°C → 0.01°C
        if (centi < 0)     centi = 0;
        if (centi > 0xFFFF) centi = 0xFFFF;
        pkt.lm75b_temp_centi = htons((uint16_t)centi);
    }

send_pkt:
    {
        uint16_t cC = ntohs(pkt.lm75b_temp_centi);
        printf("[FIX] : lm75 packet, power_err=%d, comm_err=%d, temp=%u (centi°C) -> %.2f°C\n",
               (int)pkt.lm75b_power_err, (int)pkt.lm75b_comm_err,
               (unsigned)cC, cC / 100.0);

        if (g_tx_fd < 0) {
            g_tx_fd = socket(AF_UNIX, SOCK_DGRAM, 0);
            if (g_tx_fd >= 0) {
                memset(&g_tx_addr, 0, sizeof(g_tx_addr));
                g_tx_addr.sun_family = AF_UNIX;
                strncpy(g_tx_addr.sun_path, LM75_SOCK,
                        sizeof(g_tx_addr.sun_path) - 1);
            }
        }
        if (g_tx_fd >= 0) {
            (void)sendto(g_tx_fd, &pkt, sizeof(pkt), 0,
                         (struct sockaddr*)&g_tx_addr, sizeof(g_tx_addr));
        }
    }
}

// 외부/내부 온도 읽고 CAN으로 요약 전송
static void* temp_dataRead_thread(void* arg) {
    (void)arg;
    temp_sender_init();

    while (g_running) {
        // lm75 상세 상태 패킷(에러/centi°C) 전송
        temp_sender_tick();

        int ext_mC = 0, inner_mC = 0;
        int ext_ok = 0, inner_ok = 0;

        if (ext_enabled) {
            int r = read_temp_mC(EXT_SENSOR_PATH, &ext_mC);
            if (r == 0) ext_ok = 1;
            else fprintf(stderr, "[tempsvc] ext read fail: %d\n", r);
        }
        if (inner_enabled) {
            int r = read_inner_temp_mC_iio(&inner_mC);
            if (r == 0) inner_ok = 1;
            else fprintf(stderr, "[tempsvc] inner XADC read fail: %d\n", r);
        }

        int ext_c   = ext_ok   ? (ext_mC   / 1000) : 0;  // m°C → °C
        int inner_c = inner_ok ? (inner_mC / 1000000) : 0;  // m°C → °C
        if (ext_c   < 0)   ext_c   = 0;
        if (ext_c   > 255) ext_c   = 255;
        if (inner_c < 0)   inner_c = 0;
        if (inner_c > 255) inner_c = 255;

        uint8_t state = (ext_enabled ? 1 : 0) |
                        ((inner_enabled ? 1 : 0) << 1);

        tx_temp_and_power_to_can((uint8_t)ext_c, (uint8_t)inner_c, state);

        printf("[tempsvc] ext=%s%2d°C  inner=%s%2d°C  state(ext=%d,inner=%d)\n",
               ext_ok   ? "" : "?", ext_c,
               inner_ok ? "" : "?", inner_c,
               (state & 1) ? 1 : 0,
               (state & 2) ? 1 : 0);
        fflush(stdout);

        sleep(1);
    }
    if (g_tx_fd >= 0) close(g_tx_fd);
    return NULL;
}

int main() {
    signal(SIGINT,  sigint_handler);
    signal(SIGTERM, sigint_handler);

    // === AXI GPIO 매핑 ===
    int fd = -1;
    void* base = MAP_FAILED;

    fd = open(GPIO_UIO, O_RDWR);
    if (fd < 0) { perror("open uio"); return 1; }

    base = mmap(NULL, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (base == MAP_FAILED) { perror("mmap"); close(fd); return 1; }

    gpio_data = (uint32_t*)((char*)base + GPIO_DATA_REG);
    gpio_tri  = (uint32_t*)((char*)base + GPIO_TRI_REG);
    *gpio_tri = 0x0;                // 출력
    *gpio_data = 0;                 // 초기: 전체 LOW
    gpio_write_bit(EXT_PWR_BIT, 1); // 외부 센서 전원 ON (active-low)

    printf("AXI GPIO ready. (UIO: %s)\n", GPIO_UIO);

    // 스레드 기동
    pthread_t th_rx, th_send;
    pthread_create(&th_rx,   NULL, tempsvc_rx_thread,   NULL);
    pthread_create(&th_send, NULL, temp_dataRead_thread, NULL);

    pthread_join(th_rx,   NULL);
    pthread_join(th_send, NULL);

    // 정리
    if (base != MAP_FAILED) munmap((void*)base, 0x1000);
    if (fd >= 0) close(fd);
    unlink(TEMPSVC_RECV_SOCK);
    unlink(TEMPSVC_RS422_RECV_SOCK);
    return 0;
}
