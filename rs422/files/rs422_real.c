// project-spec/meta-user/recipes-apps/rs422/files/rs422.c
#include <stdint.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <termios.h>
#include <pthread.h>
#include <errno.h>
#include <poll.h>

volatile int g_running = 1;

#define RS422_SOCKET_PATH "/run/rs422.sock"
#define DEV_PATH "/dev/ttyUL1"
#define FRAME_SIZE 8

// ---------------- GPIO ----------------
void gpio_export(const char* num) {
    int fd = open("/sys/class/gpio/export", O_WRONLY);
    if (fd >= 0) { write(fd, num, strlen(num)); close(fd); }
}

void gpio_direction(const char* num, const char* dir) {
    char path[64];
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%s/direction", num);
    int fd = open(path, O_WRONLY);
    if (fd >= 0) { write(fd, dir, strlen(dir)); close(fd); }
}

void gpio_value(const char* num, int val) {
    char path[64];
    char v = val ? '1' : '0';
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%s/value", num);
    int fd = open(path, O_WRONLY);
    if (fd >= 0) { write(fd, &v, 1); close(fd); }
}

void init_gpio(const char* gpio_de, const char* gpio_re) {
    gpio_export(gpio_de);
    gpio_export(gpio_re);
    usleep(100000);
    gpio_direction(gpio_de, "out");
    gpio_direction(gpio_re, "out");
    gpio_value(gpio_de, 1);
    gpio_value(gpio_re, 0);
}

void sigint_handler(int signo) { (void)signo; g_running = 0; }

void* sender_thread(void* arg) {
    int fd = *(int*)arg;
    uint8_t frame[FRAME_SIZE];
    uint8_t counter = 0;

    while (g_running) {
        for (int i = 0; i < FRAME_SIZE; i++)
            frame[i] = counter + i;

        int n = write(fd, frame, FRAME_SIZE);
        if (n == FRAME_SIZE) {
            printf("[RS422 TX] ");
            for (int i = 0; i < FRAME_SIZE; i++)
                printf("%02X ", frame[i]);
            printf("\n");
            fflush(stdout);
        } else {
            perror("[RS422 TX] write");
        }

        counter++;
        sleep(1);  // 1초 주기 송신
    }
    return NULL;
}

void* receiver_thread(void* arg) {
    int fd = *(int*)arg;
    struct pollfd pfd = { .fd = fd, .events = POLLIN };
    uint8_t buf[FRAME_SIZE];
    int received = 0;

    while (g_running) {
        int ret = poll(&pfd, 1, 1000);
        if (ret < 0) {
            if (errno == EINTR) continue;
            perror("poll");
            break;
        } else if (ret == 0) continue;

        if (pfd.revents & POLLIN) {
            int n = read(fd, buf + received, FRAME_SIZE - received);
            if (n > 0) {
                received += n;

                if (received == FRAME_SIZE) {
                    printf("[RS422 RX] ");
                    for (int i = 0; i < FRAME_SIZE; i++)
                        printf("%02X ", buf[i]);
                    printf("\n");
                    fflush(stdout);
                    received = 0; // 다음 프레임 준비
                }
            }
        }
    }
    return NULL;
}

// ---------------- CAN IPC 수신 스레드 ----------------
void* can_ipc_thread(void* arg) {
    int serial_fd = *(int*)arg;
    int sock_fd;
    struct sockaddr_un addr;

    struct __attribute__((packed)) {
        uint16_t id;
        uint8_t dlc;
        uint8_t data[8];
    } payload;

    unlink(RS422_SOCKET_PATH);
    sock_fd = socket(AF_UNIX, SOCK_DGRAM, 0);
    if (sock_fd < 0) {
        perror("socket");
        return NULL;
    }

    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, RS422_SOCKET_PATH, sizeof(addr.sun_path) - 1);

    if (bind(sock_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("bind");
        close(sock_fd);
        return NULL;
    }

    printf("[IPC] Listening on %s ...\n", RS422_SOCKET_PATH);

    while (g_running) {
        ssize_t n = recv(sock_fd, &payload, sizeof(payload), 0);
        if (n == sizeof(payload)) {
            printf("[IPC] Received CAN msg ID=0x%X DLC=%d\n", payload.id, payload.dlc);

            if ((payload.id == 0x100 || payload.id == 0x101) &&
                payload.dlc > 0 && payload.dlc <= 8) {

                // 8바이트 맞춰서 보냄 (나머지는 0 채움)
                uint8_t frame[FRAME_SIZE] = {0};
                memcpy(frame, payload.data, payload.dlc);

                int sent = write(serial_fd, frame, FRAME_SIZE);
                if (sent == FRAME_SIZE) {
                    printf("[RS422] Sent 8 bytes from CAN ID=0x%X\n", payload.id);
                    fflush(stdout);
                }
            }
        }
    }

    close(sock_fd);
    unlink(RS422_SOCKET_PATH);
    return NULL;
}

// ---------------- 메인 ----------------
int main() {
    const char* gpio_de = "1022"; // DE GPIO
    const char* gpio_re = "1023"; // RE GPIO

    signal(SIGINT, sigint_handler);
    init_gpio(gpio_de, gpio_re);

    int fd = open(DEV_PATH, O_RDWR | O_NOCTTY);
    if (fd < 0) { perror("open"); return -1; }

    // 시리얼 설정
    struct termios tty;
    tcgetattr(fd, &tty);
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_oflag &= ~OPOST;
    tty.c_cc[VMIN]  = 1;
    tty.c_cc[VTIME] = 0;
    tcsetattr(fd, TCSANOW, &tty);

    // 스레드 시작
    pthread_t tx_thread, rx_thread, ipc_thread;
    pthread_create(&tx_thread, NULL, sender_thread, &fd);
    pthread_create(&rx_thread, NULL, receiver_thread, &fd);
    pthread_create(&ipc_thread, NULL, can_ipc_thread, &fd);

    pthread_join(tx_thread, NULL);
    pthread_join(rx_thread, NULL);
    pthread_join(ipc_thread, NULL);

    close(fd);
    return 0;
}

