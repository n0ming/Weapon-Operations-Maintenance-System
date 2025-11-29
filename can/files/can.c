// project-spec/meta-user/recipes-apps/can/files/can.c
// C1!!!!
//
// build : gcc -std=c11 -O2 -Wall -Wextra -pthread can.c -o can
// run   : sudo ./can                         # 실제값
//         CAN_FAKE=1 sudo ./can              # 가짜(고정)
//         CAN_FAKE=1 CAN_RAND=1 sudo ./can   # 가짜(랜덤)
//         sudo ./can --fake-can [--rand]

#define _XOPEN_SOURCE 700   /* usleep, getpagesize */

#include "protocol.h"     // T_BITM_RACK_DETAIL_INFO (boardd와 통신)
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <pthread.h>
#include <signal.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <errno.h>
#include <time.h>
#include <sys/timerfd.h>
#include <poll.h>
#include <sys/stat.h>
#include <ctype.h>

/* ===== 보드 자동 판별 (MAC/환경변수) ===== */
typedef enum { BOARD_A=0, BOARD_B=1, BOARD_C=2 } board_t;
static board_t g_board = BOARD_A;

/* 로그 프리픽스 + 안전한 포맷 매크로 */
static const char* g_log_prefix = "[CAN/?] ";
#define LOGF(fmt, ...)  fprintf(stderr, "%s" fmt, g_log_prefix, ##__VA_ARGS__)

/* ===== 경로/디바이스 ===== */
#define SPI_UIO_PATH      "/dev/uio0"            // MCP2515 SPI (AXI Quad SPI)
#define SPI_MAP_SIZE      0x10000

/*================ Socket Tx Addr ===============*/
#define RS422_SOCKET_PATH "/run/rs422.sock"      // RS422 수신 서비스의 DGRAM 주소
#define TEMP_SOCKET_PATH  "/run/tempsvc.sock"
/* boardd가 CAN DETAIL을 수신하는 소켓 (boardd는 /run/can.sock 바인드) */
#define CAN_DETAIL_SOCK_PATH "/run/can.sock"
#define MIC_SOCKET_PATH "/run/mic.sock"
#define SPEAKER_SOCKET_PATH "/run/speaker.sock"
#define LED_SOCKET_PATH "/run/led.sock" //CAN에서 받는 전원제어

/*================ Socket Rx Addr ===============*/
#define SOCK_RX_TEMP  "/run/can.tempsvc.sock"
#define SOCK_RX_MIC   "/run/can.mic.sock"
#define SOCK_RX_SPEAKER  "/run/can.speaker.sock"
#define SOCK_RX_LED   "/run/can.led.sock"




/* =============== 보드별 CAN ID (런타임 변수) ================= */
static uint32_t POWER_CONTROL = 0;
static uint32_t POWER_REPLY   = 0;
static uint32_t BUS_STATUS    = 0;
static uint32_t TEMP_INFO     = 0;
static uint32_t POWER_STATUS  = 0;

/* ================== MCP2515 Commands / Registers ================== */
#define MCP_CMD_RESET            0xC0
#define MCP_CMD_READ             0x03
#define MCP_CMD_WRITE            0x02
#define MCP_CMD_BIT_MODIFY       0x05
#define MCP_CMD_READ_RXB0SIDH    0x90
#define MCP_CMD_READ_RXB1SIDH    0x92

#define MCP_REG_CANCTRL  0x0F
#define MCP_REG_CANSTAT  0x0E
#define MCP_REG_CNF1     0x2A
#define MCP_REG_CNF2     0x29
#define MCP_REG_CNF3     0x28
#define MCP_REG_TXB0CTRL 0x30
#define MCP_REG_RXB0CTRL 0x60
#define MCP_REG_RXB1CTRL 0x70
#define MCP_REG_CANINTF  0x2C
#define MCP_REG_CANINTE  0x2B
#define MCP_REG_EFLG     0x2D // Error Flag
#define MCP_REG_TEC      0x1C // Tx Error Counter
#define MCP_REG_REC      0x1D // Rx Error Counter

#define RXM_ALL         0x60
#define CANINTF_RX0IF   (1u<<0)
#define CANINTF_RX1IF   (1u<<1)
#define TXREQ           (1u<<3)
#define MODE_NORMAL     0x00
#define MODE_CONFIG     0x80

/* ================== AXI Quad SPI (typical) ================== */
#define REG_SPI_SRR_OFS  0x40
#define REG_SPI_CR_OFS   0x60
#define REG_SPI_SR_OFS   0x64
#define REG_SPI_DTR_OFS  0x68
#define REG_SPI_DRR_OFS  0x6C
#define REG_SPI_SSR_OFS  0x70

#define CR_SPE        (1u<<1)
#define CR_MST        (1u<<2)
#define CR_TXFIFO_RST (1u<<5)
#define CR_RXFIFO_RST (1u<<6)
#define CR_MANUAL_SS  (1u<<7)
#define SR_RX_EMPTY   (1u<<0)
#define SR_TX_EMPTY   (1u<<2)

/* MCP2515 Error Flag bits */
#define EFLG_RX1OVR  0x80
#define EFLG_RX0OVR  0x40
#define EFLG_TXBO    0x20  // Bus-Off
#define EFLG_TXEP    0x10  // TX Error Passive
#define EFLG_RXEP    0x08  // RX Error Passive
#define EFLG_TXWAR   0x04  // TX Warning
#define EFLG_RXWAR   0x02  // RX Warning
#define EFLG_EWARN   0x01  // Error Warning

/* ================== Types / Globals ================== */
typedef struct {
    uint16_t id;   // RS422 payload용; 11bit면 그대로, 29bit면 하위 16bit만 사용
    uint8_t  dlc;
    uint8_t  data[8];
} CAN_Message;

typedef struct {
    uint8_t bus_state;  // 0: Error Active / 1: Passive or Warning / 2: BusOff
    uint8_t TEC;
    uint8_t REC;
} CAN_Error;

static volatile uint8_t* spi_base = NULL;
static volatile int g_running = 1;
static pthread_mutex_t spi_lock = PTHREAD_MUTEX_INITIALIZER;

/* tempsvc → 우리가 받은 온도값 (centi ℃) */
static volatile int     temp_enabled   = 1;
static volatile int     inner_enabled  = 1;
static volatile int16_t g_temp_centi   = 0;
static volatile uint8_t g_ext_temp_c   = 0;
static volatile uint8_t g_inner_temp_c = 0;
static volatile uint8_t g_temp_pwr     = 0;  // bit0=외부센서, bit1=내부센서 전원
static volatile int     g_temp_valid   = 0;


/*===================== 마이크/스피커 전원제어 글로벌 변수 =====================*/
static volatile int     mic_enabled = 1;
static volatile int     speaker_enabled = 1;

static volatile uint8_t g_mic_pwr = 1;
static volatile uint8_t g_speaker_pwr = 1;

static volatile int     g_mic_valid      = 0;
static volatile int     g_speaker_valid  = 0;


/*=============================== LED 전원제어 글로벌 변수 =================================*/

static volatile int led_enabled = 1;

static volatile uint8_t g_led_pwr = 1;

static volatile int g_led_valid = 0;


/* ================= 전체 1s 카운터 =======================*/
static volatile uint32_t g_sec_counter = 0;

/* ===== 다중 RX 소켓 테이블 ============================= */
typedef struct {
    const char* name;       // "tempsvc", "mic", "svc2"
    const char* bind_path;  // 우리가 bind할 경로
    int         fd;         // 수신용 FD
} RxSock;

static RxSock g_rx[] = {
    { "tempsvc", SOCK_RX_TEMP, -1 },
    { "mic",     SOCK_RX_MIC,  -1 },
    { "speaker",    SOCK_RX_SPEAKER, -1 },
    { "led",     SOCK_RX_LED,  -1 }
};
static const int RXN = (int)(sizeof(g_rx) / sizeof(g_rx[0]));

/* === Fake 모드 토글 === */
static int g_use_fake_can  = 0;   // 1=CAN 상태 가짜 값 제공
static int g_use_rand_fake = 0;   // 1=랜덤 가짜 값

/* ================== MMIO helpers (SPI) ================== */
#define spi_read32(ofs)        (*((volatile uint32_t*)((uint8_t*)spi_base + (ofs))))
#define spi_write32(ofs, val)  (*((volatile uint32_t*)((uint8_t*)spi_base + (ofs))) = (val))

static inline void cs_select(void)   { spi_write32(REG_SPI_SSR_OFS, ~0x1u); }
static inline void cs_deselect(void) { spi_write32(REG_SPI_SSR_OFS, 0xFFFFFFFF); }

static uint8_t spi_transfer(uint8_t tx) {
    pthread_mutex_lock(&spi_lock);
    for (int i = 0; i < 1000 && !(spi_read32(REG_SPI_SR_OFS) & SR_TX_EMPTY); i++) usleep(10);
    spi_write32(REG_SPI_DTR_OFS, tx);
    for (int i = 0; i < 1000 && (spi_read32(REG_SPI_SR_OFS) & SR_RX_EMPTY); i++) usleep(10);
    uint8_t v = (uint8_t)spi_read32(REG_SPI_DRR_OFS);
    pthread_mutex_unlock(&spi_lock);
    return v;
}

/* ================== MCP2515 low-level ================== */
static void mcp_write(uint8_t addr, uint8_t val) {
    cs_select();  (void)spi_transfer(MCP_CMD_WRITE); (void)spi_transfer(addr); (void)spi_transfer(val);  cs_deselect();
}
static uint8_t mcp_read(uint8_t addr) {
    cs_select();  (void)spi_transfer(MCP_CMD_READ);  (void)spi_transfer(addr); uint8_t v = spi_transfer(0); cs_deselect(); return v;
}
static void mcp_bit_modify(uint8_t addr, uint8_t mask, uint8_t data) {
    cs_select();  (void)spi_transfer(MCP_CMD_BIT_MODIFY); (void)spi_transfer(addr); (void)spi_transfer(mask); (void)spi_transfer(data); cs_deselect();
}
static void mcp_reset(void) {
    cs_select(); (void)spi_transfer(MCP_CMD_RESET); cs_deselect(); usleep(5000);
}
static void can_set_mode(uint8_t mode) {
    mcp_bit_modify(MCP_REG_CANCTRL, 0xE0, mode);
    while ((mcp_read(MCP_REG_CANSTAT) & 0xE0) != mode) usleep(1000);
}

/* ================== SPI init ================== */
static void spi_init(void) {
    spi_write32(REG_SPI_SRR_OFS, 0x0A); usleep(100);
    spi_write32(REG_SPI_SSR_OFS, 0xFFFFFFFF);
    uint32_t cr = CR_MST | CR_SPE | CR_MANUAL_SS;
    spi_write32(REG_SPI_CR_OFS, cr | CR_TXFIFO_RST | CR_RXFIFO_RST);
    usleep(100);
    spi_write32(REG_SPI_CR_OFS, cr);
    printf("[SPI] CR=0x%08X SR=0x%08X\n", spi_read32(REG_SPI_CR_OFS), spi_read32(REG_SPI_SR_OFS));
}

/* ================== ID assemble (11/29bit) ================== */
static inline uint32_t assemble_can_id(uint8_t sidh, uint8_t sidl, uint8_t eid8, uint8_t eid0, int* is_ext) {
    int ide = (sidl >> 3) & 0x01; // 1: extended
    if (is_ext) *is_ext = ide;
    if (ide) {
        uint32_t id = 0;
        id |= ((uint32_t)sidh) << 21;
        id |= ((uint32_t)(sidl & 0xE0)) << 13;
        id |= ((uint32_t)(sidl & 0x03)) << 16;
        id |= ((uint32_t)eid8) << 8;
        id |= (uint32_t)eid0;
        return id;
    } else {
        return ((uint32_t)sidh << 3) | ((uint32_t)(sidl >> 5) & 0x07);
    }
}

/* ================== 공통 DGRAM 송신 유틸 ================== */
static int send_unix_dgram(const char* path, const void* buf, size_t len) {
    int fd = socket(AF_UNIX, SOCK_DGRAM, 0);
    if (fd < 0) { perror("socket"); return -1; }
    struct sockaddr_un to; memset(&to, 0, sizeof(to));
    to.sun_family = AF_UNIX;
    strncpy(to.sun_path, path, sizeof(to.sun_path) - 1);
    ssize_t n = sendto(fd, buf, len, 0, (struct sockaddr*)&to, sizeof(to));
    if (n < 0) fprintf(stderr, "[IPC] sendto(%s) failed: %s\n", path, strerror(errno));
    close(fd);
    return (n < 0) ? -1 : 0;
}

/* ------------------RS422 IPC tx ---------------------- */
static int tx_socket_to_rs422_payload(uint32_t id, const uint8_t* data, uint8_t dlc) {
    uint8_t payload[13];
    memcpy(payload + 0, &id, 4);
    payload[4] = dlc;
    memset(payload + 5, 0, 8);
    if (dlc > 0 && dlc <= 8 && data) memcpy(payload + 5, data, dlc);
    return send_unix_dgram(RS422_SOCKET_PATH, &payload, sizeof(payload));
}
/*---------------------tempsvc로 제어 명령----------------*/
static inline int tx_socket_to_tempsvc(const uint8_t* data, size_t len) {
    return send_unix_dgram(TEMP_SOCKET_PATH, data, len);
}

/*---------------------mic로 제어 명령----------------*/
static inline int tx_socket_to_mic(const uint8_t* data, size_t len) {
    return send_unix_dgram(MIC_SOCKET_PATH, data, len);
}

/*---------------------speaker로 제어 명령----------------*/
static inline int tx_socket_to_speaker(const uint8_t* data, size_t len) {
    return send_unix_dgram(SPEAKER_SOCKET_PATH, data, len);
}

/*---------------------led로 제어 명령----------------*/
static inline int tx_socket_to_led(const uint8_t* data, size_t len) {
    return send_unix_dgram(LED_SOCKET_PATH, data, len);
}

/* ================== TX payload helpers ================== */
static inline void set_bit(uint8_t* reg, int bit, int on) {
    if (on) *reg |= (1u << bit);
    else    *reg &= ~(1u << bit);
}

/* ===== 선언: 아래에서 정의 ===== */
static int  read_from_rxb(uint8_t read_cmd, uint32_t* id, uint8_t* data, uint8_t* dlc, int* is_ext);
static int  can_recv_any(uint32_t* id, uint8_t* data, uint8_t* dlc);
static void can_send_ext(uint32_t id, const uint8_t* data, uint8_t dlc);

/* ===== 주기 송신: 온도 ===== */
static void send_temperature(void) {
    uint32_t id = TEMP_INFO;

    const int ext_on   = (g_temp_pwr & 0x01) != 0;
    const int inner_on = (g_temp_pwr & 0x02) != 0;

    if (!g_temp_valid || (!ext_on && !inner_on)) {
        uint8_t z[2] = { 0, 0 };
        can_send_ext(id, z, 2);
        return;
    }

    uint8_t data[2] = {
        ext_on   ? (uint8_t)g_ext_temp_c   : 0,
        inner_on ? (uint8_t)g_inner_temp_c : 0
    };
    can_send_ext(id, data, 2);
}

/* ===== 주기 송신: 전원/장치 상태 ===== */ 
static volatile int BUS_ERROR_STATE = 0; /* send_bus_status에서 사용 */
static void send_device_state(void) {
    uint32_t id = POWER_STATUS;
    uint8_t data[8] = { 0 };

    set_bit(&data[5], 0, (g_temp_pwr & 0x01) ? 1 : 0);  // 외부 온도 
    set_bit(&data[5], 1, (g_temp_pwr & 0x02) ? 1 : 0);  // 내부 온도  
    

    if (g_board == BOARD_A) {
        /* SPEAKER 전원 상태 */
        //set_bit(&data[5], 2, (g_speaker_pwr & 0x04) ? 1 : 0);
        set_bit(&data[5], 2, g_speaker_pwr);
    } else if (g_board == BOARD_B) {
        /* MIC 전원 상태 */
        //set_bit(&data[5], 2, (g_mic_pwr & 0x04) ? 1 : 0);
        set_bit(&data[5], 2, g_mic_pwr);
    } else if (g_board == BOARD_C) {
        /* LED 전원 상태 */
        //set_bit(&data[5], 2, (g_led_pwr & 0x04) ? 1 : 0);
        set_bit(&data[5], 2, g_led_pwr);
    }


    set_bit(&data[7], 0, temp_enabled);
    can_send_ext(id, data, 8);

    printf("[POWER STATUS] 외부온도:%d  내부온도:%d  스피커:%d  마이크:%d\n",
        (g_temp_pwr & 0x01), (g_temp_pwr & 0x02), g_speaker_pwr, g_mic_pwr);

}

/* ===== Bus 상태 전송 ===== */
static void send_bus_status(void) {
    uint32_t id = BUS_STATUS;
    uint8_t data[1] = { 0x00 };    // 0=정상
    if (BUS_ERROR_STATE == 3)      data[0] = 0x0F; // BusOff
    else if (BUS_ERROR_STATE == 1) data[0] = 0x01; // Warning
    can_send_ext(id, data, 1);
}

/* ====== 실제/가짜 CAN_Error 제공 ====== */
static void provide_can_error(CAN_Error* e) {
    if (!g_use_fake_can) {
        uint8_t tec  = mcp_read(MCP_REG_TEC);
        uint8_t rec  = mcp_read(MCP_REG_REC);
        uint8_t eflg = mcp_read(MCP_REG_EFLG);

        int bus_off  = (eflg & 0x20) != 0;
        int epassive = (eflg & 0x18) != 0;
        int warn     = (eflg & 0x07) != 0;

        e->TEC = tec;
        e->REC = rec;
        if (bus_off)               e->bus_state = 2;  // BusOff
        else if (epassive || warn) e->bus_state = 1;  // Passive/Warning
        else                       e->bus_state = 0;  // Active
    } else {
        if (!g_use_rand_fake) { e->TEC = 3; e->REC = 1; e->bus_state = 0; }
        else {
            e->TEC = rand() % 10; e->REC = rand() % 10;
            int r = rand() % 100;
            e->bus_state = (r < 70 ? 0 : (r < 90 ? 1 : 2));
        }
    }
}

/* ===== CAN_Error → boardd DETAIL 전송 ===== */
static void notify_can_detail_from_error(const CAN_Error* e)
{
    // protocol.h: can_bus_state (0=Down, 1=Active, 2=BusOff)
    uint8_t state_proto = (e->bus_state == 0) ? 1 : ((e->bus_state == 1) ? 0 : 2);
    BUS_ERROR_STATE = (e->bus_state == 2 ? 3 : (e->bus_state == 1 ? 1 : 0));

    T_BITM_RACK_DETAIL_INFO pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.can_bus_state  = state_proto;
    pkt.can_tx_err_cnt = e->TEC;
    pkt.can_rx_err_cnt = e->REC;

    (void)send_unix_dgram(CAN_DETAIL_SOCK_PATH, &pkt, sizeof(pkt));
    printf("[CAN→boardd] can_bus_state=%u TEC=%u REC=%u -> %s\n",
           pkt.can_bus_state, pkt.can_tx_err_cnt, pkt.can_rx_err_cnt, CAN_DETAIL_SOCK_PATH);
}

/* === 매초: CAN 상태 보고 (실제/가짜) === */
static inline void report_can_status_once(void) {
    CAN_Error e;
    provide_can_error(&e);
    printf("[CAN-STATUS] (FAKE=%d RAND=%d) TEC=%u REC=%u state=%u\n",
           g_use_fake_can, g_use_rand_fake, e.TEC, e.REC, e.bus_state);
    notify_can_detail_from_error(&e);
}

/* ================== RX 공통 리더 (정의) ================== */
static int read_from_rxb(uint8_t read_cmd, uint32_t* id, uint8_t* data, uint8_t* dlc, int* is_ext)
{
    cs_select();
    (void)spi_transfer(read_cmd);          // 0x90(RXB0 SIDH) or 0x92(RXB1 SIDH)
    uint8_t sidh   = spi_transfer(0);
    uint8_t sidl   = spi_transfer(0);
    uint8_t eid8   = spi_transfer(0);
    uint8_t eid0   = spi_transfer(0);
    uint8_t dlcraw = spi_transfer(0);
    uint8_t rtr    = (dlcraw & 0x40) ? 1 : 0;
    uint8_t n      = dlcraw & 0x0F;
    if (!rtr) { for (uint8_t i = 0; i < n; i++) data[i] = spi_transfer(0); }
    else      { for (uint8_t i = 0; i < n; i++) (void)spi_transfer(0); }
    cs_deselect();

    int ext = 0;
    uint32_t rid = assemble_can_id(sidh, sidl, eid8, eid0, &ext);
    if (is_ext) *is_ext = ext;

    *id  = rid;
    *dlc = n;
    return 1;
}

/* ================== can_recv_any (정의) ================== */
static int can_recv_any(uint32_t* id, uint8_t* data, uint8_t* dlc)
{
    uint8_t flag = mcp_read(MCP_REG_CANINTF);

    if (flag & CANINTF_RX0IF) {
        int is_ext = 0;
        int ok = read_from_rxb(MCP_CMD_READ_RXB0SIDH, id, data, dlc, &is_ext);
        mcp_bit_modify(MCP_REG_CANINTF, CANINTF_RX0IF, 0x00); // clear
        if (ok) {
            printf("[RX] %s ID=0x%X DLC=%u\n", is_ext ? "EXT" : "STD", *id, *dlc);
            return 1;
        }
    }
    if (flag & CANINTF_RX1IF) {
        int is_ext = 0;
        int ok = read_from_rxb(MCP_CMD_READ_RXB1SIDH, id, data, dlc, &is_ext);
        mcp_bit_modify(MCP_REG_CANINTF, CANINTF_RX1IF, 0x00); // clear
        if (ok) {
            printf("[RX] %s ID=0x%X DLC=%u\n", is_ext ? "EXT" : "STD", *id, *dlc);
            return 1;
        }
    }
    return 0;
}

/* ================== Extended TX (정의) ================== */
static void can_send_ext(uint32_t id, const uint8_t* data, uint8_t dlc)
{
    uint8_t sidh = (uint8_t)(id >> 21);
    uint8_t sidl = ((uint8_t)(id >> 13) & 0xE0) | (1 << 3) | ((uint8_t)(id >> 16) & 0x03);
    uint8_t eid8 = (uint8_t)(id >> 8);
    uint8_t eid0 = (uint8_t)id;

    cs_select();
    (void)spi_transfer(MCP_CMD_WRITE);
    (void)spi_transfer(MCP_REG_TXB0CTRL + 1); // start at SIDH
    (void)spi_transfer(sidh);
    (void)spi_transfer(sidl);
    (void)spi_transfer(eid8);
    (void)spi_transfer(eid0);
    (void)spi_transfer(dlc & 0x0F);
    for (int i = 0; i < dlc; i++) (void)spi_transfer(data[i]);
    cs_deselect();

    mcp_bit_modify(MCP_REG_TXB0CTRL, TXREQ, TXREQ);
}

/* ================== Threads ================== */
static void* tx_thread(void* arg) {
    (void)arg;
    int tfd = timerfd_create(CLOCK_MONOTONIC, 0);
    if (tfd == -1) { perror("timerfd_create"); pthread_exit(NULL); }
    struct itimerspec its = { .it_interval = (struct timespec){1,0},
                              .it_value    = (struct timespec){1,0} };
    timerfd_settime(tfd, 0, &its, NULL);

    if (g_use_rand_fake) srand((unsigned)time(NULL) ^ getpid());

    uint64_t exp;
    while (g_running) {
        if (read(tfd, &exp, sizeof(exp)) != sizeof(exp)) continue;

        /* 전역 1s 카운터 증가 */
        g_sec_counter++;

        /* 매초: boardd로 CAN 상태 보고 */
        report_can_status_once();

        /* 하드웨어 전송은 FAKE 모드에서 호출 금지 */
        if (!g_use_fake_can) {
            send_temperature();                   // +0.00s
            usleep(333000);  send_device_state(); // +0.33s
            usleep(333000);  send_bus_status();   // +0.66s
        } else {
            /* 페이싱만 유지 */
            usleep(333000);
            usleep(333000);
        }
    }
    close(tfd);
    return NULL;
}

static void* rx_thread(void* arg) {
    (void)arg;
    uint8_t  buf[8], dlc;
    uint32_t id;

    while (g_running) {
        while (can_recv_any(&id, buf, &dlc)) {
            if (id == POWER_REPLY) continue; /* 내가 보낸 ACK는 드롭 */

            /* 제어 ID → 온도 송신 on/off */
            if (id == POWER_CONTROL && dlc >= 2) {
                temp_enabled   = (buf[1] >> 0) & 1;
                inner_enabled  = (buf[1] >> 1) & 1;
                g_temp_pwr     = (temp_enabled ? 0x01 : 0x00) | (inner_enabled ? 0x02 : 0x00);
                
                
                if (g_board == BOARD_A) {
                    speaker_enabled = (buf[1] >> 2) & 1;
                    g_speaker_pwr = speaker_enabled;
                } else if (g_board == BOARD_B) {
                    mic_enabled = (buf[1] >> 2) & 1;
                    g_mic_pwr = mic_enabled;
                } else if (g_board == BOARD_C) {
                    led_enabled = (buf[1] >> 2) & 1;
                    g_led_pwr = led_enabled;
                }

                can_send_ext(POWER_REPLY, buf, 2);

                (void)tx_socket_to_tempsvc(buf, 2);
                
                if (g_board == BOARD_A) {
                    (void)tx_socket_to_speaker(buf, 2);
                } else if (g_board == BOARD_B) {
                    (void)tx_socket_to_mic(buf, 2);
                } else if (g_board == BOARD_C) {
                    (void)tx_socket_to_led(buf, 2);
                }

                (void)tx_socket_to_rs422_payload(id, buf, 2);

                printf("[CMD] temp_enabled=%d inner_enabled=%d (pwr=0x%02X)  speaker=%d   mic=%d   led=%d \n",
                       temp_enabled, inner_enabled, g_temp_pwr, speaker_enabled, mic_enabled, led_enabled);
            }
        }
        usleep(5000); // idle
    }
    return NULL;
}

/* ====== 다중 바인드 + poll 수신 스레드 ====== */
static int rx_bind_all(void) {
    for (int i = 0; i < RXN; i++) {
        unlink(g_rx[i].bind_path);
        int fd = socket(AF_UNIX, SOCK_DGRAM, 0);
        if (fd < 0) { perror("socket"); return -1; }
        struct sockaddr_un me; memset(&me, 0, sizeof(me));
        me.sun_family = AF_UNIX;
        strncpy(me.sun_path, g_rx[i].bind_path, sizeof(me.sun_path) - 1);
        if (bind(fd, (struct sockaddr*)&me, sizeof(me)) < 0) {
            fprintf(stderr, "[IPC] bind(%s) failed: %s\n", g_rx[i].bind_path, strerror(errno));
            close(fd);
            return -1;
        }
        chmod(g_rx[i].bind_path, 0666);
        g_rx[i].fd = fd;
        printf("[IPC] recv ready: %s (%s)\n", g_rx[i].name, g_rx[i].bind_path);
    }
    return 0;
}

static void rx_handle(int idx, const uint8_t* buf, ssize_t n) {
    if (strcmp(g_rx[idx].name, "tempsvc") == 0) {
        if (n == 3) {
            g_ext_temp_c   = buf[0];
            g_inner_temp_c = buf[1];
            g_temp_pwr     = buf[2] & 0x03;
            g_temp_valid   = 1;

            printf("[tempsvc] ext=%u°C, inner=%u°C, pwr(ext=%u,inner=%u)\n",
                   g_ext_temp_c, g_inner_temp_c,
                   (g_temp_pwr & 0x01) ? 1 : 0, (g_temp_pwr & 0x02) ? 1 : 0);
        }
    }
    // mic
    if (strcmp(g_rx[idx].name, "mic") == 0) {
        if (n == 1) {
            g_mic_pwr = buf[0] & 0x04;
            g_mic_valid = 1;

            printf("[mic] pwr(mic=%u)\n",
                (g_mic_pwr & 0x04) ? 1 : 0);
        }
    }
    // speaker
    if (strcmp(g_rx[idx].name, "speaker") == 0) {
        if (n == 1) {
            g_speaker_pwr = buf[0] & 0x04;
            g_speaker_valid = 1;

            printf("[speaker] pwr(speaker=%u)\n",
                (g_speaker_pwr & 0x04) ? 1 : 0);
        }
    }
    // led
    if (strcmp(g_rx[idx].name, "led") == 0) {
        if (n == 1) {
            g_led_pwr = buf[0] & 0x04;
            g_led_valid = 1;

            printf("[led] pwr(mic=%u)\n",
                (g_led_pwr & 0x04) ? 1 : 0);
        }
    }
}

static void* rx_poll_thread(void* arg) {
    (void)arg;
    if (rx_bind_all() < 0) return NULL;

    struct pollfd pfds[8];
    int idxmap[8];
    int K = 0;
    for (int i = 0; i < RXN; i++) {
        if (g_rx[i].fd >= 0) {
            pfds[K].fd     = g_rx[i].fd;
            pfds[K].events = POLLIN;
            idxmap[K]      = i;
            K++;
        }
    }
    if (K == 0) return NULL;

    uint8_t buf[512];
    while (g_running) {
        int r = poll(pfds, K, 500);   // 0.5s 타임아웃
        if (r < 0) { if (errno == EINTR) continue; perror("poll"); break; }
        if (r == 0) continue;
        for (int k = 0; k < K; k++) {
            if (pfds[k].revents & POLLIN) {
                ssize_t n = recv(pfds[k].fd, buf, sizeof buf, 0);
                if (n > 0) rx_handle(idxmap[k], buf, n);
            }
        }
    }

    for (int i = 0; i < RXN; i++) {
        if (g_rx[i].fd >= 0) close(g_rx[i].fd);
        if (g_rx[i].bind_path) unlink(g_rx[i].bind_path);
    }
    return NULL;
}

/* ===== 유틸: MAC 읽기 & 보드 결정 ===== */
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

static void set_ids_for_board(board_t b){
    switch(b){
        case BOARD_A:
            POWER_CONTROL = 0x00084051u;
            POWER_REPLY   = 0x0A884001u;
            BUS_STATUS    = 0x0A881001u;
            TEMP_INFO     = 0x0A887001u;
            POWER_STATUS  = 0x0A880001u;
            g_log_prefix  = "[CAN/A] ";
            break;
        case BOARD_B:
            POWER_CONTROL = 0x00084052u;
            POWER_REPLY   = 0x0A904001u;
            BUS_STATUS    = 0x0A901101u;
            TEMP_INFO     = 0x0A907001u;
            POWER_STATUS  = 0x0A900001u;
            g_log_prefix  = "[CAN/B] ";
            break;
        default: /* C */
            POWER_CONTROL = 0x00084053u;
            POWER_REPLY   = 0x0A984001u;
            BUS_STATUS    = 0x0A981191u;
            TEMP_INFO     = 0x0A987001u;
            POWER_STATUS  = 0x0A980001u;
            g_log_prefix  = "[CAN/C] ";
            break;
    }
}

static void decide_board_by_mac(void){
    /* MAC 매핑표(소문자) */
    struct { const char* mac; board_t b; } map[] = {
        { "00:0a:35:00:1e:52", BOARD_A },
        { "00:0a:35:00:1e:53", BOARD_B },
        { "00:0a:35:00:1e:54", BOARD_C },
    };

    /* 1) 환경변수 강제 (CAN_BOARD 우선, 없으면 RS422_BOARD도 허용) */
    const char* env_board = getenv("CAN_BOARD");
    if(!env_board) env_board = getenv("RS422_BOARD");
    if(env_board){
        if (env_board[0]=='A'||env_board[0]=='a') g_board=BOARD_A;
        else if (env_board[0]=='B'||env_board[0]=='b') g_board=BOARD_B;
        else if (env_board[0]=='C'||env_board[0]=='c') g_board=BOARD_C;
        set_ids_for_board(g_board);
        LOGF("Board override by env: %c\n", env_board[0]);
        return;
    }

    /* 2) 인터페이스에서 MAC 읽어 자동 매핑 (CAN_IFACE 우선, 없으면 RS422_IFACE, 없으면 eth0) */
    const char* ifn = getenv("CAN_IFACE");
    if(!ifn) ifn = getenv("RS422_IFACE");
    if(!ifn) ifn = "eth0";
    char mac[64]={0};
    if(read_mac_lower(ifn, mac, sizeof(mac))==0){
        for(size_t i=0;i<sizeof(map)/sizeof(map[0]);++i){
            if(strcmp(mac, map[i].mac)==0){ g_board = map[i].b; break; }
        }
    }
    set_ids_for_board(g_board);
    LOGF("Board decided: %s (iface=%s, mac=%s)\n",
         (g_board==BOARD_A)?"A":(g_board==BOARD_B)?"B":"C", ifn, mac[0]?mac:"<NA>");
}

/* ================== MAIN ================== */
static void sig_handler(int s) { (void)s; g_running = 0; }

int main(int argc, char** argv) {
    /* 즉시 출력 */
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);

    /* 보드 자동 판별 */
    decide_board_by_mac();

    /* Fake 모드 파라미터 */
    g_use_fake_can  = (getenv("CAN_FAKE") != NULL);
    g_use_rand_fake = (getenv("CAN_RAND") != NULL);
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--fake-can")) g_use_fake_can = 1;
        else if (!strcmp(argv[i], "--rand")) g_use_rand_fake = 1;
    }
    if (g_use_rand_fake) srand((unsigned)time(NULL) ^ getpid());

    signal(SIGINT, sig_handler);
    signal(SIGTERM, sig_handler);
    printf("--- CAN Service Start (FAKE=%d, RAND=%d, BOARD=%s) ---\n",
           g_use_fake_can, g_use_rand_fake,
           (g_board==BOARD_A)?"A":(g_board==BOARD_B)?"B":"C");

    /* === FAKE 모드면 하드웨어 초기화 전부 건너뜀 === */
    if (!g_use_fake_can) {
        /* SPI UIO: mmap (map1 offset = pagesize) */
        int sfd = open(SPI_UIO_PATH, O_RDWR | O_SYNC);
        if (sfd < 0) { perror("open SPI UIO"); return 1; }
        long pg = getpagesize();
        spi_base = mmap(NULL, SPI_MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, sfd, pg);
        if (spi_base == MAP_FAILED) { perror("mmap SPI UIO"); close(sfd); return 1; }
        close(sfd);

        /* SPI/MCP 초기화 */
        spi_init();
        mcp_reset();

        /* reset 직후 응답 점검 (CANSTAT OPMOD = 0b100x) */
        uint8_t stat = mcp_read(MCP_REG_CANSTAT);
        printf("[CHK] CANSTAT=0x%02X (expect 0x8x)\n", stat);
        if ((stat & 0xE0) != 0x80) {
            fprintf(stderr, "[FATAL] MCP2515 SPI comm fail\n");
            return 1;
        }

        /* 16MHz 기준 250kbps (CNF 0x03/0xBA/0x07) */
        mcp_write(MCP_REG_CNF1, 0x03);
        mcp_write(MCP_REG_CNF2, 0xBA);
        mcp_write(MCP_REG_CNF3, 0x07);

        /* 수신 버퍼/인터럽트 설정 */
        mcp_write(MCP_REG_RXB0CTRL, RXM_ALL | 0x04); // BUKT=1
        mcp_write(MCP_REG_RXB1CTRL, RXM_ALL);
        mcp_write(MCP_REG_CANINTE, 0x03);            // RX0IE|RX1IE

        can_set_mode(MODE_NORMAL);
        printf("[OK] MCP2515 NORMAL mode (CANCTRL=0x%02X)\n", mcp_read(MCP_REG_CANCTRL));
    } else {
        printf("[FAKE] MCP2515 init skipped; running without /dev/uioX\n");
    }

    /* 스레드 */
    pthread_t th_tx, th_rx, th_rxpoll;
    pthread_create(&th_tx, NULL, tx_thread, NULL);

    /* 실제 모드에서만 수신 스레드 시작 (FAKE면 MCP 읽기 안 함) */
    if (!g_use_fake_can) {
        pthread_create(&th_rx,     NULL, rx_thread,     NULL);
        pthread_create(&th_rxpoll, NULL, rx_poll_thread,NULL);
        pthread_join(th_rx, NULL);
        // 필요시 rx_poll 스레드도 join:
        // pthread_join(th_rxpoll, NULL);
    }

    pthread_join(th_tx, NULL);

    /* Cleanup */
    if (!g_use_fake_can) {
        can_set_mode(MODE_CONFIG);
        if (spi_base && spi_base != MAP_FAILED) munmap((void*)spi_base, SPI_MAP_SIZE);
    }
    printf("[EXIT]\n");
    return 0;
}
