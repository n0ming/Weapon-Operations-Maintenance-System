#ifndef ZYBO_MAINT_PROTOCOL_H
#define ZYBO_MAINT_PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ===================== 공통 상수 ===================== */
enum {
    TCP_MAGIC_START   = 0xAAAAAAAAu,
    TCP_MAGIC_TAIL    = 0x55555555u,
    TCP_PORT_DEFAULT  = 9000
};
/* 멀티바이트 전송은 네트워크 바이트오더(BE) 권장: htons/htonl 사용 */

/* ===================== 시간 (8B) ===================== */
typedef struct __attribute__((packed)) {
    uint16_t year;                 /* BE 전송 권장 */
    uint8_t  mon, day, hour, min, sec, sec100; /* sec100: 0.01s */
} T_UtcTime;                       /* 8 B */

/* ===================== Header / Tail ===================== */
typedef struct __attribute__((packed)) {
    uint32_t  start;               /* 0xAAAAAAAA, BE */
    uint8_t   sender;              /* 보드/PC ID */
    uint8_t   receiver;            /* 대상 ID */
    uint16_t  infocode;            /* BE */
    uint32_t  length;              /* BE: Header~Tail 전체 길이 */
    T_UtcTime time;                /* 8 B */
} T_TCP_HEADER;                    /* 20 B */

typedef struct __attribute__((packed)) {
    uint32_t tail;                 /* 0x55555555, BE */
} T_TCP_TAIL;                      /* 4 B */

/* ===================== InfoCode 비트맵 =====================
   [15]   RESP(0=req,1=resp)
   [14:12]MODE (0 RT,1 NRT,2 CTRL,3 MAINT,4 TEST)
   [11:8] SUBJECT
   [7:4]  ACTION (0 Status,1 Detail,2 SelfTest,3 Start,4 Stop,5 Get,6 Set)
   [3:0]  VAR
*/
#define IC_RESP      0x8000
#define IC_MODE(m)   ( ((uint16_t)((m)&0x7u))  << 12 )
#define IC_SUBJ(s)   ( ((uint16_t)((s)&0xFu))  << 8  )
#define IC_ACT(a)    ( ((uint16_t)((a)&0xFu))  << 4  )
#define IC_VAR(v)    ( ((uint16_t)((v)&0xFu))       )
#define IC_BUILD(mode,subj,act,var,resp) \
    ( IC_MODE(mode) | IC_SUBJ(subj) | IC_ACT(act) | IC_VAR(var) | ((resp)?IC_RESP:0) )

/* MODE */
enum { IC_RT = 0, IC_NRT = 1, IC_CTRL = 2, IC_MAINT = 3, IC_TEST = 4 };

/* SUBJECT — BASIC RawState와 1:1 매핑 */
enum {
    IC_SUBJ_SYS  = 0,  /* BOARD_ERR(1), SYSTEM_ERR(2) */
    IC_SUBJ_CAN  = 1,  /* CAN_ERR(3)   */
    IC_SUBJ_ETH  = 2,  /* ETH_ERR(5)   */
    IC_SUBJ_SER  = 3,  /* SERIAL_ERR(4)*/
    IC_SUBJ_TEMP = 4,  /* TEMP_ERR(9)  */
    IC_SUBJ_MIC  = 5,  /* MIC_ERR(10)  */
    IC_SUBJ_SPK  = 6,  /* SPK_ERR(11)  */
    IC_SUBJ_LED  = 7   /* LED_ERR(12)  */
};

/* ACTION */
enum {
    IC_ACT_STATUS = 0, IC_ACT_DETAIL = 1, IC_ACT_SELFTEST = 2,
    IC_ACT_START  = 3, IC_ACT_STOP   = 4, IC_ACT_GET     = 5, IC_ACT_SET = 6
};

/* 자주 쓰는 조합 (예시) */
#define IC_RT_SYS_STATUS     IC_BUILD(IC_RT,  IC_SUBJ_SYS,  IC_ACT_STATUS, 0, 0) /* 0x0000 */
#define IC_RT_CAN_STATUS     IC_BUILD(IC_RT,  IC_SUBJ_CAN,  IC_ACT_STATUS, 0, 0) /* 0x0100 */
#define IC_RT_ETH_STATUS     IC_BUILD(IC_RT,  IC_SUBJ_ETH,  IC_ACT_STATUS, 0, 0) /* 0x0200 */
#define IC_RT_SER_STATUS     IC_BUILD(IC_RT,  IC_SUBJ_SER,  IC_ACT_STATUS, 0, 0) /* 0x0300 */
#define IC_RT_TEMP_STATUS    IC_BUILD(IC_RT,  IC_SUBJ_TEMP, IC_ACT_STATUS, 0, 0)
#define IC_RT_MIC_STATUS     IC_BUILD(IC_RT,  IC_SUBJ_MIC,  IC_ACT_STATUS, 0, 0)
#define IC_RT_SPK_STATUS     IC_BUILD(IC_RT,  IC_SUBJ_SPK,  IC_ACT_STATUS, 0, 0)
#define IC_RT_LED_STATUS     IC_BUILD(IC_RT,  IC_SUBJ_LED,  IC_ACT_STATUS, 0, 0)

#define IC_NRT_SYS_DETAIL    IC_BUILD(IC_NRT, IC_SUBJ_SYS,  IC_ACT_DETAIL, 0, 0)
#define IC_NRT_CAN_DETAIL    IC_BUILD(IC_NRT, IC_SUBJ_CAN,  IC_ACT_DETAIL, 0, 0)
#define IC_NRT_ETH_DETAIL    IC_BUILD(IC_NRT, IC_SUBJ_ETH,  IC_ACT_DETAIL, 0, 0)
#define IC_NRT_SER_DETAIL    IC_BUILD(IC_NRT, IC_SUBJ_SER,  IC_ACT_DETAIL, 0, 0)
#define IC_NRT_TEMP_DETAIL   IC_BUILD(IC_NRT, IC_SUBJ_TEMP, IC_ACT_DETAIL, 0, 0)
#define IC_NRT_MIC_DETAIL    IC_BUILD(IC_NRT, IC_SUBJ_MIC,  IC_ACT_DETAIL, 0, 0)
#define IC_NRT_SPK_DETAIL    IC_BUILD(IC_NRT, IC_SUBJ_SPK,  IC_ACT_DETAIL, 0, 0)
#define IC_NRT_LED_DETAIL    IC_BUILD(IC_NRT, IC_SUBJ_LED,  IC_ACT_DETAIL, 0, 0)

/* ===================== BASIC (30B 고정) =====================
 * 문서 테이블 그대로:
 * | OpMode(1) | CheckTime(8) | Position(1) | AssignedFunc(1) |
 * | Reserved(3) | RawStates 블록(16) |
 * RawStates 블록 = v[3](각 4B) + pad(4B) = 16B
 *  - v[0]: 상태 비트맵
 *  - v[1]: 보드 온도(℃*100, int16 하위 16비트 사용), 상위 16비트 0
 *  - v[2]: 확장용
 */
typedef struct __attribute__((packed)) {
    uint32_t v[3];  /* 모두 BE 전송 권장 */
    uint32_t _pad;  /* 16B 블록 고정 */
} T_RawStates16;    /* 16 B */

typedef struct __attribute__((packed)) {
    uint8_t     op_mode;        /* 1B */
    T_UtcTime   check_time;     /* 8B */
    uint8_t     position;       /* 1B */
    uint8_t     assigned_func;  /* 1B (1:보드1, 2:보드2, 3:보드3) */
    uint8_t     reserved[3];    /* 3B */
    T_RawStates16 raw_states;   /* 16B */
} T_BITM_BASIC_INFO;            /* 총 30 B */

/* RawState[0] 비트 (문서 표와 동일) */
enum {
    RS0_UNDEF_ERR   = 0,   /* 정의되지 않은 오류 */
    RS0_BOARD_ERR   = 1,   /* CPU/MEM/POWER/내부온도 이상 */
    RS0_SYSTEM_ERR  = 2,   /* APP/FPGA/서비스 오류 (상위 개념) */
    RS0_CAN_ERR     = 3,   /* CAN 통신 이상 */
    RS0_SERIAL_ERR  = 4,   /* RS-422 이상 */
    RS0_ETH_ERR     = 5,   /* Ethernet 이상 */
    /* 6..8 Reserved */
    RS0_TEMP_ERR    = 9,   /* LM75B 온도 오류 */
    RS0_MIC_ERR     = 10,  /* 마이크 오류 */
    RS0_SPK_ERR     = 11,  /* 스피커 출력 오류 */
    RS0_LED_ERR     = 12,  /* LED 제어 오류 */
    RS0_SVC_ERR     = 13,  /* 서비스(svc_status) 에러: 하나라도 죽으면 1 */
    /* 14..15 Reserved */
};

/* 헬퍼: 상태 비트 읽기 */
static inline int basic_state_bit(const T_BITM_BASIC_INFO* b, unsigned bit) {
    uint32_t s = b->raw_states.v[0];
    return (int)((s >> (bit & 31)) & 1u);
}

/* ===================== DETAIL (52B 고정) =====================
 * 문서 표를 내장형 고정 필드로 구체화:
 * - service_error[]: systemd 서비스 상태 바이트(0=OK, 그 외=에러코드)
 * - 필요 시 길이/의미를 프로젝트 표준으로 합의해 사용
 */
typedef struct __attribute__((packed)) {
    /* ---- System Meta ---- */
    uint8_t  app_ver[4];           /* 예: {1,0,0,0} */
    uint8_t  fpga_ver[4];
    uint8_t  cpu_usage;            /* % */
    uint8_t  mem_usage;            /* % */
    int16_t  temperature_board;    /* 0.1°C, BE */
    uint8_t  service_error[6];     /* systemd 서비스 상태 (0=OK) */
    uint32_t uptime_sec;           /* BE */

    /* ---- CAN ---- */
    uint8_t  can_bus_state;        /* 0=Down,1=Active,2=BusOff */
    uint8_t  can_tx_err_cnt;
    uint8_t  can_rx_err_cnt;

    /* ---- RS422 ---- */
    uint8_t  rs422_socket_err;     /* CAN-Serial/소켓 오류 플래그 */
    uint16_t rs422_tx_err;         /* BE */
    uint16_t rs422_rx_err;         /* BE */

    /* ---- Ethernet ---- */
    uint8_t  eth_link_up;          /* 0/1 */
    uint8_t  eth_loss_pct;         /* 0~100 */
    uint8_t  eth_ping_fail_cnt;    /* 누적 실패 */

    /* ---- LM75B Sensor ---- */
    uint8_t  lm75b_link_ok;        /* 0/1 */
    uint8_t  lm75b_power_err;      /* 0/1 */
    uint8_t  lm75b_comm_err;       /* 0/1 */
    int16_t  lm75b_temp_centi;     /* 0.01°C, BE */

    /* ---- MIC ---- */
    uint8_t  mic_power_err;        /* 0/1 */
    uint8_t  mic_comm_err;         /* 0/1 */

    /* ---- SPK ---- */
    uint8_t  spk_power_err;        /* 0/1 */
    uint8_t  spk_comm_err;         /* 0/1 */

    /* ---- LED ---- */
    uint8_t  led_link_ok;          /* 0/1 */
    uint8_t  led_power_err;        /* 0/1 */
    uint8_t  led_comm_err;         /* 0/1 */

    /* 52B 정렬/확장용 */
    uint8_t  _reserved[7];
} T_BITM_RACK_DETAIL_INFO;         /* 52 B */

/* ===================== 사이즈 가드(선택) ===================== */
/* C11 이상이면 켜서 빌드 타임 확인 권장
#if __STDC_VERSION__ >= 201112L
_Static_assert(sizeof(T_TCP_HEADER) == 20, "T_TCP_HEADER must be 20B");
_Static_assert(sizeof(T_TCP_TAIL)   == 4,  "T_TCP_TAIL must be 4B");
_Static_assert(sizeof(T_BITM_BASIC_INFO) == 30, "BASIC must be 30B");
_Static_assert(sizeof(T_BITM_RACK_DETAIL_INFO) == 52, "DETAIL must be 52B");
#endif
*/

#ifdef __cplusplus
}
#endif
#endif /* ZYBO_MAINT_PROTOCOL_H */
