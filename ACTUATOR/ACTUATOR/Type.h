/**
 * @file    types.h
 * @brief   BLSlot(약속) ATmega128A 공용 enum / struct 정의
 *
 * 기반: ATmega128A_펌웨어_식별자_v3_noEEPROM.md §2
 *
 * 표기 규칙: 상수 UPPER_SNAKE, 타입 PascalCase,
 *            함수 Module_PascalCase, 전역 g_snake_case, 정적 s_snake_case
 *
 * 소유권 (팀 분업 가이드라인 §2.1):
 *   Team A : SystemState, CmdType, ErrorCode, Frame, LastCmdRecord, SystemCtx
 *   Team B : AxisId, StepDir, DispenseResult, DispensePhase, StepperStatus,
 *            ServoStatus, HomingStatus, StepperAxis, ServoCtx, SensorCtx,
 *            HomingCtx, DispenseCtx
 *
 * 주의: 이 파일은 config.h 를 포함하지 않는다 (config.h 가 이 파일을 포함).
 *       순환 포함을 막기 위해 여기에는 config.h 의 매크로를 쓰지 않는다.
 */

#ifndef TYPES_H
#define TYPES_H

#include <stdint.h>
#include <stdbool.h>

/* ==========================================================================
 * 1. enum
 * ========================================================================== */

/* ---- Team A 소유 -------------------------------------------------------- */

/** 시스템 상태 (설계서 §2, flat 6-state) */
typedef enum {
    STATE_IDLE = 0,
    STATE_MOVING,
    STATE_AWAITING_DISPENSE,
    STATE_DISPENSING,
    STATE_AWAITING_RESULT_ACK,
    STATE_RECOVERY_REQUIRED
} SystemState;

/** 수신 명령 종류 (설계서 §3.2.1) */
typedef enum {
    CMD_NONE = 0,
    CMD_MOVE,
    CMD_DISPENSE,
    CMD_ACK,
    CMD_TIMEOUT,
    CMD_UNKNOWN
} CmdType;

/** ERROR 응답 코드 (설계서 §3.2.3) */
typedef enum {
    ERR_INVALID_FORMAT = 0,     /**< 요청번호 없이 2필드로 송신 */
    ERR_INVALID_COORD,
    ERR_INVALID_TIME,
    ERR_ID_CONFLICT,
    ERR_BUSY,
    ERR_NOT_READY,
    ERR_COORD_MISMATCH,
    ERR_RECOVERY_REQUIRED,
    ERR_STEPPER_ERROR,          /**< §6-13: 검출 조건 미확정 */
    ERR_SERVO_ERROR,            /**< §6-13: 검출 조건 미확정 */
    ERR_SENSOR_ERROR            /**< §6-13: 검출 조건 미확정 */
} ErrorCode;

/* ---- Team B 소유 -------------------------------------------------------- */

/** 축 식별자. AXIS_COUNT 는 배열 크기용 */
typedef enum {
    AXIS_X = 0,
    AXIS_Y,
    AXIS_COUNT
} AxisId;

/** 스텝 진행 방향. 스텝 인덱스 증감 부호와 일치 */
typedef enum {
    STEP_DIR_MINUS = -1,
    STEP_DIR_NONE  = 0,
    STEP_DIR_PLUS  = 1
} StepDir;

/** 배출 결과. 값은 RESULT 프레임의 R 필드와 동일 (설계서 §3.2.2) */
typedef enum {
    DISPENSE_RESULT_NONE    = 0,
    DISPENSE_RESULT_SUCCESS = 1,    /**< RESULT R=1 */
    DISPENSE_RESULT_EMPTY   = 2     /**< RESULT R=2 */
} DispenseResult;

/**
 * 압출 3-phase 내부 서브상태 (dispense.c 전용).
 * FSM 은 이 값을 보지 않고 Dispense_IsBusy() / Dispense_IsComplete() 만 쓴다.
 */
typedef enum {
    DPHASE_IDLE = 0,
    DPHASE_MOVE_MINUS,      /**< 1차: (y - a) 로 이동 */
    DPHASE_PUSH_MINUS,
    DPHASE_MOVE_CENTER,     /**< 2차: (y) */
    DPHASE_PUSH_CENTER,
    DPHASE_MOVE_PLUS,       /**< 3차: (y + a) */
    DPHASE_PUSH_PLUS,
    DPHASE_DETECT_WINDOW,   /**< 3차 0도 도달 후 5초 감지창 */
    DPHASE_DONE
} DispensePhase;

/** 스테퍼 구동 상태 (stepper.c 내부, Stepper_IsBusy() 로만 공개) */
typedef enum {
    STEPPER_IDLE = 0,
    STEPPER_RUNNING
} StepperStatus;

/** 서보 상태 (servo.c 내부, Servo_Status() 로 공개) */
typedef enum {
    SERVO_IDLE = 0,
    SERVO_RAMPING,          /**< 50도 -> 0도, 5도/500ms 하강 중 */
    SERVO_SETTLING          /**< 즉시 점프 후 SERVO_SETTLE_MS 안정 대기 */
} ServoStatus;

/** 홈 탐색 상태 (설계서 §7) */
typedef enum {
    HOMING_IDLE = 0,
    HOMING_SEARCHING,       /**< 자동 복구: 스텝 예산 내 탐색 */
    HOMING_MANUAL_WAIT,     /**< 수동 복구: 코일 해제, 사람이 밀기 대기 */
    HOMING_DONE
} HomingStatus;

/* ==========================================================================
 * 2. struct
 * ========================================================================== */

/* ---- Team A 소유 -------------------------------------------------------- */

/** 파싱된 프레임 1개 */
typedef struct {
    CmdType   cmd;
    uint32_t  req_id;
    uint8_t   x;
    uint8_t   y;
    uint32_t  allow_time_sec;   /**< MOVE 의 TTTTTT. 검증/기록 전용 */
    char      ack_target[12];   /**< "MOVE" / "DISPENSE" / "RESULT" / "TIMEOUT" */
    bool      valid;
} Frame;

/** 중복 / ID 충돌 판정용 캐시 (MOVE, DISPENSE 각 1개) */
typedef struct {
    uint32_t  req_id;
    uint8_t   x;
    uint8_t   y;
    uint32_t  allow_time_sec;
    bool      seen;
} LastCmdRecord;

/* ---- Team B 소유 -------------------------------------------------------- */

/** 축별 스테퍼 컨텍스트 (stepper.c 내부 static) */
typedef struct {
    int16_t        cur_steps;       /**< 현재 절대 스텝. open-loop 추정치 */
    int16_t        target_steps;
    uint8_t        seq_index;       /**< Full Drive 4-step 시퀀스 인덱스 */
    StepDir        dir;
    StepperStatus  status;
    uint32_t       last_tick_ms;
} StepperAxis;

/** 서보 컨텍스트 (servo.c 내부 static) */
typedef struct {
    uint8_t       cur_angle;
    uint8_t       target_angle;
    ServoStatus   status;
    uint32_t      last_tick_ms;
} ServoCtx;

/**
 * 센서 컨텍스트 (sensors.c 내부 static).
 * latch 계열은 ISR 이 쓰므로 volatile.
 */
typedef struct {
    volatile bool  ir_latched;          /**< IR1 OR IR2 감지 래치 */
    volatile bool  home_latched[AXIS_COUNT];  /**< [추가] 축 배열화 */
    uint32_t       ir_last_edge_ms;     /**< 디바운스 기준 시각 */
    bool           ir_enabled;          /**< [추가] Sensors_IrEnable() 상태 미러 */
    uint32_t       home_poll_tick_ms;   /**< [추가] 레벨 폴링 주기 기준 */
} SensorCtx;

/** 홈 탐색 컨텍스트 (homing.c 내부 static) */
typedef struct {
    HomingStatus  status;
    bool          x_done;
    bool          y_done;
    uint16_t      x_step_count;     /**< 스텝 예산 소모량 */
    uint16_t      y_step_count;
    uint32_t      last_tick_ms;     /**< [추가] HOMING_STEP_INTERVAL_MS 기준 */
} HomingCtx;

/** 배출 시퀀스 컨텍스트 (dispense.c 내부 static) */
typedef struct {
    DispensePhase   phase;
    uint8_t         target_x;
    uint8_t         target_y;
    DispenseResult  result;
    uint32_t        window_start_ms;    /**< 3차 감지창 시작 시각 */
    uint32_t        settle_tick_ms;     /**< [추가] SERVO_SETTLE_MS 기준 */
} DispenseCtx;

/* ---- Team A 소유: 시스템 전역 컨텍스트 ---------------------------------- */

/**
 * 시스템 전역 컨텍스트.
 * cached_result 는 Team B 가 정의한 DispenseResult 를 값으로만 사용한다.
 */
typedef struct {
    SystemState     state;
    uint8_t         cur_x;
    uint8_t         cur_y;

    uint32_t        active_req_id;      /**< 현재 사이클의 MOVE / DISPENSE 요청번호 */
    uint8_t         pending_x;
    uint8_t         pending_y;

    LastCmdRecord   last_move;
    LastCmdRecord   last_dispense;

    DispenseResult  cached_result;      /**< ACK 수신 후에도 유지 (§6-4) */
    uint32_t        result_req_id;
    uint8_t         result_retx_count;
    uint32_t        result_retx_tick_ms;

    bool            boot_suppress_error;    /**< 부팅 직후 ERROR 억제 */
} SystemCtx;

#endif /* TYPES_H */