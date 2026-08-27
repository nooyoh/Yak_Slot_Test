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

/** 시스템 상태 (설계서 §2, flat 6-state) */
typedef enum {
	STATE_IDLE = 0,
	STATE_MOVING,
	STATE_AWAITING_DISPENSE,
	STATE_DISPENSING,
	STATE_AWAITING_RESULT_ACK,
	STATE_RECOVERY_REQUIRED
} SystemState;


/** X/Y축 식별 */
typedef enum {
	AXIS_X = 0,
	AXIS_Y,
	AXIS_COUNT
} AxisId;

/** 스테퍼 이동 방향 */
typedef enum {
	STEP_DIR_MINUS = -1,
	STEP_DIR_NONE  = 0,
	STEP_DIR_PLUS  = 1
} StepDir;


/** 배출 결과 */
typedef enum {
	DISPENSE_RESULT_NONE    = 0,
	DISPENSE_RESULT_SUCCESS = 1,
	DISPENSE_RESULT_EMPTY   = 2
} DispenseResult;


/** 배출 시퀀스 내부 상태 */
typedef enum {
	DPHASE_IDLE = 0,

	DPHASE_MOVE_MINUS,
	DPHASE_PUSH_MINUS,

	DPHASE_MOVE_CENTER,
	DPHASE_PUSH_CENTER,

	DPHASE_MOVE_PLUS,
	DPHASE_PUSH_PLUS,

	DPHASE_DETECT_WINDOW,
	DPHASE_DONE
} DispensePhase;


/** 스테퍼 상태 */
typedef enum {
	STEPPER_IDLE = 0,
	STEPPER_RUNNING
} StepperStatus;


/** 서보 상태 */
typedef enum {
	SERVO_IDLE = 0,
	SERVO_RAMPING,
	SERVO_SETTLING
} ServoStatus;


/** 홈 탐색 상태 */
typedef enum {
	HOMING_IDLE = 0,
	HOMING_SEARCHING,
	HOMING_MANUAL_WAIT,
	HOMING_DONE
} HomingStatus;

/* ==========================================================================
 * 2. struct
 * ========================================================================== */

/** X/Y 스테퍼 축 상태 */
typedef struct {
	int16_t        cur_steps;
	int16_t        target_steps;
	uint8_t        seq_index;
	StepDir        dir;
	StepperStatus  status;
	uint32_t       last_step_ms;
} StepperAxis;

/** 서보 상태 (servo.c 내부 static) */
typedef struct {
	uint8_t       cur_angle;
	uint8_t       target_angle;
	ServoStatus   status;
	uint32_t      last_tick_ms;
} ServoCtx;

/** 센서 상태 */
typedef struct {
	volatile bool ir_latched;
	volatile bool home_x_latched;
	volatile bool home_y_latched;
	uint32_t ir_last_edge_ms;
} SensorCtx;


/** 홈 탐색 상태 */
typedef struct {
	HomingStatus status;
	bool x_done;
	bool y_done;
	uint16_t x_step_count;
	uint16_t y_step_count;
} HomingCtx;


/** 3단계 배출 시퀀스 상태 */
typedef struct {
	DispensePhase  phase;
	uint8_t        target_x;
	uint8_t        target_y;
	DispenseResult result;
	uint32_t       window_start_ms;
} DispenseCtx;


#endif /* TYPES_H */