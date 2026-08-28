# [NEW] ATmega128A 펌웨어 식별자

> 기반: `약속(藥SLOT) ATmega128A 상위설계` (v2)
이름은 전부 **후보**이며, 팀에서 확정한 이름으로 치환 예정.
표기 규칙: 상수 `UPPER_SNAKE`, 타입 `PascalCase`, 함수 `Module_PascalCase`, 전역변수 `g_snake_case`, 정적변수 `s_snake_case`.
값 옆의 `★` 는 설계서에 값이 없거나 실측이 필요해 이 문서에서 제안한 항목.
> 

---

## 0. 파일 / 모듈 구성

| 파일 | 역할 |
| --- | --- |
| `main.c` | 부팅 시퀀스, 슈퍼루프 |
| `config.h` | 핀맵, 물리 상수, 튜닝 값 (실측 조정 대상 집중) |
| `types.h` | 공용 enum / struct |
| `systick.c/h` | Timer0 1ms 틱, 경과시간 헬퍼 |
| `uart.c/h` | USART0 드라이버 + RX/TX 링버퍼 |
| `protocol.c/h` | 프레임 추출·파싱 / 응답 프레임 생성·송신 |
| `fsm.c/h` | 6상태 상태머신, 중복·충돌 판정, RESULT 재전송 타이머 |
| `dispense.c/h` | 1~3차 압출 시퀀스 컨트롤러 (스테퍼+서보+IR 인터리브) |
| `stepper.c/h` | X/Y 스테퍼 논블로킹 구동 |
| `servo.c/h` | Timer1 PWM, 각도 점프/램프 |
| `sensors.c/h` | IR ×2 (SEN0503) + 홈 센서 ×2 (SG255) 통합 드라이버 |
| `homing.c/h` | 원점 탐색 시퀀스 (자동 / 수동 복구) |

12개 모듈입니다

**`sensors.c` 를 IR / 홈으로 쪼개지 않는 이유**

이번 핀맵에서 홈X = INT0(PD0), 홈Y = INT1(PD1), IR1 = INT2(PD2) 가 **`EICRA` 한 레지스터**를 공유합니다. 게다가 같은 `PORTD` 안에서 풀업 정책이 서로 반대입니다 — 홈 센서는 외부 풀업(4.7k)이라 내부 풀업을 **꺼야** 하고, IR1은 NPN 오픈 컬렉터라 내부 풀업을 **켜야** 합니다. 모듈을 나누면 각자 `EICRA = ...` / `PORTD = ...` 로 통째 대입하다가 서로의 설정을 지웁니다. (IR2는 `EICRB`/INT4 라 별개지만 같은 파일에 둡니다)

### 타이머 자원 배정 (설계서 미명시 → 제안)

| 타이머 | 용도 |
| --- | --- |
| Timer0 | 1ms 시스템 틱 (CTC, prescaler 64, `OCR0 = 249`) ★ — 스테퍼 3ms, 서보 500ms, IR 5s, 재전송 10s 를 전부 여기서 파생 |
| Timer1 | 서보 PWM (Fast PWM Mode 14, `ICR1 = 39999`, prescaler 8) — **설계서 §5 확정** |
| Timer2 | 예비 (미사용) |

---

## 1. `config.h`

### 1.1 시스템

```c
#define F_CPU               16000000UL  /* 시스템 클럭 16MHz. UBRR·PWM 계산의 기준 */
#define SYSTICK_HZ          1000U       /* 소프트 타이머 해상도 = 1ms */
#define SYSTICK_OCR0        249U        /* ★ Timer0 CTC 비교값 (16MHz/64/250 = 1kHz) */
```

### 1.2 슬롯 그리드 / 좌표 검증

```c
#define SLOT_X_COUNT        2                       /* X 슬롯 개수 (행) */
#define SLOT_Y_COUNT        5                       /* Y 슬롯 개수 (열) */
#define COORD_X_MIN         0                       /* MOVE/DISPENSE 의 X 하한 */
#define COORD_X_MAX         (SLOT_X_COUNT - 1)      /* X 상한. 벗어나면 INVALID_COORD */
#define COORD_Y_MIN         0                       /* Y 하한 */
#define COORD_Y_MAX         (SLOT_Y_COUNT - 1)      /* Y 상한 */

#define ALLOW_TIME_MIN_SEC  1UL       /* ★ TTTTTT 하한. 설계서 미명시 — §6-9 */
#define ALLOW_TIME_MAX_SEC  999999UL  /* TTTTTT 6자리 상한. 벗어나면 INVALID_TIME */
```

### 1.3 UART / 프로토콜

```c
#define UART_BAUD           9600UL        /* 설계서 §3.1 전송 속도 */
#define UART_UBRR_VALUE     ((F_CPU / (16UL * UART_BAUD)) - 1UL)  /* UBRR0 레지스터 값 */

#define UART_RX_RING_SIZE   128U          /* ★ 수신 링버퍼 크기. 2의 거듭제곱(마스킹용) */
#define UART_TX_RING_SIZE   128U          /* ★ 송신 링버퍼 크기 */

#define FRAME_MAX_LEN       64U           /* 한 프레임 최대 길이. 초과 시 INVALID_FORMAT */
#define FRAME_TERM_LF       '\n'          /* 프레임 종료 문자 (0x0A) */
#define FRAME_TERM_CR       '\r'          /* 수신 시 발견하면 버림 */
#define FRAME_DELIM         '|'           /* 필드 구분자 */
#define FRAME_MAX_FIELDS    5U            /* 최다 필드 = MOVE (5개) */
#define TX_FRAME_BUF_LEN    40U           /* ★ 송신 조립 버퍼. 최장 응답 RESULT|8자리|XYR\n */

#define REQ_ID_HEX_LEN      8U            /* 요청번호 자릿수 (대문자 16진 8자리) */
#define REQ_ID_NONE         0x00000000UL  /* Pi 가 쓰지 않는 값 = "요청번호 없음" 표식 */
#define REQ_ID_MIN          0x00000001UL  /* 유효 요청번호 하한 */
#define REQ_ID_MAX          0xFFFFFFFFUL  /* 유효 요청번호 상한 */
```

명령 / 응답 토큰 문자열:

```c
#define TOK_MOVE            "MOVE"        /* Pi→AT: 슬롯 이동 지시 */
#define TOK_DISPENSE        "DISPENSE"    /* Pi→AT: 배출 지시 */
#define TOK_ACK             "ACK"         /* 양방향: 수신 확인 */
#define TOK_TIMEOUT         "TIMEOUT"     /* Pi→AT: DISPENSE 없이 사이클 종료 */

#define TOK_WAIT            "WAIT"        /* AT→Pi: 목표 도착, 배출 대기 */
#define TOK_RESULT          "RESULT"      /* AT→Pi: 배출 결과 (XYR) */
#define TOK_ERROR           "ERROR"       /* AT→Pi: 오류 코드 반환 */
```

RESULT 재전송 (설계서 §2.2 `AWAITING_RESULT_ACK`):

```c
#define RESULT_RETX_INTERVAL_MS  10000UL  /* ACK 미수신 시 RESULT 재전송 주기 */
#define RESULT_RETX_MAX_COUNT    6U       /* 최대 6회(누적 60초). 초과 시 무송신 IDLE 강제 전이 */
```

### 1.4 스테퍼 (설계서 §4)

```c
#define STEPPER_PORT         PORTA   /* 8핀 전부 스테퍼 전용. X/Y 니블 분할 */
#define STEPPER_DDR          DDRA    /* 전 비트 출력 설정용 */
#define STEPPER_X_SHIFT      0U      /* X축 = PA0~PA3 (하위 니블) */
#define STEPPER_Y_SHIFT      4U      /* Y축 = PA4~PA7 (상위 니블) */
#define STEPPER_NIBBLE_MASK  0x0FU   /* 한 축 갱신 시 반대 축 비트 보존용 마스크 */

#define STEP_SEQ_LEN         4U      /* Full Drive(2상 여자) 시퀀스 길이 */
#define STEP_INTERVAL_MS     3U      /* 스텝 간 딜레이 = 속도 제어. 짧으면 탈조 */

#define X_MIN_STEPS          0       /* X축 원점(슬롯 0)의 절대 스텝 */
#define X_MAX_STEPS          1420    /* X축 최대 행정. 클램프 상한 */
#define Y_MIN_STEPS          0       /* Y축 원점(슬롯 0)의 절대 스텝 */
#define DISPENSE_SUB_OFFSET_STEPS  60  /* ★ 3회 압출용 미세 이동량(=a). 실측 후 조정 */
#define Y_MAX_STEPS          (4600 + DISPENSE_SUB_OFFSET_STEPS)  /* 3차 (y+a) 까지 포함한 상한 */

#define DISPENSE_OFFSET_AXIS     AXIS_Y  /* 미세 이동을 거는 축. 설계서 §5 = Y축 */
#define STEPPER_RELEASE_ON_IDLE  0       /* ★ 1=정지 시 코일 해제, 0=유지토크 — §6-6 */
```

좌표 → 절대 스텝 변환 테이블은 `stepper.c` 내부 `static const` 로 두고, **스텝 단위 값이 모듈 밖으로 나가지 않게** 합니다. FSM 은 슬롯 인덱스만 다룹니다.

```c
/* stepper.c */
/* Full Drive 4-step 시퀀스. IN1,IN2,IN3,IN4 순서로 니블에 그대로 대입 */
static const uint8_t s_full_drive_seq[STEP_SEQ_LEN] = { 0b1100, 0b0110, 0b0011, 0b1001 };

/* 슬롯 인덱스 → 원점 기준 절대 스텝 위치 */
static const int16_t s_x_step_table[SLOT_X_COUNT] = { 0, 1420 };
static const int16_t s_y_step_table[SLOT_Y_COUNT] = { 0, 1200, 2300, 3440, 4600 };
```

### 1.5 홈 탐색 (설계서 §7)

```c
#define HOMING_STEP_INTERVAL_MS  5U      /* ★ 탐색 속도. 주행(3ms)보다 느리게 = 놓침 방지 */
#define HOMING_MAX_STEPS_X       1800U   /* ★ X 탐색 스텝 예산. 초과 시 수동 복구 전환 */
#define HOMING_MAX_STEPS_Y       6000U   /* ★ Y 탐색 스텝 예산 */
#define HOME_LEVEL_POLL_MS       10U     /* ★ 핀 레벨 직접 판독 주기(엣지 못 잡는 경우 보완) */

#define HOME_DIR_X               STEP_DIR_MINUS  /* X 홈 방향 = 스텝 인덱스 감소 */
#define HOME_DIR_Y               STEP_DIR_MINUS  /* Y 홈 방향 = 스텝 인덱스 감소 */
#define HOME_TO_ORIGIN_OFFSET_X  0               /* 센서 감지점과 (0,0) 의 보정 스텝 */
#define HOME_TO_ORIGIN_OFFSET_Y  0
```

`HOMING_MAX_STEPS` 를 축별로 나눈 이유: X 최대 행정이 1420 인데 공용 6000 을 쓰면 X 센서 고장 시 정상 행정의 4배를 더 돌고 나서야 실패로 판정합니다(5ms/step 기준 30초 낭비 + 기구 충돌 위험).

### 1.6 서보 (설계서 §5)

```c
#define SERVO_PWM_TOP            39999U  /* ICR1. 50Hz(20ms) @ prescaler 8 */
#define SERVO_PULSE_MIN_US       500U    /* 0도 펄스폭. 실측 캘리브레이션 대상 */
#define SERVO_PULSE_MAX_US       2500U   /* 180도 펄스폭. 실측 캘리브레이션 대상 */
#define SERVO_ANGLE_MIN          0U      /* 각도 하한 (=하강 종료점) */
#define SERVO_ANGLE_MAX          180U    /* 각도 상한 */

#define SERVO_ANGLE_IDLE         180U    /* 대기 각도. DISPENSE 수신 전까지 유지 */
#define SERVO_ANGLE_PUSH_START   50U     /* 배출 시작 각도. 180→50 은 즉시 점프 */
#define SERVO_ANGLE_STEP_DEG     5U      /* 램프 1스텝당 감소 각도 */
#define SERVO_STEP_INTERVAL_MS   500U    /* 램프 주기. 50→0도 총 5.0초 */
#define SERVO_SETTLE_MS          150U    /* ★ 점프 후 물리 안정 대기. 없으면 "복귀 완료" 판정 불가 — §6-4 */
```

### 1.7 IR 센서 (설계서 §6)

```c
#define IR1_PORT      PORTD   /* 내부 풀업 설정용 */
#define IR1_PIN_REG   PIND    /* 레벨 판독용 */
#define IR1_BIT       PD2     /* INT2 */
#define IR2_PORT      PORTE
#define IR2_PIN_REG   PINE
#define IR2_BIT       PE4     /* INT4 (EICRB 관할) */

#define IR_ACTIVE_LEVEL   0U  /* 낙하 감지 = LOW (NPN 오픈 컬렉터) */
#define IR_DEBOUNCE_MS    5U  /* ★ 서보 진동 오검출 차단. 실측 검증 필요 — §6-5 */

#define DISPENSE_DETECT_WINDOW_MS  5000UL  /* 3차 0도 도달 후 감지 대기창 */
```

### 1.8 홈 센서 (설계서 §7)

```c
#define HOME_X_PIN_REG  PIND
#define HOME_X_BIT      PD0   /* INT0 */
#define HOME_Y_PIN_REG  PIND
#define HOME_Y_BIT      PD1   /* INT1 */

#define HOME_ACTIVE_LEVEL     1U  /* 빔 차단(감지) = HIGH → Rising Edge 트리거 */
#define HOME_INTERNAL_PULLUP  0U  /* 외부 풀업 4.7k 사용 → 내부 풀업 반드시 OFF */
```

---

## 2. `types.h`

### 2.1 enum

```c
/* 설계서 §2 의 6상태. 한 번에 하나의 요청만 처리하는 flat 구조 */
typedef enum {
    STATE_IDLE = 0,             /* 명령 대기. MOVE 만 받아들임 */
    STATE_MOVING,               /* 스테퍼 주행 중. 도착하면 WAIT 송신 */
    STATE_AWAITING_DISPENSE,    /* 목표 도착. DISPENSE 또는 TIMEOUT 대기 */
    STATE_DISPENSING,           /* 1~3차 압출 시퀀스 수행 중 */
    STATE_AWAITING_RESULT_ACK,  /* RESULT 송신 후 Pi 의 ACK 대기 */
    STATE_RECOVERY_REQUIRED     /* 물리 위치 미확정. 홈 탐색 완료 전까지 모든 명령 거부 */
} SystemState;
```

```c
typedef enum { AXIS_X = 0, AXIS_Y, AXIS_COUNT } AxisId;  /* 축 식별자 겸 배열 인덱스 */

/* 스텝 진행 방향. 값 자체를 cur_steps 증감에 그대로 더함 */
typedef enum { STEP_DIR_MINUS = -1, STEP_DIR_NONE = 0, STEP_DIR_PLUS = 1 } StepDir;
```

```c
/* 파싱된 명령 종류 */
typedef enum {
    CMD_NONE = 0,   /* 파싱 전 초기값 */
    CMD_MOVE,       /* ID_CONFLICT 판정 대상 */
    CMD_DISPENSE,   /* ID_CONFLICT 판정 대상 */
    CMD_ACK,        /* 판정 제외 (선행 DISPENSE 의 요청번호 재사용) */
    CMD_TIMEOUT,    /* 판정 제외 (선행 MOVE 의 요청번호 재사용) */
    CMD_UNKNOWN     /* 토큰 불일치 → INVALID_FORMAT */
} CmdType;
```

```c
/* ERROR 프레임의 [CODE] 필드. Protocol_ErrorText() 로 문자열화 */
typedef enum {
    ERR_INVALID_FORMAT = 0,  /* 필드 수·문자·종료 이상. 요청번호 없이 2필드로 송신 */
    ERR_INVALID_COORD,       /* X/Y 범위 초과 */
    ERR_INVALID_TIME,        /* TTTTTT 범위 초과 */
    ERR_ID_CONFLICT,         /* 같은 요청번호에 다른 페이로드 */
    ERR_BUSY,                /* 다른 요청 수행 중 */
    ERR_NOT_READY,           /* WAIT 이전에 DISPENSE 도착 */
    ERR_COORD_MISMATCH,      /* DISPENSE 좌표 ≠ 직전 MOVE 좌표 */
    ERR_RECOVERY_REQUIRED,   /* 홈 탐색 미완료 상태에서 명령 수신 */
    ERR_STEPPER_ERROR,       /* 이동 실패를 명확히 판정 가능할 때만 — §6-11 */
    ERR_SERVO_ERROR,         /* 서보 실패 검출 가능할 때만 */
    ERR_SENSOR_ERROR         /* 센서 고장과 미감지를 구분 가능할 때만 */
} ErrorCode;
```

```c
/* RESULT 프레임의 R 필드 값 */
typedef enum {
    DISPENSE_RESULT_NONE    = 0,  /* 아직 결과 없음 */
    DISPENSE_RESULT_SUCCESS = 1,  /* R=1: 1~3차 중 IR 감지 성공 */
    DISPENSE_RESULT_EMPTY   = 2   /* R=2: 3차까지 미감지 = 슬롯 소진(추정) */
} DispenseResult;
```

압출 3-phase 내부 서브상태 (`dispense.c` 전용, 밖으로는 `bool` 만 노출):

```c
typedef enum {
    DPHASE_IDLE = 0,
    DPHASE_MOVE_MINUS,     /* 1차: (y − a) 로 스테퍼 이동 */
    DPHASE_PUSH_MINUS,     /* 1차: 서보 하강 → 0도 → 180도 복귀 */
    DPHASE_MOVE_CENTER,    /* 2차: (y) 로 이동 */
    DPHASE_PUSH_CENTER,    /* 2차: 서보 사이클 */
    DPHASE_MOVE_PLUS,      /* 3차: (y + a) 로 이동 */
    DPHASE_PUSH_PLUS,      /* 3차: 서보 사이클 */
    DPHASE_DETECT_WINDOW,  /* 3차 0도 도달 후 5초 감지 대기 */
    DPHASE_DONE            /* 결과 확정. FSM 이 회수 */
} DispensePhase;
```

액추에이터 상태 (각 모듈 내부 `static`, 공개 인터페이스는 `*_IsBusy()` / `*_Status()`):

```c
typedef enum { STEPPER_IDLE = 0, STEPPER_RUNNING } StepperStatus;  /* 목표 도달 여부 */

typedef enum {
    SERVO_IDLE = 0,   /* 목표 각도 도달 및 안정 완료 */
    SERVO_RAMPING,    /* 500ms/5도 하강 진행 중 */
    SERVO_SETTLING    /* 즉시 점프 후 물리 안정 대기 중 */
} ServoStatus;

typedef enum {
    HOMING_IDLE = 0,
    HOMING_SEARCHING,    /* 자동 복구: 스텝 예산 내 탐색 중 */
    HOMING_MANUAL_WAIT,  /* 수동 복구: 코일 해제, 사람이 밀어주기 대기 */
    HOMING_DONE          /* 두 축 모두 감지 → (0,0) 확정 */
} HomingStatus;
```

### 2.2 struct

```c
/* 수신 라인 1개를 파싱한 결과. 사용 후 버리는 일회성 구조체 */
typedef struct {
    CmdType   cmd;              /* 명령 종류 */
    uint32_t  req_id;           /* 요청번호. INVALID_FORMAT 이면 무의미 */
    uint8_t   x;                /* 슬롯 X (MOVE/DISPENSE 만) */
    uint8_t   y;                /* 슬롯 Y */
    uint32_t  allow_time_sec;   /* MOVE 의 TTTTTT. 검증·기록 전용, 타이머로 쓰지 않음 */
    char      ack_target[12];   /* ACK 의 3번째 필드 ("RESULT"/"TIMEOUT" 등) */
    bool      valid;            /* 파싱 성공 여부 */
} Frame;
```

```c
/* 중복·ID 충돌 판정용 캐시. MOVE 용/DISPENSE 용 각 1개 (설계서 §3.2.1) */
typedef struct {
    uint32_t  req_id;          /* 직전 수신한 같은 종류 명령의 요청번호 */
    uint8_t   x;               /* 그때의 페이로드 — 같은 번호에 다르면 ID_CONFLICT */
    uint8_t   y;
    uint32_t  allow_time_sec;
    bool      seen;            /* 부팅 후 한 번이라도 받은 적 있는지 */
} LastCmdRecord;
```

```c
/* 축 하나의 주행 상태. stepper.c 내부에 X/Y 두 개 */
typedef struct {
    int16_t        cur_steps;      /* 현재 절대 스텝 (명령 카운트 추정치) */
    int16_t        target_steps;   /* 목표 절대 스텝 */
    uint8_t        seq_index;      /* s_full_drive_seq 의 현재 위치 (0~3) */
    StepDir        dir;            /* 진행 방향 */
    StepperStatus  status;         /* 주행 중 / 정지 */
    uint32_t       last_step_ms;   /* 마지막 스텝 시각. 3ms 간격 판정용 */
} StepperAxis;
```

```c
typedef struct {
    uint8_t      cur_angle;     /* 현재 출력 중인 각도 */
    uint8_t      target_angle;  /* 램프의 최종 목표 각도 */
    ServoStatus  status;        /* 램프/안정/완료 */
    uint32_t     last_tick_ms;  /* 마지막 각도 갱신 시각. 500ms 판정용 */
} ServoCtx;
```

```c
/* 센서 래치. ISR 이 쓰고 메인 루프가 읽으므로 volatile 필수 */
typedef struct {
    volatile bool  ir_latched;       /* IR1 OR IR2 감지 발생. DISPENSING 진입 시 clear */
    volatile bool  home_x_latched;   /* X 홈 Rising Edge 발생 */
    volatile bool  home_y_latched;   /* Y 홈 Rising Edge 발생 */
    uint32_t       ir_last_edge_ms;  /* 디바운스 기준 시각 */
} SensorCtx;
```

```c
typedef struct {
    HomingStatus  status;         /* 탐색 / 수동대기 / 완료 */
    bool          x_done;         /* X 감지 완료 → 코일 OFF, Y 만 계속 진행 */
    bool          y_done;
    uint16_t      x_step_count;   /* 소모 스텝. 예산 초과 판정용 */
    uint16_t      y_step_count;
} HomingCtx;
```

```c
typedef struct {
    DispensePhase   phase;            /* 현재 차수/단계 */
    uint8_t         target_x;         /* 배출 대상 슬롯 */
    uint8_t         target_y;         /* 오프셋 계산의 기준 (=y) */
    DispenseResult  result;           /* 확정된 R 값 */
    uint32_t        window_start_ms;  /* 5초 감지창 시작 시각 */
} DispenseCtx;
```

```c
/* 시스템 전역 컨텍스트. FSM 이 소유 */
typedef struct {
    SystemState     state;                /* 현재 상태 */
    uint8_t         cur_x;                /* 현재 확정 슬롯 좌표 */
    uint8_t         cur_y;

    uint32_t        active_req_id;        /* 처리 중인 사이클의 요청번호 */
    uint8_t         pending_x;            /* MOVE 로 받은 목표 좌표 (DISPENSE 대조용) */
    uint8_t         pending_y;

    LastCmdRecord   last_move;            /* MOVE 중복/충돌 판정 캐시 */
    LastCmdRecord   last_dispense;        /* DISPENSE 중복/충돌 판정 캐시 */

    DispenseResult  cached_result;        /* 같은 DISPENSE 재수신 시 재송신할 결과 */
    uint32_t        result_req_id;        /* 그 결과의 요청번호 */
    uint8_t         result_retx_count;    /* RESULT 재전송 횟수 (최대 6) */
    uint32_t        result_retx_tick_ms;  /* 마지막 재전송 시각 */

    bool            boot_suppress_error;  /* 부팅 직후 쓰레기 프레임에 ERROR 응답 억제 */
} SystemCtx;
```

---

## 3. 전역 변수

`extern` 로 공개하는 것은 아래 2개뿐입니다. 나머지 컨텍스트(`StepperAxis`, `ServoCtx`, `SensorCtx`, `HomingCtx`, `DispenseCtx`)는 각 `.c` 내부 `static` 으로 숨기고 함수로만 접근합니다.

```c
extern volatile uint32_t  g_tick_ms;  /* systick.c. Timer0 ISR 이 1ms마다 증가. 모든 타이밍의 기준 */
extern SystemCtx          g_ctx;      /* fsm.c. 상태·좌표·요청번호·재전송 카운터 전부 여기 */
```

`g_tick_ms` 는 32비트라 8비트 MCU 에서 비원자적으로 읽힙니다. 직접 읽지 말고 반드시 `Systick_Now()` 를 거치세요.

---

## 4. 모듈별 함수

### 4.1 `systick`

```c
void      Systick_Init(void);                             /* Timer0 CTC 1ms 설정 + 인터럽트 허용 */
uint32_t  Systick_Now(void);                              /* g_tick_ms 를 인터럽트 차단 후 원자적 복사 */
bool      Systick_Elapsed(uint32_t since, uint32_t ms);   /* since 로부터 ms 경과했는지. 오버플로 안전 */
```

### 4.2 `uart`

```c
void  Uart_Init(void);                        /* UBRR/RXEN/TXEN/RXCIE 설정, 링버퍼 초기화 */
bool  Uart_ReadLine(char *dst, uint8_t cap);  /* LF 로 끝난 완성 라인 1개 추출. 없으면 false */
void  Uart_WriteLine(const char *src);        /* TX 링버퍼에 넣고 LF 부착. 논블로킹 */
bool  Uart_TxBusy(void);                      /* 송신 잔여분 존재 여부 */
```

### 4.3 `protocol`

```c
bool     Protocol_Parse(const char *line, Frame *out);       /* 문자열 → Frame. 형식 검증 포함 */
uint8_t  Protocol_ParseHex32(const char *s, uint32_t *out);  /* 대문자 16진 8자리 → uint32 */

void  Protocol_SendAck(uint32_t req_id, const char *target); /* ACK|req|MOVE 등 */
void  Protocol_SendWait(uint32_t req_id);                    /* WAIT|req */
void  Protocol_SendResult(uint32_t req_id, uint8_t x, uint8_t y, DispenseResult r);
                                                             /* RESULT|req|XYR */
void  Protocol_SendError(uint32_t req_id, ErrorCode code);   /* ERROR|req|CODE (3필드) */
void  Protocol_SendInvalidFormat(void);                      /* ERROR|INVALID_FORMAT (2필드) */

const char *Protocol_ErrorText(ErrorCode code);              /* ErrorCode → 문자열 */
```

### 4.4 `fsm`

```c
void         Fsm_Init(void);                    /* g_ctx 초기화 */
void         Fsm_Task(void);                    /* 매 루프 호출. 도착/완료 감지 + RESULT 재전송 */
void         Fsm_HandleFrame(const Frame *f);   /* 파싱된 명령 1개 처리. 판정→상태별 분기 */

SystemState  Fsm_State(void);                   /* 현재 상태 조회 */
void         Fsm_SetState(SystemState next);    /* 상태 전이 (단일 진입점) */
```

`fsm.c` 내부 (`static`):

```c
static bool  Fsm_CheckIdConflict(const Frame *f);  /* 같은 번호+다른 페이로드 → true */
static bool  Fsm_IsDuplicate(const Frame *f);      /* 번호·페이로드 완전 일치 → 재전송 취급 */
static void  Fsm_HandleMove(const Frame *f);       /* 상태별 MOVE 처리 (ACK/BUSY/무시) */
static void  Fsm_HandleDispense(const Frame *f);   /* 좌표 대조 후 DISPENSING 진입 */
static void  Fsm_HandleTimeout(const Frame *f);    /* AWAITING_DISPENSE·IDLE 에서 ACK 회신 */
static void  Fsm_HandleAck(const Frame *f);        /* AWAITING_RESULT_ACK 에서만 유효 */
static void  Fsm_ServiceResultRetx(void);          /* 10초 주기 RESULT 재전송, 6회 초과 시 IDLE */
```

`Fsm_SetState()` 를 유일한 전이 경로로 두는 이유: 나중에 로깅이나 저장을 다시 붙일 때 한 곳만 고치면 됩니다.

### 4.5 `dispense`

```c
void            Dispense_Start(uint8_t x, uint8_t y);  /* 3-phase 시퀀스 시작. IR 래치 clear 포함 */
void            Dispense_Task(void);                   /* phase 진행. 1회 1ms 이내 반환 */
bool            Dispense_IsBusy(void);                 /* 진행 중 여부 */
bool            Dispense_IsComplete(void);             /* DPHASE_DONE 도달 여부 */
DispenseResult  Dispense_Result(void);                 /* 확정된 R 값 회수 */
void            Dispense_Abort(void);                  /* 즉시 서보 180도 복귀 + 시퀀스 종료 */
```

### 4.6 `stepper`

```c
void     Stepper_Init(void);                                 /* DDRA 출력 설정, 컨텍스트 초기화 */
void     Stepper_Task(void);                                 /* 3ms마다 1스텝. 목표 도달 시 정지 */
bool     Stepper_MoveToSlot(uint8_t x, uint8_t y);           /* 슬롯 인덱스로 목표 지정 */
bool     Stepper_MoveToSteps(AxisId axis, int16_t target);   /* 절대 스텝 지정 (압출 오프셋용) */
void     Stepper_StepOnce(AxisId axis, StepDir dir);         /* 1스텝만. homing 전용 */
void     Stepper_Stop(AxisId axis);                          /* 해당 축 정지 (코일 상태는 유지) */
void     Stepper_ReleaseAll(void);                           /* PORTA 전체 LOW. 손으로 밀 수 있게 */
bool     Stepper_IsBusy(void);                               /* 두 축 중 하나라도 주행 중 */
int16_t  Stepper_CurSteps(AxisId axis);                      /* 현재 절대 스텝 조회 */
void     Stepper_SetOrigin(void);                            /* 홈 감지 시 cur_steps 를 0 으로 확정 */
int16_t  Stepper_SlotToSteps(AxisId axis, uint8_t idx);      /* 테이블 조회 */
int16_t  Stepper_ClampSteps(AxisId axis, int16_t v);         /* MIN/MAX 범위로 잘라냄 */
```

### 4.7 `servo`

```c
void         Servo_Init(void);                 /* Timer1 Fast PWM Mode14 설정, 180도 출력 */
void         Servo_Task(void);                 /* 램프 진행 + settle 타이머 관리 */
void         Servo_SetAngleNow(uint8_t deg);   /* OCR1A 즉시 갱신 (점프) */
void         Servo_StartPushRamp(void);        /* 50도 점프 후 5도/500ms 하강 시작 */
void         Servo_ReturnIdle(void);           /* 0 → 180도 즉시 점프 */
ServoStatus  Servo_Status(void);               /* 램프/안정/완료. phase 전환 판정에 사용 */
uint16_t     Servo_AngleToPulse(uint8_t deg);  /* 각도 → OCR1A 값 선형 변환 */
```

### 4.8 `sensors`

```c
void  Sensors_Init(void);                  /* EICRA/EICRB/EIMSK/풀업을 한 번에 설정 */
void  Sensors_Task(void);                  /* IR 디바운스 확정 + 홈 핀 레벨 주기 폴링 */

void  Sensors_IrClear(void);               /* 래치 초기화. DISPENSING 진입 시 호출 */
bool  Sensors_IrDetected(void);            /* IR1 OR IR2 감지 여부 (설계서 §6 OR 로직) */
void  Sensors_IrEnable(bool en);           /* EIMSK 의 INT2/INT4 비트만 개폐 */

bool  Sensors_HomeLevel(AxisId axis);      /* 핀 레벨 직접 판독. 이미 원점에 있을 때 필요 */
bool  Sensors_HomeLatched(AxisId axis);    /* Rising Edge 래치 확인 */
void  Sensors_HomeClear(AxisId axis);      /* 래치 해제 */
```

`Sensors_HomeLevel()` 이 따로 있는 이유: 이미 원점에 정지한 상태로 부팅하면 신호가 HIGH 로 고정되어 Rising Edge 가 아예 발생하지 않습니다. 엣지만 믿으면 그 경우 영원히 탐색합니다.

### 4.9 `homing`

```c
void          Homing_Start(void);        /* 두 축 동시 홈 방향 구동 시작 */
void          Homing_Task(void);         /* 스텝 예산 감시, 축별 도달 처리, 초과 시 수동 전환 */
HomingStatus  Homing_Status(void);       /* 탐색/수동대기/완료 조회 */
bool          Homing_IsComplete(void);   /* 두 축 완료 → FSM 이 IDLE 로 전이 */
```

### 4.10 `main`

```c
static void  Sys_Init(void);        /* 주변장치 초기화 → sei → RECOVERY_REQUIRED 진입 → 홈 탐색 */
static void  Task_UartRx(void);     /* 라인 추출 → 파싱 → Fsm_HandleFrame */
static void  Task_Actuator(void);   /* 상태에 맞는 액추에이터 tick 호출 */
static void  Task_Fsm(void);        /* 상태 전이 판정 + RESULT 재전송 타이머 */
```

---

## 5. `main.c` 스켈레톤

```c
int main(void)
{
    Sys_Init();

    for (;;) {
        Task_UartRx();      /* 수신 처리 */
        Task_Actuator();    /* 물리 동작 tick */
        Task_Fsm();         /* 상태 전이 판정 */
    }
}

static void Sys_Init(void)
{
    Systick_Init();
    Uart_Init();
    Stepper_Init();
    Servo_Init();
    Sensors_Init();
    Fsm_Init();

    sei();

    /* 설계서 §2.1: 전원 인가 시 무조건 RECOVERY_REQUIRED 진입 후 홈 탐색 */
    g_ctx.boot_suppress_error = true;
    Fsm_SetState(STATE_RECOVERY_REQUIRED);
    Homing_Start();
}

static void Task_Actuator(void)
{
    Sensors_Task();     /* 디바운스·폴링은 항상 돌려야 함 */
    Stepper_Task();
    Servo_Task();

    if (Fsm_State() == STATE_RECOVERY_REQUIRED) {
        Homing_Task();
    } else if (Fsm_State() == STATE_DISPENSING) {
        Dispense_Task();
    }
}
```

각 `*_Task()` 는 1회 호출당 **1ms 이내**에 반환해야 합니다. 블로킹 `_delay_ms()` 는 `Sys_Init()` 안에서만 허용합니다.

---