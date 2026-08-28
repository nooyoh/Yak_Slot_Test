# ATmega128A 팀 분업 가이드라인

> 기반: `ATmega128A_펌웨어_식별자_v3_noEEPROM.md`
Team A = UART/프로토콜/상태머신, Team B = 액추에이터/센서
이 문서의 목적은 "누가 어떤 코드를 언제 만져도 되는가" 를 미리 정해서, 마지막에 합칠 때 인터페이스가 어긋나지 않게 하는 것입니다.
> 

---

## 0. 분업 경계 — 왜 여기서 자르는가

`fsm.c` 는 UART 로 받은 명령을 해석해서 액추에이터 함수를 호출하는 쪽입니다. 즉 **FSM 은 호출자, 액추에이터 모듈은 피호출자**입니다. 이 방향이 분업 경계의 기준입니다.

```
[Team A 영역]                          [Team B 영역]
UART → Protocol_Parse → Frame
                          │
                          ▼
                    Fsm_HandleFrame()  ──호출──▶  Stepper_*, Servo_*,
                          │                        Sensors_*, Homing_*,
                          ▼                        Dispense_*
                    Protocol_Send*  ◀──조회──┐
                          │                   (IsBusy / Status / Result)
                          ▼
                        UART
```

FSM 이 액추에이터 내부 상태(`StepperAxis`, `ServoCtx` 등)를 직접 들여다보는 일은 없어야 합니다. FSM 이 아는 것은 **§3의 인터페이스 함수 목록뿐**입니다. 이 규칙 하나만 지키면 두 팀은 서로의 내부 구현을 몰라도 동시에 작업할 수 있습니다.

---

## 1. 담당 모듈

| 모듈 | 담당 | 비고 |
| --- | --- | --- |
| `~~systick.c/h~~` | **A** | Timer0. 양쪽 다 쓰지만 타이머 설정 자체는 UART 재전송 타이머(A)가 주 소비자 |
| `uart.c/h` | **A** |  |
| `~~protocol.c/h~~` | **A** |  |
| `fsm.c/h` | **A** | 액추에이터 함수를 **호출**만 함. 내부 구현은 모름 |
| `types.h` | **공동 소유 (§2 참고)** |  |
| `config.h` | **공동 소유, 섹션별 분할 (§2 참고)** |  |
| `stepper.c/h` | **B** |  |
| `servo.c/h` | **B** |  |
| `sensors.c/h` | **B** | IR + 홈 통합 (레지스터 공유 이유는 식별자 문서 §0 참고) |
| `homing.c/h` | **B** |  |
| `dispense.c/h` | **B** | FSM 은 `Dispense_Start/Task/IsBusy/IsComplete/Result` 만 호출 |
| `main.c` | **공동 작성** | `Sys_Init` 순서, `Task_*` 배치는 인터페이스가 확정된 뒤 마지막에 같이 씀 |

---

## 2. 공유 파일 — `types.h` / `config.h`

두 팀이 같은 파일을 동시에 고치면 충돌이 계속 납니다. 그래서 파일 단위가 아니라 **섹션 단위**로 소유권을 나눕니다.

### 2.1 `types.h`

| 섹션 | 소유 | 변경 규칙 |
| --- | --- | --- |
| `SystemState`, `CmdType`, `ErrorCode`, `Frame`, `LastCmdRecord`, `SystemCtx` | A | B 가 원하는 변경 있으면 A 에게 요청 |
| `AxisId`, `StepDir`, `DispenseResult`, `DispensePhase`, `StepperStatus`, `ServoStatus`, `HomingStatus`, `StepperAxis`, `ServoCtx`, `SensorCtx`, `HomingCtx`, `DispenseCtx` | B | A 가 원하는 변경 있으면 B 에게 요청 |

`SystemCtx` 안에 `DispenseResult cached_result` 처럼 A 구조체가 B 타입을 참조하는 지점이 있습니다. 이런 경우 **타입 정의 자체는 B 가 소유**하고, A 는 그 타입을 값으로만 씁니다 (내부 필드를 해석하지 않음).

### 2.2 `config.h`

식별자 문서의 §1.1~§1.8 을 그대로 소유권 경계로 씁니다.

| 섹션 | 소유 |
| --- | --- |
| 1.1 시스템, 1.2 슬롯 그리드, 1.3 UART/프로토콜 | A |
| 1.4 스테퍼, 1.5 홈 탐색, 1.6 서보, 1.7 IR 센서, 1.8 홈 센서 | B |

**공통 상수는 A 가 정의하고 B 가 읽습니다**: `SLOT_X_COUNT`, `SLOT_Y_COUNT`, `COORD_X_MIN/MAX`, `COORD_Y_MIN/MAX`. 좌표 범위는 프로토콜 검증(A)과 스텝 테이블 크기(B) 양쪽에서 쓰이지만, "이 시스템의 슬롯이 몇 개인가" 는 프로토콜 스펙에서 오는 값이라 A 소유로 둡니다.

### 2.3 병합 절차

1. 각자 `config.h` / `types.h` 에서 **자기 섹션만** 수정
2. PR 을 올릴 때 상대 섹션에 diff 가 없는지 스스로 확인
3. 상대 섹션을 고쳐야 하는 요청이 생기면 코드로 직접 고치지 말고 이슈로 남긴 뒤 소유 팀이 반영

---

## 3. 인터페이스 함수 목록 (얼려야 하는 부분)

이 표에 있는 시그니처는 **작업 시작 전에 먼저 합의하고, 이후에는 시그니처를 바꾸지 않는 것**을 원칙으로 합니다. 바꿔야 하면 양쪽에 미리 알립니다. 반환 타입/인자 순서가 하나라도 다르면 링크 단계에서 잡히지 않고 동작 오류로만 드러납니다.

### 3.1 A → B (FSM 이 호출)

```c
/* stepper */
bool     Stepper_MoveToSlot(uint8_t x, uint8_t y);   /* 목표 지정. 이미 이동 중이면 false */
bool     Stepper_IsBusy(void);
void     Stepper_ReleaseAll(void);                    /* RECOVERY 진입 시 A 가 호출 가능성 있음 */

/* homing */
void          Homing_Start(void);
HomingStatus  Homing_Status(void);
bool          Homing_IsComplete(void);

/* dispense */
void            Dispense_Start(uint8_t x, uint8_t y);
bool            Dispense_IsBusy(void);
bool            Dispense_IsComplete(void);
DispenseResult  Dispense_Result(void);
void            Dispense_Abort(void);
```

### 3.2 B → A (액추에이터가 FSM 상태를 참조)

원칙적으로 **없어야 합니다.** B 는 FSM 상태를 직접 읽지 않고, 필요한 값은 A 가 함수 인자로 넘겨줍니다 (`Stepper_MoveToSlot(x, y)` 처럼). 이 방향의 호출이 생기면 그 시점에 설계를 다시 봐야 한다는 신호입니다.

### 3.3 매 루프 호출 (양쪽 다 알아야 함)

```c
void  Stepper_Task(void);
void  Servo_Task(void);
void  Sensors_Task(void);
void  Homing_Task(void);      /* RECOVERY_REQUIRED 상태에서만 */
void  Dispense_Task(void);    /* DISPENSING 상태에서만 */
```

`main.c` 의 `Task_Actuator()` 안에서 어떤 상태일 때 무엇을 호출하는지는 식별자 문서 §5 스켈레톤에 이미 정해져 있습니다. B 는 이 스켈레톤을 전제로 각 `_Task()` 를 구현하면 됩니다.

---

## 4. 각자 컴파일 가능하게 만들기 — 스텁

인터페이스가 합의되면, 상대 모듈이 완성되기 전이라도 **더미 구현(스텁)** 을 넣어서 각자 빌드·테스트할 수 있습니다.

**A 가 만들 스텁** (`stub_actuator.c`, B 완성 전까지 링크용):

```c
bool Stepper_MoveToSlot(uint8_t x, uint8_t y) { (void)x; (void)y; return true; }
bool Stepper_IsBusy(void) { static uint8_t n = 0; return (++n % 20) != 0; }  /* 몇 틱 후 완료처럼 동작 */
void Homing_Start(void) {}
bool Homing_IsComplete(void) { static uint8_t n = 0; return ++n > 5; }
void Dispense_Start(uint8_t x, uint8_t y) { (void)x; (void)y; }
bool Dispense_IsBusy(void) { static uint8_t n = 0; return (++n % 10) != 0; }
DispenseResult Dispense_Result(void) { return DISPENSE_RESULT_SUCCESS; }
```

**B 가 만들 스텁** (`stub_protocol.c`, A 완성 전까지 하드웨어 단독 테스트용):

```c
/* UART 없이 하드코딩된 명령을 흘려보내서 stepper/servo/sensors 단독 검증 */
void Stub_InjectMove(uint8_t x, uint8_t y);
void Stub_InjectDispense(uint8_t x, uint8_t y);
```

스텁 파일은 최종 빌드(`main.c` 통합 시점)에서 **둘 다 제외**합니다. `Makefile`/빌드 스크립트에 `SRCS_STUB` 변수를 따로 두고, 단독 테스트 타깃과 통합 타깃을 분리하는 것을 권합니다.

---

## 5. 통합 순서 (스텁 → 실물 교체)

1. §3 인터페이스 표 합의 (착수 전, 1회)
2. 각자 스텁 기반으로 독립 개발/컴파일
3. A: UART 루프백(에코) 또는 시리얼 터미널로 `Frame` 파싱·`ERROR`/`ACK` 응답까지 단독 검증
4. B: `Stub_InjectMove/Dispense` 로 스테퍼·서보·센서 시퀀스를 하드웨어 위에서 단독 검증
5. 스텁 제거, 실물 함수로 링크 → `main.c` 공동 작성
6. 통합 후 §7 체크리스트로 회귀 확인

이 순서를 지키면 "두 사람이 같은 코드를 동시에 못 만진다" 는 문제 없이, 통합 시점에는 **인터페이스 불일치만** 확인하면 됩니다.

---