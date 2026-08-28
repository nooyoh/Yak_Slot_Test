# AT_INTEGRATION — UART(Team A) ↔ 액추에이터/센서(Team B) 통합 노트

작성일 2026-08-28 · 대상: `AT_UART/` + `AT_ACTUATOR/` → `AT_INTEGRATION/`
기준 문서: `AT_ACTUATOR/README_1.md`(상위설계) · `README_2.md`(식별자) · `README_3.md`(분업 가이드)

> **원칙**: 두 팀의 소스는 **B-1 한 줄을 제외하고** 수정하지 않았다. `main.c` 와
> `INTEGRATION.cproj` 는 이 폴더에서 새로 작성했다. 상위설계와 어긋나는 나머지
> 부분은 아래에 정리만 하고 고치지 않았다 — 각 항목의 조치 주체(Team A / Team B /
> 팀 합의)를 표기했다.
>
> **[2026-08-28 갱신] B-1(systick 프리스케일러) 은 사용자 지시로 통합본에서 수정 완료.**
> 원본 `AT_UART/systick.c` 는 그대로 두었고 `AT_INTEGRATION/systick.c:17` 한 줄만 정정.

---

## A. 통합 구조 / 병합 결정

| 파일 | 출처 | 비고 |
| --- | --- | --- |
| `config.h` | AT_UART ＝ AT_ACTUATOR | **두 폴더가 byte 단위로 동일.** 병합 충돌 없음 |
| `types.h` | AT_UART | Team A 판이 Team B 판의 **상위집합** (Team A 가 `CmdType`/`ErrorCode`/`Frame`/`LastCmdRecord`/`SystemCtx` 추가). 그대로 채택 |
| `systick.h` | AT_UART (Team A 소유) | 인터페이스 그대로 |
| `systick.c` | AT_UART (Team A 소유) + **B-1 1줄 수정** | `TCCR0` 프리스케일러 `/32`→`/64` 정정 완료 (2026-08-28, 사용자 지시). 원본 `AT_UART/systick.c` 는 유지 |
| `uart.c/h` `protocol.c/h` `fsm.c/h` | AT_UART | Team A 산출물 |
| `sensors.c/h` `stepper.c/h` `servo.c/h` `homing.c/h` `dispense.c/h` | AT_ACTUATOR | Team B 산출물 |
| `main.c` | **신규 작성** | README_2 §4.10 / §5 스켈레톤 + AT_ACTUATOR/main.c 의 상태 게이팅 방식 |
| `INTEGRATION.cproj` | 신규 (ACTUATOR_VS.cproj 기반) | 위 15개 소스 등록 |

`AT_UART/main.c`(빈 템플릿), `AT_ACTUATOR/main.c`(단독빌드용), `team_a_stub.c`,
임시 `uart.h`/`fsm.h` 는 통합에서 제외.

---

## B. 상위설계 불일치 / 결함 (임의 수정 안 함)

심각도: **BLOCKER**(동작 불가) · **MAJOR**(특정 시나리오 오동작) · **MINOR**(경미/방어적)

---

### B-1. [BLOCKER · ✅ 통합본에서 수정 완료] `systick.c` Timer0 프리스케일러가 /32 로 설정됨

원본 `AT_UART/systick.c:17` (수정 전):
```c
TCCR0 = (1U << WGM01) | (1U << CS01) | (1U << CS00); //CTC모드, 64분주 설정
```
- ATmega128 Timer0 `CS02:CS01:CS00` 인코딩: `011` = clk/**32**, `100` = clk/**64**.
- `CS01|CS00` = `011` = **clk/32**. 주석("64분주")·`README_2 §1.1`("16MHz/64/250 = 1kHz")과 모순.
- 방치 시: 틱이 1ms 아닌 **0.5ms** → `Systick_*` 의존 타이밍 전부 2배 빠름
  (RESULT 재전송 10s→5s, IR 감지창 5s→2.5s, 스텝 3ms→1.5ms 탈조 위험, 서보 램프 500ms→250ms).

**수정 (2026-08-28, 사용자 지시 — `AT_INTEGRATION/systick.c:17`):**
```c
TCCR0 = (1U << WGM01) | (1U << CS02); //CTC모드, 64분주(CS02:0:0=100) 설정 — [통합 B-1 수정]
```
- 원본 `AT_UART/systick.c` 는 손대지 않았다. Team A 가 upstream 에도 동일 수정을 반영해야 한다.
- `AT_ACTUATOR/systick.c` 는 원래부터 올바른 값(`CS02`)이었음 — 이번 수정으로 세 곳이 일치.

---

### B-2. [MAJOR · main 통합에서 처리함] `Fsm_Init()` 이 `boot_suppress_error` 를 세우지 않음

`AT_UART/fsm.c:26-30` — `Fsm_Init()` 은 `memset(&g_ctx, 0, …)` 만 하므로 `boot_suppress_error = false`.
- `AT_ACTUATOR/main.c:76-78` 주석: "이 플래그는 `Fsm_Init()`(Team A)이 자체적으로 세팅하는 것으로 가정".
  → **가정이 틀림.** Team A 는 안 한다.
- `README_2 §5` 스켈레톤은 `Sys_Init()` 에서 `g_ctx.boot_suppress_error = true;` 를 명시.
- **통합 조치**: `AT_INTEGRATION/main.c` 의 `Sys_Init()` 에서 `Fsm_Init()` 직후 `g_ctx.boot_suppress_error = true;`
  를 세운다(스켈레톤 준수). `Fsm_Task()` 가 홈 완료 시 `false` 로 내림(`fsm.c:315`).
- **합의 필요**: 이 책임을 `Fsm_Init()` 안으로 넣을지(권장), main 이 계속 세울지 팀에서 확정.

---

### B-3. [MAJOR · Team A] 좌표/시간 검증이 상태 판정보다 먼저 실행됨

`AT_UART/fsm.c:83-96` — `Fsm_HandleFrame()` 은 상태와 무관하게 먼저
`INVALID_COORD`(`f->x > COORD_X_MAX …`), `INVALID_TIME` 을 검사한 뒤 상태별 핸들러로 분기한다.
- 그래서 `STATE_RECOVERY_REQUIRED`(홈 미완료) 중에 **좌표가 범위 밖인** MOVE 가 오면
  `RECOVERY_REQUIRED` 가 아니라 `INVALID_COORD` 로 응답한다. `STATE_MOVING` 중이면 `BUSY` 대신
  `INVALID_COORD`/`INVALID_TIME`.
- `상위설계 §2.2` 상태 전이표는 해당 상태에서 `RECOVERY_REQUIRED` / `BUSY` 를 우선 반환하는 것처럼 읽힌다.
  `§3.2.1` 판정 순서는 `포맷 → ID_CONFLICT → 상태별` 만 명시하고 좌표/시간 검증 위치는 안 정함.
- **판단 필요(팀 합의)**: 범위 검증과 상태 거부(RECOVERY_REQUIRED/BUSY) 중 무엇이 우선인지 확정.
  Pi 구현이 "RECOVERY 중엔 무조건 RECOVERY_REQUIRED" 를 기대하면 Team A 가 순서를 바꿔야 함.

---

### B-4. [MINOR · Team A] `Uart_ReadLine` 의 초과 프레임 처리가 `INVALID_FORMAT` 를 내지 않음

`AT_UART/uart.c:103-114` — LF 없이 RX 링버퍼가 꽉 차면(127B) `s_rx_tail = s_rx_head` 로
**조용히 폐기**하고 `false` 반환. `상위설계 §3.1`("최대 프레임 64B, 초과 시 INVALID_FORMAT")과 달리
응답 프레임이 없다(Pi 는 타임아웃으로만 인지).
- `uart.h:16-18` 주석은 "잘린 결과는 항상 `FRAME_MAX_LEN` 초과 → 길이검사에서 걸림" 이라 하지만,
  잘린 길이는 `cap-1` 로 **클램프**되므로 `cap == FRAME_MAX_LEN` 이면 63자가 되어 `> 63` 검사를
  아슬아슬하게 통과할 수 있다(그 뒤 필드 파싱에서 대개 걸리긴 함).
- **통합 완화**: `main.c` 의 수신 버퍼를 `FRAME_MAX_LEN + 1`(65)로 잡고 `cap=65` 로 호출 →
  64자 이상 라인이 `Protocol_Parse` 길이검사(`> 63`)에서 **항상** `INVALID_FORMAT`.
  LF 없는 초과 버스트의 무응답 폐기는 그대로 남음(Team A 판단 사항).

---

### B-5. [MINOR · Team A] `STATE_IDLE` 에서 `Stepper_MoveToSlot()` 실패를 `STEPPER_ERROR` 로 응답

`AT_UART/fsm.c:130-132` — `상위설계 §3.2.3` 는 `STEPPER_ERROR` 를 "이동 실패를 명확히
판단할 수 있는 경우에만" 쓰라고 제한. IDLE 에서 `Stepper_MoveToSlot` 이 `false` 인 건 보통
"슬롯 범위 밖"(이미 `INVALID_COORD` 로 걸러짐) 또는 "이미 이동 중"(IDLE 에선 비정상) 이라
`STEPPER_ERROR` 라기보다 내부 상태 이상에 가깝다. 실사용상 거의 도달 불가 경로. 표현만 정정 권장.

---

### B-6. [MINOR · Team A] ID_CONFLICT 캐시가 거부된 프레임·중복 프레임으로도 갱신됨

`AT_UART/fsm.c:46-65` `Fsm_CheckIdConflict()` — 충돌이 아니면 무조건 `last_move`/`last_dispense`
캐시를 갱신한다. `BUSY` 로 거부될 MOVE(예: `STATE_MOVING` 중 다른 MOVE)도 캐시를 덮어쓴다.
`§3.2.1`("직전 수신한 동일 명령 종류") 해석상 큰 문제는 아니나, 이후 재전송 대조에 영향 가능.
또한 캐시가 사이클 종료 시 리셋되지 않아, Pi 요청번호가 `§3.2.1` 표현대로 "순환"하여 옛 번호를
다른 페이로드로 재사용하면 정상 요청이 `ID_CONFLICT` 로 걸린다(Pi 가 재사용 안 하면 무해).

---

### B-7. [MINOR · 빌드] `protocol.c` 의 `snprintf("%08lX")` 는 full `vfprintf` 필요

`INTEGRATION.cproj` 는 축소 printf(`-Wl,-u,vfprintf -lprintf_min`)를 지정하지 않으므로
기본(full) printf 가 링크됨 → 정상. **주의**: 나중에 flash 절약용으로 `libprintf_min` 을
켜면 폭 지정자/`l` 수정자가 무시되어 모든 `ACK`/`RESULT`/`ERROR` 프레임이 깨진다.

---

### B-8. [정보 · 이름 정합] `Uart_Task()` → `Task_UartRx()`

`AT_ACTUATOR/main.c` 는 `Uart_Task()` 를 호출하지만 Team A 인터페이스(`README_2 §4.2`)에는
그런 함수가 없다. Team A 는 `Uart_ReadLine()` 만 제공하고, "라인→파싱→`Fsm_HandleFrame`"
조립은 main 이 담당(`§4.10 Task_UartRx`). 통합 `main.c` 가 이 규약대로 구현했다. 실제 충돌 아님.

---

## C. 인터페이스 대조 (README_3 §3 얼린 시그니처) — 전부 일치 ✅

| 구분 | 함수 | 선언 | 사용처 | 일치 |
| --- | --- | --- | --- | --- |
| A→B | `bool Stepper_MoveToSlot(uint8_t,uint8_t)` | `stepper.h:43` | `fsm.c:130` | ✅ |
| A→B | `bool Stepper_IsBusy(void)` | `stepper.h:46` | `fsm.c:283` | ✅ |
| A→B | `void Dispense_Start(uint8_t,uint8_t)` | `dispense.h:41` | `fsm.c:192` | ✅ |
| A→B | `bool Dispense_IsComplete(void)` | `dispense.h:50` | `fsm.c:294` | ✅ |
| A→B | `DispenseResult Dispense_Result(void)` | `dispense.h:58` | `fsm.c:296` | ✅ |
| A→B | `bool Homing_IsComplete(void)` | `homing.h:44` | `fsm.c:312` | ✅ |
| 매루프 | `Stepper_Task`/`Servo_Task`/`Sensors_Task` | 각 헤더 | `main.c Task_Actuator` | ✅ |
| 매루프 | `Homing_Task`(RECOVERY only)/`Dispense_Task`(DISPENSING only) | 각 헤더 | `main.c Task_Actuator` | ✅ |
| main→A | `Uart_Init`/`Uart_ReadLine`/`Uart_WriteLine`/`Uart_TxBusy` | `uart.h` | `main.c`, `protocol.c` | ✅ (§4.2) |
| main→A | `Fsm_Init`/`Fsm_Task`/`Fsm_HandleFrame`/`Fsm_State`/`Fsm_SetState` | `fsm.h` | `main.c` | ✅ (§4.4) |
| A 내부 | `Protocol_Parse`/`Protocol_Send*`/`Protocol_ErrorText` | `protocol.h` | `fsm.c`, `main.c` | ✅ (§4.3) |
| 전역 | `extern volatile uint32_t g_tick_ms` / `extern SystemCtx g_ctx` | `systick.h`/`fsm.h` | 다수 | ✅ (§3) |

- `types.h` enum/struct: Team A 판이 Team B 판을 완전히 포함. 값·순서 동일. ✅
- `config.h`: 두 폴더 동일. ✅
- ISR 벡터 충돌 없음: `USART0_RX`/`USART0_UDRE`(uart) · `TIMER0_COMP`(systick) · `INT0`/`INT1`/`INT2`/`INT4`(sensors). ✅

---

## D. 프로토콜 동작 대조 (상위설계 §2.2 / §3) — 검토 결과

Team A `fsm.c` 상태 전이는 `상위설계 §2.2` 표와 대조 시 **의미상 일치**. 확인한 항목:

- IDLE: 새 MOVE→ACK→MOVING / DISPENSE·기타→무시 ✅
- MOVING: 같은 MOVE→ACK 재전송 / 도착→WAIT→AWAITING_DISPENSE / 다른 MOVE→BUSY / DISPENSE→NOT_READY ✅
- AWAITING_DISPENSE: 같은 MOVE→WAIT 재전송 / 좌표일치 DISPENSE→ACK→DISPENSING / 좌표불일치→COORD_MISMATCH / TIMEOUT(req 일치)→ACK→IDLE ✅
- DISPENSING: 같은 DISPENSE→ACK 재전송 / 완료→RESULT→AWAITING_RESULT_ACK / MOVE→BUSY ✅
- AWAITING_RESULT_ACK: 같은 DISPENSE→RESULT 재전송 / ACK(req 일치)→IDLE(캐시 유지) / 10초×6회 재전송 후 무송신 IDLE ✅
- RECOVERY_REQUIRED: MOVE/DISPENSE→RECOVERY_REQUIRED(단 B-3 예외) / 홈 완료→IDLE ✅
- TIMEOUT: AWAITING_DISPENSE→ACK 후 IDLE / IDLE→번호 대조 없이 ACK(멱등) / 그 외→무응답 ✅
- ACK: AWAITING_RESULT_ACK 외 무시 ✅
- 프레임 포맷: `RESULT|RRRRRRRR|XYR`, `ERROR|RRRRRRRR|CODE`, `ERROR|INVALID_FORMAT`(2필드), 대문자 16진 8자리, `|` 구분, LF 종료, CR 제거, 64B 상한 — 전부 `§3.1`/`§3.2` 준수 ✅

미해결은 위 B-1~B-8 뿐.

---

## E. 통합 후 검증 체크리스트 (README_3 §5 순서)

1. ~~B-1 프리스케일러~~ → ✅ 통합본 수정 완료. Team A upstream 반영만 남음.
2. 빌드: Microchip Studio 에서 `INTEGRATION.cproj` 열고 Debug 빌드. `.hex` 생성 확인.
3. UART 단독: 시리얼 터미널로 `MOVE|00000001|0|0|003600\n` 전송 → `ACK|00000001|MOVE` 수신 확인.
   깨진 프레임(`MOVE|xx`) → `ERROR|INVALID_FORMAT` 확인 (홈 완료 이후여야 함 — B-2).
4. 부팅: 전원 인가 → `STATE_RECOVERY_REQUIRED` → 홈 탐색 → 두 축 감지 → `STATE_IDLE`.
   (홈 완료 전 MOVE → `ERROR|reqid|RECOVERY_REQUIRED`)
5. 풀 사이클: MOVE→ACK→(이동)→WAIT→DISPENSE→ACK→(1~3차 배출)→RESULT→ACK→IDLE.
6. IR: DISPENSING 중 빔 차단 → 즉시 서보 180도 복귀 + `RESULT|…|xy1`.
7. 재전송: RESULT 후 ACK 안 보냄 → 10초 간격 재전송 6회 후 IDLE.
