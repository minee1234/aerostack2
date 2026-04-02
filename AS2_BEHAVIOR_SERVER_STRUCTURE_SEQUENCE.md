# BehaviorServer 구조 및 동작 시퀀스

> 파일 경로: `as2_behaviors/as2_behavior/include/as2_behavior/__impl/behavior_server__impl.hpp`
> 분석 기준 브랜치: `main`
> 분석 일자: 2026-04-02

---

## 목차

1. [한 문장 비유](#1-한-문장-비유)
2. [전체 구조 — 3개 레이어](#2-전체-구조--3개-레이어)
3. [생성자에서 만들어지는 것들](#3-생성자에서-만들어지는-것들)
4. [정상 실행 시퀀스](#4-정상-실행-시퀀스)
5. [상태 머신 — 4가지 상태](#5-상태-머신--4가지-상태)
6. [run() 내부 분기 로직](#6-run-내부-분기-로직)
7. [외부 제어 시퀀스 (pause / resume / stop)](#7-외부-제어-시퀀스-pause--resume--stop)
8. [타이머 2개의 생애주기](#8-타이머-2개의-생애주기)
9. [파생 클래스가 구현해야 하는 것](#9-파생-클래스가-구현해야-하는-것)

---

## 1. 한 문장 비유

> **BehaviorServer = "주문 받아서 반복 작업하는 주방장"**
>
> - BT 노드가 "takeoff 해줘 (height=2, speed=0.5)" 라고 주문(Goal)을 넣으면
> - 주방장이 수락하고 타이머 맞춰 10Hz로 조리(on_run)하다가
> - 완성되면 "다 됐어요(succeed)" 신호를 보냄

---

## 2. 전체 구조 — 3개 레이어

```
┌─────────────────────────────────────────────────────────────┐
│  레이어 1: 통신 인터페이스 (BehaviorServer가 자동 생성)        │
│                                                             │
│  ┌──────────────────┐   ┌──────────────────────────────┐   │
│  │  ROS2 Action 서버 │   │  제어 서비스 4개               │   │
│  │  /drone0/        │   │  /_behavior/pause             │   │
│  │  TakeoffBehavior │   │  /_behavior/resume            │   │
│  │                  │   │  /_behavior/stop              │   │
│  │  Goal →          │   │  /_behavior/modify            │   │
│  │  Feedback ←      │   └──────────────────────────────┘   │
│  │  Result ←        │                                      │
│  └──────────────────┘   ┌──────────────────────────────┐   │
│                          │  상태 토픽 (10Hz 상시 발행)    │   │
│                          │  /_behavior/behavior_status  │   │
│                          │  IDLE / RUNNING / PAUSED     │   │
│                          └──────────────────────────────┘   │
├─────────────────────────────────────────────────────────────┤
│  레이어 2: 실행 엔진 (BehaviorServer 내부)                    │
│                                                             │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  run_timer_ (10Hz)                                   │  │
│  │  ┌──────────────────────────────────────────────┐   │  │
│  │  │  run() → on_run() 반복 호출                   │   │  │
│  │  │  ExecutionStatus 확인 → succeed/abort/feedback│   │  │
│  │  └──────────────────────────────────────────────┘   │  │
│  └──────────────────────────────────────────────────────┘  │
├─────────────────────────────────────────────────────────────┤
│  레이어 3: 파생 클래스 구현 (TakeoffBehavior 등)              │
│                                                             │
│  on_activate()      → 목표 좌표 계산, 플러그인 초기화        │
│  on_run()           → 위치 명령 전송, 도달 여부 확인         │
│  on_execution_end() → FSM 상태 변경 (FLYING 등)             │
└─────────────────────────────────────────────────────────────┘
```

---

## 3. 생성자에서 만들어지는 것들

```
BehaviorServer("TakeoffBehavior") 생성 시
│
├─ ① Action 서버 생성
│       /drone0/TakeoffBehavior
│       └─ handleGoal / handleCancel / handleAccepted 콜백 등록
│
├─ ② 제어 서비스 4개 생성 (항상 대기 중)
│       /drone0/takeoff_behavior/_behavior/pause
│       /drone0/takeoff_behavior/_behavior/resume
│       /drone0/takeoff_behavior/_behavior/stop
│       /drone0/takeoff_behavior/_behavior/modify
│
├─ ③ 상태 퍼블리셔 생성
│       /drone0/takeoff_behavior/_behavior/behavior_status
│
└─ ④ 상태 발행 타이머 시작 (100ms, 노드 살아있는 동안 계속)
        → publish_behavior_status() 반복 호출
```

---

## 4. 정상 실행 시퀀스

```
BT 노드                    BehaviorServer               플러그인
  │                             │                          │
  │  async_send_goal(goal)      │                          │
  │ ─────────────────────────► │                          │
  │                             │ handleGoal()             │
  │                             │ → activate(goal)         │
  │                             │   → on_activate(goal) ──►│
  │                             │                          │ 목표 좌표 계산
  │                             │   ◄── true ──────────────│
  │                             │   register_run_timer()   │
  │                             │   status = RUNNING       │
  │                             │ → ACCEPT_AND_EXECUTE     │
  │  GoalHandle 승인            │                          │
  │ ◄───────────────────────── │                          │
  │                             │                          │
  │  [매 100ms]                 │ [매 100ms]               │
  │                             │ run_timer_ 발동           │
  │                             │ → run()                  │
  │                             │   → on_run() ────────────►│
  │                             │                          │ 위치 명령 전송
  │                             │                          │ 고도 확인
  │                             │   ◄── RUNNING ───────────│
  │                             │   publish_feedback()     │
  │  feedback 수신              │                          │
  │ ◄───────────────────────── │                          │
  │  (RUNNING 유지)             │                          │
  │     :                       │     :                    │
  │     :  (반복)               │     : (반복)             │
  │     :                       │     :                    │
  │                             │ → on_run() ──────────────►│
  │                             │                          │ 목표 고도 도달!
  │                             │   ◄── SUCCESS ───────────│
  │                             │   goal_handle_->succeed()│
  │                             │   cleanup_run_timer()    │
  │                             │   → on_execution_end()   │
  │  Result 수신 (SUCCEEDED)    │                          │
  │ ◄───────────────────────── │                          │
  │  on_success() 호출          │                          │
  │  → BT::NodeStatus::SUCCESS  │                          │
```

---

## 5. 상태 머신 — 4가지 상태

```
                   ┌─────────────────────────────────────────┐
                   │                                         │
   생성자 호출     │         stop 서비스 호출                 │
        │          │         or cancel_goal()                 │
        ▼          │               │                         │
   ┌─────────┐     │          ┌────▼────┐                    │
   │  IDLE   │─────┼─────────►│  IDLE   │◄───────────────────┘
   └────┬────┘     │          └─────────┘
        │          │          SUCCESS / FAILURE / ABORTED 시
        │ Goal 수신, activate() 성공
        │ register_run_timer()
        ▼
   ┌─────────┐  pause() 호출  ┌─────────┐
   │ RUNNING │───────────────►│ PAUSED  │
   │         │◄───────────────│         │
   └─────────┘ resume() 호출  └─────────┘
```

**각 상태에서 run()의 동작:**

| 상태 | run() 호출 여부 | on_run() 호출 여부 |
|---|---|---|
| `IDLE` | X (타이머 없음) | X |
| `RUNNING` | O (10Hz) | O |
| `PAUSED` | O (타이머 살아있음) | X (첫 줄에서 조기 return) |

**BehaviorStatus 값:**

```
IDLE    = 0  ← 대기 중 (Goal 없음)
RUNNING = 1  ← on_run() 반복 실행 중
PAUSED  = 2  ← 일시 정지 (run_timer_ 살아있으나 run() 조기 반환)
```

---

## 6. `run()` 내부 분기 로직

```
run() 호출 (10Hz)
    │
    ├─ status != RUNNING ? ──────────────────► return (PAUSED일 때)
    │
    ├─ on_run(goal, feedback, result) 호출
    │
    │   반환값
    │   ├─ RUNNING
    │   │     publish_feedback(feedback)  ← BT에 진행 상황 전달
    │   │     status 유지
    │   │     → 다음 타이머까지 대기
    │   │
    │   ├─ SUCCESS
    │   │     goal_handle_->succeed(result)  ← BT에 완료 신호
    │   │     status = IDLE
    │   │     → cleanup_run_timer(SUCCESS)
    │   │           on_execution_end(SUCCESS)
    │   │           goal_handle_.reset()
    │   │           run_timer_.reset()  ← 타이머 해제
    │   │
    │   ├─ FAILURE
    │   │     goal_handle_->abort(result)   ← BT에 실패 신호
    │   │     status = IDLE
    │   │     → cleanup_run_timer(FAILURE)
    │   │
    │   └─ ABORTED
    │         goal_handle_->abort(result)
    │         status = IDLE
    │         → cleanup_run_timer(ABORTED)
    │
    └─ status != RUNNING 이면 cleanup_run_timer() 호출
```

**ExecutionStatus 반환값별 동작:**

| 반환값 | `goal_handle_` 처리 | `behavior_status_` | BT 노드 결과 |
|---|---|---|---|
| `SUCCESS` | `succeed(result)` | IDLE | `on_success()` → SUCCESS |
| `RUNNING` | `publish_feedback()` | RUNNING | RUNNING 유지 |
| `FAILURE` | `abort(result)` | IDLE | `on_aborted()` → FAILURE |
| `ABORTED` | `abort(result)` | IDLE | `on_aborted()` → FAILURE |

**FAILURE vs ABORTED 의미 차이:**

| 상태 | 발생 원인 | 발생 위치 |
|---|---|---|
| `FAILURE` | `on_run()`이 스스로 실패 반환 | 파생 클래스 내부 |
| `ABORTED` | 외부 `deactivate()` 호출로 강제 중단 | `deactivate()` → `cleanup_run_timer(ABORTED)` |

---

## 7. 외부 제어 시퀀스 (pause / resume / stop)

```
외부                       BehaviorServer
  │                             │
  │  pause 서비스 호출           │
  │ ─────────────────────────► │
  │                             │ status == RUNNING ? → OK
  │                             │ on_pause() 호출
  │                             │ status = PAUSED
  │  success: true              │
  │ ◄───────────────────────── │
  │                             │
  │                             │ [run_timer_는 계속 동작]
  │                             │ run() 호출되지만 첫 줄에서 return
  │                             │ (on_run() 호출 안 됨)
  │                             │
  │  resume 서비스 호출          │
  │ ─────────────────────────► │
  │                             │ status == PAUSED ? → OK
  │                             │ on_resume() 호출
  │                             │ status = RUNNING
  │  success: true              │
  │ ◄───────────────────────── │
  │                             │
  │                             │ [run() 다시 on_run() 호출 시작]
  │                             │
  │  stop 서비스 호출            │
  │ ─────────────────────────► │
  │                             │ on_deactivate() 호출
  │                             │ cleanup_run_timer(ABORTED)
  │                             │   → on_execution_end(ABORTED)
  │                             │   → goal_handle_->abort()  ← BT에 알림
  │                             │   → run_timer_.reset()
  │                             │ status = IDLE
  │  success: true              │
  │ ◄───────────────────────── │
```

**상태 전이 가드 조건:**

| 호출 | 유효 상태 | 거부 조건 |
|---|---|---|
| `pause()` | RUNNING | RUNNING이 아닌 경우 |
| `resume()` | PAUSED | PAUSED가 아닌 경우 |
| `deactivate()` | 모든 상태 | 없음 |
| `modify()` | 모든 상태 | 없음 |

**외부 stop 서비스와 BT cancel_goal()의 코드 경로 통합:**

```
ros2 service call .../stop  Trigger    ─┐
                                        ├─→ deactivate() → cleanup_run_timer(ABORTED)
BT Action cancel_goal() 요청           ─┘
  → handleCancel() → deactivate()
```

---

## 8. 타이머 2개의 생애주기

```
노드 시작                                              노드 종료
   │                                                      │
   ▼                                                      ▼
behavior_status_timer_ ────────────────────────────────► 소멸
   (100ms, 항상)

                Goal 수신        종료
                   │              │
                   ▼              ▼
run_timer_         ┌──────────────┐
   (100ms, 조건)   │   동작 중    │  → reset() 후 소멸
                   └──────────────┘
```

| 타이머 | 주기 | 생성 시점 | 해제 시점 | 역할 |
|---|---|---|---|---|
| `behavior_status_timer_` | 100ms (10Hz) | 생성자 | 노드 소멸 시 | 상태 토픽 상시 발행 |
| `run_timer_` | `1/run_frequency` | `activate()` 성공 시 | `cleanup_run_timer()` 호출 시 | `on_run()` 반복 실행 |

---

## 9. 파생 클래스가 구현해야 하는 것

```cpp
class MyBehavior : public BehaviorServer<as2_msgs::action::MyAction>
{
public:
    MyBehavior(const rclcpp::NodeOptions& options)
    : BehaviorServer(as2_names::actions::behaviors::my_action, options) {}

    // ✅ 핵심 - 반드시 구현
    ExecutionStatus on_run(
        const std::shared_ptr<const Goal>& goal,
        std::shared_ptr<Feedback>& feedback,
        std::shared_ptr<Result>& result) override
    {
        // 매 100ms 호출됨
        // SUCCESS / RUNNING / FAILURE 중 하나 반환
        if (목표_달성()) return ExecutionStatus::SUCCESS;
        if (실패_조건()) return ExecutionStatus::FAILURE;
        제어_명령_전송();
        return ExecutionStatus::RUNNING;
    }

    // 🔲 선택적 구현 (기본값: return true)
    bool on_activate(
        std::shared_ptr<const Goal> goal) override
    {
        // Goal 수신 시 1회 호출 - 초기화 작업
        // false 반환 시 Goal REJECT
        return true;
    }

    void on_execution_end(
        const ExecutionStatus& state) override
    {
        // 종료 시 1회 호출 - 정리 작업
        // state로 SUCCESS/FAILURE/ABORTED 구분 가능
    }

    bool on_deactivate(
        const std::shared_ptr<std::string>& message) override { ... }

    bool on_pause(
        const std::shared_ptr<std::string>& message) override { ... }

    bool on_resume(
        const std::shared_ptr<std::string>& message) override { ... }

    bool on_modify(
        std::shared_ptr<const Goal> goal) override { ... }
};
```

**메서드 호출 시점 요약:**

| 메서드 | 호출 시점 | 횟수 | 반환값 의미 |
|---|---|---|---|
| `on_activate()` | Goal 수신 시 | 1회 | `true`: 수락 / `false`: 거절 |
| `on_run()` | 10Hz 타이머 | 반복 | RUNNING/SUCCESS/FAILURE/ABORTED |
| `on_execution_end()` | 종료 시 | 1회 | 없음 (void) |
| `on_deactivate()` | stop/cancel 시 | 1회 | `true`: 성공 |
| `on_pause()` | pause 서비스 시 | 1회 | `true`: 성공 |
| `on_resume()` | resume 서비스 시 | 1회 | `true`: 성공 |
| `on_modify()` | modify 서비스 시 | 1회 | `true`: 성공 |
