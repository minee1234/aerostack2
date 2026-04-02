# behavior_server__impl.hpp 전체 분석

> 파일 경로: `as2_behaviors/as2_behavior/include/as2_behavior/__impl/behavior_server__impl.hpp`
> 분석 기준 브랜치: `main`
> 분석 일자: 2026-04-02

---

## 목차

1. [파일 구조 개요](#1-파일-구조-개요)
2. [생성자 BehaviorServer()](#2-생성자-behaviorserver--47~60)
3. [register_action()](#3-register_action--62~76)
4. [Goal 수신 흐름](#4-goal-수신-흐름-78~112)
5. [register_service_servers()](#5-register_service_servers--120~135)
6. [register_publishers() / register_timers()](#6-register_publishers--register_timers--137~151)
7. [register_run_timer() / cleanup_run_timer()](#7-register_run_timer--cleanup_run_timer--153~168)
8. [가상 함수 기본 구현](#8-가상-함수-기본-구현-170~210)
9. [상태 제어 함수들](#9-상태-제어-함수들)
10. [run() — 핵심 실행 루프](#10-run--핵심-실행-루프-287~326)
11. [publish_behavior_status()](#11-publish_behavior_status--328~334)
12. [전체 상태 전이 다이어그램](#12-전체-상태-전이-다이어그램)

---

## 1. 파일 구조 개요

```
behavior_server__impl.hpp (337줄)
│
├── 생성자 BehaviorServer() ─────────────────── :47~60
├── [등록 함수들]
│   ├── register_action() ────────────────── :62~76
│   ├── register_service_servers() ───────── :120~135
│   ├── register_publishers() ────────────── :137~144
│   ├── register_timers() ────────────────── :146~151  (상시 타이머)
│   ├── register_run_timer() ─────────────── :153~161  (실행 타이머)
│   └── cleanup_run_timer() ─────────────── :162~168
│
├── [가상 함수 기본 구현 - 파생 클래스가 오버라이드]
│   ├── on_activate() ────────────────────── :170~174
│   ├── on_modify() ──────────────────────── :176~180
│   ├── on_deactivate() ─────────────────── :182~186
│   ├── on_pause() ───────────────────────── :187~191
│   ├── on_resume() ─────────────────────── :192~196
│   └── on_run() ────────────────────────── :203~210
│
├── [상태 제어 함수들 - 외부에서 호출]
│   ├── activate() ───────────────────────── :212~222
│   ├── deactivate() ────────────────────── :223~236
│   ├── modify() ────────────────────────── :238~248
│   ├── pause() ─────────────────────────── :250~267
│   └── resume() ────────────────────────── :268~285
│
├── [실행 루프]
│   └── run() ───────────────────────────── :287~326
│
└── publish_behavior_status() ───────────── :328~334
```

---

## 2. 생성자 `BehaviorServer()` :47~60

```cpp
BehaviorServer(const std::string& name, const rclcpp::NodeOptions& options)
: as2::Node(name, options), action_name_(name)
{
    // run_frequency 파라미터 등록 (이미 있으면 건너뜀)
    if (!this->has_parameter("run_frequency")) {
        this->declare_parameter<float>("run_frequency", 10.0);  // 기본 10Hz
    }

    register_action();           // Action 서버 생성
    register_service_servers();  // pause/resume/stop/modify 서비스 등록
    register_publishers();       // behavior_status 퍼블리셔 생성
    register_timers();           // behavior_status 발행 타이머 (상시)
}
```

**핵심 설계:** `action_name_`이 곧 ROS2 노드 이름이 됩니다.

`TakeoffBehavior`는 `BehaviorServer("TakeoffBehavior")`로 생성되므로
Action 서버 이름 = `generate_global_name("TakeoffBehavior")` = `/drone0/TakeoffBehavior`

**초기화 순서:**

```
1. as2::Node 초기화 (ROS2 노드 기반 설정)
2. run_frequency 파라미터 선언 (10.0 Hz)
3. register_action()          → Action 서버 생성
4. register_service_servers() → 제어 서비스 4개 등록
5. register_publishers()      → behavior_status 퍼블리셔
6. register_timers()          → 상시 상태 발행 타이머 (100ms)
```

---

## 3. `register_action()` :62~76

```cpp
void register_action()
{
    // MutuallyExclusive: 동시에 하나의 콜백만 실행 (스레드 안전)
    auto action_server_group =
        this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    this->action_server_ = rclcpp_action::create_server<actionT>(
        this,
        this->generate_global_name(action_name_),  // "/drone0/TakeoffBehavior"
        std::bind(&BehaviorServer::handleGoal,     this, _1, _2),
        std::bind(&BehaviorServer::handleCancel,   this, _1),
        std::bind(&BehaviorServer::handleAccepted, this, _1));
}
```

**3개 콜백 바인딩:**

| 콜백 | 호출 시점 | 하는 일 |
|---|---|---|
| `handleGoal` | Goal 수신 즉시 | `activate()` 호출 → ACCEPT/REJECT 결정 |
| `handleCancel` | 취소 요청 수신 | `deactivate()` 호출 → ACCEPT/REJECT 결정 |
| `handleAccepted` | Goal 승인 후 | `goal_handle_` 멤버 변수에 저장 |

---

## 4. Goal 수신 흐름 :78~112

### `handleGoal()` :78~89

```cpp
rclcpp_action::GoalResponse handleGoal(uuid, goal)
{
    // activate()가 성공하면 즉시 실행 시작
    if (this->activate(goal)) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    return rclcpp_action::GoalResponse::REJECT;
}
```

**중요:** `ACCEPT_AND_EXECUTE`는 별도 스레드를 만들지 않고
**이미 실행 중**임을 선언합니다. 실제 반복 실행은 타이머가 담당합니다.

### `handleCancel()` :91~106

```cpp
rclcpp_action::CancelResponse handleCancel(goal_handle)
{
    // deactivate()를 Trigger 서비스 형식으로 호출
    auto req = make_shared<Trigger::Request>();
    auto res = make_shared<Trigger::Response>();
    deactivate(req, res);

    return res->success ? CancelResponse::ACCEPT : CancelResponse::REJECT;
}
```

**설계 특이점:** 취소 요청을 `deactivate()` 서비스와 **동일한 코드 경로**로 처리합니다.
외부 서비스 호출과 ROS2 Action 취소가 완전히 동일하게 동작합니다.

### `handleAccepted()` :108~112

```cpp
void handleAccepted(goal_handle)
{
    goal_handle_ = goal_handle;   // 멤버 변수에 저장
    // run()은 타이머가 호출 → 여기서 스레드 생성 없음
}
```

---

## 5. `register_service_servers()` :120~135

```cpp
void register_service_servers()
{
    // 서비스 이름: /{node_name}/_behavior/{action}
    // 예: /drone0/takeoff_behavior/_behavior/pause

    pause_srv_  = create_service<Trigger>    (generate_name("pause"),  &pause);
    resume_srv_ = create_service<Trigger>    (generate_name("resume"), &resume);
    stop_srv_   = create_service<Trigger>    (generate_name("stop"),   &deactivate);
    modify_srv_ = create_service<modify_srv> (generate_name("modify"), &modify);
}

// generate_name() :114~118
std::string generate_name(const std::string& name)
{
    // "takeoff_behavior" + "/_behavior/" + "pause"
    return std::string(this->get_name()) + "/_behavior/" + name;
}
```

**등록되는 서비스 전체 목록** (TakeoffBehavior 예시):

```
/drone0/takeoff_behavior/_behavior/pause     (std_srvs/srv/Trigger)
/drone0/takeoff_behavior/_behavior/resume    (std_srvs/srv/Trigger)
/drone0/takeoff_behavior/_behavior/stop      (std_srvs/srv/Trigger)
/drone0/takeoff_behavior/_behavior/modify    (actionT::SendGoalService)
```

---

## 6. `register_publishers()` / `register_timers()` :137~151

```cpp
void register_publishers()
{
    // 주석 처리된 것들: feedback, goal_status 는 현재 미사용
    // feedback_pub_    = ...  ← 주석 처리
    // goal_status_pub_ = ...  ← 주석 처리

    behavior_status_pub_ = create_publisher<BehaviorStatus>(
        generate_name("behavior_status"), 10);
    // → /drone0/takeoff_behavior/_behavior/behavior_status
}

void register_timers()
{
    // 상시 실행 타이머: 100ms(10Hz)로 상태 발행
    behavior_status_timer_ = create_timer(
        std::chrono::milliseconds(100),
        std::bind(&BehaviorServer::publish_behavior_status, this));
}
```

---

## 7. `register_run_timer()` / `cleanup_run_timer()` :153~168

```cpp
// Goal 수신 후 activate() 성공 시 1회 호출
void register_run_timer()
{
    float run_frequency;
    this->get_parameter("run_frequency", run_frequency);  // 기본 10.0 Hz

    run_timer_ = create_timer(
        std::chrono::duration<double>(1.0f / run_frequency),
        std::bind(&BehaviorServer::timer_callback, this));
    // timer_callback() → run(goal_handle_) 호출
}

// 실행 종료 시 (SUCCESS/FAILURE/ABORTED) 호출
void cleanup_run_timer(const ExecutionStatus& state)
{
    on_execution_end(state);   // 파생 클래스 정리 로직 (FSM 변경 등)
    goal_handle_.reset();      // Goal 핸들 해제
    run_timer_.reset();        // 타이머 해제 → run() 더 이상 호출 안 됨
}
```

**타이머 2개 구분:**

| 타이머 | 주기 | 생성 시점 | 해제 시점 |
|---|---|---|---|
| `behavior_status_timer_` | 100ms (10Hz) | 생성자 | 노드 소멸 시 |
| `run_timer_` | `1/run_frequency` | `activate()` 성공 시 | `cleanup_run_timer()` 호출 시 |

---

## 8. 가상 함수 기본 구현 :170~210

모두 **아무 것도 하지 않는 기본 구현**을 제공합니다.
파생 클래스가 필요한 것만 선택적으로 오버라이드합니다.

```cpp
// 기본값: 항상 성공 반환
bool on_activate(goal)   { return true; }   // :171~174
bool on_modify(goal)     { return true; }   // :177~180
bool on_deactivate(msg)  { return true; }   // :183~186
bool on_pause(msg)       { return true; }   // :188~191
bool on_resume(msg)      { return true; }   // :193~196

// 기본값: 즉시 성공 반환 (실제 구현은 파생 클래스에서)
ExecutionStatus on_run(goal, feedback, result)
{
    return ExecutionStatus::SUCCESS;         // :204~210
}
```

**파생 클래스 오버라이드 패턴 (TakeoffBehavior 예시):**

```cpp
bool on_activate(goal) override {
    return takeoff_plugin_->on_activate(goal);  // 플러그인 초기화
}

ExecutionStatus on_run(goal, feedback, result) override {
    return takeoff_plugin_->on_run(goal, feedback, result);  // 반복 실행
}

void on_execution_end(state) override {
    sendEventFSME(PSME::TOOK_OFF);   // 플랫폼 FSM 상태 전환
}
```

---

## 9. 상태 제어 함수들

### `activate()` :212~222

```cpp
bool activate(goal)
{
    RCLCPP_INFO(logger, "START");
    if (on_activate(goal)) {          // 파생 클래스 구현 호출
        register_run_timer();         // 반복 실행 타이머 시작
        behavior_status_.status = BehaviorStatus::RUNNING;
        return true;
    }
    return false;  // on_activate 실패 시 Goal REJECT
}
```

### `deactivate()` :223~236

```cpp
void deactivate(req, result)
{
    RCLCPP_INFO(logger, "STOP");
    auto msg = make_shared<string>();
    result->success = on_deactivate(msg);
    result->message = *msg;
    if (result->success) {
        cleanup_run_timer(ExecutionStatus::ABORTED);  // 타이머 해제
        behavior_status_.status = BehaviorStatus::IDLE;
    }
}
```

### `modify()` :238~248

```cpp
void modify(request, response)
{
    RCLCPP_INFO(logger, "MODIFY");
    // request->goal에서 새 Goal 추출
    const auto goal = make_shared<actionT::Goal>(request->goal);
    bool success = on_modify(goal);
    response->accepted = success;
    // ※ run_timer_는 계속 실행 중 → 다음 on_run()부터 새 Goal 적용
}
```

### `pause()` :250~267

```cpp
void pause(req, result)
{
    // RUNNING 상태가 아니면 거부
    if (behavior_status_.status != BehaviorStatus::RUNNING) {
        result->success = false;
        result->message = "Behavior is not running";
        return;
    }
    result->success = on_pause(msg);
    if (result->success) {
        behavior_status_.status = BehaviorStatus::PAUSED;
        // ※ run_timer_는 해제하지 않음 → run()은 계속 호출되지만
        //   run() 첫 줄의 status 체크에서 조기 return됨
    }
}
```

### `resume()` :268~285

```cpp
void resume(req, result)
{
    // PAUSED 상태가 아니면 거부
    if (behavior_status_.status != BehaviorStatus::PAUSED) {
        result->success = false;
        result->message = "Behavior is not paused";
        return;
    }
    result->success = on_resume(msg);
    if (result->success) {
        behavior_status_.status = BehaviorStatus::RUNNING;
        // run_timer_는 이미 살아있으므로 자동으로 재개
    }
}
```

**PAUSE 시 run_timer_ 동작 설명:**

```
PAUSED 상태에서도 run_timer_는 계속 동작함
    ↓
timer_callback() → run() 호출
    ↓
run() :291
if (behavior_status_.status != BehaviorStatus::RUNNING) {
    return;  ← PAUSED이므로 즉시 반환 (아무 것도 안 함)
}
```

**상태 전이 가드 조건:**

| 호출 | 유효 상태 | 거부 조건 |
|---|---|---|
| `pause()` | RUNNING | RUNNING이 아닌 경우 |
| `resume()` | PAUSED | PAUSED가 아닌 경우 |
| `deactivate()` | 모든 상태 | 없음 |
| `modify()` | 모든 상태 | 없음 |

---

## 10. `run()` — 핵심 실행 루프 :287~326

```cpp
void run(const shared_ptr<GoalHandleAction>& goal_handle_action)
{
    // [가드 조건] PAUSED 또는 IDLE이면 즉시 반환
    if (behavior_status_.status != BehaviorStatus::RUNNING) {
        return;
    }

    // 매번 새로운 feedback/result 객체 생성
    auto goal     = goal_handle_action->get_goal();
    auto feedback = make_shared<actionT::Feedback>();
    auto result   = make_shared<actionT::Result>();

    // 파생 클래스의 on_run() 호출 (핵심 로직)
    ExecutionStatus status = on_run(goal, feedback, result);

    switch (status) {
        case SUCCESS:
            behavior_status_.status = BehaviorStatus::IDLE;
            goal_handle_->succeed(result);           // BT에 성공 신호 전달
            break;

        case RUNNING:
            // 5초마다 한 번 로그 출력 (THROTTLE)
            RCLCPP_INFO_THROTTLE(logger, *clk, 5000, "RUNNING");
            goal_handle_action->publish_feedback(feedback);  // BT에 피드백 전달
            behavior_status_.status = BehaviorStatus::RUNNING;
            break;

        case FAILURE:
            behavior_status_.status = BehaviorStatus::IDLE;
            goal_handle_->abort(result);             // BT에 실패 신호 전달
            break;

        case ABORTED:
            behavior_status_.status = BehaviorStatus::IDLE;
            goal_handle_->abort(result);             // FAILURE와 동일 처리
            break;
    }

    // RUNNING이 아닌 상태가 되면 타이머 정리
    if (behavior_status_.status != BehaviorStatus::RUNNING) {
        cleanup_run_timer(status);
    }
}
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

## 11. `publish_behavior_status()` :328~334

```cpp
void publish_behavior_status()
{
    BehaviorStatus msg;
    msg.status = behavior_status_.status;
    behavior_status_pub_->publish(msg);
    // 토픽: /drone0/takeoff_behavior/_behavior/behavior_status
    // 주기: 100ms (10Hz) 상시 발행
}
```

**BehaviorStatus 값:**

```
IDLE    = 0  ← 대기 중 (Goal 없음)
RUNNING = 1  ← on_run() 반복 실행 중
PAUSED  = 2  ← 일시 정지 (run_timer_ 살아있으나 run() 조기 반환)
```

**외부에서 상태 모니터링:**

```bash
ros2 topic echo /drone0/takeoff_behavior/_behavior/behavior_status
```

---

## 12. 전체 상태 전이 다이어그램

```
                    [생성자 호출]
                         │
                         │ register_action()
                         │ register_service_servers()
                         │ register_publishers()
                         │ register_timers() → behavior_status_timer_ (상시)
                         │
                      [IDLE]
                         │
          Goal 수신 → handleGoal()
                         │
                  activate(goal)
                   │           │
            성공(true)      실패(false)
                 │                 │
    register_run_timer()       REJECT → BT FAILURE
    behavior_status=RUNNING
                 │
             [RUNNING] ◄─────────────────────────────────────┐
                 │                                           │
          timer_callback()                           resume() 서비스 호출
          → run() 호출 (10Hz)                                │
                 │                                       [PAUSED]
        on_run() 반환값                                      │
   ┌─────────────┼───────────────┐                   pause() 서비스 호출
   │             │               │                          │
SUCCESS       RUNNING       FAILURE/ABORTED       behavior_status=PAUSED
   │             │               │                   run()은 조기 return
   │     publish_feedback()      │
   │     behavior_status=RUNNING │
   │                             │
succeed(result)             abort(result)
behavior_status=IDLE        behavior_status=IDLE
   │                             │
   └──────────────┬──────────────┘
                  │
         cleanup_run_timer(state)
           ├─ on_execution_end(state)  ← 파생 클래스 정리
           ├─ goal_handle_.reset()
           └─ run_timer_.reset()
                  │
               [IDLE]
```

**외부 서비스 호출과 Action 취소의 코드 경로 통합:**

```
ros2 service call .../stop  Trigger    ─┐
                                        ├─→ deactivate() → cleanup_run_timer(ABORTED)
BT Action cancel_goal() 요청           ─┘
  → handleCancel() → deactivate()
```
