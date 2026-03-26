# as2_behavior 패키지 상세 분석

**버전**: 1.1.3
**라이선스**: BSD-3-Clause
**작성자**: Miguel Fernández Cortizas, Pedro Arias Pérez, David Pérez Saura, Rafael Pérez Seguí

---

## 1. 패키지 개요

`as2_behavior`는 Aerostack2 전체 behavior 시스템의 **핵심 프레임워크 패키지**다.
모든 구체적인 behavior 패키지(`as2_behaviors_motion`, `as2_behaviors_platform` 등)는 이 패키지의 `BehaviorServer<actionT>` 템플릿 클래스를 상속하여 구현된다.

### 역할
- ROS2 Action Server를 기반으로 한 behavior 실행 프레임워크 제공
- Behavior 생명주기(IDLE → RUNNING → PAUSED → IDLE) 관리
- Pause/Resume/Stop/Modify 서비스 자동 등록
- BehaviorStatus 퍼블리셔 자동 등록

---

## 2. 파일 구조

```
as2_behavior/
├── include/as2_behavior/
│   ├── behavior_server.hpp              # 진입점 헤더 (impl만 포함)
│   ├── behavior_utils.hpp              # ExecutionStatus enum 정의
│   ├── as2_basic_behavior.hpp          # 레거시 기본 클래스 (스레드 기반)
│   ├── __detail/
│   │   └── behavior_server__class.hpp  # BehaviorServer 클래스 선언
│   └── __impl/
│       └── behavior_server__impl.hpp   # BehaviorServer 전체 구현 (템플릿)
├── src/
│   ├── behavior_server.cpp             # 링크용 빈 소스
│   └── behavior_client.cpp             # 클라이언트 유틸리티
├── tests/
│   ├── CMakeLists.txt
│   └── as2_behavior_gtest.cpp          # 단위 테스트
├── CMakeLists.txt
└── package.xml
```

---

## 3. 의존성

```xml
<!-- package.xml -->
<depend>rclcpp</depend>
<depend>rclcpp_action</depend>
<depend>as2_core</depend>
<depend>as2_msgs</depend>
<depend>std_srvs</depend>
<depend>std_msgs</depend>
```

---

## 4. 핵심 타입: `ExecutionStatus` enum

**파일**: `include/as2_behavior/behavior_utils.hpp:52`

```cpp
namespace as2_behavior {
  enum class ExecutionStatus { SUCCESS, RUNNING, FAILURE, ABORTED };
}
```

| 상태 | 의미 | BehaviorServer 처리 |
|------|------|---------------------|
| `SUCCESS` | 목표 달성 | `goal_handle_->succeed(result)` |
| `RUNNING` | 실행 중 | `goal_handle_->publish_feedback(feedback)` |
| `FAILURE` | 목표 실패 | `goal_handle_->abort(result)` |
| `ABORTED` | 외부 중단 | `goal_handle_->abort(result)` |

---

## 5. `BehaviorServer<actionT>` 클래스

**파일**: `include/as2_behavior/__detail/behavior_server__class.hpp`

### 클래스 선언

```cpp
namespace as2_behavior {

template<typename actionT>
class BehaviorServer : public as2::Node
{
public:
  using GoalHandleAction = rclcpp_action::ServerGoalHandle<actionT>;

  // 내부 타입 별칭
  using BehaviorStatus = as2_msgs::msg::BehaviorStatus;
  using start_srv   = typename actionT::Impl::SendGoalService;
  using modify_srv  = start_srv;
  using result_srv  = typename actionT::Impl::GetResultService;
  using feedback_msg = typename actionT::Impl::FeedbackMessage;
  using goal_status_msg = typename actionT::Impl::GoalStatusMessage;
  using cancel_srv  = typename actionT::Impl::CancelGoalService;

  std::string action_name_;
  typename rclcpp_action::Server<actionT>::SharedPtr action_server_;

  // ────── 오버라이드 대상 메서드 (서브클래스가 구현) ──────
  virtual bool on_activate(std::shared_ptr<const typename actionT::Goal> goal);
  virtual bool on_modify(std::shared_ptr<const typename actionT::Goal> goal);
  virtual bool on_deactivate(const std::shared_ptr<std::string>& message);
  virtual bool on_pause(const std::shared_ptr<std::string>& message);
  virtual bool on_resume(const std::shared_ptr<std::string>& message);
  virtual void on_execution_end(const ExecutionStatus& state) {}
  virtual ExecutionStatus on_run(
    const std::shared_ptr<const typename actionT::Goal>& goal,
    std::shared_ptr<typename actionT::Feedback>& feedback_msg,
    std::shared_ptr<typename actionT::Result>& result_msg);

private:
  // 서비스 서버
  typename rclcpp::Service<start_srv>::SharedPtr start_srv_;
  typename rclcpp::Service<modify_srv>::SharedPtr modify_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resume_srv_;

  // 퍼블리셔
  rclcpp::Publisher<BehaviorStatus>::SharedPtr behavior_status_pub_;

  // 타이머
  rclcpp::TimerBase::SharedPtr behavior_status_timer_;  // 100ms 주기
  rclcpp::TimerBase::SharedPtr run_timer_;               // run_frequency 기반
};

} // namespace as2_behavior
```

### 생성자 동작

```cpp
BehaviorServer<actionT>::BehaviorServer(const std::string& name, const rclcpp::NodeOptions& options)
  : as2::Node(name, options), action_name_(name)
{
  // 파라미터 선언 (기본값: 10.0 Hz)
  if (!this->has_parameter("run_frequency"))
    this->declare_parameter<float>("run_frequency", 10.0);

  register_action();          // 글로벌 이름으로 Action Server 등록
  register_service_servers(); // pause/resume/stop/modify 서비스 등록
  register_publishers();      // behavior_status 퍼블리셔 등록
  register_timers();          // 100ms behavior_status 타이머 등록
}
```

---

## 6. ROS2 인터페이스 자동 등록

### Action Server
```
/<namespace>/<action_name>  (generate_global_name 사용)
```

### Service Servers (로컬 이름)
```
/<node_name>/_behavior/pause    [std_srvs/Trigger]
/<node_name>/_behavior/resume   [std_srvs/Trigger]
/<node_name>/_behavior/stop     [std_srvs/Trigger]
/<node_name>/_behavior/modify   [actionT::Impl::SendGoalService]
```

### Publishers
```
/<node_name>/_behavior/behavior_status  [as2_msgs/BehaviorStatus]  (100ms 주기)
```

---

## 7. 생명주기 상태 머신

```
           [Action Goal 수신]
                 │
                 ▼
           on_activate() ─── false ──► [REJECT 응답]
                 │ true
                 ▼
           register_run_timer()
                 │
           status = RUNNING
                 │
    ┌────────────▼────────────┐
    │  on_run() → RUNNING     │ ◄─── run_timer_ 주기 (run_frequency Hz)
    │  publish_feedback()     │
    └────────────┬────────────┘
         │              │              │
      SUCCESS         FAILURE        ABORTED
         │              │              │
    succeed(result)  abort(result)  abort(result)
         │              │              │
         └──────────────┴──────────────┘
                        │
                on_execution_end(state)
                goal_handle_.reset()
                run_timer_.reset()
                status = IDLE
```

### Pause/Resume 흐름
```
pause_srv 수신 → on_pause() → status = PAUSED
  └─ run() 내에서: if(status != RUNNING) return 즉시 종료

resume_srv 수신 → on_resume() → status = RUNNING
  └─ run() 재개
```

### Stop 흐름
```
stop_srv 수신 → on_deactivate() → cleanup_run_timer(ABORTED)
             → status = IDLE
             → goal_handle_.reset()
```

---

## 8. 핵심 구현: `run()` 메서드

**파일**: `include/as2_behavior/__impl/behavior_server__impl.hpp:288`

```cpp
template<typename actionT>
void BehaviorServer<actionT>::run(
  const typename std::shared_ptr<GoalHandleAction>& goal_handle_action)
{
  // PAUSED 상태 → 아무것도 안 함
  if (behavior_status_.status != BehaviorStatus::RUNNING) return;

  auto goal = goal_handle_action->get_goal();
  auto feedback = std::make_shared<typename actionT::Feedback>();
  auto result = std::make_shared<typename actionT::Result>();

  // 서브클래스의 on_run() 호출
  ExecutionStatus status = on_run(goal, feedback, result);

  switch (status) {
    case ExecutionStatus::SUCCESS:
      behavior_status_.status = BehaviorStatus::IDLE;
      goal_handle_->succeed(result);
      break;
    case ExecutionStatus::RUNNING:
      goal_handle_action->publish_feedback(feedback);  // 5초마다 RUNNING 로그
      break;
    case ExecutionStatus::FAILURE:
    case ExecutionStatus::ABORTED:
      behavior_status_.status = BehaviorStatus::IDLE;
      goal_handle_->abort(result);
      break;
  }

  // SUCCESS/FAILURE/ABORTED → 타이머 정리
  if (behavior_status_.status != BehaviorStatus::RUNNING)
    cleanup_run_timer(status);
}
```

---

## 9. 서브클래스 구현 패턴

모든 구체적인 behavior는 다음 패턴으로 구현된다:

```cpp
class MyBehavior : public as2_behavior::BehaviorServer<as2_msgs::action::MyAction>
{
public:
  MyBehavior() : BehaviorServer("my_behavior_name") {
    // 파라미터 선언, 구독자, 서비스 클라이언트 생성
  }

  bool on_activate(std::shared_ptr<const MyAction::Goal> goal) override {
    // 목표 검증 및 초기화
    return true; // false면 REJECT
  }

  bool on_deactivate(const std::shared_ptr<std::string>& msg) override {
    // 안전하게 정지
    return true;
  }

  ExecutionStatus on_run(
    const std::shared_ptr<const MyAction::Goal>& goal,
    std::shared_ptr<MyAction::Feedback>& feedback,
    std::shared_ptr<MyAction::Result>& result) override
  {
    // 목표 달성 여부 확인
    if (goal_reached()) {
      result->success = true;
      return ExecutionStatus::SUCCESS;
    }
    // 피드백 업데이트
    feedback->progress = current_progress;
    return ExecutionStatus::RUNNING;
  }

  void on_execution_end(const ExecutionStatus& state) override {
    // 정리 작업 (호버, FSM 이벤트 등)
  }
};
```

---

## 10. `as2_basic_behavior.hpp` (레거시 클래스)

별도 스레드에서 action을 실행하는 구형 기반 클래스.
현재 `as2_behavior_tree` 패키지의 BT Action Node에서 참조하나, 신규 behavior는 `BehaviorServer`를 사용한다.

```cpp
template<typename MessageT>
class BasicBehavior : public as2::Node {
  void handleAccepted(const shared_ptr<GoalHandleAction> goal_handle) {
    if (execution_thread_.joinable()) execution_thread_.join();
    execution_thread_ = std::thread(...);  // 별도 스레드 실행
  }
};
```

---

## 11. 설계 패턴

| 패턴 | 적용 위치 | 설명 |
|------|-----------|------|
| **Template Method** | `BehaviorServer::run()` | `on_run()` 호출 구조 고정, 구현은 서브클래스 |
| **Strategy** | pluginlib 기반 behavior | 알고리즘을 런타임에 교체 |
| **State Machine** | `behavior_status_` | IDLE/RUNNING/PAUSED 전환 |
| **CRTP-like** | 템플릿 특화 | 컴파일 타임 타입 안전성 |

---

## 12. 알려진 이슈 및 주의사항

1. **`on_activate()` 기본 구현이 `true` 반환**: 서브클래스가 오버라이드하지 않으면 항상 성공으로 간주됨. TODO 주석에 "CONVERT INTO PURE VIRTUAL FUNCTIONS"라고 명시.

2. **`run_frequency` 파라미터**: 기본값 10Hz. 고속 컨트롤이 필요한 behavior는 launch 파일에서 명시적으로 설정 필요.

3. **Modify 서비스 타입이 Start와 동일**: `modify_srv = start_srv`. Goal 구조체 안에 modify 요청이 포함됨.

4. **`feedback_pub_`, `goal_status_pub_` 주석 처리됨**: 구현 파일 140-141라인에서 피드백/상태 퍼블리셔가 비활성화됨. Action Client가 내부 ROS2 메커니즘으로 피드백을 수신.

5. **`behavior_status_` 접근 동시성**: `run_timer_` 콜백과 서비스 콜백이 같은 executor에서 실행되면 race condition 없음. 그러나 MultiThreadedExecutor 사용 시 주의 필요.
