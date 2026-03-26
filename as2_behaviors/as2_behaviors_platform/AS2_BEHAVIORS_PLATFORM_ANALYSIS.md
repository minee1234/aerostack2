# as2_behaviors_platform 패키지 상세 분석

**버전**: 1.1.3
**라이선스**: BSD-3-Clause

---

## 1. 패키지 개요

`as2_behaviors_platform`은 드론 플랫폼의 **저수준 상태 제어** behavior 2종을 제공하는 패키지다.
Arming(모터 활성화)과 Offboard 모드 전환을 ROS2 Action 인터페이스로 노출시켜, 상위 시스템(BehaviorTree, 미션 플래너)이 일관된 방식으로 호출할 수 있게 한다.

### 특징
- **Instant Behavior**: Pause/Resume/Modify/Stop을 지원하지 않는 단발성 behavior
- 플러그인 없이 단일 헤더 파일에 전체 구현
- 비동기 서비스 호출 + Future 폴링 방식

---

## 2. 파일 구조

```
as2_behaviors_platform/
├── include/as2_behaviors_platform/
│   ├── set_arming_state_behavior.hpp      # ARM/DISARM behavior (헤더 전용 구현)
│   └── set_offboard_mode_behavior.hpp     # Offboard 모드 behavior (헤더 전용 구현)
├── src/
│   ├── set_arming_state_behavior_main.cpp   # main() 진입점
│   └── set_offboard_mode_behavior_main.cpp  # main() 진입점
├── launch/
│   ├── as2_platform_behaviors_launch.py     # 두 behavior 동시 실행
│   └── test.launch.py
├── tests/
│   ├── CMakeLists.txt
│   └── as2_behaviors_platform_gtest.cpp
├── CMakeLists.txt
└── package.xml
```

---

## 3. 의존성

```xml
<depend>rclcpp</depend>
<depend>rclcpp_action</depend>
<depend>as2_behavior</depend>
<depend>as2_msgs</depend>
<depend>std_srvs</depend>
```

---

## 4. `SetArmingStateBehavior`

**파일**: `include/as2_behaviors_platform/set_arming_state_behavior.hpp`

### 클래스 전체 구현

```cpp
class SetArmingStateBehavior
  : public as2_behavior::BehaviorServer<as2_msgs::action::SetArmingState>
{
public:
  SetArmingStateBehavior()
  : as2_behavior::BehaviorServer<as2_msgs::action::SetArmingState>("set_arming_state")
  {
    client_ = this->create_client<std_srvs::srv::SetBool>("set_arming_state");
  }

  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future_;

  // ── on_activate ──────────────────────────────────────────────
  bool on_activate(std::shared_ptr<const as2_msgs::action::SetArmingState::Goal> goal) override
  {
    auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
    req->data = goal->request;  // true = ARM, false = DISARM

    if (!client_->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_INFO(get_logger(), "service not available");
      return false;
    }
    future_ = client_->async_send_request(req).share();
    if (!future_.valid()) return false;
    return true;
  }

  // ── on_run ───────────────────────────────────────────────────
  ExecutionStatus on_run(const Goal&, Feedback&, Result& result_msg) override
  {
    if (future_.valid() &&
        future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    {
      auto result = future_.get();
      result_msg->success = true;  // 주의: 항상 true (버그 가능성)
      return result->success ? ExecutionStatus::SUCCESS : ExecutionStatus::FAILURE;
    }
    return ExecutionStatus::RUNNING;
  }

  // ── 지원하지 않는 동작들 ──────────────────────────────────────
  bool on_modify(...)  override { RCLCPP_WARN(..., "Cannot modify a service request"); return false; }
  bool on_deactivate(...) override { *message = "Unable to deactivate InstantBehavior"; return false; }
  bool on_pause(...)   override { *message = "Unable to pause InstantBehavior"; return false; }
  bool on_resume(...)  override { *message = "Unable to resume InstantBehavior"; return false; }
  void on_execution_end(...) override {}
};
```

---

## 5. `SetOffboardModeBehavior`

**파일**: `include/as2_behaviors_platform/set_offboard_mode_behavior.hpp`

### 구조 (SetArmingStateBehavior와 동일한 패턴)

```cpp
class SetOffboardModeBehavior
  : public as2_behavior::BehaviorServer<as2_msgs::action::SetOffboardMode>
{
public:
  SetOffboardModeBehavior()
  : as2_behavior::BehaviorServer<as2_msgs::action::SetOffboardMode>("set_offboard_mode")
  {
    client_ = this->create_client<std_srvs::srv::SetBool>("set_offboard_mode");
  }

  // on_activate(): goal->request (true = offboard ON, false = OFF)
  // on_run(): future 폴링
  // 모두 SetArmingStateBehavior와 동일한 패턴
};
```

---

## 6. ROS2 인터페이스

### SetArmingStateBehavior

| 종류 | 이름 | 타입 |
|------|------|------|
| Action Server | `/<ns>/set_arming_state` | `as2_msgs/SetArmingState` |
| Service Client | `/<node>/set_arming_state` | `std_srvs/SetBool` |
| Publisher | `/<node>/_behavior/behavior_status` | `as2_msgs/BehaviorStatus` |

### SetOffboardModeBehavior

| 종류 | 이름 | 타입 |
|------|------|------|
| Action Server | `/<ns>/set_offboard_mode` | `as2_msgs/SetOffboardMode` |
| Service Client | `/<node>/set_offboard_mode` | `std_srvs/SetBool` |
| Publisher | `/<node>/_behavior/behavior_status` | `as2_msgs/BehaviorStatus` |

---

## 7. Action 타입 정의

### `as2_msgs/action/SetArmingState`
```
# Goal
bool request    # true = ARM, false = DISARM
---
# Result
bool success
---
# Feedback
(없음)
```

### `as2_msgs/action/SetOffboardMode`
```
# Goal
bool request    # true = Offboard ON, false = Offboard OFF
---
# Result
bool success
---
# Feedback
(없음)
```

---

## 8. 실행 흐름

```
[Action 클라이언트 Goal 전송]
        │
        ▼
on_activate()
  ├── wait_for_service(5s)  ─── 타임아웃 → false → REJECT
  └── async_send_request()  ─── 성공 → ACCEPT
        │
        ▼
[run_timer_ 10Hz 폴링]
on_run()
  ├── future_.wait_for(0s) == ready? ─── 아니오 → RUNNING (계속 폴링)
  └── 예 → result->success?
            ├── true  → ExecutionStatus::SUCCESS
            └── false → ExecutionStatus::FAILURE
```

---

## 9. Launch 파일

```python
# as2_platform_behaviors_launch.py
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='as2_behaviors_platform',
            executable='set_arming_state_behavior',
            namespace=LaunchConfiguration('namespace'),
        ),
        Node(
            package='as2_behaviors_platform',
            executable='set_offboard_mode_behavior',
            namespace=LaunchConfiguration('namespace'),
        ),
    ])
```

---

## 10. 알려진 이슈 및 주의사항

1. **`result_msg->success = true` 버그 (line 113)**:
   ```cpp
   result_msg->success = true;  // ← 항상 true!
   return result->success ? SUCCESS : FAILURE;
   ```
   서비스가 실패(`success=false`)를 반환해도 `result_msg->success`는 항상 `true`로 설정됨. Action 결과의 `success` 필드가 실제 서비스 결과를 반영하지 못하는 버그.

2. **Offboard 전환 타이밍**: 플랫폼(PX4, ArduPilot)이 Offboard 수락 조건(setpoint 스트리밍 선행)을 요구하는 경우, `SetOffboardModeBehavior` 단독으로는 부족. 플랫폼 드라이버가 자동 스트리밍을 처리해야 함.

3. **중단 불가**: `on_deactivate()` false 반환. ARM/Offboard 서비스는 한 번 전송되면 취소 불가. 플랫폼 드라이버가 결정.

4. **서비스 이름 하드코드**: 생성자에서 `"set_arming_state"`, `"set_offboard_mode"` 고정. 네임스페이스 적용은 ROS2 노드 설정으로 처리됨.
