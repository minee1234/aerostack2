# as2_behaviors_motion 패키지 상세 분석

**버전**: 1.1.3
**라이선스**: BSD-3-Clause

---

## 1. 패키지 개요

`as2_behaviors_motion`은 드론의 **기동(Motion) 관련 behavior** 5종을 구현하는 패키지다.
각 behavior는 `as2_behavior::BehaviorServer<actionT>`를 상속하며, `pluginlib`를 통해 런타임에 알고리즘 플러그인을 교체할 수 있다.

### 포함 Behavior 목록

| Behavior | Action 타입 | 플러그인 수 |
|----------|------------|-----------|
| `TakeoffBehavior` | `as2_msgs::action::Takeoff` | 4 |
| `LandBehavior` | `as2_msgs::action::Land` | 3 |
| `GoToBehavior` | `as2_msgs::action::GoToWaypoint` | 2 |
| `FollowPathBehavior` | `as2_msgs::action::FollowPath` | 2 |
| `FollowReferenceBehavior` | `as2_msgs::action::FollowReference` | 없음 (단일 구현) |

---

## 2. 파일 구조

```
as2_behaviors_motion/
├── takeoff_behavior/
│   ├── include/takeoff_behavior/
│   │   ├── takeoff_behavior.hpp    # BehaviorServer 서브클래스 선언
│   │   └── takeoff_base.hpp        # 플러그인 기반 클래스
│   ├── plugins/
│   │   ├── takeoff_plugin_platform.cpp
│   │   ├── takeoff_plugin_position.cpp
│   │   ├── takeoff_plugin_speed.cpp
│   │   └── takeoff_plugin_trajectory.cpp
│   ├── src/
│   │   ├── takeoff_behavior.cpp         # 구현
│   │   └── takeoff_behavior_node.cpp    # main 진입점
│   └── config/config_default.yaml
├── land_behavior/
│   ├── include/land_behavior/
│   │   ├── land_behavior.hpp
│   │   └── land_base.hpp
│   ├── plugins/
│   │   ├── land_plugin_platform.cpp
│   │   ├── land_plugin_speed.cpp
│   │   └── land_plugin_trajectory.cpp
│   └── ...
├── go_to_behavior/
│   ├── include/go_to_behavior/
│   │   ├── go_to_behavior.hpp
│   │   └── go_to_base.hpp
│   ├── plugins/
│   │   ├── go_to_plugin_position.cpp
│   │   └── go_to_plugin_trajectory.cpp
│   └── config/config_default.yaml
├── follow_path_behavior/
│   ├── include/follow_path_behavior/
│   │   ├── follow_path_behavior.hpp
│   │   └── follow_path_base.hpp
│   ├── plugins/
│   │   ├── follow_path_plugin_position.cpp
│   │   └── follow_path_plugin_trajectory.cpp
│   └── config/config_default.yaml
├── follow_reference_behavior/
│   ├── include/follow_reference_behavior/
│   │   └── follow_reference_behavior.hpp
│   ├── src/
│   │   ├── follow_reference_behavior.cpp
│   │   └── follow_reference_behavior_node.cpp
│   └── config/config_default.yaml
├── launch/
│   ├── motion_behaviors_launch.py
│   ├── takeoff_behavior_launch.py
│   ├── land_behavior_launch.py
│   ├── go_to_behavior_launch.py
│   ├── follow_path_behavior_launch.py
│   ├── follow_reference_behavior_launch.py
│   └── composable_motion_behaviors.launch.py
├── plugins.xml                          # pluginlib 등록 XML
├── CMakeLists.txt
└── package.xml
```

---

## 3. 의존성

```xml
<depend>rclcpp</depend>
<depend>rclcpp_action</depend>
<depend>pluginlib</depend>
<depend>as2_behavior</depend>
<depend>as2_core</depend>
<depend>as2_msgs</depend>
<depend>as2_motion_reference_handlers</depend>
<depend>geometry_msgs</depend>
<depend>Eigen3</depend>
<depend>tf2_ros</depend>
```

---

## 4. Takeoff Behavior

### 4.1 클래스 구조

```
BehaviorServer<as2_msgs::action::Takeoff>
        │
        └── TakeoffBehavior
                │  (위임)
                └── TakeoffBase (플러그인 인터페이스)
                        ├── takeoff_plugin_platform::Plugin
                        ├── takeoff_plugin_speed::Plugin
                        ├── takeoff_plugin_position::Plugin
                        └── takeoff_plugin_trajectory::Plugin
```

### 4.2 `TakeoffBehavior` 클래스

**파일**: `takeoff_behavior/include/takeoff_behavior/takeoff_behavior.hpp`

```cpp
class TakeoffBehavior : public as2_behavior::BehaviorServer<as2_msgs::action::Takeoff>
{
  using PSME = as2_msgs::msg::PlatformStateMachineEvent;

  // 플러그인 시스템
  std::shared_ptr<pluginlib::ClassLoader<takeoff_base::TakeoffBase>> loader_;
  std::shared_ptr<takeoff_base::TakeoffBase> takeoff_plugin_;

  // TF 처리
  std::shared_ptr<as2::tf::TfHandler> tf_handler_;
  std::string base_link_frame_id_;

  // ROS2 인터페이스
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  as2::SynchronousServiceClient<as2_msgs::srv::SetPlatformStateMachineEvent>::SharedPtr platform_cli_;
};
```

### 4.3 생성자 초기화 순서

```
1. BehaviorServer("takeoff") 생성
2. 파라미터 선언: plugin_name, takeoff_height, takeoff_speed, takeoff_threshold
3. pluginlib::ClassLoader 생성 (패키지: "as2_behaviors_motion", 기반: TakeoffBase)
4. TfHandler 생성
5. 플러그인 인스턴스 생성: plugin_name + "::Plugin"
6. TakeoffBase::initialize(node, tf_handler, params) 호출
7. SynchronousServiceClient 생성 (SetPlatformStateMachineEvent)
8. TwistStamped 구독 (self_localization/twist)
```

### 4.4 `on_activate()` 동작 (process_goal 포함)

```cpp
bool TakeoffBehavior::process_goal(goal, new_goal) {
  // 1. 높이 음수 검증
  if (goal->takeoff_height < 0.0f) return false;

  // 2. 속도 기본값 적용
  new_goal.takeoff_speed = (goal->takeoff_speed != 0.0f) ?
    goal->takeoff_speed : params["takeoff_speed"];

  // 3. FSM에 TAKE_OFF 이벤트 전송 (동기 서비스 호출)
  if (!sendEventFSME(PSME::TAKE_OFF)) return false;

  return true;
}
```

### 4.5 `on_execution_end()` 동작

```cpp
void TakeoffBehavior::on_execution_end(const ExecutionStatus& state) {
  if (state == SUCCESS)
    sendEventFSME(PSME::TOOK_OFF);   // FSM → FLYING
  else
    sendEventFSME(PSME::EMERGENCY);  // FSM → EMERGENCY
}
```

### 4.6 `TakeoffBase` 플러그인 인터페이스

**파일**: `takeoff_behavior/include/takeoff_behavior/takeoff_base.hpp`

```cpp
struct takeoff_plugin_params {
  double takeoff_height;
  double takeoff_speed;
  double takeoff_threshold;
};

class TakeoffBase {
protected:
  // 필수 구현 (순수 가상)
  virtual bool own_activate(as2_msgs::action::Takeoff::Goal& goal) = 0;
  virtual bool own_deactivate(const std::shared_ptr<std::string>& message) = 0;
  virtual void own_execution_end(const as2_behavior::ExecutionStatus& state) = 0;
  virtual as2_behavior::ExecutionStatus own_run() = 0;

  // 선택 구현 (기본: false 반환 + 로그)
  virtual bool own_modify(as2_msgs::action::Takeoff::Goal& goal);
  virtual bool own_pause(const std::shared_ptr<std::string>& message);
  virtual bool own_resume(const std::shared_ptr<std::string>& message);
  virtual void ownInit() {}

  // 내부 제공 유틸리티
  void sendHover();  // HoverMotion 핸들러 호출

protected:
  as2::Node* node_ptr_;
  as2_msgs::action::Takeoff::Goal goal_;
  as2_msgs::action::Takeoff::Feedback feedback_;
  as2_msgs::action::Takeoff::Result result_;
  takeoff_plugin_params params_;
  geometry_msgs::msg::PoseStamped actual_pose_;
  bool localization_flag_;  // state_callback 수신 여부

  // processGoal()에서 localization_flag_ 체크
  // → false이면 "no localization" 에러로 reject
};
```

### 4.7 플러그인 상세 분석

#### `takeoff_plugin_platform` (플랫폼 네이티브 이륙)
```cpp
void ownInit() {
  platform_takeoff_cli_ = node_ptr_->create_client<std_srvs::srv::SetBool>(
    as2_names::services::platform::takeoff);
}

bool own_activate(Goal& goal) {
  // platform/takeoff 서비스 비동기 호출
  platform_takeoff_future_ = platform_takeoff_cli_->async_send_request(...).share();
  return true;
}

ExecutionStatus own_run() {
  // future 완료 여부 확인 (논블로킹)
  if (future_ready) return future_result->success ? SUCCESS : FAILURE;
  return RUNNING;
}
// 주의: own_deactivate() → "Takeoff can not be cancelled", false 반환
```

#### `takeoff_plugin_speed` (속도 명령 이륙)
```cpp
ExecutionStatus own_run() {
  if (checkGoalCondition()) {
    // |goal_height - actual_height| < takeoff_threshold
    return SUCCESS;
  }
  // SpeedMotion: z 방향 속도 명령 송출
  speed_motion_handler_->sendSpeedCommandWithYawSpeed(
    "earth", 0.0, 0.0, goal_.takeoff_speed, 0.0);
  return RUNNING;
}

bool own_deactivate(msg) {
  sendHover();  // 취소 시 호버
  return true;
}
```

### 4.8 ROS2 인터페이스 (Takeoff)

| 종류 | 이름 | 타입 |
|------|------|------|
| Action Server | `/<ns>/TakeoffBehavior` | `as2_msgs/Takeoff` |
| Subscription | `/<ns>/self_localization/twist` | `geometry_msgs/TwistStamped` |
| Service Client | `/<ns>/state_machine_event` | `as2_msgs/srv/SetPlatformStateMachineEvent` |
| Service Server | `/<node>/_behavior/pause` | `std_srvs/Trigger` |
| Service Server | `/<node>/_behavior/resume` | `std_srvs/Trigger` |
| Service Server | `/<node>/_behavior/stop` | `std_srvs/Trigger` |
| Publisher | `/<node>/_behavior/behavior_status` | `as2_msgs/BehaviorStatus` |

---

## 5. Land Behavior

### 5.1 구조 (Takeoff와 대칭)

```
BehaviorServer<as2_msgs::action::Land>
        └── LandBehavior
                └── LandBase (플러그인 인터페이스)
                        ├── land_plugin_platform::Plugin
                        ├── land_plugin_speed::Plugin
                        └── land_plugin_trajectory::Plugin
```

### 5.2 FSM 이벤트 처리

```
on_activate():  sendEventFSME(PSME::LAND)     → FSM: FLYING → LANDING
on_execution_end(SUCCESS):  sendEventFSME(PSME::LANDED)   → FSM: LANDING → LANDED
                            sendEventFSME(PSME::DISARM)    → FSM: LANDED → DISARMED
on_execution_end(!SUCCESS): sendEventFSME(PSME::EMERGENCY) → FSM → EMERGENCY
```

### 5.3 Feedback

```cpp
// LandBase::state_callback()에서 업데이트
feedback_.actual_land_height  = actual_pose_.pose.position.z;
feedback_.actual_land_speed   = twist_msg.twist.linear.z;
```

---

## 6. GoTo Behavior

### 6.1 클래스

```cpp
class GoToBehavior : public as2_behavior::BehaviorServer<as2_msgs::action::GoToWaypoint>
{
  std::shared_ptr<go_to_base::GoToBase> go_to_plugin_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  rclcpp::Subscription<as2_msgs::msg::PlatformInfo>::SharedPtr platform_info_sub_;
};
```

### 6.2 GoToBase 플러그인 파라미터

```cpp
struct go_to_plugin_params {
  double go_to_speed;          // 기본: 0.5 m/s
  double go_to_threshold;      // 기본: 0.2 m
  double tf_timeout_threshold; // 기본: 0.05 s
};
```

### 6.3 플러그인

| 플러그인 | 전략 |
|----------|------|
| `go_to_plugin_position` | PositionMotion 핸들러로 목표 위치 명령 |
| `go_to_plugin_trajectory` | TrajectoryMotion 핸들러로 부드러운 이동 |

### 6.4 목표 좌표 변환

```cpp
// GoToBase::on_activate()에서:
// 목표 PoseStamped를 tf_handler_로 "earth" 프레임으로 변환
auto goal_earth = tf_handler_->convert(goal.target_pose, "earth");
```

### 6.5 Feedback

```cpp
feedback_.actual_speed             = current_speed;
feedback_.actual_distance_to_goal  = distance(current_pos, goal_pos);
```

---

## 7. FollowPath Behavior

### 7.1 목표 구조

```
as2_msgs::action::FollowPath::Goal:
  - header
  - path: PoseWithID[]  (각 waypoint에 ID 존재)
  - max_speed: float
  - yaw_mode: YawMode
```

### 7.2 Feedback

```cpp
feedback_.actual_speed;
feedback_.actual_distance_to_next_waypoint;
feedback_.remaining_waypoints;
feedback_.next_waypoint_id;
```

### 7.3 플러그인

| 플러그인 | 전략 |
|----------|------|
| `follow_path_plugin_position` | 순차적 GoTo 방식으로 각 waypoint 이동 |
| `follow_path_plugin_trajectory` | 전체 경로를 trajectory 생성기로 실행 |

---

## 8. FollowReference Behavior

### 8.1 특징 (플러그인 없음)

플러그인 시스템을 사용하지 않고 단일 구현으로 작성된 behavior.
외부에서 지속적으로 참조 위치/속도를 제공하는 경우 사용.

```cpp
class FollowReferenceBehavior
  : public as2_behavior::BehaviorServer<as2_msgs::action::FollowReference>
{
  as2_msgs::action::FollowReference::Goal goal_;

  // 두 가지 Motion Handler
  std::shared_ptr<as2::motionReferenceHandlers::PositionMotion> position_motion_handler_;
  std::shared_ptr<as2::motionReferenceHandlers::HoverMotion> hover_motion_handler_;

  std::shared_ptr<as2::tf::TfHandler> tf_handler_;
  geometry_msgs::msg::PoseStamped actual_pose_;
  geometry_msgs::msg::TwistStamped actual_twist;
  int platform_state_;
  bool localization_flag_;
};
```

### 8.2 Yaw 계산 모드

```cpp
bool computeYaw(uint8_t yaw_mode, const Point& target, const Point& actual, float& yaw) {
  switch (yaw_mode) {
    case YawMode::PATH_FACING:
      yaw = atan2(target.y - actual.y, target.x - actual.x);
      break;
    case YawMode::FIXED_YAW:
      yaw = goal_.yaw_angle;
      break;
    case YawMode::KEEP_YAW:
      yaw = getActualYaw();
      break;
  }
}
```

### 8.3 `on_modify()` 지원

FollowReferenceBehavior는 `on_modify()`를 구현하여 실행 중 목표 참조를 변경할 수 있다.
(다른 behavior는 대부분 modify 미지원)

---

## 9. 플러그인 등록 (`plugins.xml`)

```xml
<library path="takeoff_plugin_platform">
  <class name="takeoff_plugin_platform::Plugin" type="takeoff_plugin_platform::Plugin"
         base_class_type="takeoff_base::TakeoffBase"/>
</library>
<library path="takeoff_plugin_speed">
  <class name="takeoff_plugin_speed::Plugin" .../>
</library>
<!-- ... 기타 플러그인 -->
```

---

## 10. 설정 파일

### `takeoff_behavior/config/config_default.yaml`
```yaml
takeoff_height: 1.0       # [m]
takeoff_speed: 0.5        # [m/s]
takeoff_threshold: 0.05   # [m] 목표 도달 판정 거리
plugin_name: "takeoff_plugin_speed"
run_frequency: 10.0       # [Hz]
```

### `go_to_behavior/config/config_default.yaml`
```yaml
go_to_speed: 0.5          # [m/s]
go_to_threshold: 0.2      # [m]
tf_timeout_threshold: 0.05 # [s]
plugin_name: "go_to_plugin_position"
run_frequency: 10.0
```

---

## 11. Launch 시스템

### 단일 behavior 실행
```bash
ros2 launch as2_behaviors_motion takeoff_behavior_launch.py \
  namespace:=drone0 \
  plugin_name:=takeoff_plugin_speed
```

### 전체 motion behavior 실행
```bash
ros2 launch as2_behaviors_motion motion_behaviors_launch.py \
  namespace:=drone0
```

### Composable Node (컨테이너 공유)
```bash
ros2 launch as2_behaviors_motion composable_motion_behaviors.launch.py \
  namespace:=drone0 \
  container_name:=motion_container
```

---

## 12. 클래스 계층 요약

```
as2::Node
  └── as2_behavior::BehaviorServer<ActionT>
        ├── TakeoffBehavior
        │     └── [위임] TakeoffBase
        │           ├── takeoff_plugin_platform::Plugin
        │           ├── takeoff_plugin_speed::Plugin
        │           ├── takeoff_plugin_position::Plugin
        │           └── takeoff_plugin_trajectory::Plugin
        ├── LandBehavior
        │     └── [위임] LandBase
        │           ├── land_plugin_platform::Plugin
        │           ├── land_plugin_speed::Plugin
        │           └── land_plugin_trajectory::Plugin
        ├── GoToBehavior
        │     └── [위임] GoToBase
        │           ├── go_to_plugin_position::Plugin
        │           └── go_to_plugin_trajectory::Plugin
        ├── FollowPathBehavior
        │     └── [위임] FollowPathBase
        │           ├── follow_path_plugin_position::Plugin
        │           └── follow_path_plugin_trajectory::Plugin
        └── FollowReferenceBehavior (단일 구현, 플러그인 없음)
```

---

## 13. 알려진 이슈 및 주의사항

1. **`localization_flag_` 초기 검사**: `processGoal()`에서 localization_flag_가 false이면 behavior reject. twist 토픽 수신 전에 goal을 보내면 실패함.

2. **`takeoff_plugin_platform::own_deactivate()` 취소 불가**: "Takeoff can not be cancelled" 로그 출력 후 `false` 반환. 이미 플랫폼 명령이 전송된 상태.

3. **TF 프레임 hardcode**: `state_callback`에서 `"earth"` 프레임으로 TF 변환 강제. 다른 좌표계 사용 시 수정 필요.

4. **FSM 이벤트 동기 호출**: `sendEventFSME()`는 `SynchronousServiceClient`를 사용하여 최대 3초 블로킹. on_activate 내에서 타임아웃 발생 가능.

5. **FollowPath 플러그인의 경로 변환 미지원**: on_modify() 기본 구현이 false 반환. 경로 수정 불가.
