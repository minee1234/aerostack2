# as2_behaviors_payload 패키지 상세 분석

**버전**: 1.1.3
**라이선스**: BSD-3-Clause

---

## 1. 패키지 개요

`as2_behaviors_payload`는 드론에 장착된 **페이로드(탑재 장비) 제어** behavior 2종을 제공한다.

| Behavior | Action 타입 | 설명 |
|----------|------------|------|
| `GripperBehavior` | `as2_msgs/GripperHandler` | 그리퍼(집게) 개폐 제어 |
| `PointGimbalBehavior` | `as2_msgs/PointGimbal` | 짐벌을 3D 지점으로 향하게 제어 |

---

## 2. 파일 구조

```
as2_behaviors_payload/
├── gripper_behavior/
│   ├── include/gripper_behavior/
│   │   ├── gripper_behavior.hpp              # BehaviorServer 서브클래스
│   │   └── gripper_behavior_plugin_base.hpp  # 플러그인 기반 클래스
│   ├── plugins/
│   │   ├── dc_servo/
│   │   │   ├── include/dc_servo/dc_servo.hpp
│   │   │   ├── src/dc_servo.cpp
│   │   │   └── config/plugin_default.yaml
│   │   └── two_fingers/
│   │       ├── include/two_fingers/two_fingers.hpp
│   │       ├── src/two_fingers.cpp
│   │       └── config/plugin_default.yaml
│   ├── src/
│   │   ├── gripper_behavior.cpp
│   │   └── as2_gripper_behavior_node.cpp
│   ├── launch/
│   │   ├── gripper_behavior_launch.py
│   │   ├── dc_servo_gripper.launch.py
│   │   └── two_fingers.launch.py
│   ├── tests/
│   │   └── as2_behaviors_gripper_gtest.cpp
│   └── plugins.xml
├── point_gimbal_behavior/
│   ├── include/point_gimbal_behavior/
│   │   └── point_gimbal_behavior.hpp
│   ├── src/
│   │   ├── point_gimbal_behavior.cpp
│   │   └── point_gimbal_behavior_node.cpp
│   ├── config/
│   │   └── config_default.yaml
│   ├── launch/
│   │   └── point_gimbal_behavior.launch.py
│   └── tests/
│       └── point_gimbal_gtest.cpp
├── CMakeLists.txt
└── package.xml
```

---

## 3. Gripper Behavior

### 3.1 클래스 구조

```
BehaviorServer<as2_msgs::action::GripperHandler>
        └── GripperBehavior
                └── [위임] GripperBase (플러그인)
                        ├── dc_servo::Plugin    (DC 서보 모터)
                        └── two_fingers::Plugin (2-핑거 그리퍼)
```

### 3.2 `GripperBase` 플러그인 인터페이스

**파일**: `gripper_behavior/include/gripper_behavior/gripper_behavior_plugin_base.hpp`

```cpp
namespace gripper_behavior_plugin_base {

class GripperBase {
public:
  void initialize(as2::Node* node_ptr) {
    node_ptr_ = node_ptr;
    ownInit();  // 플러그인 초기화 훅
  }

  // 내부에서 own_* 메서드로 위임
  bool on_activate(Goal goal);
  bool on_modify(Goal goal);
  bool on_deactivate(message);
  bool on_pause(message);
  bool on_resume(message);
  void on_execution_end(state);
  ExecutionStatus on_run(Goal, Feedback&, Result&);

protected:
  // 필수 구현 (순수 가상)
  virtual bool own_activate(as2_msgs::action::GripperHandler::Goal& goal) = 0;
  virtual bool own_modify(as2_msgs::action::GripperHandler::Goal& goal) = 0;
  virtual bool own_deactivate(const std::shared_ptr<std::string>& message) = 0;
  virtual bool own_pause(const std::shared_ptr<std::string>& message) = 0;
  virtual bool own_resume(const std::shared_ptr<std::string>& message) = 0;
  virtual void own_execution_end(const as2_behavior::ExecutionStatus& state) = 0;
  virtual as2_behavior::ExecutionStatus own_run() = 0;

  // 선택 구현
  virtual void ownInit() {}

protected:
  as2::Node* node_ptr_;
  as2_msgs::action::GripperHandler::Goal goal_;
  as2_msgs::action::GripperHandler::Feedback feedback_;
  as2_msgs::action::GripperHandler::Result result_;
};

} // namespace gripper_behavior_plugin_base
```

### 3.3 Action 타입

```
# as2_msgs/action/GripperHandler
# Goal
bool gripper_goal   # true = 닫기(grip), false = 열기(release)
---
# Result
bool gripper_success
---
# Feedback
bool gripper_state   # 현재 그리퍼 상태
```

### 3.4 플러그인: `dc_servo`

```cpp
// DC 서보 모터 기반 그리퍼
class Plugin : public GripperBase {
  void ownInit() {
    // 서보 제어 서비스/토픽 초기화
    // 설정: plugin_default.yaml (서보 핀 번호, PWM 범위 등)
  }

  bool own_activate(Goal& goal) {
    // PWM 신호로 서보 위치 명령
    // goal.gripper_goal: true = close, false = open
    return true;
  }

  ExecutionStatus own_run() {
    // 서보 위치 도달 여부 확인
    // 피드백: feedback_.gripper_state
    return (servo_reached_target) ? SUCCESS : RUNNING;
  }
};
```

### 3.5 플러그인: `two_fingers`

```cpp
// 2-핑거 그리퍼 (보통 전기모터 또는 공압)
class Plugin : public GripperBase {
  // 손가락 위치 제어
  // 파라미터: 완전 개방/폐쇄 위치, 속도
};
```

### 3.6 ROS2 인터페이스 (Gripper)

| 종류 | 이름 | 타입 |
|------|------|------|
| Action Server | `/<ns>/gripper_handler` | `as2_msgs/GripperHandler` |
| Publisher | `/<node>/_behavior/behavior_status` | `as2_msgs/BehaviorStatus` |

---

## 4. PointGimbal Behavior

### 4.1 클래스

**파일**: `point_gimbal_behavior/include/point_gimbal_behavior/point_gimbal_behavior.hpp`

```cpp
namespace point_gimbal_behavior {

struct gimbal_status {
  geometry_msgs::msg::Vector3 orientation;  // roll, pitch, yaw [rad]
};

class PointGimbalBehavior
  : public as2_behavior::BehaviorServer<as2_msgs::action::PointGimbal>
{
  // ── TF 처리 ─────────────────────────────────────────────────
  as2::tf::TfHandler tf_handler_;
  std::string base_link_frame_id_;
  std::string gimbal_name_;
  std::string gimbal_base_frame_id_;  // 짐벌 베이스 프레임
  std::string gimbal_frame_id_;       // 짐벌 현재 프레임

  // ── 짐벌 한계 ────────────────────────────────────────────────
  double gimbal_roll_min_,  gimbal_roll_max_;
  double gimbal_pitch_min_, gimbal_pitch_max_;
  double gimbal_yaw_min_,   gimbal_yaw_max_;

  // ── 목표 및 현재 상태 ─────────────────────────────────────────
  geometry_msgs::msg::PointStamped desired_goal_position_;   // gimbal_base_frame에서
  geometry_msgs::msg::PointStamped current_goal_position_;   // gimbal_frame에서
  geometry_msgs::msg::Vector3Stamped gimbal_angles_current_; // 피드백용
  geometry_msgs::msg::Vector3Stamped gimbal_angles_offset_;  // ENU → base_link 오프셋

  // ── 타임아웃 ────────────────────────────────────────────────
  rclcpp::Time goal_init_time_;
  rclcpp::Duration behavior_timeout_;
  double gimbal_threshold_;  // 목표 도달 허용 오차 [rad]

  // ── 제어 명령 ────────────────────────────────────────────────
  as2_msgs::msg::GimbalControl gimbal_control_msg_;
  rclcpp::Publisher<as2_msgs::msg::GimbalControl>::SharedPtr gimbal_control_pub_;
};

} // namespace point_gimbal_behavior
```

### 4.2 Action 타입

```
# as2_msgs/action/PointGimbal
# Goal
geometry_msgs/PointStamped goal_point   # 짐벌이 향할 3D 지점 (임의 프레임)
---
# Result
bool success
---
# Feedback
geometry_msgs/Vector3 gimbal_angles     # 현재 짐벌 각도 [roll, pitch, yaw]
```

### 4.3 동작 흐름

```
on_activate():
  1. goal.goal_point를 gimbal_base_frame으로 TF 변환
     → desired_goal_position_
  2. goal_init_time_ = now()
  3. update_gimbal_state() → 현재 짐벌 각도 취득

on_run():
  1. update_gimbal_state()  ← TF에서 현재 짐벌 포즈 취득
  2. (타임아웃 체크) now() - goal_init_time_ > behavior_timeout_
     → FAILURE
  3. 목표 지점까지의 roll/pitch/yaw 계산
     - 벡터: desired_goal_position_ - gimbal_current_pos
     - yaw   = atan2(dy, dx)
     - pitch = atan2(-dz, sqrt(dx²+dy²))
  4. check_gimbal_limits(roll, pitch, yaw)
     → 한계 초과 시 클리핑
  5. GimbalControl 퍼블리시
  6. point_check_finished()
     - |computed_angle - current_angle| < gimbal_threshold_ → SUCCESS

on_deactivate():
  현재 짐벌 각도 유지 (추가 명령 없음)
```

### 4.4 `GimbalControl` 메시지

```
as2_msgs/msg/GimbalControl:
  std_msgs/Header header
  float64 roll   # [rad]
  float64 pitch  # [rad]
  float64 yaw    # [rad]
  uint8   control_mode  # POSITION / VELOCITY / ...
```

### 4.5 ROS2 인터페이스 (PointGimbal)

| 종류 | 이름 | 타입 |
|------|------|------|
| Action Server | `/<ns>/point_gimbal` | `as2_msgs/PointGimbal` |
| Publisher | `/<ns>/gimbal_control` | `as2_msgs/GimbalControl` |
| Publisher | `/<node>/_behavior/behavior_status` | `as2_msgs/BehaviorStatus` |

### 4.6 설정 파라미터

**파일**: `point_gimbal_behavior/config/config_default.yaml`

```yaml
gimbal_name: "gimbal"
gimbal_base_frame_id: "gimbal_base"
gimbal_frame_id: "gimbal"
gimbal_threshold: 0.02          # [rad] 목표 도달 허용 오차
behavior_timeout: 10.0          # [s] 타임아웃
gimbal_roll_min:  -0.5          # [rad]
gimbal_roll_max:   0.5
gimbal_pitch_min: -1.57         # ≈ -π/2
gimbal_pitch_max:  1.57
gimbal_yaw_min:   -1.57
gimbal_yaw_max:    1.57
run_frequency: 10.0
```

---

## 5. 알려진 이슈 및 주의사항

### Gripper Behavior

1. **`on_activate()` 버그**: `gripper_behavior_plugin_base.hpp:70`
   ```cpp
   if (own_activate(goal_candidate)) {
     goal_ = goal_candidate;
     return true;
   }
   return true;  // ← 항상 true! own_activate 실패해도 성공으로 처리
   ```

2. **플러그인 Modify/Pause/Resume**: 플러그인이 구현해야 하며, 기본 구현 없음. 미구현 시 런타임 오류 가능.

### PointGimbal Behavior

1. **TF 프레임 의존성**: 짐벌 프레임이 TF 트리에 없으면 `update_gimbal_state()` 실패. 짐벌 드라이버가 반드시 TF를 브로드캐스트해야 함.

2. **ENU → base_link 오프셋**: `gimbal_angles_offset_`이 수동으로 설정되어야 함. 자동 보정 없음.

3. **타임아웃 동작**: 타임아웃 발생 시 짐벌이 마지막 명령 각도를 유지. 안전한 기본값으로 돌아가는 로직 없음.
