# as2_behaviors_trajectory_generation 패키지 상세 분석

**버전**: 1.1.3
**라이선스**: BSD-3-Clause

---

## 1. 패키지 개요

`as2_behaviors_trajectory_generation`은 드론의 **다항식 궤적(Polynomial Trajectory) 생성** behavior를 제공하는 패키지다.
외부 라이브러리 `dynamic_trajectory_generator`와 통합하여 부드럽고 동역학적으로 가능한 궤적을 생성하고, 실시간으로 컨트롤러에 궤적 설정값(TrajectorySetpoints)을 전송한다.

### 핵심 특징
- `dynamic_traj_generator::DynamicTrajectory` 라이브러리 기반 다항식 궤적 생성
- 실행 중 waypoint 동적 수정 지원 (`on_modify()`)
- 다중 Yaw 모드 지원 (경로 추종, 고정, 토픽 구독)
- Map-to-Odom 프레임 오프셋 실시간 보정
- 디버그 시각화 (RViz Marker, nav_msgs/Path)

---

## 2. 파일 구조

```
as2_behaviors_trajectory_generation/
├── generate_polynomial_trajectory_behavior/
│   ├── include/generate_polynomial_trajectory_behavior/
│   │   └── generate_polynomial_trajectory_behavior.hpp  # 전체 클래스 선언
│   ├── src/
│   │   ├── generate_polynomial_trajectory_behavior.cpp     # 구현
│   │   └── generate_polynomial_trajectory_behavior_node.cpp # main
│   ├── config/
│   │   └── config_default.yaml          # 기본 파라미터
│   ├── launch/
│   │   ├── generate_polynomial_trajectory_behavior_launch.py
│   │   └── composable_generate_polynomial_trajectory_behavior.launch.py
│   ├── tests/
│   │   ├── CMakeLists.txt
│   │   ├── behavior_test.py
│   │   └── generate_polynomial_trajectory_behavior_gtest.cpp
│   └── CMakeLists.txt
├── CMakeLists.txt
└── package.xml
```

---

## 3. 의존성

```xml
<depend>rclcpp</depend>
<depend>as2_behavior</depend>
<depend>as2_core</depend>
<depend>as2_msgs</depend>
<depend>as2_motion_reference_handlers</depend>
<depend>dynamic_trajectory_generator</depend>  <!-- 외부 궤적 생성 라이브러리 -->
<depend>Eigen3</depend>
<depend>geometry_msgs</depend>
<depend>nav_msgs</depend>
<depend>visualization_msgs</depend>
<depend>std_msgs</depend>
<depend>tf2_ros</depend>
```

---

## 4. 핵심 클래스: `DynamicPolynomialTrajectoryGenerator`

**파일**: `generate_polynomial_trajectory_behavior.hpp:80`

```cpp
class DynamicPolynomialTrajectoryGenerator
  : public as2_behavior::BehaviorServer<as2_msgs::action::GeneratePolynomialTrajectory>
{
  // ── 구독자 ───────────────────────────────────────────────────
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr state_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr yaw_sub_;
  rclcpp::Subscription<as2_msgs::msg::PoseStampedWithIDArray>::SharedPtr mod_waypoint_sub_;

  // ── 타이머 ───────────────────────────────────────────────────
  rclcpp::TimerBase::SharedPtr timer_update_frame_;  // 프레임 오프셋 갱신
  double frequency_update_frame_ = 0.0;

  // ── 궤적 생성기 ──────────────────────────────────────────────
  std::shared_ptr<dynamic_traj_generator::DynamicTrajectory> trajectory_generator_;

  // ── Motion Handlers ──────────────────────────────────────────
  as2::motionReferenceHandlers::HoverMotion hover_motion_handler_;
  as2::motionReferenceHandlers::TrajectoryMotion trajectory_motion_handler_;
  as2::tf::TfHandler tf_handler_;

  // ── 파라미터 ─────────────────────────────────────────────────
  std::string base_link_frame_id_;   // 예: "drone0/base_link"
  std::string desired_frame_id_;     // 목표 좌표계
  std::string map_frame_id_;         // "map"
  int sampling_n_ = 1;               // 궤적 샘플링 수
  double sampling_dt_ = 0.0;         // 샘플 간 시간 간격
  int path_length_ = 0;              // 0 = 전체 궤적
  float yaw_threshold_ = 0;          // Yaw 계산 활성화 거리 임계값
  float transform_threshold_ = 1.0;  // 프레임 오프셋 갱신 임계값 [m]
  double yaw_speed_threshold_ = 2.0; // 최대 Yaw 속도 [rad/s]
  double wp_close_threshold_ = 0.0;  // Waypoint 지평선 [s]

  // ── Yaw 처리 ────────────────────────────────────────────────
  as2_msgs::msg::YawMode yaw_mode_;
  bool has_yaw_from_topic_ = false;
  float yaw_from_topic_ = 0.0f;
  float init_yaw_angle_ = 0.0f;

  // ── 상태 변수 ────────────────────────────────────────────────
  Eigen::Vector3d current_position_;
  geometry_msgs::msg::TransformStamped current_map_to_odom_transform_;
  geometry_msgs::msg::TransformStamped last_map_to_odom_transform_;
  double current_yaw_;

  // ── 시간 추적 ────────────────────────────────────────────────
  rclcpp::Duration eval_time_ = rclcpp::Duration(0, 0);
  rclcpp::Time time_zero_;
  bool first_run_ = false;
  bool has_odom_ = false;
  dynamic_traj_generator::DynamicWaypoint::Deque waypoints_to_set_;

  // ── 디버그 퍼블리셔 ─────────────────────────────────────────
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr debug_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_waypoints_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr debug_ref_point_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr debug_end_ref_point_pub_;
  bool enable_debug_ = false;
  std::thread plot_thread_;  // 별도 스레드에서 시각화
};
```

---

## 5. Action 타입

### `as2_msgs::action::GeneratePolynomialTrajectory`

```
# Goal
as2_msgs/msg/PoseStampedWithIDArray waypoints   # 경유점 배열
float32 max_speed                               # 최대 속도 [m/s]
as2_msgs/msg/YawMode yaw_mode                  # Yaw 제어 모드
---
# Result
bool trajectory_generator_success
---
# Feedback
float32 actual_speed
float32 remaining_trajectory_time              # 남은 궤적 시간 [s]
```

---

## 6. Behavior 메서드 구현

### 6.1 `on_activate()`

```
1. goalToDynamicWaypoint() → waypoints_to_set_ 변환
2. updateFrame() → map-to-odom TF 취득
3. trajectory_generator_->setWaypoints(waypoints_to_set_)
4. trajectory_generator_->generate() → 다항식 궤적 계산
5. time_zero_ = now()
6. first_run_ = true
7. (enable_debug_) → plotTrajectory() 디버그 시각화
```

### 6.2 `on_modify()`

```
1. 새 목표의 waypoints → 추가 waypoints_to_set_ 준비
2. trajectory_generator_->setWaypoints() 런타임 수정
→ 실행 중 waypoint 추가/변경 가능 (DynamicTrajectory 기능)
```

### 6.3 `on_run()`

```
1. has_odom_ 체크 (위치 정보 없으면 FAILURE)
2. eval_time_ = now() - time_zero_
3. evaluateTrajectory(eval_time_) 호출
4. 궤적 완료 여부 체크 → SUCCESS
5. Yaw 계산 (yaw_mode에 따라)
6. trajectory_motion_handler_.sendTrajectorySetpoint() 호출
7. feedback_.actual_speed, remaining_trajectory_time 업데이트
```

### 6.4 `on_pause()`

```
1. hover_motion_handler_.sendHover() → 현재 위치 유지
2. eval_time_ 중지 (time_zero_ 업데이트로 resume 시 재개 가능)
```

### 6.5 `on_execution_end()`

```
1. SUCCESS  → hover_motion_handler_.sendHover()
2. 기타 상태 → hover_motion_handler_.sendHover() (안전 정지)
3. 타이머 정리, trajectory_generator_ 리셋
```

---

## 7. 궤적 생성 내부 메커니즘

### 7.1 `goalToDynamicWaypoint()`

```cpp
bool goalToDynamicWaypoint(goal, waypoints) {
  for (auto& wp : goal->waypoints.poses) {
    dynamic_traj_generator::DynamicWaypoint dyn_wp;

    // ID를 waypoint에 부여
    dyn_wp.setName(wp.id);

    // 위치 설정
    dyn_wp.resetWaypoint(Eigen::Vector3d(
      wp.pose.position.x,
      wp.pose.position.y,
      wp.pose.position.z));

    // 속도/가속도가 있으면 설정
    if (wp.pose.has_velocity)
      dyn_wp.setVelocity(Eigen::Vector3d(...));

    waypoints.push_back(dyn_wp);
  }
}
```

### 7.2 `evaluateTrajectory(eval_time)`

```cpp
bool evaluateTrajectory(double eval_time) {
  dynamic_traj_generator::References refs;

  // 궤적에서 현재 시간의 위치/속도/가속도 추출
  bool finished = trajectory_generator_->evaluateTrajectory(eval_time, refs);

  // TrajectorySetpoint 메시지 구성
  trajectory_command_.positions    = refs.position;
  trajectory_command_.velocities   = refs.velocity;
  trajectory_command_.accelerations = refs.acceleration;
  trajectory_command_.yaw          = computedYaw;

  return finished;
}
```

### 7.3 `updateFrame()` (Map-to-Odom 보정)

```
배경: Map 프레임과 Odom 프레임 사이의 오프셋이 시간에 따라 변화
     (특히 GPS 기반 localization에서 SLAM 보정 적용 시)

동작:
1. tf_handler_->getTransform("map", "odom") 취득
2. 현재 transform과 마지막 transform의 거리 계산
3. transform_threshold_ 이상 차이 → waypoints 좌표 재계산
4. trajectory_generator_->setWaypoints() 재호출
```

### 7.4 `computeYawFaceReference()`

```cpp
double computeYawFaceReference() {
  // 현재 위치에서 다음 참조 위치까지의 방향 계산
  auto next_ref = trajectory_generator_->evaluateTrajectory(eval_time_ + 0.1s);
  double dx = next_ref.position.x() - current_position_.x();
  double dy = next_ref.position.y() - current_position_.y();

  // 거리가 yaw_threshold_ 미만이면 현재 yaw 유지
  if (sqrt(dx*dx + dy*dy) < yaw_threshold_)
    return current_yaw_;

  return atan2(dy, dx);
}
```

---

## 8. Yaw 모드 지원

| YawMode | 동작 |
|---------|------|
| `PATH_FACING` | `computeYawFaceReference()`로 진행 방향 추종 |
| `FIXED_YAW` | `goal_.yaw_mode.angle` 고정 |
| `KEEP_YAW` | `current_yaw_` (초기 yaw) 유지 |
| `YAW_FROM_TOPIC` | `yaw_sub_`로 수신한 `Float32` 값 사용 |

---

## 9. Waypoint 동적 수정

실행 중 `PoseStampedWithIDArray` 토픽으로 waypoint를 실시간 수정 가능:

```cpp
void modifyWaypointCallback(const PoseStampedWithIDArray::SharedPtr msg) {
  for (auto& pose : msg->poses) {
    dynamic_traj_generator::DynamicWaypoint dyn_wp;
    generateDynamicPoint(pose, dyn_wp);  // ID 기반으로 기존 wp 교체
    waypoints_to_set_.push_back(dyn_wp);
  }
  // 다음 on_run()에서 setWaypoints() 적용
}
```

---

## 10. ROS2 인터페이스

| 종류 | 이름 | 타입 |
|------|------|------|
| Action Server | `/<ns>/generate_polynomial_trajectory` | `as2_msgs/GeneratePolynomialTrajectory` |
| Subscription | `/<ns>/self_localization/twist` | `geometry_msgs/TwistStamped` |
| Subscription | `/<ns>/yaw` | `std_msgs/Float32` (optional) |
| Subscription | `/<ns>/waypoint_modify` | `as2_msgs/PoseStampedWithIDArray` |
| Publisher (debug) | `debug/traj_generated` | `nav_msgs/Path` |
| Publisher (debug) | `debug/waypoints` | `visualization_msgs/MarkerArray` |
| Publisher (debug) | `debug/ref_traj_point` | `visualization_msgs/Marker` |
| Publisher (debug) | `debug/ref_traj_end_point` | `visualization_msgs/Marker` |

---

## 11. 설정 파라미터

**파일**: `config/config_default.yaml`

```yaml
# 궤적 샘플링
sampling_n: 1               # 궤적 평가 샘플 수
sampling_dt: 0.01           # 샘플 간격 [s]
path_length: 0              # 0 = 전체 궤적

# Yaw 제어
yaw_threshold: 0.1          # Yaw 계산 활성화 최소 거리 [m]
yaw_speed_threshold: 6.0    # 최대 Yaw 속도 [rad/s]

# 프레임 보정
frequency_update_frame: 0.0 # 프레임 오프셋 갱신 주기 [Hz], 0 = 비활성
transform_threshold: 1.0    # 갱신 트리거 거리 [m]
wp_close_threshold: 0.0     # Waypoint 지평선 [s]

# 디버그
enable_debug: false

# BehaviorServer 기본 파라미터
run_frequency: 100.0        # 실행 루프 주기 [Hz]
```

---

## 12. 테스트

### Python 통합 테스트 (`behavior_test.py`)
```python
# 테스트 시나리오:
# 1. TakeoffBehavior 실행 (2m 고도)
# 2. GeneratePolynomialTrajectory 실행 (여러 waypoint)
# 3. 궤적 완료 대기
# 4. LandBehavior 실행
```

---

## 13. 설계 패턴

| 패턴 | 적용 |
|------|------|
| **Strategy** | YawMode별 Yaw 계산 전략 |
| **Observer** | PoseStampedWithIDArray 구독으로 waypoint 동적 수정 |
| **Template Method** | BehaviorServer의 on_activate/on_run/on_execution_end |

---

## 14. 알려진 이슈 및 주의사항

1. **`plot_thread_` 종료 미처리**: 디버그 모드에서 `plot_thread_`가 detach되거나 join 없이 소멸될 수 있음. `on_execution_end()`에서 정리 코드 필요.

2. **`sampling_n_`, `sampling_dt_` 조합 주의**: `sampling_n_ > 1` + `sampling_dt_ > 0`이면 여러 설정값을 한 번에 전송. 컨트롤러가 배열 형태의 TrajectorySetpoints를 지원해야 함.

3. **Map 프레임 의존성**: `frequency_update_frame_ > 0`이면 `"map"` 프레임이 반드시 존재해야 함. GPS 없는 실내 환경에서는 비활성화 필요.

4. **고주파 실행 (100Hz)**: `run_frequency: 100.0` 기본값. CPU 부하가 큰 환경에서는 낮춰야 함.

5. **Yaw 속도 제한**: `yaw_speed_threshold_`로 급격한 yaw 변화를 제한하지만, 구현 내에서 실제 clamping 로직 확인 필요.
