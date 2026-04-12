# Aerostack2 프레임워크 설계 문서

> 전체 코드베이스 기반 아키텍처 설계 분석
> 버전: 1.1.3 | ROS 2 Humble | C++17 / Python 3

---

## 목차

1. [설계 철학](#1-설계-철학)
2. [전체 시스템 아키텍처](#2-전체-시스템-아키텍처)
3. [핵심 추상화 계층 — as2_core](#3-핵심-추상화-계층--as2_core)
4. [메시지 인터페이스 설계 — as2_msgs](#4-메시지-인터페이스-설계--as2_msgs)
5. [플랫폼 추상화 계층](#5-플랫폼-추상화-계층)
6. [좌표계 및 TF 설계](#6-좌표계-및-tf-설계)
7. [드론 상태 머신 설계](#7-드론-상태-머신-설계)
8. [제어 아키텍처](#8-제어-아키텍처)
9. [행동(Behavior) 시스템 설계](#9-행동behavior-시스템-설계)
10. [Behavior Tree 통합](#10-behavior-tree-통합)
11. [Python API 설계](#11-python-api-설계)
12. [전체 데이터 흐름](#12-전체-데이터-흐름)
13. [플러그인 시스템 설계](#13-플러그인-시스템-설계)
14. [다중 드론 설계](#14-다중-드론-설계)
15. [핵심 설계 패턴 요약](#15-핵심-설계-패턴-요약)

---

## 1. 설계 철학

### 1.1 핵심 목표

```
┌─────────────────────────────────────────────────────────────────────┐
│                      Aerostack2 설계 목표                            │
│                                                                     │
│   1. 하드웨어 독립성                                                  │
│      "같은 미션 코드가 Gazebo 시뮬과 실기체에서 동일하게 동작"          │
│                                                                     │
│   2. 모듈 교체 가능성                                                 │
│      "제어 알고리즘, 상태 추정, 경로 계획을 런타임에 교체"              │
│                                                                     │
│   3. 계층적 추상화                                                    │
│      "각 계층은 아래 계층의 구현을 몰라도 된다"                         │
│                                                                     │
│   4. 다중 드론 확장성                                                 │
│      "1대부터 N대까지 네임스페이스 분리로 자연스럽게 확장"               │
│                                                                     │
│   5. 표준 인터페이스                                                  │
│      "모든 드론 동작은 ROS2 Action으로 표준화"                         │
└─────────────────────────────────────────────────────────────────────┘
```

### 1.2 설계 원칙

| 원칙 | 적용 방식 |
|------|----------|
| **관심사 분리** | 플랫폼 / 제어 / 행동 / 미션 계층 분리 |
| **의존성 역전** | 상위 계층은 추상(인터페이스)에만 의존 |
| **개방-폐쇄 원칙** | 플러그인으로 확장, 기존 코드 수정 불필요 |
| **단일 책임** | 각 패키지는 하나의 명확한 역할 수행 |

---

## 2. 전체 시스템 아키텍처

### 2.1 계층 아키텍처

```
╔══════════════════════════════════════════════════════════════════════╗
║  [ MISSION LAYER ]  미션 레이어                                       ║
║                                                                      ║
║  ┌──────────────────┐  ┌──────────────────┐  ┌───────────────────┐  ║
║  │ Python Script    │  │ BehaviorTree XML │  │ Keyboard Teleop   │  ║
║  │ drone.takeoff()  │  │ <Sequence>       │  │ WASD 입력         │  ║
║  │ drone.go_to()    │  │   <TakeOff/>     │  │                   │  ║
║  │ drone.land()     │  │   <GoTo/>        │  │ RViz Plugin       │  ║
║  └────────┬─────────┘  └────────┬─────────┘  └─────────┬─────────┘  ║
╚═══════════╪════════════════════╪═════════════════════╪══════════════╝
            │                    │                     │
╔═══════════╪════════════════════╪═════════════════════╪══════════════╗
║  [ API LAYER ]  인터페이스 레이어                                      ║
║           │                    │                     │               ║
║  ┌────────▼────────────────────▼─────────────────────▼───────────┐  ║
║  │                     as2_python_api                            │  ║
║  │   DroneInterface → TakeoffModule / GoToModule / LandModule    │  ║
║  │                     BehaviorHandler (Action Client 래퍼)       │  ║
║  └────────────────────────────┬──────────────────────────────────┘  ║
║                               │  as2_behavior_tree                   ║
║                               │  (BT 노드 → Action 클라이언트)       ║
╚═══════════════════════════════╪══════════════════════════════════════╝
                                │  ROS2 Action
╔═══════════════════════════════╪══════════════════════════════════════╗
║  [ BEHAVIOR LAYER ]  행동 레이어                                      ║
║                               │                                      ║
║  ┌────────────────────────────▼──────────────────────────────────┐  ║
║  │              as2_behavior::BehaviorServer<ActionT>            │  ║
║  │    IDLE ──► ACTIVATING ──► RUNNING ──► SUCCESS / FAILURE      ║  ║
║  └───────────────────────────────────────────────────────────────┘  ║
║                                                                      ║
║  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌────────┐  ┌────────┐  ║
║  │ Takeoff  │  │  Land    │  │  GoTo    │  │ Follow │  │  Path  │  ║
║  │ Behavior │  │ Behavior │  │ Behavior │  │  Path  │  │Planner │  ║
║  │ + Plugin │  │ + Plugin │  │ + Plugin │  │+Plugin │  │+Plugin │  ║
║  └──────────┘  └──────────┘  └──────────┘  └────────┘  └────────┘  ║
╚═══════════════════════════════╪══════════════════════════════════════╝
                                │  motion_reference/* 토픽
╔═══════════════════════════════╪══════════════════════════════════════╗
║  [ CONTROL LAYER ]  제어 레이어                                       ║
║                               │                                      ║
║  ┌────────────────────────────▼──────────────────────────────────┐  ║
║  │  as2_motion_reference_handlers                                │  ║
║  │  Hover / Position / Speed / SpeedInPlane / ACRO / Trajectory  │  ║
║  └────────────────────────────┬──────────────────────────────────┘  ║
║                               │                                      ║
║  ┌────────────────────────────▼──────────────────────────────────┐  ║
║  │  as2_motion_controller (ControllerManager)                    │  ║
║  │  Plugin: DifferentialFlatness / PID                           │  ║
║  └────────────────────────────┬──────────────────────────────────┘  ║
╚═══════════════════════════════╪══════════════════════════════════════╝
                                │  actuator_command/* 토픽
╔═══════════════════════════════╪══════════════════════════════════════╗
║  [ ESTIMATION LAYER ]  추정 레이어                                    ║
║                                                                      ║
║  ┌──────────────────────────────┐  ┌────────────────────────────┐   ║
║  │   as2_state_estimator        │  │    as2_map_server          │   ║
║  │   Plugin: raw_odom /         │  │    Plugin: depth / lidar   │   ║
║  │           ground_truth /     │  │    → Occupancy Grid        │   ║
║  │           mocap_pose         │  └────────────────────────────┘   ║
║  │   → self_localization/pose   │                                    ║
║  │   → TF: earth→odom→base_link │                                    ║
║  └──────────────────────────────┘                                    ║
╚═══════════════════════════════╪══════════════════════════════════════╝
                                │
╔═══════════════════════════════╪══════════════════════════════════════╗
║  [ PLATFORM LAYER ]  플랫폼 레이어                                    ║
║                               │                                      ║
║  ┌────────────────────────────▼──────────────────────────────────┐  ║
║  │  as2::AerialPlatform (추상 클래스)                             │  ║
║  │  ownSendCommand() / ownSetArmingState() / configureSensors()  │  ║
║  └───────────────────────────────────────────────────────────────┘  ║
║                                                                      ║
║  ┌──────────────────┐  ┌──────────────────┐  ┌──────────────────┐  ║
║  │  GazeboPlatform  │  │ MultirotorSim    │  │  PX4 / ArduPilot │  ║
║  │  (Gazebo 연동)    │  │ (내장 물리 시뮬)  │  │  (실기체 드라이버) │  ║
║  └──────────────────┘  └──────────────────┘  └──────────────────┘  ║
╚══════════════════════════════════════════════════════════════════════╝
```

---

## 3. 핵심 추상화 계층 — as2_core

### 3.1 클래스 계층 구조

```
rclcpp::Node  /  rclcpp_lifecycle::LifecycleNode
        │
        └── as2::Node
                │   + loop_frequency_
                │   + loop_rate_ptr_
                │   + generate_local_name()
                │   + generate_global_name()
                │   + sleep()
                │
                ├── as2::AerialPlatform
                │       │   + state_machine_ (PlatformStateMachine)
                │       │   + available_control_modes_
                │       │   + cmd_freq_, info_freq_
                │       │   + platform_info_msg_
                │       │
                │       │   [순수 가상 - 반드시 구현]
                │       │   + configureSensors()
                │       │   + ownSendCommand()
                │       │   + ownSetArmingState(bool)
                │       │   + ownSetOffboardControl(bool)
                │       │   + ownSetPlatformControlMode(ControlMode)
                │       │   + ownKillSwitch()
                │       │   + ownStopPlatform()
                │       │
                │       │   [가상 - 선택적 구현]
                │       │   + ownTakeoff()
                │       │   + ownLand()
                │       │
                │       ├── GazeboPlatform
                │       ├── MultirotorSimulatorPlatform
                │       └── (PX4Platform, ArduPilotPlatform...)
                │
                ├── StateEstimator
                ├── ControllerManager
                ├── BehaviorServer<T>
                └── (기타 모든 AS2 노드)
```

### 3.2 as2::Node 핵심 기능

```
as2::Node 제공 기능:

  ┌──────────────────────────────────────────────────────────┐
  │  네임스페이스 관리                                         │
  │  generate_local_name("pose")  → "drone0/pose"            │
  │  generate_global_name("map")  → "/map"                   │
  └──────────────────────────────────────────────────────────┘

  ┌──────────────────────────────────────────────────────────┐
  │  주파수 제어                                               │
  │  preset_loop_frequency(100.0)  → 100Hz 루프 설정          │
  │  sleep()                       → 지정 주파수 유지          │
  └──────────────────────────────────────────────────────────┘

  ┌──────────────────────────────────────────────────────────┐
  │  라이프사이클 (LifecycleNode 모드)                         │
  │  on_configure() → on_activate() → on_deactivate()        │
  │  on_cleanup()   → on_shutdown() → on_error()             │
  └──────────────────────────────────────────────────────────┘
```

### 3.3 센서 추상화 시스템

```
as2::sensor 계층:

  SensorData<T>                    ← 기본 데이터 발행기
     └── GenericSensor             ← 주파수 기반 발행기
            └── Sensor<T>          ← 템플릿 센서 핸들러
                   │
                   ├── Odometry    = Sensor<nav_msgs::msg::Odometry>
                   ├── Imu         = Sensor<sensor_msgs::msg::Imu>
                   ├── GPS         = Sensor<sensor_msgs::msg::NavSatFix>
                   ├── Lidar       = Sensor<sensor_msgs::msg::LaserScan>
                   ├── Battery     = Sensor<sensor_msgs::msg::BatteryState>
                   ├── Barometer   = Sensor<sensor_msgs::msg::FluidPressure>
                   ├── Compass     = Sensor<sensor_msgs::msg::MagneticField>
                   ├── RangeFinder = Sensor<sensor_msgs::msg::Range>
                   ├── Camera      ← 이미지 + CameraInfo + TF 통합
                   ├── GroundTruth ← 시뮬 ground truth
                   └── Gimbal      ← 짐벌 자세 + TF

  사용 패턴 (플랫폼 구현 내):
  configureSensors() {
    imu_sensor_    = std::make_unique<as2::sensor::Imu>("imu", this, 200.0);
    camera_sensor_ = std::make_unique<as2::sensor::Camera>("front_cam", this, 30.0);
    gps_sensor_    = std::make_unique<as2::sensor::GPS>("gps", this, 10.0);
  }

  // 데이터 업데이트 시
  imu_sensor_->updateData(imu_msg);        // → sensor_measurements/imu 발행
  camera_sensor_->updateData(img_msg);     // → sensor_measurements/front_cam 발행
```

### 3.4 토픽 / 서비스 / 액션 이름 상수

```cpp
// as2_core/names/topics.hpp
namespace as2_names::topics {

  // 센서 → 상태 추정기 입력
  sensor_measurements::imu       = "sensor_measurements/imu"
  sensor_measurements::gps       = "sensor_measurements/gps"
  sensor_measurements::odom      = "sensor_measurements/odom"
  ground_truth::pose             = "ground_truth/pose"

  // 상태 추정기 → 행동/제어기 입력
  self_localization::pose        = "self_localization/pose"
  self_localization::twist       = "self_localization/twist"

  // 행동 → 제어기 입력
  motion_reference::pose         = "motion_reference/pose"
  motion_reference::twist        = "motion_reference/twist"
  motion_reference::trajectory   = "motion_reference/trajectory"
  motion_reference::thrust       = "motion_reference/thrust"

  // 제어기 → 플랫폼 입력
  actuator_command::pose         = "actuator_command/pose"
  actuator_command::twist        = "actuator_command/twist"
  actuator_command::thrust       = "actuator_command/thrust"
  actuator_command::trajectory   = "actuator_command/trajectory"

  // 플랫폼 상태
  platform::info                 = "platform/info"
  alert_event                    = "alert_event"
}

// as2_core/names/actions.hpp
namespace as2_names::actions::behaviors {
  takeoff          = "TakeoffBehavior"
  gotowaypoint     = "GoToBehavior"
  followreference  = "FollowReferenceBehavior"
  followpath       = "FollowPathBehavior"
  land             = "LandBehavior"
  trajectorygenerator = "TrajectoryGeneratorBehavior"
}
```

---

## 4. 메시지 인터페이스 설계 — as2_msgs

### 4.1 제어 모드 인코딩

```
ControlMode.msg — 3차원 제어 모드 명세

┌─────────────────────────────────────────────────────────────────┐
│  control_mode (8비트)                                           │
│                                                                 │
│  0: UNSET           → 미설정                                    │
│  1: HOVER           → 현재 위치 유지                             │
│  2: POSITION        → (x, y, z) 위치 제어                       │
│  3: SPEED           → (vx, vy, vz) 속도 제어                    │
│  4: SPEED_IN_A_PLANE→ (vx, vy) + z 고도 제어                   │
│  5: ATTITUDE        → (roll, pitch, yaw) 자세 제어              │
│  6: ACRO            → (roll_rate, pitch_rate, yaw_rate) + thrust│
│  7: TRAJECTORY      → (pos + vel + acc) 완전 상태 제어          │
└─────────────────────────────────────────────────────────────────┘
┌─────────────────────────────────────────────────────────────────┐
│  yaw_mode (8비트)                                               │
│                                                                 │
│  0: NONE      → 요 제어 없음                                    │
│  1: YAW_ANGLE → 절대 요 각도 지정                               │
│  2: YAW_SPEED → 요 각속도 지정                                  │
└─────────────────────────────────────────────────────────────────┘
┌─────────────────────────────────────────────────────────────────┐
│  reference_frame (8비트)                                        │
│                                                                 │
│  0: UNDEFINED_FRAME    → 미정의                                 │
│  1: LOCAL_ENU_FRAME    → 로컬 ENU 좌표계 (x:East, y:North, z:Up)│
│  2: BODY_FLU_FRAME     → 기체 좌표계 (x:Front, y:Left, z:Up)   │
│  3: GLOBAL_LAT_LONG    → GPS 전역 좌표계                        │
└─────────────────────────────────────────────────────────────────┘

사용 예시:
  control_mode   = POSITION (2)
  yaw_mode       = YAW_ANGLE (1)
  reference_frame= LOCAL_ENU_FRAME (1)
  → "ENU 기준 (x,y,z) 위치로 이동하고 특정 방향을 바라봐라"
```

### 4.2 플랫폼 상태 메시지

```
PlatformInfo.msg — 플랫폼 전체 상태 스냅샷

  ┌────────────────────────────────────────────┐
  │  header: std_msgs/Header                   │
  │  connected: bool          ← 통신 연결 여부  │
  │  armed: bool              ← 모터 무장 여부  │
  │  offboard: bool           ← 오프보드 모드   │
  │  status: PlatformStatus   ← FSM 상태        │
  │  current_control_mode: ControlMode          │
  └────────────────────────────────────────────┘

PlatformStatus.msg — 상태 머신 현재 상태
  EMERGENCY  = -1
  DISARMED   =  0
  LANDED     =  1
  TAKING_OFF =  2
  FLYING     =  3
  LANDING    =  4

AlertEvent.msg — 긴급 이벤트
  KILL_SWITCH      = -1  (즉시 모터 정지)
  EMERGENCY_HOVER  = -2  (긴급 제자리 비행)
  EMERGENCY_LAND   = -3  (긴급 착륙)
  INFO_ALERT       =  0  (정보 알림)
  FORCE_HOVER      =  1  (강제 제자리 비행)
  FORCE_LAND       =  2  (강제 착륙)
```

### 4.3 YawMode 메시지 (행동 레이어용)

```
YawMode.msg — 행동 수행 중 드론이 바라볼 방향

  0: KEEP_YAW          → 현재 방향 유지
  1: PATH_FACING       → 이동 방향으로 자동 회전
  2: FIXED_YAW         → 절대 각도 고정
  3: YAW_FROM_TOPIC    → 외부 토픽에서 수신
  4: YAW_FROM_ORIENTATION → 쿼터니언에서 추출
  5: YAW_TO_FRAME      → 특정 TF 프레임 방향으로
  6: FACE_REFERENCE    → 참조점을 바라봄
```

### 4.4 전체 액션 인터페이스

```
15개 ROS2 Action 목록:

┌────────────────────────────────────────────────────────────────┐
│  플랫폼 기본 (2개)                                              │
├──────────────────────┬─────────────────────────────────────────┤
│ SetArmingState       │ Goal: bool request                      │
│                      │ Result: bool success                    │
├──────────────────────┼─────────────────────────────────────────┤
│ SetOffboardMode      │ Goal: bool request                      │
│                      │ Result: bool success                    │
└──────────────────────┴─────────────────────────────────────────┘

┌────────────────────────────────────────────────────────────────┐
│  이동 행동 (5개)                                                │
├──────────────────────┬─────────────────────────────────────────┤
│ Takeoff              │ Goal: takeoff_height, takeoff_speed     │
│                      │ Feedback: actual_speed, actual_height   │
│                      │ Result: takeoff_success                 │
├──────────────────────┼─────────────────────────────────────────┤
│ Land                 │ Goal: land_speed                        │
│                      │ Feedback: actual_speed, actual_height   │
│                      │ Result: land_success                    │
├──────────────────────┼─────────────────────────────────────────┤
│ GoToWaypoint         │ Goal: target_pose(PointStamped),        │
│                      │       yaw(YawMode), max_speed           │
│                      │ Feedback: actual_speed, distance_to_goal│
│                      │ Result: go_to_success                   │
├──────────────────────┼─────────────────────────────────────────┤
│ FollowPath           │ Goal: path(PoseWithID[]), yaw, max_speed│
│                      │ Feedback: actual_speed,                 │
│                      │           distance_to_next_waypoint,    │
│                      │           remaining_waypoints           │
│                      │ Result: follow_path_success             │
├──────────────────────┼─────────────────────────────────────────┤
│ FollowReference      │ Goal: target_pose, yaw,                 │
│                      │       max_speed_x/y/z                   │
│                      │ Feedback: actual_speed, distance        │
└──────────────────────┴─────────────────────────────────────────┘

┌────────────────────────────────────────────────────────────────┐
│  고급 이동 (2개)                                                │
├──────────────────────┬─────────────────────────────────────────┤
│ GeneratePolynomial   │ Goal: path(PoseStampedWithID[]),        │
│ Trajectory           │       yaw, max_speed                    │
│                      │ Feedback: next_waypoint_id, remaining   │
├──────────────────────┼─────────────────────────────────────────┤
│ NavigateToPoint      │ Goal: point, yaw, navigation_speed      │
│ (장애물 회피)         │ Feedback: current_pose/speed,           │
│                      │           distance_remaining            │
└──────────────────────┴─────────────────────────────────────────┘

┌────────────────────────────────────────────────────────────────┐
│  페이로드 / 인식 / 군집 (6개)                                    │
├──────────────────────┬─────────────────────────────────────────┤
│ GripperHandler       │ Goal: bool request_gripper              │
│ PointGimbal          │ Goal: GimbalControl, follow_mode        │
│ DetectArucoMarkers   │ Goal: uint16[] target_ids               │
│ ForceEstimation      │ Goal: bool request                      │
│ MassEstimation       │ Goal: bool request                      │
│ SwarmFlocking        │ Goal: virtual_centroid,                 │
│                      │       swarm_formation(PoseWithID[]),    │
│                      │       drones_namespace[]                │
└──────────────────────┴─────────────────────────────────────────┘
```

---

## 5. 플랫폼 추상화 계층

### 5.1 AerialPlatform 설계 원리

```
AerialPlatform 핵심 설계:

  "플랫폼은 명령을 수신하고 상태를 발행한다"

  입력 (구독):
  ┌──────────────────────────────────────────────────┐
  │  actuator_command/pose       → command_pose_msg_ │
  │  actuator_command/twist      → command_twist_msg_│
  │  actuator_command/thrust     → command_thrust_msg│
  │  actuator_command/trajectory → command_traj_msg_ │
  │  alert_event                 → 긴급 처리          │
  └──────────────────────────────────────────────────┘

  출력 (발행):
  ┌──────────────────────────────────────────────────┐
  │  platform/info               ← platform_info_msg_│
  │  sensor_measurements/*       ← configureSensors()│
  └──────────────────────────────────────────────────┘

  서비스 (응답):
  ┌──────────────────────────────────────────────────┐
  │  set_arming_state            → ownSetArmingState()│
  │  set_offboard_mode           → ownSetOffboard()  │
  │  set_platform_control_mode   → ownSetPlatformCtrl│
  │  platform/takeoff            → ownTakeoff()      │
  │  platform/land               → ownLand()         │
  └──────────────────────────────────────────────────┘
```

### 5.2 플랫폼 구현체 비교

```
┌──────────────────────────────────────────────────────────────────┐
│                    플랫폼 구현체 비교                              │
├────────────────────┬──────────────────┬──────────────────────────┤
│  항목              │ GazeboPlatform   │ MultirotorSimulator       │
├────────────────────┼──────────────────┼──────────────────────────┤
│  물리 엔진         │ Gazebo (ODE/     │ 내장 수학적 모델          │
│                    │  Bullet 등)      │ Simulator<double, 4>      │
├────────────────────┼──────────────────┼──────────────────────────┤
│  GPU 필요          │ 필요             │ 불필요 (CPU만)            │
├────────────────────┼──────────────────┼──────────────────────────┤
│  업데이트 주파수   │ Gazebo 설정 따름 │ 1000Hz (물리)             │
│                    │                  │ 100Hz (제어)              │
│                    │                  │ 100Hz (상태 발행)         │
├────────────────────┼──────────────────┼──────────────────────────┤
│  센서 모델         │ Gazebo 플러그인  │ IMU/GPS/GroundTruth/Odom  │
├────────────────────┼──────────────────┼──────────────────────────┤
│  사용 시나리오     │ 정밀 시뮬레이션  │ 빠른 알고리즘 개발/테스트  │
└────────────────────┴──────────────────┴──────────────────────────┘
```

### 5.3 플랫폼이 수행하는 제어 모드 협상

```
제어 모드 설정 흐름:

  행동(Behavior) / 제어기
       │  SetControlMode 서비스 호출
       │  { control_mode: POSITION, yaw_mode: YAW_ANGLE, frame: LOCAL_ENU }
       ▼
  ControllerManager
       │  available_modes_config 확인
       │  입력 모드 ↔ 출력 모드 매핑
       ▼
  AerialPlatform::ownSetPlatformControlMode()
       │  플랫폼이 해당 모드 지원 여부 확인
       │  available_control_modes_ 목록 대조
       ▼
  성공 시: 명령 수신 및 ownSendCommand() 호출 시작
  실패 시: 에러 반환 → 제어기가 다른 출력 모드 시도
```

---

## 6. 좌표계 및 TF 설계

### 6.1 TF 트리 구조

```
전역 좌표계 (고정):
  earth  ──── 지구 중심 또는 임의 전역 원점
     │        (geographiclib로 GPS ↔ ENU 변환)
     │
  map    ──── 점유 격자(Occupancy Grid) 원점
     │        (as2_map_server가 생성)
     │
  odom   ──── 드론 이륙 위치 기준 로컬 원점
     │        (as2_state_estimator가 생성 및 갱신)
     │
  base_link ─ 드론 기체 중심 (동적으로 이동)
     │
     ├── camera_link     ← 카메라 마운트 위치
     ├── lidar_link      ← 라이다 마운트 위치
     ├── gimbal_base     ← 짐벌 베이스
     │      └── gimbal   ← 짐벌 현재 자세
     └── (기타 센서)

프레임 ID 파라미터:
  base_frame_id:    "base_link"   (드론 본체)
  odom_frame_id:    "odom"        (로컬 기준)
  global_ref_frame: "earth"       (전역 기준)
  map_frame_id:     "map"         (지도 기준)
```

### 6.2 좌표계 규약

```
ENU (East-North-Up) 좌표계:        FLU (Front-Left-Up) 기체 좌표계:

  Z (Up)                              Z (Up)
  │                                   │
  │      Y (North)                    │    X (Front)
  │    /                              │  /
  │  /                                │/
  └────── X (East)                    └────── Y (Left)

  LOCAL_ENU_FRAME = 1                 BODY_FLU_FRAME = 2
  (기본 참조 프레임)                   (기체 상대 명령)
```

### 6.3 StateEstimator 변환 체인

```
StateEstimatorBase 플러그인이 관리하는 변환:

  earth_to_map_   : tf2::Transform  ← GPS 원점 설정 시 고정
  map_to_odom_    : tf2::Transform  ← 드리프트 보정
  odom_to_base_   : tf2::Transform  ← 실시간 갱신

  변환 갱신 주기:
  ┌─────────────────────────────────────────────┐
  │  raw_odom 플러그인:  Odometry 수신마다       │
  │  ground_truth 플러그인: 시뮬 GT 수신마다     │
  │  mocap_pose 플러그인: MoCap 수신마다         │
  └─────────────────────────────────────────────┘

  발행 토픽:
  → /drone0/self_localization/pose  (PoseStamped in earth)
  → /drone0/self_localization/twist (TwistStamped in body)
  → TF broadcast: odom → base_link
  → TF static: earth → odom (초기화 후 고정)
```

---

## 7. 드론 상태 머신 설계

### 7.1 상태 전이 다이어그램

```
                         EMERGENCY (-1)
                         ┌───────────┐
                         │  EMERGENCY│◄──── EMERGENCY 이벤트
                         │  (긴급)   │      (어느 상태에서든)
                         └───────────┘

  ┌───────────┐  ARM(0)   ┌───────────┐  TAKE_OFF(2)  ┌─────────────┐
  │ DISARMED  │──────────►│  LANDED   │──────────────►│ TAKING_OFF  │
  │   (0)     │           │   (1)     │               │    (2)      │
  │ 모터 정지  │◄──────────│ 착륙 상태  │               │  이륙 중     │
  └───────────┘  DISARM(1)└─────┬─────┘               └──────┬──────┘
                                │  ▲                          │
                                │  │ LANDED(5)        TOOK_OFF(3)
                                │  │                          │
                           LAND(4) │                          ▼
                                │  │               ┌──────────────────┐
                                │  │               │    FLYING  (3)   │
                                │  │               │    정상 비행      │
                                │  │               └──────────────────┘
                                │  │                          │
                                ▼  │                   LAND(4)│
                         ┌─────────┴──┐                      │
                         │  LANDING   │◄─────────────────────┘
                         │   (4)      │
                         │ 착륙 중     │
                         └────────────┘

상태 전이 표:
┌─────────────┬──────────┬─────────────┐
│ 현재 상태    │ 이벤트    │ 다음 상태   │
├─────────────┼──────────┼─────────────┤
│ DISARMED    │ ARM      │ LANDED      │
│ LANDED      │ DISARM   │ DISARMED    │
│ LANDED      │ TAKE_OFF │ TAKING_OFF  │
│ TAKING_OFF  │ TOOK_OFF │ FLYING      │
│ FLYING      │ LAND     │ LANDING     │
│ LANDING     │ LANDED   │ LANDED      │
│ 임의        │ EMERGENCY│ EMERGENCY   │
└─────────────┴──────────┴─────────────┘
```

### 7.2 상태 이벤트 발생 주체

```
이벤트 발생 주체:
  ARM       → SetArmingStateBehavior::on_run() 성공 후
  DISARM    → LandBehavior::on_execution_end(SUCCESS) 후
  TAKE_OFF  → TakeoffBehavior::on_activate() 에서
  TOOK_OFF  → TakeoffBehavior::on_execution_end(SUCCESS) 후
  LAND      → LandBehavior::on_activate() 에서
  LANDED    → LandBehavior::on_execution_end(SUCCESS) 후
  EMERGENCY → AlertEvent 수신 또는 타임아웃 발생 시

이벤트 전송 경로:
  BehaviorServer
      └── platform_cli_->sendRequest(SetPlatformStateMachineEvent)
              └── AerialPlatform::state_machine_.processEvent(event)
```

---

## 8. 제어 아키텍처

### 8.1 제어 신호 흐름

```
Behavior (행동)
    │
    │  motionReferenceHandlers 사용
    │
    ├──► HoverMotion::sendHover()
    │         → ControlMode = HOVER
    │         → motion_reference/twist (vx=0,vy=0,vz=0)
    │
    ├──► PositionMotion::sendPositionCommandWithYawAngle()
    │         → ControlMode = POSITION / YAW_ANGLE / LOCAL_ENU
    │         → motion_reference/pose
    │
    ├──► SpeedMotion::sendSpeedCommandWithYawSpeed()
    │         → ControlMode = SPEED / YAW_SPEED / BODY_FLU
    │         → motion_reference/twist
    │
    ├──► SpeedInAPlaneMotion::sendSpeedInAPlaneCommandWithYawAngle()
    │         → ControlMode = SPEED_IN_A_PLANE
    │         → motion_reference/pose + motion_reference/twist
    │
    ├──► ACROMotion::sendACRO()
    │         → ControlMode = ACRO
    │         → motion_reference/thrust
    │
    └──► TrajectoryMotion::sendTrajectoryCommandWithYawAngle()
              → ControlMode = TRAJECTORY
              → motion_reference/trajectory (pos+vel+acc+yaw)

                    │
                    ▼
          MotionReferenceHandler (BasicMotionReferenceHandler)
          - SetControlMode 서비스 호출 (제어 모드 협상)
          - 실제 토픽 발행
                    │
                    ▼
          ControllerManager (as2_motion_controller)
          - 구독: motion_reference/* + self_localization/*
          - 플러그인 선택 및 실행
                    │
         ┌──────────┴──────────┐
         │                     │
    DifferentialFlatness     PID
    플러그인                  플러그인
    (고정밀 궤적 추종)         (위치/속도 제어)
         │                     │
         └──────────┬──────────┘
                    │
                    ▼
          actuator_command/* 토픽 발행
                    │
                    ▼
          AerialPlatform::ownSendCommand()
                    │
                    ▼
          하드웨어 / 시뮬레이터
```

### 8.2 ControllerBase 인터페이스

```cpp
class ControllerBase {
  // [입력] 현재 드론 상태 (self_localization 수신)
  virtual void updateState(
    const PoseStamped & pose,
    const TwistStamped & twist) = 0;

  // [입력] 목표값 갱신 (motion_reference 수신)
  virtual void updateReference(const PoseStamped & ref);
  virtual void updateReference(const TwistStamped & ref);
  virtual void updateReference(const TrajectorySetpoints & ref);
  virtual void updateReference(const Thrust & ref);

  // [출력] 제어 명령 계산 (cmd_freq_ 주파수로 호출)
  virtual bool computeOutput(
    double dt,
    PoseStamped & pose_out,    // actuator_command/pose
    TwistStamped & twist_out,  // actuator_command/twist
    Thrust & thrust_out        // actuator_command/thrust
  ) = 0;

  // [설정] 제어 모드 설정
  virtual bool setMode(ControlMode in, ControlMode out) = 0;

  // [설정] 파라미터 실시간 업데이트
  virtual bool updateParams(const vector<Parameter> & params) = 0;

  // [설정] 프레임 ID 반환
  virtual string getDesiredPoseFrameId()  { return "odom"; }
  virtual string getDesiredTwistFrameId() { return "base_link"; }
};
```

### 8.3 제어기 모드 매핑 (available_modes_config.yaml)

```
입력 모드 (행동에서 요청)    →    출력 모드 (플랫폼이 수락)

  POSITION + LOCAL_ENU     →    POSITION + LOCAL_ENU
  POSITION + LOCAL_ENU     →    SPEED + BODY_FLU         (변환 필요)
  SPEED + LOCAL_ENU        →    SPEED + LOCAL_ENU
  TRAJECTORY               →    TRAJECTORY
  TRAJECTORY               →    ATTITUDE                 (변환 필요)
  ATTITUDE                 →    ATTITUDE
  ACRO                     →    ACRO

매칭 우선순위:
  MATCH_ALL > MATCH_MODE_AND_FRAME > MATCH_MODE_AND_YAW > MATCH_MODE
```

---

## 9. 행동(Behavior) 시스템 설계

### 9.1 BehaviorServer 아키텍처

```
BehaviorServer<ActionT> 구성 요소:

  ┌──────────────────────────────────────────────────────────────┐
  │  ROS2 Action Server                                          │
  │  → /<namespace>/<behavior_name> (e.g. /drone0/TakeoffBehavior)│
  │  → handleGoal() / handleCancel() / handleAccepted()         │
  └──────────────────────────────────────────────────────────────┘

  ┌──────────────────────────────────────────────────────────────┐
  │  서비스 서버 (행동 제어용)                                     │
  │  → /<behavior_name>/stop     (Trigger)   강제 중단           │
  │  → /<behavior_name>/pause    (Trigger)   일시 정지           │
  │  → /<behavior_name>/resume   (Trigger)   재개                │
  │  → /<behavior_name>/modify   (ModifySrv) 목표 수정           │
  └──────────────────────────────────────────────────────────────┘

  ┌──────────────────────────────────────────────────────────────┐
  │  상태 발행자                                                  │
  │  → /<behavior_name>/behavior_status (BehaviorStatus)        │
  │     IDLE(0) / RUNNING(1) / PAUSED(2)                        │
  │     주기: behavior_status_timer_ (기본 10Hz)                 │
  └──────────────────────────────────────────────────────────────┘

  ┌──────────────────────────────────────────────────────────────┐
  │  실행 타이머                                                  │
  │  → run_timer_ (주파수: run_frequency 파라미터, 기본 10Hz)    │
  │  → on_run() 을 주기적으로 호출                               │
  └──────────────────────────────────────────────────────────────┘
```

### 9.2 BehaviorServer 생명주기

```
클라이언트          BehaviorServer              서브클래스
    │                    │                          │
    │── Goal ───────────►│                          │
    │                    │── on_activate(goal) ────►│
    │                    │◄── true/false ────────── │
    │◄── GoalAccepted ───│                          │
    │                    │                          │
    │                    │  [run_timer_ 콜백 시작]   │
    │                    │── on_run(goal,fb,result)►│
    │◄── Feedback ───────│◄── RUNNING ────────────── │
    │                    │── on_run() ─────────────►│
    │◄── Feedback ───────│◄── RUNNING ────────────── │
    │                    │                          │
    │  (수정 요청)         │                          │
    │── /modify ────────►│── on_modify(new_goal) ──►│
    │                    │                          │
    │  (일시정지 요청)      │                          │
    │── /pause ─────────►│── on_pause() ───────────►│
    │                    │  [run_timer_ 일시 중지]   │
    │── /resume ────────►│── on_resume() ──────────►│
    │                    │  [run_timer_ 재시작]      │
    │                    │                          │
    │                    │── on_run() ─────────────►│
    │                    │◄── SUCCESS ─────────────── │
    │                    │── on_execution_end(✓) ──►│
    │◄── Result(✓) ──────│                          │
    │                    │  [run_timer_ 중지, IDLE]  │
```

### 9.3 TakeoffBehavior — 플러그인 구조 상세

```
TakeoffBehavior 초기화 흐름:

  생성자 호출
    │
    ├── BehaviorServer 초기화 (액션서버, 서비스서버, 타이머)
    │
    ├── 파라미터 선언:
    │   plugin_name, takeoff_height, takeoff_speed, takeoff_threshold
    │
    ├── pluginlib::ClassLoader<TakeoffBase> 생성
    │
    ├── loader_->createSharedInstance(plugin_name + "::Plugin")
    │   예) "as2_behaviors_motion/TakeoffPluginSpeed::Plugin"
    │
    ├── tf_handler_ 생성 (TF 변환 처리)
    │
    ├── twist_sub_ 생성 → self_localization/twist 구독
    │   → state_callback() → tf 변환 후 플러그인에 전달
    │
    └── platform_cli_ 생성
        → set_platform_state_machine_event 서비스 클라이언트

on_activate(goal):
    ├── 목표값 유효성 검사 (고도 > 0)
    ├── sendEventFSME(TAKE_OFF) → 플랫폼 FSM 이벤트
    └── takeoff_plugin_->on_activate(goal)

on_run(goal, feedback, result):
    └── takeoff_plugin_->on_run(goal, feedback, result)

on_execution_end(state):
    ├── state == SUCCESS → sendEventFSME(TOOK_OFF)
    ├── state == FAILURE → sendEventFSME(EMERGENCY)
    └── takeoff_plugin_->on_execution_end(state)
```

### 9.4 4가지 이륙 플러그인 비교

```
┌──────────────────────────────────────────────────────────────────┐
│  takeoff_plugin_platform                                         │
│                                                                  │
│  ownActivate():                                                  │
│    platform_takeoff_cli_->async_send_request(request)           │
│    → /platform/takeoff 서비스 호출                               │
│  ownRun():                                                       │
│    future 완료 확인 → SUCCESS/RUNNING                            │
│  장점: 기체 내장 이륙 로직 사용                                    │
│  단점: 플랫폼이 takeoff 서비스를 지원해야 함                       │
└──────────────────────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────────────────────┐
│  takeoff_plugin_speed                                            │
│                                                                  │
│  ownActivate(): (즉시 true)                                      │
│  ownRun():                                                       │
│    if |actual_height - target_height| < threshold → SUCCESS     │
│    else:                                                         │
│      speed_motion_handler_->sendSpeedCommandWithYawSpeed(        │
│        "earth", 0, 0, takeoff_speed, 0)                         │
│  장점: 단순, 빠름                                                 │
│  단점: 정밀도 낮음 (오버슈트 가능)                                 │
└──────────────────────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────────────────────┐
│  takeoff_plugin_position                                         │
│                                                                  │
│  ownRun():                                                       │
│    position_motion_handler_->sendPositionCommandWithYawAngle(    │
│      "earth", current_x, current_y, target_height, current_yaw) │
│  장점: 목표 고도 정밀 제어                                         │
│  단점: 위치 제어기 튜닝 필요                                        │
└──────────────────────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────────────────────┐
│  takeoff_plugin_trajectory                                       │
│                                                                  │
│  ownRun():                                                       │
│    trajectory_motion_handler_->sendTrajectoryCommand(            │
│      pos, vel, acc)  ← 다항식 궤적으로 부드럽게 이륙              │
│  장점: 가장 부드러운 이륙                                          │
│  단점: 궤적 생성 계산 필요                                         │
└──────────────────────────────────────────────────────────────────┘
```

---

## 10. Behavior Tree 통합

### 10.1 BT 노드 타입 및 매핑

```
BehaviorTree.CPP 노드 타입        Aerostack2 구현

Action 노드:
  ┌────────────────────────────────────────────────────────────────┐
  │  XML 태그      │ C++ 클래스               │ ROS2 통신          │
  ├────────────────┼──────────────────────────┼────────────────────┤
  │  <Arm/>        │ ArmService               │ SetArmingState Srv │
  │  <Disarm/>     │ DisarmService            │ SetArmingState Srv │
  │  <Offboard/>   │ OffboardService          │ SetOffboardMode Srv│
  │  <TakeOff/>    │ TakeoffAction            │ TakeoffBehavior Act│
  │  <Land/>       │ LandAction               │ LandBehavior Act   │
  │  <GoTo/>       │ GoToAction               │ GoToBehavior Act   │
  │  <GoToGps/>    │ GoToGpsAction            │ GoToBehavior Act   │
  │  <FollowPath/> │ FollowPathAction         │ FollowPathBehavior │
  │  <SendEvent/>  │ SendEvent                │ 토픽 발행          │
  │  <Echo/>       │ Echo                     │ 로그 출력          │
  │  <SetOrigin/>  │ SetOrigin                │ SetOrigin 서비스   │
  │  <GetOrigin/>  │ GetOrigin                │ GetOrigin 서비스   │
  │  <GpsToCart/>  │ GpsToCartesian           │ GPS 변환 서비스     │
  └────────────────┴──────────────────────────┴────────────────────┘

Condition 노드:
  ┌────────────────┬──────────────────────────┬────────────────────┐
  │  <IsFlying/>   │ IsFlyingCondition        │ platform/info 구독 │
  └────────────────┴──────────────────────────┴────────────────────┘

Decorator 노드:
  ┌────────────────┬──────────────────────────┬────────────────────┐
  │ <WaitForEvent/>│ WaitForEvent             │ 토픽 구독 대기     │
  │ <WaitForAlert/>│ WaitForAlert             │ alert_event 대기   │
  └────────────────┴──────────────────────────┴────────────────────┘
```

### 10.2 TakeoffAction 구현 원리

```cpp
// TakeoffAction — BT Action Node 구현

// 포트 정의 (XML 속성과 매핑)
static BT::PortsList providedPorts() {
  return providedBasicPorts({
    BT::InputPort<double>("height"),  // <TakeOff height="2.0" .../>
    BT::InputPort<double>("speed")    // <TakeOff speed="0.5" .../>
  });
}

// 매 틱마다 ROS2 Action Goal 설정
void on_tick() override {
  goal_.takeoff_height = getInput<double>("height").value_or(1.0);
  goal_.takeoff_speed  = getInput<double>("speed").value_or(0.5);
}

// Action 서버에서 결과 수신 후 처리
BT::NodeStatus on_success() {
  std::this_thread::sleep_for(500ms);  // 안정화 대기
  return BT::NodeStatus::SUCCESS;
}
```

### 10.3 BT 실행 엔진

```
bt_manager 노드 (as2_behavior_tree_node.cpp):

  1. ROS2 파라미터에서 XML 파일 경로 읽기
     tree = "/path/to/mission.xml"

  2. BT::BehaviorTreeFactory에 노드 타입 등록 (16개)

  3. Blackboard 생성 및 공유 데이터 설정:
     blackboard["node"]             = rclcpp::Node 포인터
     blackboard["server_timeout"]   = 10000ms
     blackboard["bt_loop_duration"] = 10ms

  4. factory.createTreeFromFile(tree, blackboard)

  5. (선택) Groot2Publisher 연결 → 실시간 시각화

  6. 메인 루프:
     while (rclcpp::ok() && result == RUNNING) {
       result = tree.tickWhileRunning();
       loopRate.sleep();  // 10ms 주기
     }
     → tree 노드들은 내부에서 ROS2 Action/Service 호출
```

---

## 11. Python API 설계

### 11.1 클래스 계층

```
DroneInterfaceBase (drone_interface_base.py)
  │  SingleThreadedExecutor로 백그라운드 스핀
  │  구독: platform/info, self_localization/pose, self_localization/twist
  │  서비스: arm/disarm, offboard/manual
  │
  └── DroneInterface (drone_interface.py)
        │  모듈 조합으로 완전한 인터페이스 제공
        │
        ├── self.takeoff      = TakeoffModule(drone=self)
        ├── self.land         = LandModule(drone=self)
        ├── self.go_to        = GoToModule(drone=self)
        └── self.follow_path  = FollowPathModule(drone=self)

             DroneInterfaceGPS (drone_interface_gps.py)
               └── GPS 좌표 기반 이동 추가

             DroneInterfaceTeleop (drone_interface_teleop.py)
               └── 텔레오퍼레이션 기능 추가
```

### 11.2 모듈 설계 패턴

```
모든 모듈은 두 클래스를 다중 상속:
  ModuleBase + BehaviorHandler<ActionT>

class TakeoffModule(ModuleBase, TakeoffBehavior):
  alias = 'takeoff'

  __call__(height=1.0, speed=0.5, wait=True):
    return self.start(height=height, speed=speed, wait_result=wait)
         │
         └── BehaviorHandler.start()
               → Action Client.send_goal()
               → wait=True이면 blocking (결과 대기)
               → wait=False이면 non-blocking (백그라운드 실행)

BehaviorHandler 기능:
  - pause()  → /TakeoffBehavior/pause 서비스 호출
  - resume() → /TakeoffBehavior/resume 서비스 호출
  - stop()   → /TakeoffBehavior/stop 서비스 호출
  - status   → BehaviorStatus 구독으로 실시간 상태 확인
```

### 11.3 GoToModule 전체 API

```python
class GoToModule:
  # 기본 이동 (절대 좌표)
  def __call__(x, y, z, speed,
               yaw_mode=KEEP_YAW, yaw_angle=None,
               frame_id='earth', wait=True) → bool

  # 단순 이동 (yaw 유지)
  def go_to_point(point: [x,y,z], speed,
                  frame_id='earth') → bool

  # 특정 방향 바라보기
  def go_to_with_yaw(x, y, z, speed, yaw_angle,
                     frame_id='earth') → bool

  # 이동 방향으로 자동 회전
  def go_to_point_path_facing(point, speed,
                               frame_id='earth') → bool

# 사용 예시
drone.go_to(x=5.0, y=3.0, z=2.0, speed=1.0,
            yaw_mode=YawMode.PATH_FACING)
drone.go_to.go_to_point([5, 3, 2], speed=1.0)
```

### 11.4 MissionInterpreter

```
MissionInterpreter — 복잡한 미션 순차 실행

  미션 정의 → 파싱 → 실행 스택

  missions = {
    0: Mission([
      TakeoffTask(height=2.0, speed=0.5),
      GoToTask(x=5.0, y=0.0, z=2.0, speed=1.0),
      GoToTask(x=5.0, y=5.0, z=2.0, speed=1.0),
      LandTask(speed=0.3)
    ])
  }

  interpreter.start(mission_id=0)
    → 각 Task를 순서대로 실행
    → current_behavior 추적
    → pause/resume/stop 지원
    → 실행 스레드 분리
```

---

## 12. 전체 데이터 흐름

### 12.1 수직 데이터 흐름

```
┌─────────────────────────────────────────────────────────────────────┐
│  상향 흐름 (센서 → 사용자)                                           │
│                                                                     │
│  하드웨어 센서                                                       │
│    │ 로우 데이터 (IMU 가속도/자이로, GPS NMEA, 카메라 픽셀)            │
│    ▼                                                                │
│  as2::AerialPlatform::configureSensors()                            │
│    │ sensor_measurements/imu, /gps, /camera 등                      │
│    ▼                                                                │
│  as2_state_estimator (플러그인)                                      │
│    │ self_localization/pose (PoseStamped, earth 프레임)              │
│    │ self_localization/twist (TwistStamped, body 프레임)             │
│    │ TF: earth → odom → base_link                                   │
│    ▼                                                                │
│  BehaviorServer::on_run() + ControllerBase::updateState()           │
│    │ feedback (현재 고도, 속도, 거리 등)                              │
│    ▼                                                                │
│  Python API / BehaviorTree                                          │
│    │ 사용자 코드에서 feedback 수신                                    │
└─────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────┐
│  하향 흐름 (사용자 → 하드웨어)                                        │
│                                                                     │
│  사용자 코드                                                         │
│    │ drone.takeoff(height=2.0)                                      │
│    ▼                                                                │
│  BehaviorHandler (Action Client)                                    │
│    │ Takeoff Action Goal → /drone0/TakeoffBehavior                  │
│    ▼                                                                │
│  TakeoffBehavior::on_activate() + on_run()                          │
│    │ motionReferenceHandler.sendSpeedCommand(vz=0.5)                │
│    ▼                                                                │
│  BasicMotionReferenceHandler                                        │
│    │ SetControlMode 서비스 → ControllerManager                      │
│    │ motion_reference/twist 토픽 발행                                │
│    ▼                                                                │
│  ControllerBase::updateReference() + computeOutput()               │
│    │ actuator_command/thrust, /pose, /twist                         │
│    ▼                                                                │
│  AerialPlatform::ownSendCommand()                                   │
│    │ 모터 PWM / MAVLink 메시지 / Gazebo 물리 명령                    │
└─────────────────────────────────────────────────────────────────────┘
```

### 12.2 이륙 전체 시퀀스

```
  사용자            Python API        TakeoffBehavior     Platform FSM
    │                   │                   │                   │
    │─ takeoff(h=2.0) ─►│                   │                   │
    │                   │─ SendGoal ────────►│                   │
    │                   │                   │─ sendEventFSME ──►│
    │                   │                   │    (TAKE_OFF)     │─► TAKING_OFF
    │                   │◄─ GoalAccepted ───│                   │
    │                   │                   │─ Plugin.activate()│
    │                   │                   │                   │
    │                   │                   │ [run_timer 10Hz]  │
    │                   │                   │─ Plugin.run() ──┐ │
    │                   │◄─ Feedback(0.3m)  │◄─ RUNNING ──────┘ │
    │                   │◄─ Feedback(0.8m)  │                   │
    │                   │◄─ Feedback(1.5m)  │                   │
    │                   │◄─ Feedback(2.0m)  │                   │
    │                   │                   │─ Plugin.run() ──┐ │
    │                   │                   │◄─ SUCCESS ───────┘ │
    │                   │                   │─ HoverMotion      │
    │                   │                   │─ sendEventFSME ──►│
    │                   │                   │    (TOOK_OFF)     │─► FLYING
    │                   │◄─ Result(✓) ──────│                   │
    │◄─ True ───────────│                   │                   │
```

### 12.3 토픽 흐름 요약

```
네임스페이스: /drone0/

[센서 계층]
  Platform ─────────────────────────────────────────────────────►
    sensor_measurements/imu          (200Hz)
    sensor_measurements/gps          (10Hz)
    sensor_measurements/camera/image (30Hz)
    ground_truth/pose                (100Hz, 시뮬만)

[추정 계층]
  StateEstimator ────────────────────────────────────────────────►
    self_localization/pose  (PoseStamped, earth frame)   (100Hz)
    self_localization/twist (TwistStamped, body frame)   (100Hz)
    /tf (odom → base_link 동적 변환)                     (100Hz)

[제어 계층]
  BehaviorPlugin / motionReferenceHandler ───────────────────────►
    motion_reference/pose         (PoseStamped)
    motion_reference/twist        (TwistStamped)
    motion_reference/trajectory   (TrajectorySetpoints)
    motion_reference/thrust       (Thrust)

  ControllerManager ─────────────────────────────────────────────►
    actuator_command/pose         (100Hz)
    actuator_command/twist        (100Hz)
    actuator_command/thrust       (100Hz)
    controller/info               (ControllerInfo, 10Hz)

[플랫폼 계층]
  Platform ─────────────────────────────────────────────────────►
    platform/info                 (PlatformInfo)         (10Hz)
    alert_event                   (AlertEvent)           (이벤트)
```

---

## 13. 플러그인 시스템 설계

### 13.1 pluginlib 기반 플러그인 아키텍처

```
pluginlib 플러그인 등록 및 로딩:

등록 (CMakeLists.txt + plugins.xml):
  ┌────────────────────────────────────────────────────┐
  │  plugins.xml:                                      │
  │  <class name="as2_behaviors_motion/               │
  │               TakeoffPluginSpeed"                  │
  │   type="takeoff_speed::Plugin"                     │
  │   base_class_type="takeoff_base::TakeoffBase">     │
  │  </class>                                          │
  │                                                    │
  │  소스코드 끝:                                       │
  │  PLUGINLIB_EXPORT_CLASS(takeoff_speed::Plugin,     │
  │                         takeoff_base::TakeoffBase) │
  └────────────────────────────────────────────────────┘

로딩 (런타임):
  ┌────────────────────────────────────────────────────┐
  │  loader_ = make_shared<                            │
  │    ClassLoader<TakeoffBase>>(                      │
  │    "as2_behaviors_motion",  ← 패키지명             │
  │    "takeoff_base::TakeoffBase"); ← 기반 클래스      │
  │                                                    │
  │  plugin_ = loader_->createSharedInstance(          │
  │    "as2_behaviors_motion/TakeoffPluginSpeed");     │
  └────────────────────────────────────────────────────┘
```

### 13.2 전체 플러그인 목록

```
패키지                     기반 클래스                   플러그인들
───────────────────────────────────────────────────────────────────
as2_motion_controller      ControllerBase               DifferentialFlatness
                                                         PID

as2_state_estimator        StateEstimatorBase           raw_odometry
                                                         ground_truth
                                                         mocap_pose

as2_behaviors_motion       takeoff_base::TakeoffBase    TakeoffPluginSpeed
                                                         TakeoffPluginPosition
                                                         TakeoffPluginPlatform
                                                         TakeoffPluginTrajectory
                           land_base::LandBase          LandPluginSpeed
                                                         LandPluginPlatform
                                                         LandPluginTrajectory
                           go_to_base::GoToBase         GoToPluginPosition
                                                         GoToPluginTrajectory
                           follow_path_base::...        FollowPathPluginPosition
                                                         FollowPathPluginTrajectory

as2_behaviors_path_planning PluginBase                  AStarPlanner
                                                         VoronoiPlanner

as2_behaviors_payload      GripperBase                  DcServoGripper
                                                         TwoFingersGripper

as2_map_server             MapServerBase               DepthImageMap
                                                         PointcloudMap
```

### 13.3 플러그인 선택 방법

```yaml
# config.yaml — 런타임 플러그인 선택
/**:
  ros__parameters:
    plugin_name: "as2_behaviors_motion/TakeoffPluginSpeed"
    #            "as2_behaviors_motion/TakeoffPluginPosition"
    #            "as2_behaviors_motion/TakeoffPluginTrajectory"
    takeoff_height: 2.0
    takeoff_speed: 0.5
    takeoff_threshold: 0.1
```

---

## 14. 다중 드론 설계

### 14.1 네임스페이스 기반 격리

```
다중 드론 배포 구조:

  /drone0/bt_manager          ← 드론0 BT 실행기
  /drone0/TakeoffBehavior     ← 드론0 이륙 행동
  /drone0/LandBehavior        ← 드론0 착륙 행동
  /drone0/platform_node       ← 드론0 플랫폼 드라이버
  /drone0/state_estimator     ← 드론0 상태 추정기
  /drone0/motion_controller   ← 드론0 모션 제어기

  /drone1/bt_manager          ← 드론1 BT 실행기
  /drone1/TakeoffBehavior     ← 드론1 이륙 행동
  ...

  /swarm_manager              ← 군집 관리자 (단일)
    └── SwarmFlockingBehavior
          ├── /drone0/FollowReferenceBehavior Action Client
          ├── /drone1/FollowReferenceBehavior Action Client
          └── /drone2/FollowReferenceBehavior Action Client
```

### 14.2 SwarmFlocking 설계

```
SwarmFlockingBehavior:

  입력 (Action Goal):
    virtual_centroid: geometry_msgs/PoseStamped
      → 무리의 가상 중심점 위치/속도
    swarm_formation: PoseWithID[]
      → 각 드론의 중심점 대비 상대 오프셋
      [{"id":"drone0", offset:(0,0,0)},
       {"id":"drone1", offset:(1,0,0)},
       {"id":"drone2", offset:(-1,0,0)}]
    drones_namespace: string[]
      → ["drone0", "drone1", "drone2"]

  내부 동작:
    for each drone in drones_namespace:
      follow_ref_client_[drone]->send_goal(
        target_pose = virtual_centroid + formation_offset[drone]
      )

  결과:
    swarm_pose: geometry_msgs/Pose
      → 무리 전체 현재 중심 위치
```

### 14.3 Python API 다중 드론

```python
# 독립 제어
drone0 = DroneInterface("drone0")
drone1 = DroneInterface("drone1")

drone0.takeoff(height=2.0)
drone1.takeoff(height=2.0)

# 동기화 이동 (wait=False로 병렬 시작)
drone0.go_to(x=5.0, y=0.0, z=2.0, speed=1.0, wait=False)
drone1.go_to(x=5.0, y=2.0, z=2.0, speed=1.0, wait=False)
drone0.go_to.wait_result()  # 완료 대기
drone1.go_to.wait_result()
```

---

## 15. 핵심 설계 패턴 요약

### 15.1 사용된 설계 패턴

```
┌─────────────────┬────────────────────────────────────────────────┐
│  패턴           │  적용 위치                                      │
├─────────────────┼────────────────────────────────────────────────┤
│ Strategy        │ 플러그인 시스템                                  │
│ (전략 패턴)      │ 제어기/추정기/행동 플러그인을 런타임에 교체       │
├─────────────────┼────────────────────────────────────────────────┤
│ Template Method │ BehaviorServer, ControllerBase, StateEstimator │
│ (템플릿 메서드)   │ 기반 클래스가 흐름 정의, 서브클래스가 세부 구현  │
├─────────────────┼────────────────────────────────────────────────┤
│ Observer        │ ROS2 토픽 구독/발행                             │
│ (관찰자 패턴)    │ 센서 → 추정기 → 제어기 → 플랫폼                │
├─────────────────┼────────────────────────────────────────────────┤
│ State Machine   │ PlatformStateMachine                           │
│ (상태 머신)      │ DISARMED/LANDED/FLYING/EMERGENCY 전이          │
├─────────────────┼────────────────────────────────────────────────┤
│ Command         │ BehaviorServer + ROS2 Action                   │
│ (커맨드 패턴)    │ 행동 요청을 객체화, 취소/수정 지원               │
├─────────────────┼────────────────────────────────────────────────┤
│ Facade          │ Python DroneInterface, as2_python_api          │
│ (퍼사드 패턴)    │ 복잡한 ROS2 통신을 단순 메서드로 래핑            │
├─────────────────┼────────────────────────────────────────────────┤
│ Composite       │ BehaviorTree XML                               │
│ (복합 패턴)      │ Sequence/Selector로 복잡한 미션 구성            │
├─────────────────┼────────────────────────────────────────────────┤
│ Adapter         │ motionReferenceHandlers                        │
│ (어댑터 패턴)    │ 행동의 고수준 명령 → 제어기 저수준 신호 변환     │
└─────────────────┴────────────────────────────────────────────────┘
```

### 15.2 계층 간 결합 방식

```
계층 간 통신 방식:

사용자 ↔ 행동      : ROS2 Action   (비동기, Feedback, Cancel)
행동 ↔ 제어기      : ROS2 Topic    (단방향, 고주파)
                    + ROS2 Service (제어 모드 협상)
제어기 ↔ 플랫폼    : ROS2 Topic    (단방향, 고주파)
                    + ROS2 Service (설정 변경)
행동 ↔ 플랫폼 FSM  : ROS2 Service (동기, FSM 이벤트)
이벤트 전파         : ROS2 Topic    (alert_event, 전체 브로드캐스트)
```

### 15.3 확장 포인트

```
새 플랫폼 드라이버 추가:
  → as2::AerialPlatform 상속
  → 7개 순수 가상 메서드 구현
  → configureSensors() 에서 센서 초기화

새 제어 알고리즘 추가:
  → ControllerBase 상속
  → updateState(), computeOutput(), setMode() 구현
  → plugins.xml 등록

새 행동 추가:
  → BehaviorServer<MyAction> 상속
  → on_activate(), on_run() 구현
  → as2_msgs/action/*.action 정의 추가

새 상태 추정기 추가:
  → StateEstimatorBase 상속
  → on_setup(), on_update_data() 구현
  → plugins.xml 등록

새 BT 노드 추가:
  → BT::SyncActionNode 또는 BT::ActionNode 상속
  → tick() 또는 on_tick() 구현
  → factory.registerNodeType<MyNode>("MyTag") 등록
```

---

## 부록 — 핵심 파일 인덱스

| 역할 | 파일 경로 |
|------|----------|
| 드론 추상 클래스 | `as2_core/include/as2_core/aerial_platform.hpp` |
| 기본 노드 | `as2_core/include/as2_core/node.hpp` |
| 상태 머신 | `as2_core/include/as2_core/platform_state_machine.hpp` |
| 센서 추상화 | `as2_core/include/as2_core/sensor.hpp` |
| 토픽 이름 상수 | `as2_core/include/as2_core/names/topics.hpp` |
| 행동 서버 (클래스) | `as2_behaviors/as2_behavior/include/as2_behavior/__detail/behavior_server__class.hpp` |
| 행동 서버 (구현) | `as2_behaviors/as2_behavior/include/as2_behavior/__impl/behavior_server__impl.hpp` |
| 이륙 행동 | `as2_behaviors/as2_behaviors_motion/takeoff_behavior/src/takeoff_behavior.cpp` |
| 이륙 플러그인 (속도) | `as2_behaviors/as2_behaviors_motion/takeoff_behavior/plugins/takeoff_plugin_speed.cpp` |
| 모션 핸들러 기반 | `as2_motion_reference_handlers/include/as2_motion_reference_handlers/basic_motion_references.hpp` |
| 제어기 기반 | `as2_motion_controller/include/as2_motion_controller/controller_base.hpp` |
| 상태 추정기 기반 | `as2_state_estimator/include/as2_state_estimator/plugin_base.hpp` |
| BT 실행 노드 | `as2_behavior_tree/src/as2_behavior_tree_node.cpp` |
| Python API | `as2_python_api/as2_python_api/drone_interface.py` |
| Gazebo 플랫폼 | `as2_aerial_platforms/as2_platform_gazebo/include/as2_platform_gazebo/as2_platform_gazebo.hpp` |
