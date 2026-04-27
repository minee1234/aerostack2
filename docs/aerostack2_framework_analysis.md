# AeroStack2 전체 프레임워크 분석

> 분석 일자: 2026-04-27  
> 대상 브랜치: main  
> 작성자: 코드베이스 자동 분석

---

## 1. 전체 패키지 구조

```
aerostack2/
├── as2_core/                       # 기반 클래스 및 유틸리티
├── as2_msgs/                       # 메시지·서비스·액션 정의
├── as2_motion_controller/          # 모션 제어 파이프라인
├── as2_motion_reference_handlers/  # 모션 레퍼런스 추상화
├── as2_aerial_platforms/           # 플랫폼 추상화 레이어
├── as2_behaviors/                  # 비헤이비어 시스템 (플러그인)
├── as2_behavior_tree/              # Behavior Tree 통합
├── as2_state_estimator/            # 상태 추정 (플러그인)
├── as2_map_server/                 # 맵 관리 (플러그인)
├── as2_python_api/                 # 사용자 Python API
├── as2_hardware_drivers/           # 하드웨어 드라이버
├── as2_user_interfaces/            # 텔레오퍼레이션·시각화
├── as2_utilities/                  # 지오존·GPS·TF 유틸
├── as2_simulation_assets/          # 시뮬레이션 자산
└── as2_cli/                        # CLI 도구
```

### 패키지 카테고리

| 카테고리 | 패키지 | 역할 |
|---|---|---|
| 기반 | `as2_core`, `as2_msgs` | 베이스 클래스, 메시지 정의 |
| 모션 파이프라인 | `as2_motion_controller`, `as2_motion_reference_handlers` | 제어·명령 생성 |
| 인지·상태 | `as2_state_estimator`, `as2_hardware_drivers` | 센서 퓨전·상태 추정 |
| 비헤이비어 | `as2_behavior*`, `as2_behaviors_*` | 재사용 가능한 태스크 구현 |
| 플랫폼 | `as2_aerial_platforms` | 하드웨어 추상화 |
| 유틸리티 | `as2_utilities/*` | GPS, 지오존, TF 유틸 |
| 통합 | `as2_behavior_tree` | 비헤이비어 조합 |
| 인터페이스 | `as2_python_api`, `as2_cli` | 사용자 접근 |

---

## 2. as2_core — 프레임워크 기반

### 클래스 계층 구조

```
as2::Node  (rclcpp::Node 또는 rclcpp_lifecycle::LifecycleNode 상속)
│  - 주파수 기반 루프 (node_frequency 파라미터)
│  - 라이프사이클 콜백 (on_configure / on_activate / ...)
│  - 토픽 이름 생성 유틸리티 (local / global namespace)
│
└── as2::AerialPlatform  (as2::Node 상속, 모든 플랫폼의 공통 기반)
       - 플랫폼 상태머신 관리
       - 명령 수신 (pose / twist / thrust / trajectory)
       - platform/info 퍼블리시
       - 서비스: arm/disarm, offboard, control_mode, takeoff/land
       - 추상 메서드: ownSendCommand(), ownSetArmingState(), ownTakeoff() 등
```

### 플랫폼 상태머신 (6-state FSM)

```
DISARMED ──ARM──► LANDED ──TAKE_OFF──► TAKING_OFF ──TOOK_OFF──► FLYING
                    ▲                                               │
                    └──────────────LANDED────── LANDING ◄──LAND────┘
                                                    │
                                               EMERGENCY
```

| 상태 | 설명 |
|---|---|
| `DISARMED` | 비활성, 모터 정지 |
| `LANDED` | 지상에서 arm 완료 |
| `TAKING_OFF` | 이륙 중 |
| `FLYING` | 정상 비행 중 |
| `LANDING` | 착륙 중 |
| `EMERGENCY` | 비상 상태 |

---

## 3. 토픽·서비스·액션 네이밍 컨벤션

### 토픽 (`as2_names::topics`)

| 네임스페이스 | 주요 토픽 | 설명 |
|---|---|---|
| `sensor_measurements/` | imu, gps, lidar, battery | 센서 원시 데이터 |
| `ground_truth/` | pose, twist | 시뮬레이터 진실값 |
| `self_localization/` | pose, twist, odom | 상태 추정 결과 |
| `motion_reference/` | thrust, pose, twist, trajectory | 제어 레퍼런스 |
| `actuator_command/` | pose, twist, thrust | 플랫폼 액추에이터 명령 |
| `platform/` | info | 플랫폼 상태 브로드캐스트 |
| `controller/` | info | 컨트롤러 정보 |

### 서비스 (`as2_names::services`)

| 경로 | 서비스 |
|---|---|
| `platform/` | set_arming_state, set_offboard_mode, set_platform_control_mode, list_control_modes |
| `controller/` | set_control_mode, list_control_modes |
| `gps/` | get_origin, set_origin |
| `behavior/` | dynamic_land, dynamic_follower, package_pickup, package_unpick |

### 액션 (`as2_names::actions::behaviors`)

- `Takeoff`, `Land`, `GoToWaypoint`, `FollowPath`, `FollowReference`
- `GeneratePolynomialTrajectory`, `DetectArucoMarkers`
- `SwarmFlocking`, `ForceEstimation`, `MassEstimation`
- `SetArmingState`, `SetOffboardMode`, `GripperHandler`, `PointGimbal`

---

## 4. 모션 제어 파이프라인

### 전체 흐름

```
[Python API / Mission]
         │ 목표 설정 (Action)
         ▼
[Behavior (e.g. GoToBehavior)]
         │ PositionMotion.sendPositionCommandWithYawAngle()
         ▼
  motion_reference/pose  (토픽 publish)
         │
         ▼
[ControllerHandler]  ◄─── self_localization/pose (현재 상태)
         │            ◄─── platform/info (현재 모드)
         │  제어 모드 협상 → 플러그인 로드
         ▼
[Controller Plugin (pluginlib)]
    updateState()     ← 현재 위치·속도
    updateReference() ← 목표 setpoint
    computeOutput()   → Thrust / Twist
         │
         ▼
  actuator_command/thrust  (토픽 publish)
         │
         ▼
[AerialPlatform]
    ownSendCommand()  ← 플랫폼별 구현
         │
         ▼
[Hardware / Gazebo / Simulator]
         │
         ▼ sensor_measurements/*
[State Estimator] ──► self_localization/*  (피드백 루프)
```

### 제어 모드 3차원 구조 (`ControlMode.msg`)

| 차원 | 값 | 설명 |
|---|---|---|
| **Yaw** | NONE / YAW_ANGLE / YAW_SPEED | Yaw 제어 방식 |
| **Main** | HOVER / POSITION / SPEED / SPEED_IN_A_PLANE / ATTITUDE / ACRO / TRAJECTORY | 주 제어 방식 |
| **Frame** | LOCAL_ENU / BODY_FLU / GLOBAL_GPS | 기준 좌표계 |

### 컨트롤러 플러그인

| 플러그인 | 알고리즘 |
|---|---|
| `differential_flatness_controller` | 미분 평탄성 이론 기반 |
| `pid_speed_controller` | 캐스케이드 PID |

### 모션 레퍼런스 핸들러 계층

```
BasicMotionReferenceHandler (기반)
├── PositionMotion          → sendPositionCommandWithYawAngle()
├── SpeedMotion             → sendSpeedCommandWithYawSpeed()
├── SpeedInAPlaneMotion     → 수평 속도 + 수직 위치 혼합
├── TrajectoryMotion        → TrajectorySetpoints 전체
├── AcroMotion              → attitude rate 명령
└── HoverMotion             → 레퍼런스 없음 (수동 호버)
```

---

## 5. 비헤이비어 아키텍처

### 템플릿 기반 액션 서버

```cpp
template<class ActionType>
class BehaviorServer : public as2::Node {
    rclcpp_action::Server<ActionType>

    // 구현체가 오버라이드해야 하는 추상 메서드
    virtual bool on_activate(goal)          // 비헤이비어 시작
    virtual bool on_modify(goal)            // 실행 중 목표 변경
    virtual bool on_deactivate(message)     // 정지
    virtual bool on_pause(message)          // 일시 정지
    virtual bool on_resume(message)         // 재개
    virtual ExecutionStatus on_run()        // 실행 루프 (매 틱)
    virtual void on_execution_end(status)   // 종료 후 정리

    // 자동 제공 서비스
    // /<behavior>/_behavior/pause
    // /<behavior>/_behavior/resume
    // /<behavior>/_behavior/stop
    // /<behavior>/_behavior/modify

    // 자동 제공 토픽
    // /<behavior>/_behavior/behavior_status
}
```

### 이중 플러그인 구조

```
TakeoffBehavior (BehaviorServer 상속)
└── pluginlib::ClassLoader<TakeoffBase>
    ├── takeoff_plugin_platform  → 플랫폼 네이티브 이륙
    ├── takeoff_plugin_position  → 목표 고도로 위치 이동
    └── takeoff_plugin_speed     → 수직 속도 램프업
```

### 비헤이비어 카테고리

| 카테고리 | 비헤이비어 | 플러그인 |
|---|---|---|
| 모션 | Takeoff, Land, GoTo, FollowPath, FollowReference | O |
| 경로 계획 | A*, Voronoi, RRT | O |
| 인지 | ArucoMarker 감지 | O |
| 페이로드 | Gripper, PointGimbal | O |
| 궤적 | 다항식 궤적 생성 | X |
| 군집 | Swarm Flocking | X |

---

## 6. 플랫폼 추상화

### 계층 구조

```
as2::AerialPlatform (추상)
├── as2_platform_gazebo            → geometry_msgs/Twist → Gazebo
├── as2_platform_multirotor_sim    → 내장 멀티로터 시뮬레이터
└── (실제 하드웨어 플랫폼)           → PX4 / MAVLink 등
```

### 새 플랫폼 구현 시 오버라이드 메서드

| 메서드 | 역할 |
|---|---|
| `configureSensors()` | 센서 토픽 설정 |
| `ownSendCommand()` | 실제 명령 전송 |
| `ownSetArmingState(bool)` | arm / disarm |
| `ownSetOffboardControl(bool)` | offboard 전환 |
| `ownSetPlatformControlMode()` | 제어 모드 설정 |
| `ownTakeoff()` / `ownLand()` | 네이티브 이착륙 |
| `ownKillSwitch()` | 긴급 정지 |
| `ownStopPlatform()` | 플랫폼 정지 |

### 플랫폼이 수신하는 명령 토픽

| 토픽 | 메시지 타입 |
|---|---|
| `actuator_command/pose` | `geometry_msgs/PoseStamped` |
| `actuator_command/twist` | `geometry_msgs/TwistStamped` |
| `actuator_command/thrust` | `as2_msgs/Thrust` |
| `actuator_command/trajectory` | `as2_msgs/TrajectorySetpoints` |
| `alert_event` | `as2_msgs/AlertEvent` |

---

## 7. as2_msgs 메시지·서비스·액션 정의

### 메시지 파일 (26개)

| 메시지 | 역할 |
|---|---|
| `Acro` | Acrobatic 제어 (angle rate + thrust) |
| `AlertEvent` | 경고/비상 이벤트 |
| `BehaviorStatus` | 비헤이비어 실행 상태 |
| `ControlMode` | 제어 모드 (3차원) |
| `ControllerInfo` | 컨트롤러 정보 |
| `FollowTargetInfo` | 타겟 추종 정보 |
| `Geozone` | 지오존 정의 |
| `GimbalControl` | 짐벌 제어 |
| `MissionEvent` / `MissionUpdate` | 미션 이벤트/업데이트 |
| `NodeStatus` | 노드 상태 |
| `PlatformInfo` | 플랫폼 정보 (광역 브로드캐스트) |
| `PlatformStateMachineEvent` | FSM 이벤트 |
| `PlatformStatus` | FSM 현재 상태 |
| `PolygonList` | 지오존 폴리곤 목록 |
| `PoseStampedWithID` / `PoseWithID` (+ Array 변형) | ID 포함 포즈 |
| `Speed` / `Thrust` | 속도/추력 |
| `TrajGenInfo` / `TrajectoryPoint` / `TrajectorySetpoints` | 궤적 관련 |
| `UInt16MultiArrayStamped` | 스탬프 배열 |
| `YawMode` | Yaw 제어 모드 |

### 메시지별 Publisher / Subscriber

| 메시지 | Publisher | Subscriber |
|---|---|---|
| `Acro` | `as2_platform_gazebo` | `as2_gazebo_assets` |
| `AlertEvent` | `as2_geozones`, `as2_rviz_plugins` | `as2_core`, `as2_behavior_tree` |
| `BehaviorStatus` | `as2_behavior` | `as2_python_api` |
| `ControllerInfo` | `as2_motion_controller` | `as2_motion_reference_handlers`, `as2_alphanumeric_viewer` |
| `GimbalControl` | `as2_behaviors_payload` | `as2_platform_multirotor_sim`, `as2_gazebo_assets` |
| `PlatformInfo` | `as2_core` | `as2_motion_controller`, `as2_behaviors_motion`, `as2_behavior_tree`, `as2_alphanumeric_viewer`, `as2_python_api` |
| `PoseStampedWithIDArray` | `as2_behaviors_perception` | `as2_behaviors_trajectory_generation` |
| `PoseWithIDArray` | — | `as2_behaviors_swarm_flocking` |
| `PolygonList` | `as2_geozones` | `as2_visualization` |
| `Thrust` | `as2_motion_controller`, `as2_motion_reference_handlers` | `as2_core`, `as2_behaviors_param_estimation`, `as2_alphanumeric_viewer` |
| `TrajectorySetpoints` | `as2_motion_reference_handlers`, `as2_motion_controller` | `as2_core`, `as2_motion_controller` |

---

## 8. Python API

### 클래스 구조

```
DroneInterfaceBase (rclpy.node.Node 상속)
│  - 백그라운드 스레드 자동 스핀
│  - SUB: platform/info, self_localization/pose·twist
│  - PUB: alert_event
│  - SRV: arm, disarm, offboard, manual
│  - 동적 모듈 로딩: load_module(pkg)
│
└── DroneInterface
       - 사전 로드 모듈
         ├── takeoff   (TakeoffModule)
         ├── go_to     (GoToModule)
         ├── follow_path (FollowPathModule)
         └── land      (LandModule)
```

### 사용 예시

```python
import rclpy
from as2_python_api.drone_interface import DroneInterface

rclpy.init()
drone = DroneInterface(drone_id='drone0', use_sim_time=True)

drone.takeoff(height=2.0, speed=0.5)
drone.go_to.go_to(x=5, y=5, z=2, speed=1.0, yaw_mode=YawMode.FIXED_YAW)
drone.follow_path.follow_path(path_points, speed=1.0)
drone.land(speed=0.3)

drone.destroy_node()
rclpy.shutdown()
```

---

## 9. 런치 시스템

### 설정 기반 런치 패턴

```python
# 모든 패키지의 공통 패턴
DeclareLaunchArgumentsFromConfigFile(
    name='config_file', source_file=platform_config_file)

Node(
    package='as2_platform_multirotor_simulator',
    executable='as2_platform_multirotor_simulator_node',
    namespace=LaunchConfiguration('namespace'),
    parameters=[
        {'use_sim_time': LaunchConfiguration('use_sim_time')},
        LaunchConfigurationFromConfigFile('config_file'),
        LaunchConfigurationFromConfigFile('uav_config'),
    ]
)
```

### 주요 설정 파일

| 파일 | 내용 |
|---|---|
| `platform_config_file.yaml` | 플랫폼 파라미터 |
| `control_modes.yaml` | 지원 제어 모드 목록 |
| `uav_config.yaml` | 기체 물리 특성 |
| `world_config.yaml` | 환경/세계 설정 |

---

## 10. 핵심 설계 패턴

| 패턴 | 적용 위치 | 목적 |
|---|---|---|
| **pluginlib 플러그인** | 컨트롤러, 비헤이비어, 상태추정, 맵서버 | 런타임 알고리즘 교체 |
| **템플릿 액션서버** | 모든 비헤이비어 | 타입 안전한 제네릭 비헤이비어 |
| **라이프사이클 노드** | 컴파일 타임 스위치 (`AS2_NODE_FATHER`) | 표준/라이프사이클 노드 선택 |
| **6-상태 FSM** | AerialPlatform | 플랫폼 상태 일관성 보장 |
| **동기식 서비스 클라이언트** | 전체 서비스 호출 | 블로킹 래퍼 (`as2::SynchronousServiceClient`) |
| **TF 프레임 변환** | 컨트롤러, 비헤이비어 | ENU / FLU / GPS 좌표계 통일 |
| **YAML 설정 기반 런치** | 모든 패키지 | 파라미터 외부화 |
| **네임스페이스 격리** | 전체 시스템 | 다중 드론 동시 운용 (`/drone0/`, `/drone1/`) |

---

## 11. 확장 방법

### 새 비헤이비어 추가

1. `as2_behaviors_<category>/<behavior_name>` 패키지 생성
2. `BehaviorBase`를 상속하는 플러그인 기반 클래스 작성
3. `BehaviorServer<ActionType>` 상속 후 추상 메서드 구현
4. `plugins.xml`에 플러그인 등록
5. 런치 파일 생성

### 새 플랫폼 추가

1. `as2_aerial_platforms/<platform_name>` 패키지 생성
2. `as2::AerialPlatform` 상속
3. 추상 메서드 구현 (`ownSendCommand` 등 8개)
4. 설정 파일 및 런치 파일 작성

### 새 컨트롤러 추가

1. `as2_motion_controller_plugin_base::ControllerBase` 상속
2. 필수 메서드 구현: `ownInitialize()`, `updateState()`, `updateReference()`, `computeOutput()`, `setMode()`
3. `as2_motion_controller/plugins.xml`에 등록

---

## 12. 전체 데이터 흐름

```
┌─────────────┐     Action      ┌─────────────────────┐
│  Python API │ ─────────────► │      Behaviors       │
│  (사용자)    │                 │ (GoTo / Takeoff / ..)│
└─────────────┘                 └──────────┬──────────┘
                                           │ motion_reference/*
                                           ▼
                                ┌─────────────────────┐
                                │  Motion Reference   │
                                │     Handlers        │
                                └──────────┬──────────┘
                                           │ TrajectorySetpoints / Thrust
                                           ▼
                                ┌─────────────────────┐
                        ┌──────►  ControllerHandler  │◄─── platform/info
                        │       │  + Plugin (PID 등)  │◄─── self_localization/*
                        │       └──────────┬──────────┘
                        │                  │ actuator_command/*
                        │                  ▼
                        │       ┌─────────────────────┐
                 ControllerInfo │    AerialPlatform    │ ──► platform/info
                        │       │   (HW 추상화)        │
                        └───────┴──────────┬──────────┘
                                           │
                                ┌──────────▼──────────┐
                                │  Hardware / Gazebo  │
                                └──────────┬──────────┘
                                           │ sensor_measurements/*
                                           ▼
                                ┌─────────────────────┐
                                │   State Estimator   │ ──► self_localization/*
                                └─────────────────────┘
```

**핵심 특징:**
- 모든 레이어가 pluginlib으로 교체 가능한 개방형 구조
- 네임스페이스 기반 격리로 단일 ROS2 시스템에서 다중 드론 동시 운용
- YAML 설정 파일로 코드 변경 없이 시스템 구성 변경 가능
- Python API로 복잡한 ROS2 세부 사항을 추상화하여 사용자 편의성 제공
