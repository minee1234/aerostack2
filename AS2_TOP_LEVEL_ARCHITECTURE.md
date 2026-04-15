# Aerostack2 프레임워크 상위 개념설계

> 전체 코드베이스 분석 기반 · ROS 2 Humble · 2026-04-14

---

## 1. 전체 시스템 개요

Aerostack2는 무인 항공기(UAV)를 위한 **모듈형 다층 ROS 2 프레임워크**다.  
하드웨어 추상화부터 사용자 인터페이스까지 9개 계층으로 구성되며,  
단일 드론과 다중 드론 군집 운용을 모두 지원한다.

```
┌──────────────────────────────────────────────────────────────────────────┐
│                        AEROSTACK2 FRAMEWORK                              │
│                                                                          │
│   ┌─────────────────────────────────────────────────────────────────┐   │
│   │  L9  사용자 인터페이스 (Keyboard, RViz, Terminal Viewer)          │   │
│   └───────────────────────────┬─────────────────────────────────────┘   │
│   ┌───────────────────────────▼─────────────────────────────────────┐   │
│   │  L8  Python API  (DroneInterface, Modules, BehaviorManager)      │   │
│   └───────────────────────────┬─────────────────────────────────────┘   │
│   ┌───────────────────────────▼─────────────────────────────────────┐   │
│   │  L7  Behavior Tree  (XML 기반 미션 계획, BehaviorTree.CPP)        │   │
│   └───────────────────────────┬─────────────────────────────────────┘   │
│   ┌───────────────────────────▼─────────────────────────────────────┐   │
│   │  L6  Behaviors  (Takeoff/Land/GoTo/FollowPath/SwarmFlocking…)    │   │
│   └───────────┬───────────────────────────────────┬─────────────────┘   │
│   ┌───────────▼────────────┐         ┌────────────▼─────────────────┐   │
│   │  L5a Motion Controller │         │  L5b Motion Ref Handlers     │   │
│   │  (PID / DiffFlat)      │         │  (Pose/Twist 변환)            │   │
│   └───────────┬────────────┘         └──────────────────────────────┘   │
│   ┌───────────▼─────────────────┐  ┌───────────────────────────────┐    │
│   │  L4a State Estimator        │  │  L4b Map Server               │    │
│   │  (Odom/GroundTruth/MoCap)   │  │  (Occupancy Grid)             │    │
│   └───────────┬─────────────────┘  └───────────────────────────────┘    │
│   ┌───────────▼─────────────────────────────────────────────────────┐   │
│   │  L3  Platform & Hardware                                         │   │
│   │  (Gazebo Sim / Multirotor Sim / RealSense / USB Camera)          │   │
│   └───────────┬─────────────────────────────────────────────────────┘   │
│   ┌───────────▼─────────────────────────────────────────────────────┐   │
│   │  L2  as2_core  (Node, AerialPlatform, Sensor<T>, PlatformFSM)   │   │
│   └───────────┬─────────────────────────────────────────────────────┘   │
│   ┌───────────▼─────────────────────────────────────────────────────┐   │
│   │  L1  as2_msgs  (15 Actions · 18 Messages · 18 Services)         │   │
│   └─────────────────────────────────────────────────────────────────┘   │
└──────────────────────────────────────────────────────────────────────────┘
```

---

## 2. 레이어별 상세 블럭도

### 2-A. 데이터 플로우 (전체 흐름)

```
  ┌─────────────────────────────────────────────────────────────────────┐
  │  사용자 스크립트 / 미션 파일                                           │
  │  drone.takeoff(2.0) · drone.go_to(x,y,z) · drone.land()            │
  └────────────────────────────┬────────────────────────────────────────┘
                               │ Python 함수 호출
                               ▼
  ┌───────────────────────────────────────────────────────────────────┐
  │  as2_python_api  (L8)                                             │
  │  DroneInterface ──► TakeoffModule / GoToModule / FollowPathModule │
  │                       │ ROS 2 Action Goal 송신                     │
  └───────────────────────┼───────────────────────────────────────────┘
                          │ Action Client → Server
                          ▼
  ┌───────────────────────────────────────────────────────────────────┐
  │  as2_behaviors  (L6)                                              │
  │  TakeoffBehavior / GoToBehavior / FollowPathBehavior / …          │
  │   ├─ 현재 위치 구독: /self_localization/pose                        │
  │   ├─ 참조 발행:      /motion_reference/pose  or  /twist            │
  │   └─ 경로계획 요청:  /navigate_to_point (Action)                   │
  └───────────────────────┬───────────────────────────────────────────┘
                          │ motion_reference/* 토픽
                          ▼
  ┌───────────────────────────────────────────────────────────────────┐
  │  as2_motion_controller  (L5a)                                     │
  │  플러그인: DifferentialFlatness / PID_Speed                        │
  │   ├─ 입력: motion_reference/pose · twist · trajectory             │
  │   └─ 출력: actuator_command/thrust · attitude                     │
  └───────────────────────┬───────────────────────────────────────────┘
                          │ actuator_command/* 토픽
                          ▼
  ┌───────────────────────────────────────────────────────────────────┐
  │  as2_aerial_platforms  (L3)                                       │
  │  GazeboPlatform / MultirotorSimulator / (PX4 / ArduPilot)        │
  │   ├─ 명령 수신: actuator_command/*                                 │
  │   └─ 센서 발행: sensor_measurements/* · ground_truth/*            │
  └───────────────────────┬───────────────────────────────────────────┘
                          │ sensor_measurements/* 토픽
                          ▼
  ┌───────────────────────────────────────────────────────────────────┐
  │  as2_state_estimator  (L4a)                                       │
  │  플러그인: raw_odom / ground_truth / mocap                         │
  │   └─ 출력: self_localization/pose · twist · odom                  │
  └───────────────────────┬───────────────────────────────────────────┘
                          │ self_localization/* 토픽
                          └────────────► Behaviors 피드백 루프
```

---

### 2-B. 패키지 의존성 블럭도

```
                         ┌─────────────┐
                         │  as2_msgs   │  ◄── 모든 패키지가 의존
                         │  (L1 기반)  │
                         └──────┬──────┘
                                │
                         ┌──────▼──────┐
                         │  as2_core   │  ◄── 프레임워크 핵심 추상
                         │  (L2 기반)  │       Node, AerialPlatform
                         └──┬───┬───┬──┘       Sensor<T>, FSM
                            │   │   │
              ┌─────────────┘   │   └──────────────┐
              │                 │                  │
    ┌─────────▼───────┐  ┌──────▼──────┐  ┌───────▼────────┐
    │ as2_aerial_     │  │ as2_state_  │  │ as2_motion_    │
    │ platforms  (L3) │  │ estimator   │  │ controller (L5)│
    │ Gazebo/Sim/HW   │  │ (L4a)       │  │ PID/DiffFlat   │
    └─────────────────┘  └─────────────┘  └───────┬────────┘
                                                   │
                                    ┌──────────────┘
                                    │
                         ┌──────────▼──────────┐
                         │ as2_behavior (base) │
                         │ BasicBehavior<T>    │
                         │ BehaviorServer      │
                         └──────────┬──────────┘
                                    │
          ┌──────────────┬──────────┼──────────────┬──────────────┐
          │              │          │              │              │
  ┌───────▼──────┐ ┌─────▼─────┐ ┌─▼────────┐ ┌───▼──────┐ ┌───▼──────┐
  │ behaviors_   │ │behaviors_ │ │behaviors_│ │behaviors_│ │behaviors_│
  │ motion       │ │platform   │ │path_plan │ │payload   │ │perception│
  │ Takeoff/Land │ │Arm/Offbd  │ │A*/Voron  │ │Gripper   │ │ArUco     │
  │ GoTo/Follow  │ └───────────┘ └──────────┘ └──────────┘ └──────────┘
  └───────┬──────┘
          │
   ┌──────┴──────────────────────┐
   │                             │
┌──▼──────────────┐  ┌──────────▼──────────┐
│ as2_python_api  │  │ as2_behavior_tree   │
│ DroneInterface  │  │ BT XML Executor     │
│ TakeoffModule…  │  │ BT Behavior Nodes   │
└──────┬──────────┘  └─────────────────────┘
       │
┌──────▼──────────────┐
│ as2_user_interfaces │
│ Keyboard / RViz /   │
│ Terminal Viewer     │
└─────────────────────┘
```

---

### 2-C. 플랫폼 상태 머신 (FSM)

```
                    ┌─────────────────────────────────┐
                    │          [비상 이벤트]             │
              ┌─────▼─────────────────────────────────▼─────┐
              │              EMERGENCY                        │
              │  (자동 비상 착륙 / Hover)                      │
              └──────────────────────────────────────────────┘
                              ▲ 어느 상태에서든
                              │ Emergency 이벤트 발생 시
                              │

  ┌──────────┐  Arm   ┌──────────┐ Takeoff ┌──────────────┐
  │ DISARMED │──────►│  LANDED  │────────►│  TAKING_OFF  │
  └──────────┘        └──────────┘         └──────┬───────┘
       ▲                   ▲                       │ 목표 고도 도달
       │ Disarm             │ Land Success          ▼
       │               ┌───┴──────┐          ┌──────────┐
       └───────────────│ LANDING  │◄─────────│  FLYING  │
                        └──────────┘  Land    └──────────┘
                                      명령

  상태별 가능한 동작:
  ┌──────────────┬────────────────────────────────────────┐
  │ DISARMED     │ Arm 명령만 수락                          │
  ├──────────────┼────────────────────────────────────────┤
  │ LANDED       │ Takeoff, Disarm 수락                    │
  ├──────────────┼────────────────────────────────────────┤
  │ TAKING_OFF   │ 상태 모니터링 중 (목표 고도 확인)         │
  ├──────────────┼────────────────────────────────────────┤
  │ FLYING       │ GoTo, FollowPath, Land, 모든 이동 명령  │
  ├──────────────┼────────────────────────────────────────┤
  │ LANDING      │ 착지 감지 대기                           │
  ├──────────────┼────────────────────────────────────────┤
  │ EMERGENCY    │ 자동 안전 동작                           │
  └──────────────┴────────────────────────────────────────┘
```

---

### 2-D. 플러그인 아키텍처

```
  ┌────────────────────────────────────────────────────────────────┐
  │                     PLUGIN SYSTEM (pluginlib)                  │
  │                                                                │
  │  ┌─────────────────────┐    ┌──────────────────────────────┐  │
  │  │  Motion Controller  │    │      State Estimator         │  │
  │  │  ─────────────────  │    │  ──────────────────────────  │  │
  │  │  [ PID Speed      ] │    │  [ raw_odom              ]   │  │
  │  │  [ DiffFlatness   ] │    │  [ ground_truth          ]   │  │
  │  │                     │    │  [ mocap (MoCap4ROS2)    ]   │  │
  │  │  interface:         │    │                              │  │
  │  │  MotionController   │    │  interface:                  │  │
  │  └──────────┬──────────┘    │  StateEstimatorBase          │  │
  │             │               └──────────────────────────────┘  │
  │             │                                                  │
  │  ┌──────────▼──────────────────────────────────────────────┐  │
  │  │              Behaviors (각 패키지별 플러그인)              │  │
  │  │  ┌──────────────────┐  ┌──────────────────┐             │  │
  │  │  │ Motion Behaviors │  │Platform Behaviors│             │  │
  │  │  │  · TakeoffBeh.   │  │  · ArmBehavior   │             │  │
  │  │  │  · LandBeh.      │  │  · OffboardBeh.  │             │  │
  │  │  │  · GoToBeh.      │  └──────────────────┘             │  │
  │  │  │  · FollowPathBeh.│  ┌──────────────────┐             │  │
  │  │  │  · FollowRefBeh. │  │  Path Planning   │             │  │
  │  │  └──────────────────┘  │  · A* Plugin     │             │  │
  │  │  ┌──────────────────┐  │  · Voronoi Plugin│             │  │
  │  │  │Trajectory Gen    │  └──────────────────┘             │  │
  │  │  │  · Polynomial    │  ┌──────────────────┐             │  │
  │  │  └──────────────────┘  │   Perception     │             │  │
  │  │                         │  · ArUco Detect  │             │  │
  │  │                         └──────────────────┘             │  │
  │  └─────────────────────────────────────────────────────────┘  │
  └────────────────────────────────────────────────────────────────┘
```

---

### 2-E. ROS 2 통신 인터페이스 맵

```
  네임스페이스 규칙: /{drone_id}/... (예: /drone0/...)

  ┌──────────────────────────────────────────────────────────────────┐
  │                      TOPIC MAP                                   │
  │                                                                  │
  │  PLATFORM ──► sensor_measurements/imu          ──► StateEst     │
  │  PLATFORM ──► sensor_measurements/gps          ──► StateEst     │
  │  PLATFORM ──► sensor_measurements/odom         ──► StateEst     │
  │  PLATFORM ──► sensor_measurements/lidar        ──► Perception   │
  │  PLATFORM ──► sensor_measurements/camera       ──► Perception   │
  │  PLATFORM ──► sensor_measurements/battery      ──► Monitor      │
  │  PLATFORM ──► ground_truth/pose                ──► StateEst     │
  │                                                                  │
  │  StateEst ──► self_localization/pose           ──► Behaviors    │
  │  StateEst ──► self_localization/twist          ──► Behaviors    │
  │  StateEst ──► self_localization/odom           ──► Behaviors    │
  │                                                                  │
  │  Behaviors ──► motion_reference/pose          ──► Controller    │
  │  Behaviors ──► motion_reference/twist         ──► Controller    │
  │  Behaviors ──► motion_reference/trajectory    ──► Controller    │
  │  Behaviors ──► motion_reference/thrust        ──► Controller    │
  │                                                                  │
  │  Controller ──► actuator_command/thrust       ──► Platform      │
  │  Controller ──► actuator_command/attitude     ──► Platform      │
  │                                                                  │
  │  Platform ──► platform/info                   ──► Monitor       │
  │  Platform ──► platform/status                 ──► Monitor       │
  │  Platform ──► alert_event                     ──► Monitor       │
  └──────────────────────────────────────────────────────────────────┘

  ┌──────────────────────────────────────────────────────────────────┐
  │                      SERVICE MAP                                 │
  │                                                                  │
  │  Python API  ──► set_arming_state              ──► Platform FSM │
  │  Python API  ──► set_offboard_mode             ──► Platform FSM │
  │  Behaviors   ──► controller/set_control_mode   ──► Controller   │
  │  Behaviors   ──► controller/list_control_modes ──► Controller   │
  │  Map Server  ──► set_origin / get_origin        ──► GPS Ref     │
  │  Map Server  ──► path_to_geopath               ──► Coord Conv  │
  │  Utilities   ──► set_geozone / get_geozone     ──► Zone Mgmt   │
  └──────────────────────────────────────────────────────────────────┘

  ┌──────────────────────────────────────────────────────────────────┐
  │                      ACTION MAP                                  │
  │                                                                  │
  │  Client            Action Server               Behavior Node     │
  │  ──────            ─────────────               ─────────────     │
  │  PythonAPI ──► takeoff              ──► TakeoffBehavior         │
  │  PythonAPI ──► land                 ──► LandBehavior            │
  │  PythonAPI ──► go_to               ──► GoToBehavior            │
  │  PythonAPI ──► follow_path         ──► FollowPathBehavior       │
  │  PythonAPI ──► follow_reference    ──► FollowReferenceBehavior  │
  │  PythonAPI ──► navigate_to_point   ──► NavigateBehavior         │
  │  BT Node   ──► generate_poly_traj  ──► TrajGenBehavior          │
  │  BT Node   ──► swarm_flocking      ──► SwarmFlock Behavior      │
  │  BT Node   ──► detect_aruco_markers──► ArucoDetectBehavior     │
  │  BT Node   ──► point_gimbal        ──► GimbalBehavior           │
  │  BT Node   ──► gripper_handler     ──► GripperBehavior          │
  │  Diagnostic ──► mass_estimation   ──► MassEstimBehavior         │
  │  Diagnostic ──► force_estimation  ──► ForceEstimBehavior        │
  └──────────────────────────────────────────────────────────────────┘
```

---

### 2-F. 다중 드론 (Multi-Robot) 구조

```
  ┌────────────────────────────────────────────────────────────────┐
  │                   MULTI-ROBOT SYSTEM                           │
  │                                                                │
  │  ┌─────────────────────────────────────────────────────────┐  │
  │  │            Ground Station / Mission Planner             │  │
  │  │   Python Script  or  Behavior Tree XML                  │  │
  │  └──────┬────────────────────┬────────────────────┬────────┘  │
  │         │                   │                    │            │
  │   drone0│             drone1│              drone2│            │
  │         ▼                   ▼                    ▼            │
  │  ┌──────────────┐  ┌────────────────┐  ┌────────────────┐    │
  │  │  /drone0/    │  │   /drone1/     │  │   /drone2/     │    │
  │  │  Behaviors   │  │   Behaviors    │  │   Behaviors    │    │
  │  │  Controller  │  │   Controller   │  │   Controller   │    │
  │  │  StateEst    │  │   StateEst     │  │   StateEst     │    │
  │  │  Platform    │  │   Platform     │  │   Platform     │    │
  │  └──────┬───────┘  └───────┬────────┘  └───────┬────────┘    │
  │         │                  │                   │             │
  │         └──────────────────┼───────────────────┘             │
  │                            │                                  │
  │                  SwarmFlocking Action                         │
  │                  (드론 간 상대 위치 공유)                        │
  │                  PoseWithID / PoseStampedWithID               │
  └────────────────────────────────────────────────────────────────┘
```

---

### 2-G. 미션 실행 시퀀스 (Takeoff 예시)

```
  사용자                Python API           Behavior           Controller        Platform
   │                      │                    │                   │                │
   │  drone.takeoff(2.0)   │                    │                   │                │
   │─────────────────────►│                    │                   │                │
   │                       │  Action Goal       │                   │                │
   │                       │  (height=2.0)      │                   │                │
   │                       │───────────────────►│                   │                │
   │                       │                    │ sub: pose/twist   │                │
   │                       │                    │◄──────────────────┼────────────────│
   │                       │                    │                   │                │
   │                       │                    │ pub: motion_ref/  │                │
   │                       │                    │      pose         │                │
   │                       │                    │──────────────────►│                │
   │                       │                    │                   │ actuator_cmd/  │
   │                       │                    │                   │ thrust         │
   │                       │                    │                   │───────────────►│
   │                       │  Feedback          │                   │                │
   │                       │  (height=0.5m)     │                   │  sensor_meas/  │
   │                       │◄───────────────────│◄──────────────────┼────────────────│
   │                       │                    │  (state update)   │                │
   │                (반복: 목표 고도까지)         │                   │                │
   │                       │  Result: Success   │                   │                │
   │                       │◄───────────────────│                   │                │
   │  return True          │                    │                   │                │
   │◄──────────────────────│                    │                   │                │
```

---

## 3. 핵심 설계 원칙

### 3-A. 관심사 분리 (Separation of Concerns)

```
  ┌─────────────────────────────────────────────────────────────┐
  │                  설계 원칙 매트릭스                            │
  │                                                             │
  │  계층         역할              기술            인터페이스     │
  │  ─────────    ────────────────  ────────────    ──────────── │
  │  메시지 (L1)  데이터 정의       IDL (.msg/.srv  패키지 의존성  │
  │                               /.action)                    │
  │  코어  (L2)   추상 클래스       C++ 템플릿      상속/구현      │
  │  플랫폼(L3)   HW 추상화        C++ + ROS 2     토픽 발행/구독 │
  │  추정  (L4)   상태 추정         플러그인         토픽 발행      │
  │  제어  (L5)   동역학 제어        플러그인         토픽 구독/발행 │
  │  행동  (L6)   임무 원자 단위     플러그인         Action 서버   │
  │  BT   (L7)   미션 합성          XML 파일        Action 클라이언│
  │  API  (L8)   사용자 인터페이스  Python          함수 호출      │
  │  UI   (L9)   시각화/조작        RViz/Keyboard   토픽 발행/구독 │
  └─────────────────────────────────────────────────────────────┘
```

### 3-B. 확장 포인트 (Extension Points)

```
  새 드론 플랫폼 추가
  ─────────────────────────────────────────────────────
  as2::AerialPlatform 상속
  → configureSensors() 구현
  → ownSetArmingState() 구현
  → ownSendCommand() 구현
  → CMakeLists.txt에 등록

  새 컨트롤러 알고리즘 추가
  ─────────────────────────────────────────────────────
  MotionController 인터페이스 구현
  → plugins.xml 등록
  → 파라미터 파일 작성

  새 Behavior 추가
  ─────────────────────────────────────────────────────
  as2::BasicBehavior<ActionT> 상속
  → onAccepted() 구현
  → onExecute() 구현
  → onCancel() 구현
  → 대응 Action 메시지 정의

  새 추정기 추가
  ─────────────────────────────────────────────────────
  StateEstimatorBase 구현
  → plugins.xml 등록
  → 센서 데이터 구독 설정
```

---

## 4. 기술 스택 요약

```
  ┌────────────────────────────────────────────────────────────┐
  │                   TECHNOLOGY STACK                         │
  │                                                            │
  │  Runtime          ROS 2 Humble (Ubuntu 22.04)              │
  │  Language         C++ 17 / Python 3.10                     │
  │  Build            CMake 3 + ament_cmake                    │
  │  Actions/Srv      rclcpp_action, rclcpp_lifecycle          │
  │  Plugins          pluginlib (동적 로딩)                     │
  │  Mission Logic    BehaviorTree.CPP v3/v4                   │
  │  Math/Geometry    Eigen3, tf2, tf2_ros                     │
  │  Simulation       Gazebo (Fortress/Garden), Custom Sim     │
  │  GPS              pymap3d (geodetic ↔ ENU 변환)             │
  │  Validation       pydantic (Python API)                    │
  │  CI/CD            GitHub Actions                           │
  │  Docs             Doxygen                                  │
  └────────────────────────────────────────────────────────────┘
```

---

## 5. 패키지 목록 및 역할 요약

| 패키지 | 레이어 | 역할 | 언어 |
|---|---|---|---|
| `as2_msgs` | L1 | Actions·Messages·Services 정의 (15+18+18) | IDL |
| `as2_core` | L2 | Node·AerialPlatform·Sensor·FSM 추상 클래스 | C++ |
| `as2_platform_gazebo` | L3 | Gazebo 시뮬레이터 인터페이스 | C++ |
| `as2_platform_multirotor_simulator` | L3 | 경량 물리 시뮬레이터 | C++ |
| `as2_realsense_interface` | L3 | Intel RealSense 드라이버 | C++ |
| `as2_usb_camera_interface` | L3 | USB 카메라 드라이버 | C++ |
| `as2_gazebo_assets` | L3 | Gazebo 3D 모델 및 월드 | XML/SDF |
| `as2_state_estimator` | L4 | 상태 추정 (플러그인: odom/GT/mocap) | C++ |
| `as2_map_server` | L4 | 점유 격자 맵 서버 | C++ |
| `as2_motion_reference_handlers` | L5 | 참조 신호 변환 핸들러 | C++/Python |
| `as2_motion_controller` | L5 | PID·DiffFlat 제어기 (플러그인) | C++ |
| `as2_behavior` | L6 | Behavior 기반 클래스 (BehaviorServer) | C++ |
| `as2_behaviors_motion` | L6 | Takeoff·Land·GoTo·FollowPath·FollowRef | C++ |
| `as2_behaviors_platform` | L6 | Arm·Offboard 행동 | C++ |
| `as2_behaviors_path_planning` | L6 | A*·Voronoi 경로 계획 | C++ |
| `as2_behaviors_trajectory_generation` | L6 | 다항식 궤적 생성 | C++ |
| `as2_behaviors_swarm_flocking` | L6 | 군집 비행 행동 | C++ |
| `as2_behaviors_payload` | L6 | Gripper·Gimbal 페이로드 제어 | C++ |
| `as2_behaviors_perception` | L6 | ArUco 마커 검출 | C++ |
| `as2_behaviors_param_estimation` | L6 | 질량·힘 파라미터 추정 | C++ |
| `as2_behavior_tree` | L7 | BehaviorTree.CPP 기반 미션 실행 | C++ |
| `as2_python_api` | L8 | DroneInterface·Modules·BehaviorManager | Python |
| `as2_keyboard_teleoperation` | L9 | 키보드 원격 조작 | Python |
| `as2_alphanumeric_viewer` | L9 | 터미널 상태 뷰어 | C++ |
| `as2_visualization` | L9 | RViz 플러그인 | C++ |
| `as2_geozones` | Util | 비행 금지 구역 관리 | C++ |
| `as2_external_object_to_tf` | Util | 외부 객체 TF 변환 | C++ |
| `as2_cli` | Util | Bash 커맨드라인 도구 | Bash |

---

*이 문서는 `E:/git/aerostack2` 전체 코드베이스 정적 분석을 기반으로 생성됨.*
