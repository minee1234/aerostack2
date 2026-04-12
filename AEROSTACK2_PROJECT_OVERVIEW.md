# Aerostack2 프로젝트 전체 구조 이해 자료

> ROS 2 기반 다중 항공 로봇 프레임워크 — 아키텍처 및 구성 요소 완전 분석

- **버전**: 1.1.3
- **라이선스**: BSD-3-Clause
- **개발**: CVAR-UPM (Universidad Politécnica de Madrid)
- **ROS 버전**: ROS 2 Humble (Ubuntu 22.04)

---

## 목차

1. [한 줄 요약 — Aerostack2란?](#1-한-줄-요약--aerostack2란)
2. [전체 계층 구조](#2-전체-계층-구조)
3. [패키지 구성 지도](#3-패키지-구성-지도)
4. [각 레이어 상세 설명](#4-각-레이어-상세-설명)
   - [메시지 레이어 (as2_msgs)](#41-메시지-레이어--as2_msgs)
   - [코어 레이어 (as2_core)](#42-코어-레이어--as2_core)
   - [플랫폼 레이어](#43-플랫폼-레이어)
   - [상태 추정 / 맵 레이어](#44-상태-추정--맵-레이어)
   - [제어 레이어](#45-제어-레이어)
   - [행동 레이어 (as2_behaviors)](#46-행동-레이어--as2_behaviors)
   - [Behavior Tree 레이어](#47-behavior-tree-레이어)
   - [Python API 레이어](#48-python-api-레이어)
   - [사용자 인터페이스 레이어](#49-사용자-인터페이스-레이어)
   - [유틸리티 / CLI](#410-유틸리티--cli)
5. [ROS2 통신 전체 지도](#5-ros2-통신-전체-지도)
6. [드론 상태 머신](#6-드론-상태-머신)
7. [데이터 흐름 — 이륙 예시](#7-데이터-흐름--이륙-예시)
8. [패키지 의존성 전체 지도](#8-패키지-의존성-전체-지도)
9. [플러그인 시스템 구조](#9-플러그인-시스템-구조)
10. [시뮬레이션 vs 실기체 구성 차이](#10-시뮬레이션-vs-실기체-구성-차이)
11. [코드 규모 통계](#11-코드-규모-통계)
12. [디렉토리 전체 트리](#12-디렉토리-전체-트리)

---

## 1. 한 줄 요약 — Aerostack2란?

```
"드론 한 대부터 수십 대의 무리 비행까지,
 하드웨어에 관계없이 통일된 인터페이스로 제어할 수 있는
 ROS 2 기반 항공 로봇 미들웨어 프레임워크"
```

핵심 철학:

```
┌─────────────────────────────────────────────────────────────────┐
│                                                                 │
│  개발자가 "무엇을 할지"만 기술하면                                │
│  Aerostack2가 "어떻게 할지"를 처리한다                           │
│                                                                 │
│  예:  drone.takeoff(height=2.0)                                  │
│       → 무장 → 오프보드 전환 → 고도 제어 → 완료 자동 처리          │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

---

## 2. 전체 계층 구조

```
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  [ 사용자 영역 ]

  ┌─────────────────────────────────────────────────────────────┐
  │  Layer 9 │  사용자 인터페이스                                │
  │          │  as2_user_interfaces                             │
  │          │  키보드 조종 / RViz / 텍스트 뷰어                 │
  └────────────────────────────┬────────────────────────────────┘
                               │
  ┌────────────────────────────▼────────────────────────────────┐
  │  Layer 8 │  Python API                                      │
  │          │  as2_python_api                                  │
  │          │  drone.takeoff() / drone.go_to() / ...           │
  └────────────────────────────┬────────────────────────────────┘
                               │
  ┌────────────────────────────▼────────────────────────────────┐
  │  Layer 7 │  Behavior Tree                                   │
  │          │  as2_behavior_tree                               │
  │          │  XML 기반 미션 로직 / BehaviorTree.CPP            │
  └────────────────────────────┬────────────────────────────────┘

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  [ 로직 영역 ]

  ┌────────────────────────────▼────────────────────────────────┐
  │  Layer 6 │  Behaviors (행동)                                │
  │          │  as2_behaviors_*                                 │
  │          │  이륙/착륙/이동/경로계획/궤적/무리비행/인식/페이로드│
  └────────────────────────────┬────────────────────────────────┘

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  [ 제어 영역 ]

  ┌────────────────────────────▼────────────────────────────────┐
  │  Layer 5 │  제어 (Control)                                  │
  │          │  as2_motion_controller                           │
  │          │  PID / DifferentialFlatness                      │
  │          │  as2_motion_reference_handlers                   │
  │          │  위치/속도/자세 참조 신호 처리                     │
  └────────────────────────────┬────────────────────────────────┘
                               │
  ┌────────────────────────────▼────────────────────────────────┐
  │  Layer 4 │  상태 추정 / 맵                                   │
  │          │  as2_state_estimator  (위치/자세 추정)            │
  │          │  as2_map_server       (Occupancy Grid)           │
  └────────────────────────────┬────────────────────────────────┘

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  [ 기반 영역 ]

  ┌────────────────────────────▼────────────────────────────────┐
  │  Layer 3 │  Platform (플랫폼)                               │
  │          │  as2_aerial_platforms                            │
  │          │  Gazebo / 물리시뮬 / 실제 드론 드라이버           │
  │          │  as2_hardware_drivers  (RealSense, USB 카메라)   │
  │          │  as2_simulation_assets (3D 모델, world 파일)     │
  └────────────────────────────┬────────────────────────────────┘
                               │
  ┌────────────────────────────▼────────────────────────────────┐
  │  Layer 2 │  Core (핵심 프레임워크)                           │
  │          │  as2_core                                        │
  │          │  Node / AerialPlatform / Sensor / TF / FSM      │
  └────────────────────────────┬────────────────────────────────┘
                               │
  ┌────────────────────────────▼────────────────────────────────┐
  │  Layer 1 │  Messages (메시지 정의)                           │
  │          │  as2_msgs                                        │
  │          │  15 Actions / 18 Messages / 18 Services          │
  └─────────────────────────────────────────────────────────────┘

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
```

---

## 3. 패키지 구성 지도

```
aerostack2/ (루트 — 메타패키지)
│
├── as2_msgs/                          ← [L1] 메시지 정의
│
├── as2_core/                          ← [L2] 핵심 프레임워크
│
├── as2_aerial_platforms/              ← [L3] 플랫폼
│   ├── as2_platform_gazebo/           ←   Gazebo 시뮬레이터
│   └── as2_platform_multirotor_simulator/ ← 경량 물리 시뮬레이터
│
├── as2_hardware_drivers/              ← [L3] 하드웨어 드라이버
│   ├── as2_realsense_interface/       ←   Intel RealSense
│   └── as2_usb_camera_interface/      ←   USB 카메라
│
├── as2_simulation_assets/             ← [L3] 시뮬레이션 에셋
│   └── as2_gazebo_assets/             ←   3D 모델, Gazebo world
│
├── as2_state_estimator/               ← [L4] 상태 추정기
│   └── plugins/: raw_odom, ground_truth, mocap
│
├── as2_map_server/                    ← [L4] 맵 서버
│
├── as2_motion_reference_handlers/     ← [L5] 참조 신호 처리기
│
├── as2_motion_controller/             ← [L5] 모션 제어기
│   └── plugins/: differential_flatness, pid
│
├── as2_behaviors/                     ← [L6] 행동 패키지 묶음
│   ├── as2_behavior/                  ←   기반 클래스 (BehaviorServer)
│   ├── as2_behaviors_motion/          ←   이륙/착륙/이동/경로추종
│   ├── as2_behaviors_platform/        ←   Arm/Offboard
│   ├── as2_behaviors_path_planning/   ←   A*, Voronoi
│   ├── as2_behaviors_trajectory_generation/ ← 다항식 궤적
│   ├── as2_behaviors_swarm_flocking/  ←   군집 비행
│   ├── as2_behaviors_payload/         ←   그리퍼, 짐벌
│   ├── as2_behaviors_perception/      ←   ArUco 마커 감지
│   └── as2_behaviors_param_estimation/ ← 질량/힘 추정
│
├── as2_behavior_tree/                 ← [L7] Behavior Tree 실행기
│
├── as2_python_api/                    ← [L8] Python 고수준 API
│
├── as2_user_interfaces/               ← [L9] 사용자 인터페이스
│   ├── as2_keyboard_teleoperation/    ←   키보드 조종
│   ├── as2_alphanumeric_viewer/       ←   터미널 상태 뷰어
│   └── as2_visualization/             ←   RViz 플러그인
│
├── as2_utilities/                     ← 유틸리티
│   ├── as2_external_object_to_tf/     ←   외부 객체 → TF 변환
│   └── as2_geozones/                  ←   비행 금지 구역 관리
│
└── as2_cli/                           ← CLI 도구 (Bash 기반)
```

---

## 4. 각 레이어 상세 설명

### 4.1 메시지 레이어 — as2_msgs

모든 패키지가 공유하는 **공통 언어(인터페이스)** 정의 패키지.

```
as2_msgs/
├── action/ (15개)               ← 비동기 장시간 작업
│   ├── SetArmingState.action    ← 모터 무장/해제
│   ├── SetOffboardMode.action   ← 오프보드 모드 설정
│   ├── Takeoff.action           ← 이륙
│   ├── Land.action              ← 착륙
│   ├── GoToWaypoint.action      ← 좌표 이동
│   ├── FollowPath.action        ← 경로 추종
│   ├── FollowReference.action   ← 참조점 추종
│   ├── GeneratePolynomialTrajectory.action ← 궤적 생성
│   ├── NavigateToPoint.action   ← 장애물 회피 항법
│   ├── SwarmFlocking.action     ← 군집 비행
│   ├── DetectArucoMarkers.action← ArUco 감지
│   ├── GripperHandler.action    ← 그리퍼 제어
│   ├── PointGimbal.action       ← 짐벌 포인팅
│   ├── ForceEstimation.action   ← 외부 힘 추정
│   └── MassEstimation.action    ← 질량 추정
│
├── msg/ (18개)                  ← 상태/데이터 메시지
│   ├── PlatformStatus.msg       ← 플랫폼 상태
│   ├── PlatformInfo.msg         ← 플랫폼 정보
│   ├── ControlMode.msg          ← 제어 모드
│   ├── BehaviorStatus.msg       ← 행동 상태
│   ├── AlertEvent.msg           ← 알림 이벤트
│   ├── MissionEvent.msg         ← 미션 이벤트
│   └── ...
│
└── srv/ (18개)                  ← 동기 서비스 요청
    ├── SetControlMode.srv       ← 제어 모드 변경
    ├── SetPlatformStateMachineEvent.srv ← FSM 이벤트
    ├── SetOrigin.srv / GetOrigin.srv    ← GPS 원점 설정
    ├── SetGeozone.srv / GetGeozone.srv  ← 지오존 관리
    └── ...
```

**ControlMode 메시지 구조:**

```
ControlMode.msg
├── control_mode:  POSITION(0) / SPEED(1) / SPEED_IN_A_PLANE(2)
│                  ACCEL(3) / ATTITUDE(4) / ACRO(5)
├── yaw_mode:      YAW_ANGLE(0) / YAW_SPEED(1) / FIXED_YAW(2)
└── reference_frame: LOCAL_ENU(0) / BODY_FLU(1) / GLOBAL_ENU(2)
```

---

### 4.2 코어 레이어 — as2_core

Aerostack2의 **핵심 추상화 계층**. 모든 패키지가 이 라이브러리에 의존.

```
as2_core/include/as2_core/
│
├── node.hpp                  ← as2::Node (rclcpp::Node 확장)
│   └── 네임스페이스 자동 처리, 파라미터 편의 메서드
│
├── aerial_platform.hpp       ← as2::AerialPlatform (드론 추상 클래스)
│   ├── 가상 메서드: configureSensors(), ownSetArmingState()
│   │               ownSetOffboardMode(), ownSendCommand()
│   └── 플랫폼 FSM 자동 연결
│
├── platform_state_machine.hpp← 드론 상태 머신
│   └── DISARMED/LANDED/TAKING_OFF/FLYING/LANDING/EMERGENCY
│
├── sensor.hpp                ← as2::Sensor<T> (센서 추상화 템플릿)
│   └── 자동 구독 + 데이터 동기화
│
├── names/                    ← 토픽/서비스/액션 이름 상수
│   ├── topics.hpp            ← e.g., "self_localization/pose"
│   ├── services.hpp
│   └── actions.hpp
│
└── utils/
    ├── tf_utils.hpp          ← TF 변환 헬퍼
    ├── math_utils.hpp        ← 수학 유틸리티
    └── frame_utils.hpp       ← 좌표계 변환
```

**as2::AerialPlatform 상속 구조:**

```
rclcpp::Node
     │
     └── as2::Node              (네임스페이스, 파라미터 확장)
              │
              └── as2::AerialPlatform   (드론 공통 인터페이스)
                       │
                       ├── GazeboPlatform      (Gazebo 시뮬레이터)
                       ├── MultirotorSimulator  (내장 물리 시뮬레이터)
                       ├── PX4Platform          (PX4 펌웨어 기체)
                       └── ArduPilotPlatform    (ArduPilot 기체)
```

---

### 4.3 플랫폼 레이어

드론 하드웨어(또는 시뮬레이터)와 Aerostack2를 연결하는 **어댑터 계층**.

```
┌──────────────────────────────────────────────────────────┐
│                   Platform Layer                         │
│                                                          │
│  ┌─────────────────────┐   ┌─────────────────────────┐  │
│  │  as2_platform_gazebo │   │ as2_platform_multirotor │  │
│  │                     │   │     _simulator          │  │
│  │  Gazebo-ROS2 연동   │   │                         │  │
│  │  실제 물리 시뮬       │   │  경량 내장 시뮬레이터    │  │
│  │  (GPU 필요)         │   │  (CPU만으로 동작)        │  │
│  └─────────────────────┘   └─────────────────────────┘  │
│                                                          │
│  ┌─────────────────────────────────────────────────┐    │
│  │  실제 드론 플랫폼 (외부 프로젝트)                  │    │
│  │  as2_platform_px4 / as2_platform_ardupilot 등   │    │
│  └─────────────────────────────────────────────────┘    │
│                                                          │
│  ┌─────────────────────────────────────────────────┐    │
│  │  as2_simulation_assets                          │    │
│  │  ├── 드론 3D 모델 (SDF/URDF)                   │    │
│  │  ├── Gazebo World 파일                          │    │
│  │  └── 카메라, 라이다 센서 모델                    │    │
│  └─────────────────────────────────────────────────┘    │
│                                                          │
│  ┌─────────────────────────────────────────────────┐    │
│  │  as2_hardware_drivers                           │    │
│  │  ├── as2_realsense_interface (Intel D435 등)    │    │
│  │  └── as2_usb_camera_interface (USB 카메라)      │    │
│  └─────────────────────────────────────────────────┘    │
└──────────────────────────────────────────────────────────┘
```

**플랫폼이 제공하는 토픽:**

| 토픽 | 방향 | 설명 |
|------|------|------|
| `/<ns>/self_localization/pose` | Platform → 상위 | 드론 현재 위치/자세 |
| `/<ns>/self_localization/twist` | Platform → 상위 | 드론 현재 속도 |
| `/<ns>/platform/info` | Platform → 상위 | 플랫폼 상태 정보 |
| `/<ns>/actuator_command/thrust` | 상위 → Platform | 추력 명령 |
| `/<ns>/actuator_command/pose` | 상위 → Platform | 자세 명령 |
| `/<ns>/actuator_command/speed` | 상위 → Platform | 속도 명령 |

---

### 4.4 상태 추정 / 맵 레이어

#### as2_state_estimator — 상태 추정기

```
센서 입력                     as2_state_estimator              출력
─────────────                ─────────────────────            ──────
Odometry      ──────────────►                                 /self_localization/pose
IMU           ──────────────►  플러그인 선택:                  /self_localization/twist
GPS/GNSS      ──────────────►  ┌──────────────────┐          /tf (earth→odom→base_link)
MoCap 시스템  ──────────────►  │ raw_odom         │
Ground Truth  ──────────────►  │ ground_truth     │
                               │ mocap_pose       │
                               └──────────────────┘
```

**TF 트리 구조:**

```
earth (고정 전역 좌표계)
   └── odom (로컬 원점 좌표계)
          └── base_link (드론 본체)
                 ├── camera_link
                 ├── lidar_link
                 └── (기타 센서)
```

#### as2_map_server — 맵 서버

```
Depth Camera (포인트클라우드)  ──────►
LiDAR (2D/3D 스캔)           ──────►  as2_map_server  ──► Occupancy Grid Map
                                       (플러그인 기반)      /map 토픽
```

---

### 4.5 제어 레이어

#### as2_motion_reference_handlers — 참조 신호 처리기

행동(Behavior)에서 내린 고수준 명령을 제어기가 이해할 수 있는 형태로 변환.

```
Behavior (행동) 레이어
        │
        │  "3m 위로 이동"
        ▼
as2_motion_reference_handlers
        │
        ├── HoverMotion          → 현재 위치 유지
        ├── PositionMotion       → /motion_reference/pose
        ├── SpeedMotion          → /motion_reference/speed
        ├── SpeedInAPlaneMotion  → /motion_reference/speed (XY) + Z 고정
        └── TrajectoryMotion     → /motion_reference/trajectory
        │
        ▼
as2_motion_controller
```

#### as2_motion_controller — 모션 제어기

참조 신호를 받아 실제 액추에이터 명령으로 변환.

```
/motion_reference/* 구독
          │
          ▼
   MotionController
          │
          ├── differential_flatness 플러그인
          │   └── 미분 평탄화 이론 기반 정밀 제어
          │
          └── pid 플러그인
              └── 위치/속도 PID 제어
          │
          ▼
   /actuator_command/* 발행 → Platform
```

---

### 4.6 행동 레이어 — as2_behaviors

드론이 수행할 수 있는 **의미 있는 작업 단위** 구현.

```
                    as2_behavior
               (BehaviorServer<ActionT>)
                         │
          ┌──────────────┼──────────────┐
          │              │              │
  ┌───────▼──────┐ ┌─────▼──────┐ ┌───▼──────────┐
  │   motion     │ │  platform  │ │ path_planning│
  │              │ │            │ │              │
  │ ┌──────────┐ │ │ Arm        │ │ ┌──────────┐ │
  │ │ Takeoff  │ │ │ Offboard   │ │ │  A*      │ │
  │ │   ├─speed│ │ └────────────┘ │ │ Voronoi  │ │
  │ │   ├─pos  │ │                │ └──────────┘ │
  │ │   ├─traj │ │                └──────────────┘
  │ │   └─plat │ │
  │ ├──────────┤ │  ┌─────────────────┐
  │ │ Land     │ │  │ trajectory_gen  │
  │ ├──────────┤ │  │ (다항식 궤적)    │
  │ │ GoTo     │ │  └─────────────────┘
  │ ├──────────┤ │
  │ │FollowPath│ │  ┌─────────────────┐
  │ ├──────────┤ │  │ swarm_flocking  │
  │ │FollowRef │ │  │ (군집 비행)      │
  │ └──────────┘ │  └─────────────────┘
  └──────────────┘
          │              ┌─────────────────┐   ┌─────────────────┐
          │              │    payload       │   │   perception    │
          │              │ ┌─────────────┐ │   │ ArUco 마커 감지  │
          │              │ │  gripper    │ │   └─────────────────┘
          │              │ │  ├─dc_servo │ │
          │              │ │  └─2fingers │ │   ┌─────────────────┐
          │              │ ├─────────────┤ │   │ param_estimation│
          │              │ │  gimbal     │ │   │ 질량/힘 추정     │
          │              │ └─────────────┘ │   └─────────────────┘
          │              └─────────────────┘
```

**BehaviorServer 상태 관리:**

```
      Goal 수신
          │
     ┌────▼────┐
     │  IDLE   │
     └────┬────┘
          │ on_activate()
     ┌────▼────────┐
     │  ACTIVATING │
     └────┬────────┘
          │ 성공
     ┌────▼────┐ ◄── on_modify() (목표 수정)
     │ RUNNING │──┐
     └────┬────┘  │ on_run() 반복
          │◄──────┘
          │
    ┌─────┴──────┐
    │            │
  SUCCESS     FAILURE / ABORTED
    │            │
    └────┬───────┘
         │
    on_execution_end()
         │
     ┌───▼───┐
     │ IDLE  │
     └───────┘
```

---

### 4.7 Behavior Tree 레이어

복잡한 미션을 **XML 파일**로 선언적으로 정의하고 실행.

```
takeoff.xml:
  <BehaviorTree>
    <Sequence>
      <WaitForEvent event="start"/>
      <IsFlying/>? ← 이미 비행 중이면 스킵
      <Arm/>
      <Offboard/>
      <TakeOff height="2.0" speed="0.5"/>
    </Sequence>
  </BehaviorTree>

          ┌─────────────────────────────────┐
          │      as2_behavior_tree_node      │
          │  (bt_manager ROS2 Node)          │
          │                                  │
          │  BT::BehaviorTreeFactory         │
          │  ├── 등록된 노드 타입:            │
          │  │   Action: Arm, Disarm,        │
          │  │           Offboard, TakeOff,  │
          │  │           Land, GoTo,         │
          │  │           GoToGps, FollowPath  │
          │  │   Condition: IsFlying         │
          │  │   Decorator: WaitForEvent     │
          │  │              WaitForAlert     │
          │  │   Action: SendEvent, Echo     │
          │  │           SetOrigin, GetOrigin│
          │  │           GpsToCartesian      │
          │  │           FollowPath          │
          │  │                               │
          │  createTreeFromFile("takeoff.xml")│
          │         │                        │
          │    tickWhileRunning() 루프        │
          │    (10ms 주기)                   │
          └─────────────────────────────────┘
                         │
          각 노드가 ROS2 Action/Service 호출
```

---

### 4.8 Python API 레이어

Python 개발자를 위한 **고수준 드론 제어 인터페이스**.

```python
from as2_python_api.drone_interface import DroneInterface

# 드론 연결
drone = DroneInterface("drone0", use_sim_time=True)

# 상태 확인
print(drone.get_info())
# → {"connected": True, "armed": False, "offboard": False,
#    "state": "LANDED", "control_mode": "POSITION"}

# 비행 시퀀스
drone.arm()                              # 모터 무장
drone.offboard()                         # 오프보드 모드
drone.takeoff(height=2.0, speed=0.5)     # 이륙

drone.go_to(x=5.0, y=3.0, z=2.0,        # 이동
            speed=1.0,
            yaw_mode=YawMode.PATH_FACING)

drone.follow_path(                        # 경로 추종
    path=[(0,0,2), (5,0,2), (5,5,2)],
    speed=1.5
)

drone.land(speed=0.3)                    # 착륙
drone.disarm()                           # 모터 해제
```

**DroneInterface 내부 구조:**

```
DroneInterface
  │
  ├── 상태 구독:
  │   ├── /drone0/self_localization/pose
  │   ├── /drone0/self_localization/twist
  │   └── /drone0/platform/info
  │
  ├── 행동 클라이언트:
  │   ├── SetArmingState Action Client
  │   ├── SetOffboardMode Action Client
  │   ├── Takeoff Action Client
  │   ├── Land Action Client
  │   ├── GoToWaypoint Action Client
  │   └── FollowPath Action Client
  │
  └── GPS 유틸:
      ├── set_home() → /drone0/set_origin 서비스
      └── go_to_gps() → GPS 좌표 → ENU 변환 → GoToWaypoint
```

---

### 4.9 사용자 인터페이스 레이어

```
as2_user_interfaces/
│
├── as2_keyboard_teleoperation/
│   └── Python + as2_python_api
│       WASD: 이동, Q/E: 요우, R/F: 고도 조절
│
├── as2_alphanumeric_viewer/
│   └── C++ + ncurses
│       터미널에서 드론 상태 실시간 표시
│       ┌─────────────────────────────┐
│       │ DRONE: drone0               │
│       │ STATE:  FLYING              │
│       │ POS:    x=1.2 y=3.4 z=2.0  │
│       │ VEL:    vx=0.1 vy=0.0 vz=0 │
│       │ YAW:    45.2°               │
│       └─────────────────────────────┘
│
└── as2_visualization/
    ├── as2_rviz_plugins/
    │   └── RViz 패널: 드론 제어 UI 버튼
    └── as2_visualization/
        └── Gazebo/RViz 통합 시각화 설정
```

---

### 4.10 유틸리티 / CLI

```
as2_utilities/
├── as2_external_object_to_tf/
│   └── MoCap 마커, 사람 감지 결과 등을
│       TF 변환으로 브로드캐스트
│
└── as2_geozones/
    └── 비행 허용/금지 구역 정의
        YAML 파일로 다각형 영역 설정
        침범 시 경보 발생

as2_cli/
└── as2.bash (Bash 기반 CLI)
    ├── as2 build         ← colcon 빌드 래퍼
    ├── as2 launch        ← launch 파일 단축 실행
    └── as2 setup         ← 환경 변수 설정
```

---

## 5. ROS2 통신 전체 지도

```
네임스페이스: /drone0/

━━━━━━━━━━━━━━━ 토픽 (Topic) ━━━━━━━━━━━━━━━━━━━━━━━━━━━━

[플랫폼 → 상위 레이어]
  /drone0/self_localization/pose         (PoseStamped)
  /drone0/self_localization/twist        (TwistStamped)
  /drone0/platform/info                  (PlatformInfo)
  /drone0/sensor_measurements/imu        (Imu)
  /drone0/sensor_measurements/gps        (NavSatFix)
  /drone0/sensor_measurements/camera/*   (Image, CameraInfo)

[제어기 → 플랫폼]
  /drone0/actuator_command/thrust        (Thrust)
  /drone0/actuator_command/pose          (PoseStamped)
  /drone0/actuator_command/speed         (TwistStamped)

[행동 → 제어기]
  /drone0/motion_reference/pose          (PoseStamped)
  /drone0/motion_reference/speed         (TwistStamped)
  /drone0/motion_reference/trajectory    (JointTrajectoryPoint)

[이벤트]
  /drone0/alert_event                    (AlertEvent)
  /drone0/mission_event                  (MissionEvent)

━━━━━━━━━━━━━━━ 서비스 (Service) ━━━━━━━━━━━━━━━━━━━━━━━

  /drone0/set_control_mode              ← 제어 모드 변경
  /drone0/set_platform_state_machine_event ← FSM 이벤트
  /drone0/set_origin                    ← GPS 원점 설정
  /drone0/get_origin                    ← GPS 원점 조회
  /drone0/set_speed                     ← 최대 속도 설정

━━━━━━━━━━━━━━━ 액션 (Action) ━━━━━━━━━━━━━━━━━━━━━━━━━━

  /drone0/SetArmingState                ← 무장/해제
  /drone0/SetOffboardMode               ← 오프보드 모드
  /drone0/TakeoffBehavior               ← 이륙
  /drone0/LandBehavior                  ← 착륙
  /drone0/GoToBehavior                  ← 좌표 이동
  /drone0/FollowPathBehavior            ← 경로 추종
  /drone0/FollowReferenceBehavior       ← 참조 추종
  /drone0/GeneratePolynomialTrajectory  ← 궤적 생성
  /drone0/NavigateToPoint               ← 장애물 회피 항법
  /drone0/DetectArucoMarkers            ← ArUco 감지
  /drone0/GripperHandlerBehavior        ← 그리퍼 제어
  /drone0/PointGimbalBehavior           ← 짐벌 포인팅
  /drone0/SwarmFlockingBehavior         ← 군집 비행
```

---

## 6. 드론 상태 머신

```
                    [EMERGENCY]
                        ▲
                        │ 비상 이벤트
                        │
  ┌──────────┐  ARM  ┌──▼──────┐  TAKE_OFF  ┌────────────┐
  │ DISARMED │──────►│ LANDED  │───────────►│ TAKING_OFF │
  └──────────┘       └──┬──────┘            └─────┬──────┘
        ▲               │  ▲                      │
        │  DISARM        │  │ LAND_COMPLETE        │ TOOK_OFF
        │               │  │                      │
        │          ┌────▼──┴─────┐          ┌─────▼──────┐
        └──────────│   LANDING   │◄─────────│   FLYING   │
                   └─────────────┘  LAND    └────────────┘
                                             (정상 비행)

  상태 전이 이벤트:
  ARM           → DISARMED → LANDED
  TAKE_OFF      → LANDED → TAKING_OFF
  TOOK_OFF      → TAKING_OFF → FLYING
  LAND          → FLYING → LANDING
  LAND_COMPLETE → LANDING → LANDED
  DISARM        → LANDED → DISARMED
  EMERGENCY     → 임의 상태 → EMERGENCY
```

---

## 7. 데이터 흐름 — 이륙 예시

### Python API 호출 시

```
1. 사용자 코드
   drone.takeoff(height=2.0, speed=0.5)

2. as2_python_api
   → Takeoff Action Goal 전송
     { takeoff_height: 2.0, takeoff_speed: 0.5 }

3. TakeoffBehavior Node (/drone0)
   on_activate():
     → /drone0/set_platform_state_machine_event 서비스 호출 (TAKE_OFF 이벤트)
     → takeoff_plugin_->own_activate(goal)
   on_run():
     → takeoff_plugin_->own_run()

4. TakeoffPlugin (예: speed 플러그인)
   → as2_motion_reference_handlers::SpeedMotion 사용
   → /drone0/motion_reference/speed 발행
     { vz: 0.5 }  (위로 0.5m/s)

5. MotionController
   → /drone0/motion_reference/speed 구독
   → PID 또는 DifferentialFlatness 알고리즘 실행
   → /drone0/actuator_command/thrust 발행

6. Platform (Gazebo / 실기체)
   → actuator_command 수신
   → 모터 출력 조절 → 상승

7. StateEstimator
   → 센서 데이터로 현재 고도 계산
   → /drone0/self_localization/pose 발행

8. TakeoffPlugin
   → /drone0/self_localization/pose 구독 (state_callback)
   → 목표 고도 도달 확인 (|현재고도 - 2.0| < threshold)
   → ExecutionStatus::SUCCESS 반환

9. TakeoffBehavior
   → on_execution_end(SUCCESS) 호출
   → HoverMotion으로 전환 (현재 위치 유지)
   → Action Result { takeoff_success: true } 발행

10. drone.takeoff() 반환
```

---

## 8. 패키지 의존성 전체 지도

```
    [외부 라이브러리]
    rclcpp/rclpy │ tf2 │ pluginlib │ Eigen │ OpenCV │ yaml-cpp │ BehaviorTree.CPP
         │              │
         ▼              ▼
    ┌─────────────────────┐
    │      as2_msgs        │ ←── 모든 패키지의 공통 기반
    └──────────┬──────────┘
               │
    ┌──────────▼──────────┐
    │      as2_core        │ ←── 모든 패키지가 의존
    └──────┬───────────────┘
           │
    ┌──────┴─────────────────────────────────────────────────────┐
    │                                                            │
    ▼                                                            ▼
┌─────────────────────────┐                    ┌────────────────────────┐
│  as2_motion_reference   │                    │  as2_aerial_platforms  │
│       _handlers         │                    │  as2_hardware_drivers  │
└─────────┬───────────────┘                    │  as2_simulation_assets │
          │                                    └────────────┬───────────┘
          │                                                 │
    ┌─────▼──────────────┐                    ┌────────────▼───────────┐
    │  as2_motion_        │                    │  as2_state_estimator   │
    │    controller       │                    │  as2_map_server        │
    └─────────────────────┘                    └────────────────────────┘
                │                                           │
                └──────────────┬────────────────────────────┘
                               │
                    ┌──────────▼─────────────┐
                    │      as2_behavior        │ (BehaviorServer 기반 클래스)
                    └──────────┬──────────────┘
                               │
              ┌────────────────┼─────────────────────────┐
              │                │                         │
              ▼                ▼                         ▼
    ┌──────────────┐  ┌──────────────────┐  ┌─────────────────────┐
    │ as2_behaviors│  │as2_behaviors_    │  │ as2_behaviors_      │
    │   _motion    │  │  path_planning   │  │   trajectory_gen    │
    └──────────────┘  └──────────────────┘  └─────────────────────┘
              │
    ┌─────────┼───────────────────────────────┐
    │         │                               │
    ▼         ▼                               ▼
┌──────────┐ ┌──────────────┐  ┌──────────────────────────────┐
│ payload  │ │  perception  │  │ swarm_flocking, param_est... │
└──────────┘ └──────────────┘  └──────────────────────────────┘
                    │
         ┌──────────▼──────────┐
         │  as2_behavior_tree  │ (BT 노드가 각 행동의 Action 클라이언트 사용)
         └──────────┬──────────┘
                    │
         ┌──────────▼──────────┐
         │  as2_python_api     │ (Python 래퍼)
         └──────────┬──────────┘
                    │
         ┌──────────▼──────────┐
         │ as2_user_interfaces │
         └─────────────────────┘
```

---

## 9. 플러그인 시스템 구조

Aerostack2의 핵심 확장 메커니즘. **런타임에 알고리즘을 교체** 가능.

```
플러그인을 사용하는 패키지:

  as2_motion_controller
  └── 제어 알고리즘 플러그인
      ├── differential_flatness   (고정밀 궤적 추종)
      └── pid                     (간단한 PID 제어)

  as2_state_estimator
  └── 추정 알고리즘 플러그인
      ├── raw_odom                (원본 Odometry)
      ├── ground_truth            (시뮬 Ground Truth)
      └── mocap_pose              (모션 캡처)

  as2_behaviors_motion
  └── 행동 구현 플러그인 (11개)
      ├── takeoff: speed, position, platform, trajectory
      ├── land:    speed, platform, trajectory
      ├── go_to:   position, trajectory
      └── follow_path: position, trajectory

  as2_behaviors_path_planning
  └── 경로 계획 플러그인
      ├── a_star                  (최단 경로)
      └── voronoi                 (최대 안전 마진)

  as2_behaviors_payload
  └── 그리퍼 플러그인
      ├── dc_servo                (DC 서보 모터)
      └── two_fingers             (2축 손가락)

  as2_map_server
  └── 맵 생성 플러그인
      ├── depth_image             (뎁스 카메라)
      └── pointcloud              (포인트클라우드)
```

**플러그인 선택 방법 (YAML 설정):**

```yaml
# takeoff_behavior.yaml
takeoff_behavior:
  ros__parameters:
    plugin_name: "as2_behaviors_motion/TakeoffPluginSpeed"
    takeoff_height: 2.0
    takeoff_speed: 0.5
    takeoff_threshold: 0.1
```

---

## 10. 시뮬레이션 vs 실기체 구성 차이

```
┌────────────────────────────────────────────────────────────────┐
│                     공통 구성 요소                              │
│  as2_core / as2_msgs / as2_behaviors_* / as2_motion_controller │
│  as2_state_estimator / as2_behavior_tree / as2_python_api      │
└────────────────────────────────────────────────────────────────┘

        시뮬레이션                          실기체
        ──────────                          ──────
┌─────────────────────┐           ┌─────────────────────────┐
│ as2_platform_gazebo │           │ as2_platform_px4        │
│ (또는 multirotor_   │           │ (또는 ardupilot 등)      │
│  simulator)         │           │                         │
├─────────────────────┤           ├─────────────────────────┤
│ as2_simulation_     │           │ as2_hardware_drivers    │
│ assets              │           │ (RealSense, USB 카메라) │
│ (3D 모델, world)    │           │                         │
├─────────────────────┤           ├─────────────────────────┤
│ state_estimator     │           │ state_estimator         │
│ plugin: ground_truth│           │ plugin: mocap_pose      │
│        (시뮬 위치)   │           │         raw_odom        │
└─────────────────────┘           └─────────────────────────┘

  use_sim_time: true                use_sim_time: false
```

---

## 11. 코드 규모 통계

| 항목 | 수치 |
|------|------|
| 전체 프로젝트 크기 | 131 MB |
| C++ 소스 코드 | 약 50,579줄 |
| Python 소스 코드 | 약 21,461줄 |
| CMakeLists.txt | 72개 |
| ROS2 Action 정의 | 15개 |
| ROS2 Message 정의 | 18개 |
| ROS2 Service 정의 | 18개 |
| 총 패키지 수 | 약 35개 |
| pluginlib 플러그인 | 20개 이상 |

---

## 12. 디렉토리 전체 트리

```
aerostack2/ (메타패키지)
as2_aerial_platforms/
  └── as2_platform_gazebo/
  └── as2_platform_multirotor_simulator/
as2_behavior_tree/
  ├── include/as2_behavior_tree/
  │   ├── action/          (Arm, Takeoff, Land, GoTo, FollowPath, ...)
  │   ├── condition/        (IsFlying)
  │   └── decorator/        (WaitForEvent, WaitForAlert)
  ├── plugins/
  ├── resource/             (takeoff.xml 등 예제 XML)
  └── src/                  (as2_behavior_tree_node.cpp)
as2_behaviors/
  ├── as2_behavior/         (BehaviorServer 기반 클래스)
  ├── as2_behaviors_motion/
  │   ├── follow_path_behavior/
  │   ├── follow_reference_behavior/
  │   ├── go_to_behavior/
  │   ├── land_behavior/
  │   ├── takeoff_behavior/
  │   └── launch/
  ├── as2_behaviors_param_estimation/
  ├── as2_behaviors_path_planning/
  │   ├── common/
  │   ├── plugins/a_star/
  │   └── plugins/voronoi/
  ├── as2_behaviors_payload/
  │   ├── gripper_behavior/
  │   └── point_gimbal_behavior/
  ├── as2_behaviors_perception/
  │   └── detect_aruco_markers_behavior/
  ├── as2_behaviors_platform/
  ├── as2_behaviors_swarm_flocking/
  └── as2_behaviors_trajectory_generation/
as2_cli/                    (Bash CLI 도구)
as2_core/
  ├── include/as2_core/
  │   ├── aerial_platform.hpp
  │   ├── node.hpp
  │   ├── platform_state_machine.hpp
  │   ├── sensor.hpp
  │   ├── names/            (토픽/서비스/액션 이름 상수)
  │   └── utils/
  └── src/
as2_hardware_drivers/
  ├── as2_realsense_interface/
  └── as2_usb_camera_interface/
as2_map_server/
as2_motion_controller/
  ├── include/
  ├── plugins/
  └── src/
as2_motion_reference_handlers/
  ├── include/
  └── src/
as2_msgs/
  ├── action/ (15개)
  ├── msg/    (18개)
  └── srv/    (18개)
as2_python_api/
  ├── as2_python_api/       (Python 패키지)
  ├── examples/
  └── tests/
as2_simulation_assets/
  └── as2_gazebo_assets/    (SDF 모델, world 파일)
as2_state_estimator/
  ├── include/
  ├── plugins/
  └── src/
as2_user_interfaces/
  ├── as2_alphanumeric_viewer/
  ├── as2_keyboard_teleoperation/
  └── as2_visualization/
      ├── as2_rviz_plugins/
      └── as2_visualization/
as2_utilities/
  ├── as2_external_object_to_tf/
  └── as2_geozones/
```

---

## 참고 문서 위치

| 문서 | 경로 |
|------|------|
| 프로젝트 개요 (이 파일) | `AEROSTACK2_PROJECT_OVERVIEW.md` |
| 아키텍처 다이어그램 | `ARCHITECTURE_DIAGRAMS.md` |
| BT 실행 흐름 분석 | `BT_EXECUTION_FLOW_ANALYSIS.md` |
| as2_core 패키지 분석 | `as2_core/AS2_CORE_ANALYSIS.md` |
| as2_behavior_tree 분석 | `as2_behavior_tree/AS2_BEHAVIOR_TREE_ANALYSIS.md` |
| as2_behaviors 분석 | `as2_behaviors/AS2_BEHAVIORS_ANALYSIS.md` |
| 소스 분석 로드맵 | `SOURCE_ANALYSIS_ROADMAP.md` |
