# as2_behaviors 패키지 분석

> Aerostack2의 행동(Behavior) 시스템 전체 구조 이해 자료

---

## 목차

1. [전체 패키지 구성](#1-전체-패키지-구성)
2. [아키텍처 개요](#2-아키텍처-개요)
3. [as2_behavior — 기반 라이브러리](#3-as2_behavior--기반-라이브러리)
4. [as2_behaviors_motion — 움직임 행동](#4-as2_behaviors_motion--움직임-행동)
5. [as2_behaviors_platform — 플랫폼 제어](#5-as2_behaviors_platform--플랫폼-제어)
6. [as2_behaviors_path_planning — 경로 계획](#6-as2_behaviors_path_planning--경로-계획)
7. [as2_behaviors_trajectory_generation — 궤적 생성](#7-as2_behaviors_trajectory_generation--궤적-생성)
8. [as2_behaviors_swarm_flocking — 무리 제어](#8-as2_behaviors_swarm_flocking--무리-제어)
9. [as2_behaviors_payload — 페이로드 제어](#9-as2_behaviors_payload--페이로드-제어)
10. [as2_behaviors_perception — 인식](#10-as2_behaviors_perception--인식)
11. [as2_behaviors_param_estimation — 파라미터 추정](#11-as2_behaviors_param_estimation--파라미터-추정)
12. [플러그인 시스템](#12-플러그인-시스템)
13. [ROS2 통신 패턴](#13-ros2-통신-패턴)
14. [실행 흐름 — Takeoff 예시](#14-실행-흐름--takeoff-예시)
15. [패키지 의존성 지도](#15-패키지-의존성-지도)

---

## 1. 전체 패키지 구성

```
as2_behaviors/
├── as2_behavior/                          ← 기반 클래스 라이브러리 (모든 행동이 상속)
│
├── as2_behaviors_motion/                  ← 드론 이동 관련 행동
│   ├── takeoff_behavior/                  ← 이륙
│   ├── land_behavior/                     ← 착륙
│   ├── go_to_behavior/                    ← 목표 지점 이동
│   ├── follow_path_behavior/              ← 경로 추종
│   └── follow_reference_behavior/         ← 참조 위치 추종
│
├── as2_behaviors_platform/                ← 플랫폼 상태 제어
│   ├── set_arming_state_behavior          ← 모터 무장/해제
│   └── set_offboard_mode_behavior         ← 오프보드 모드 설정
│
├── as2_behaviors_path_planning/           ← 장애물 회피 경로 계획
│   └── plugins/
│       ├── a_star/                        ← A* 알고리즘
│       └── voronoi/                       ← Voronoi 다이어그램 기반
│
├── as2_behaviors_trajectory_generation/   ← 부드러운 궤적 생성
│   └── generate_polynomial_trajectory_behavior/
│
├── as2_behaviors_swarm_flocking/          ← 다중 드론 무리 제어
│
├── as2_behaviors_payload/                 ← 탑재 장비 제어
│   ├── gripper_behavior/                  ← 그리퍼 (집게)
│   └── point_gimbal_behavior/             ← 짐벌 포인팅
│
├── as2_behaviors_perception/              ← 환경 인식
│   └── detect_aruco_markers_behavior/     ← ArUco 마커 감지
│
└── as2_behaviors_param_estimation/        ← 드론 파라미터 추정
    ├── mass_estimation_behavior/          ← 질량 추정
    └── force_estimation_behavior/         ← 외부 힘 추정
```

---

## 2. 아키텍처 개요

### 2.1 계층 구조

```
┌──────────────────────────────────────────────────────────────────┐
│                       사용자 애플리케이션                          │
│         (BehaviorTree, Python 스크립트, 미션 플래너 등)            │
└──────────────────────────┬───────────────────────────────────────┘
                           │  ROS2 Action (Goal/Feedback/Result)
┌──────────────────────────▼───────────────────────────────────────┐
│                   Behavior Server Layer                           │
│                                                                  │
│  ┌─────────────┐  ┌──────────┐  ┌──────────┐  ┌─────────────┐  │
│  │   Takeoff   │  │  GoTo    │  │FollowPath│  │ PathPlanner │  │
│  │  Behavior   │  │ Behavior │  │ Behavior │  │  Behavior   │  │
│  └──────┬──────┘  └────┬─────┘  └────┬─────┘  └──────┬──────┘  │
│         │              │             │                │          │
│  ┌──────▼──────────────▼─────────────▼────────────────▼──────┐  │
│  │              as2_behavior::BehaviorServer<T>               │  │
│  │           (ROS2 Action Server 래퍼, 상태 관리)              │  │
│  └──────────────────────────────────────────────────────────┘  │
│                           │                                      │
│  ┌────────────────────────▼──────────────────────────────────┐  │
│  │                    Plugin System                           │  │
│  │  ┌──────────────┐  ┌──────────────┐  ┌─────────────────┐  │  │
│  │  │  position    │  │  trajectory  │  │    platform     │  │  │
│  │  │   plugin     │  │    plugin    │  │     plugin      │  │  │
│  │  └──────────────┘  └──────────────┘  └─────────────────┘  │  │
│  └────────────────────────────────────────────────────────────┘  │
└──────────────────────────┬───────────────────────────────────────┘
                           │
┌──────────────────────────▼───────────────────────────────────────┐
│                       as2_core Layer                             │
│  ┌──────────────────┐  ┌───────────────┐  ┌───────────────────┐  │
│  │  MotionReference │  │   TfHandler   │  │  PlatformFSM      │  │
│  │    Handlers      │  │ (좌표 변환)    │  │ (상태 머신 클라이언트│  │
│  └──────────────────┘  └───────────────┘  └───────────────────┘  │
└──────────────────────────┬───────────────────────────────────────┘
                           │
┌──────────────────────────▼───────────────────────────────────────┐
│                    Platform Layer                                │
│          (하드웨어 드라이버, 시뮬레이터 인터페이스)                   │
└──────────────────────────────────────────────────────────────────┘
```

### 2.2 행동 실행 상태 머신

```
                   ┌──────────┐
                   │  IDLE    │◄─────────────────────────┐
                   └────┬─────┘                          │
                        │ Goal 수신                       │
                   ┌────▼─────┐                          │
                   │ ACTIVATING│                         │
                   └────┬─────┘                          │
                        │ on_activate() 성공              │
                   ┌────▼─────┐                          │
              ┌───►│ RUNNING  │◄──── on_modify() (목표 수정)
              │    └────┬─────┘                          │
    on_run()  │         │ on_run() 반환값                 │
    RUNNING   │    ┌────┴────────────────────┐           │
              │    │                         │           │
              └────┘            ┌────────────▼──────┐    │
                           ┌───►│   PAUSING         │    │
                           │    └────────────┬──────┘    │
                           │                 │ Pause 완료 │
                           │    ┌────────────▼──────┐    │
                           └────│   PAUSED          │    │
                                └────────────┬──────┘    │
                                             │ Resume     │
                                        (RUNNING 복귀)    │
                                                         │
             SUCCESS / FAILURE / ABORTED ───────────────►┘
```

---

## 3. as2_behavior — 기반 라이브러리

### 3.1 디렉토리 구조

```
as2_behavior/
├── include/as2_behavior/
│   ├── as2_basic_behavior.hpp    ← 기본 행동 클래스 (레거시)
│   ├── behavior_server.hpp       ← BehaviorServer 메인 헤더
│   ├── behavior_utils.hpp        ← ExecutionStatus 등 유틸리티
│   ├── __impl/
│   │   └── behavior_server__impl.hpp  ← 템플릿 구현
│   └── __detail/
│       └── (세부 구현 파일들)
└── src/
    └── behavior_server.cpp
```

### 3.2 핵심 클래스 구조

```cpp
namespace as2_behavior {

// 실행 상태를 나타내는 열거형
enum class ExecutionStatus {
    SUCCESS,   // 행동 성공 완료
    RUNNING,   // 행동 실행 중
    FAILURE,   // 행동 실패
    ABORTED    // 행동 강제 중단
};

// 모든 행동의 기반 클래스 (템플릿)
// T = as2_msgs::action::Takeoff 등의 ROS2 Action 타입
template<class ActionT>
class BehaviorServer : public as2::Node {
public:
    // 서브클래스에서 반드시 구현해야 하는 메서드들
    virtual bool on_activate(std::shared_ptr<const Goal> goal) = 0;
    virtual bool on_modify(std::shared_ptr<const Goal> goal) = 0;
    virtual bool on_deactivate(const std::shared_ptr<std::string>& message) = 0;
    virtual bool on_pause(const std::shared_ptr<std::string>& message) = 0;
    virtual bool on_resume(const std::shared_ptr<std::string>& message) = 0;
    virtual ExecutionStatus on_run(
        const std::shared_ptr<const Goal>& goal,
        std::shared_ptr<Feedback>& feedback_msg,
        std::shared_ptr<Result>& result_msg) = 0;
    virtual void on_execution_end(const ExecutionStatus& state) {}

private:
    rclcpp_action::Server<ActionT>::SharedPtr action_server_;
    std::thread execution_thread_;
};

} // namespace as2_behavior
```

### 3.3 BehaviorServer가 제공하는 기능

| 기능 | 설명 |
|------|------|
| ROS2 Action Server 자동 생성 | `/<namespace>/<behavior_name>` 토픽 자동 등록 |
| 별도 스레드 실행 | `on_run()`이 블로킹 루프에서 실행되어 ROS2 스핀 방해 없음 |
| 취소 처리 | 클라이언트가 Cancel 요청 시 자동으로 `on_deactivate()` 호출 |
| 상태 전이 관리 | IDLE → ACTIVATING → RUNNING → SUCCESS/FAILURE 자동 관리 |
| 피드백 자동 발행 | `on_run()` 루프에서 Feedback 메시지 자동 발행 |

---

## 4. as2_behaviors_motion — 움직임 행동

### 4.1 패키지 내부 구조

```
as2_behaviors_motion/
│
├── takeoff_behavior/
│   ├── include/
│   │   ├── takeoff_behavior.hpp     ← BehaviorServer 구현체
│   │   └── takeoff_base.hpp         ← 플러그인 인터페이스
│   ├── src/
│   │   ├── takeoff_behavior.cpp     ← 메인 로직
│   │   └── takeoff_behavior_node.cpp← 실행파일 진입점
│   └── plugins/
│       ├── takeoff_plugin_speed/    ← 속도 제어 방식 이륙
│       ├── takeoff_plugin_position/ ← 위치 제어 방식 이륙
│       ├── takeoff_plugin_platform/ ← 플랫폼 내장 이륙
│       └── takeoff_plugin_trajectory/← 궤적 기반 이륙
│
├── land_behavior/ (유사한 구조)
├── go_to_behavior/ (유사한 구조)
├── follow_path_behavior/ (유사한 구조)
├── follow_reference_behavior/
│
└── launch/
    ├── motion_behaviors_launch.py   ← 전체 모션 행동 한번에 실행
    ├── takeoff_behavior_launch.py
    ├── land_behavior_launch.py
    ├── go_to_behavior_launch.py
    ├── follow_path_behavior_launch.py
    └── follow_reference_behavior_launch.py
```

### 4.2 Takeoff Behavior 상세 구조

```
┌─────────────────────────────────────────────────────────┐
│                  TakeoffBehavior                        │
│  (BehaviorServer<as2_msgs::action::Takeoff>)            │
│                                                         │
│  멤버 변수:                                              │
│  ├── takeoff_plugin_     : TakeoffBase 플러그인 인스턴스  │
│  ├── tf_handler_         : 좌표 변환 처리                │
│  ├── twist_sub_          : 드론 속도 구독                │
│  └── platform_cli_       : 플랫폼 상태 머신 클라이언트    │
│                                                         │
│  주요 메서드:                                            │
│  ├── on_activate()       : TAKE_OFF FSM 이벤트 발송      │
│  │                         플러그인 활성화               │
│  ├── on_run()            : 플러그인의 on_run() 위임       │
│  ├── on_deactivate()     : 플러그인 비활성화             │
│  ├── state_callback()    : twist → tf 변환 → 플러그인 전달│
│  └── sendEventFSME()     : 플랫폼 FSM 이벤트 전송        │
└─────────────────────────────────────────────────────────┘
```

### 4.3 모션 플러그인 선택 기준

```
이륙(Takeoff) 플러그인 선택:

  ┌─────────────────────────────────────────────────────┐
  │                                                     │
  │  takeoff_plugin_platform  ← 기체 자체 이륙 제어 사용  │
  │  (하드웨어가 이륙 제어 지원 시)                        │
  │                                                     │
  │  takeoff_plugin_speed     ← 속도 명령으로 고도 상승    │
  │  (단순하고 빠름, 정밀도 낮음)                         │
  │                                                     │
  │  takeoff_plugin_position  ← 위치 제어로 목표 고도     │
  │  (정밀도 높음, 위치 제어기 필요)                       │
  │                                                     │
  │  takeoff_plugin_trajectory← 궤적 생성 후 추종         │
  │  (가장 부드러운 이륙)                                 │
  └─────────────────────────────────────────────────────┘

착륙(Land) 플러그인:
  land_plugin_speed / land_plugin_platform / land_plugin_trajectory

GoTo 플러그인:
  go_to_plugin_position / go_to_plugin_trajectory

FollowPath 플러그인:
  follow_path_plugin_position / follow_path_plugin_trajectory
```

### 4.4 TakeoffBase 플러그인 인터페이스

```cpp
namespace takeoff_base {

struct takeoff_plugin_params {
    double takeoff_height;      // 목표 이륙 고도 (m)
    double takeoff_speed;       // 이륙 속도 (m/s)
    double takeoff_threshold;   // 완료 판정 오차 (m)
};

class TakeoffBase {
public:
    // 플러그인 초기화
    virtual void ownInit() {}

    // 상태 업데이트 콜백 (BehaviorServer에서 주기적으로 호출)
    void state_callback(
        const geometry_msgs::msg::PoseStamped& pose,
        const geometry_msgs::msg::TwistStamped& twist
    );

    // 행동 제어 메서드 (서브클래스 구현)
    virtual bool own_activate(as2_msgs::action::Takeoff::Goal& goal) = 0;
    virtual bool own_modify(as2_msgs::action::Takeoff::Goal& goal) = 0;
    virtual bool own_deactivate(const std::string& message) = 0;
    virtual bool own_pause(const std::string& message) = 0;
    virtual bool own_resume(const std::string& message) = 0;
    virtual as2_behavior::ExecutionStatus own_run(
        const as2_msgs::action::Takeoff::Goal& goal,
        as2_msgs::action::Takeoff::Feedback& feedback,
        as2_msgs::action::Takeoff::Result& result
    ) = 0;

protected:
    as2::Node* node_ptr_;
    as2::motionReferenceHandlers::HoverMotion* hover_motion_handler_;

    // 현재 드론 상태 (state_callback이 업데이트)
    geometry_msgs::msg::PoseStamped actual_pose_;
    float actual_takeoff_height;
    float actual_takeoff_speed;
};

} // namespace takeoff_base
```

### 4.5 모션 행동 ROS2 액션 인터페이스

| 행동 | Action 타입 | Goal 주요 필드 | Result |
|------|------------|--------------|--------|
| Takeoff | `as2_msgs/action/Takeoff` | `takeoff_height`, `takeoff_speed` | `takeoff_success` |
| Land | `as2_msgs/action/Land` | `land_speed` | `land_success` |
| GoTo | `as2_msgs/action/GoToWaypoint` | `target_pose`, `max_speed`, `yaw_mode` | `go_to_success` |
| FollowPath | `as2_msgs/action/FollowPath` | `path`, `max_speed`, `yaw_mode` | `follow_path_success` |
| FollowReference | `as2_msgs/action/FollowReference` | `reference_facing_angle`, `max_speed` | - |

---

## 5. as2_behaviors_platform — 플랫폼 제어

### 5.1 역할

드론의 **저수준 플랫폼 상태**를 제어하는 행동들로, 비행 전 필수 단계를 담당합니다.

```
비행 시작 시퀀스:

  [드론 전원 ON]
       │
       ▼
  arm_behavior ──── SetArmingState 서비스 호출 ──► [모터 스핀업]
       │
       ▼
  offboard_behavior ─ SetOffboardMode 서비스 호출 ► [오프보드 모드]
       │
       ▼
  takeoff_behavior ── TakeOff Action 호출 ──────► [비행 중]
```

### 5.2 구조

```
as2_behaviors_platform/
├── include/as2_behaviors_platform/
│   ├── set_arming_state_behavior.hpp     ← Arm/Disarm 행동
│   └── set_offboard_mode_behavior.hpp    ← Offboard 모드 설정
├── src/
│   ├── set_arming_state_behavior_main.cpp
│   └── set_offboard_mode_behavior_main.cpp
└── launch/
    └── as2_platform_behaviors_launch.py  ← 모든 플랫폼 행동 실행
```

---

## 6. as2_behaviors_path_planning — 경로 계획

### 6.1 역할과 위치

```
사용자 앱
    │
    │ NavigateToPoint Action (목적지만 지정)
    ▼
PathPlannerBehavior
    │
    ├─ 플러그인으로 경로 계산 (A* 또는 Voronoi)
    │   └─ 결과: waypoint 리스트
    │
    └─ FollowPath Action 호출 (계산된 경로 전달)
            │
            ▼
    FollowPathBehavior (as2_behaviors_motion)
```

### 6.2 디렉토리 구조

```
as2_behaviors_path_planning/
├── include/as2_behaviors_path_planning/
│   ├── path_planner_behavior.hpp      ← 메인 행동 클래스
│   └── path_planner_plugin_base.hpp   ← 플러그인 인터페이스
├── src/
│   ├── path_planner_behavior.cpp
│   └── path_planner_behavior_node.cpp
├── common/include/
│   ├── cell_node.hpp                  ← 격자 셀 노드
│   ├── graph_searcher.hpp             ← 그래프 탐색 베이스
│   └── utils.hpp                      ← 좌표 변환 유틸
└── plugins/
    ├── a_star/
    │   ├── include/
    │   │   ├── a_star.hpp             ← A* 플러그인 구현
    │   │   ├── a_star_algorithm.hpp   ← A* 알고리즘
    │   │   └── a_star_searcher.hpp    ← 탐색기
    │   └── launch/
    └── voronoi/
        ├── include/
        │   ├── voronoi.hpp            ← Voronoi 플러그인
        │   └── voronoi_searcher.hpp   ← Voronoi 탐색기
        └── launch/
```

### 6.3 플러그인 인터페이스

```cpp
namespace as2_behaviors_path_planning {

class PluginBase {
public:
    // 초기화 (노드, TF 버퍼 주입)
    virtual void initialize(
        as2::Node* node_ptr,
        std::shared_ptr<tf2_ros::Buffer> tf_buffer) = 0;

    // 경로 계획 시작 (드론 현재 위치, 목표 지점)
    virtual bool on_activate(
        geometry_msgs::msg::PoseStamped drone_pose,
        as2_msgs::action::NavigateToPoint::Goal goal) = 0;

    // 반복 실행 (경로 계획 완료 시 SUCCESS 반환)
    virtual as2_behavior::ExecutionStatus on_run() = 0;

protected:
    // 계산된 경로 (waypoint 리스트)
    std::vector<geometry_msgs::msg::Point> path_;
};

} // namespace as2_behaviors_path_planning
```

### 6.4 알고리즘 비교

```
A* 알고리즘:
  ┌─────────────────────────────────────────────┐
  │  격자 기반 탐색                              │
  │  S ─ ─ ─ ─ ─ ─ ┐                           │
  │  │  ████████   │                            │
  │  │  ████████   │                            │
  │  └ ─ ─ ─ ─ ─ ► E                            │
  │                                             │
  │  장점: 최단 경로 보장                         │
  │  단점: 계산량이 많음 (격자 크기에 비례)        │
  └─────────────────────────────────────────────┘

Voronoi 다이어그램 기반:
  ┌─────────────────────────────────────────────┐
  │  장애물 경계선에서 최대한 멀리 이동            │
  │  S                                          │
  │   \                                         │
  │    \  ████████                              │
  │     \─────────────────► E                   │
  │         ████████                            │
  │                                             │
  │  장점: 안전 마진이 최대 (장애물 회피 우선)     │
  │  단점: 최단 경로 아닐 수 있음                 │
  └─────────────────────────────────────────────┘
```

---

## 7. as2_behaviors_trajectory_generation — 궤적 생성

### 7.1 역할

단순한 waypoint 이동이 아닌 **물리적으로 부드러운 궤적**을 생성하여 드론이 급격한 방향 전환 없이 날 수 있게 합니다.

```
입력:  waypoint 리스트 [P1, P2, P3, P4]

단순 직선 이동:          다항식 궤적:
  P1─────P2              P1
  P2─────P3                 ╰─╮
  P3─────P4                    ╰─╮P2─╮
                                   ╰──P3──╮
                                          P4
  (급격한 방향 전환)       (부드러운 곡선 이동)
```

### 7.2 구조

```
as2_behaviors_trajectory_generation/
└── generate_polynomial_trajectory_behavior/
    ├── include/
    │   └── generate_polynomial_trajectory_behavior.hpp
    ├── src/
    │   ├── generate_polynomial_trajectory_behavior.cpp
    │   └── generate_polynomial_trajectory_behavior_node.cpp
    └── launch/
        ├── generate_polynomial_trajectory_behavior_launch.py
        └── composable_generate_polynomial_trajectory_behavior.launch.py
```

### 7.3 핵심 클래스

```cpp
class DynamicPolynomialTrajectoryGenerator :
    public as2_behavior::BehaviorServer<
        as2_msgs::action::GeneratePolynomialTrajectory>
{
    // 외부 라이브러리 (dynamic_trajectory_generator)
    dynamic_traj_generator::DynamicTrajectory traj_generator_;

    // 모션 핸들러
    as2::motionReferenceHandlers::HoverMotion hover_motion_handler_;
    as2::motionReferenceHandlers::TrajectoryMotion trajectory_motion_handler_;

    // Waypoint 동적 수정 구독
    rclcpp::Subscription<as2_msgs::msg::TrajectoryWaypointModifier>::SharedPtr
        waypoint_modifier_sub_;

    // Yaw 각도 계산 모드
    // - PATH_FACING: 이동 방향으로 자동 회전
    // - FIXED: 고정 각도 유지
    // - KEEP_YAW: 현재 각도 유지
    as2_msgs::msg::YawMode yaw_mode_;
};
```

---

## 8. as2_behaviors_swarm_flocking — 무리 제어

### 8.1 개념

**여러 드론**이 하나의 무리(swarm)처럼 움직이게 합니다. 한 드론이 리더 역할을 하며 나머지는 상대적 위치를 유지하며 따라갑니다.

```
           [가상 중심점]
                │
        ┌───────┼───────┐
        │       │       │
      [D1]    [D2]    [D3]
    (drone0)(drone1)(drone2)

  → 가상 중심점이 이동하면 모든 드론이 포메이션 유지하며 이동
```

### 8.2 구조

```
as2_behaviors_swarm_flocking/
├── include/as2_behaviors_swarm_flocking/
│   ├── swarm_flocking_behavior.hpp    ← 무리 행동 서버
│   └── drone_swarm.hpp                ← 개별 드론 관리
├── src/
│   ├── swarm_flocking_behavior.cpp
│   ├── swarm_flocking_behavior_node.cpp
│   └── drone_swarm.cpp
└── launch/
    └── swarm_flocking_behavior.launch.py
```

### 8.3 통신 구조

```
SwarmFlockingBehavior (리더 드론에서 실행)
       │
       ├── FollowReference Action Client ──► drone0/FollowReferenceBehavior
       ├── FollowReference Action Client ──► drone1/FollowReferenceBehavior
       └── FollowReference Action Client ──► drone2/FollowReferenceBehavior

각 FollowReferenceBehavior는 상대 위치 오프셋을 유지하며 이동
```

---

## 9. as2_behaviors_payload — 페이로드 제어

### 9.1 구성

```
as2_behaviors_payload/
├── gripper_behavior/              ← 집게 제어
│   ├── include/
│   │   ├── gripper_behavior.hpp
│   │   └── gripper_behavior_plugin_base.hpp
│   └── plugins/
│       ├── dc_servo/              ← DC 서보 모터 방식
│       └── two_fingers/           ← 두 손가락 방식
│
└── point_gimbal_behavior/         ← 카메라 짐벌 제어
    └── include/point_gimbal_behavior.hpp
```

### 9.2 그리퍼 플러그인 구조

```
┌─────────────────────────────────────────┐
│        GripperHandlerBehavior           │
│  (BehaviorServer<GripperHandler>)       │
│                                         │
│  플러그인 로드 (pluginlib)               │
│     │                                   │
│     ├── dc_servo 플러그인               │
│     │   └── PWM 신호로 서보 제어         │
│     │                                   │
│     └── two_fingers 플러그인            │
│         └── 2축 제어로 집기/놓기         │
└─────────────────────────────────────────┘
```

---

## 10. as2_behaviors_perception — 인식

### 10.1 ArUco 마커 감지

```
as2_behaviors_perception/
└── detect_aruco_markers_behavior/
    ├── include/detect_aruco_markers_behavior.hpp
    ├── src/
    │   ├── detect_aruco_markers_behavior.cpp
    │   └── detect_aruco_markers_behavior_node.cpp
    └── launch/
        ├── detect_aruco_markers_behavior_real_launch.py   ← 실 하드웨어
        └── detect_aruco_markers_behavior_sim_launch.py    ← 시뮬레이터
```

### 10.2 동작 원리

```
카메라 이미지 구독 (image_transport)
         │
         ▼
OpenCV ArUco 마커 감지
         │
         ▼
카메라 보정 파라미터로 3D 위치 추정
(camera_matrix, dist_coefficients)
         │
         ▼
마커 ID 필터링 (원하는 ID만 선택)
         │
         ▼
Action Result로 마커 위치/자세 반환
```

### 10.3 핵심 파라미터

| 파라미터 | 설명 |
|---------|------|
| `camera_matrix` | 카메라 내부 행렬 (초점 거리, 주점) |
| `dist_coefficients` | 렌즈 왜곡 계수 |
| `marker_size` | ArUco 마커 실제 크기 (m) |
| `marker_ids` | 감지할 마커 ID 목록 |

---

## 11. as2_behaviors_param_estimation — 파라미터 추정

### 11.1 구성

```
as2_behaviors_param_estimation/
├── mass_estimation_behavior/     ← 드론 질량 추정
│   └── include/
│       ├── mass_estimation_behavior.hpp
│       └── param_estimation.hpp
└── force_estimation_behavior/    ← 외부 힘 추정
    └── include/
        ├── force_estimation_behavior.hpp
        └── force_estimation.hpp
```

### 11.2 사용 목적

```
질량 추정 (mass_estimation):
  - 페이로드 장착 시 변경된 드론 전체 질량 추정
  - 제어기 파라미터 자동 조정에 활용

외부 힘 추정 (force_estimation):
  - 바람, 충돌 등 외부 교란력 추정
  - 강인 제어 (Robust Control)에 활용
```

---

## 12. 플러그인 시스템

### 12.1 플러그인 등록 구조

```
CMakeLists.txt:
  pluginlib_export_plugin_description_file(
    ${PROJECT_NAME} plugins.xml)

plugins.xml:
  <class_libraries>
    <library path="takeoff_plugin_speed_lib">
      <class name="as2_behaviors_motion/TakeoffPluginSpeed"
             type="takeoff_speed::Plugin"
             base_class_type="takeoff_base::TakeoffBase">
      </class>
    </library>
  </class_libraries>

소스 코드 끝에:
  #include <pluginlib/class_list_macros.hpp>
  PLUGINLIB_EXPORT_CLASS(takeoff_speed::Plugin, takeoff_base::TakeoffBase)
```

### 12.2 플러그인 로딩 흐름

```
TakeoffBehavior 생성자
      │
      │ plugin_name 파라미터 읽기
      │ (예: "as2_behaviors_motion/TakeoffPluginSpeed")
      │
      ▼
pluginlib::ClassLoader<TakeoffBase> 생성
      │
      ▼
loader_->createSharedInstance(plugin_name)
      │
      ▼
TakeoffBase* 인터페이스로 사용
  (런타임에 구체 클래스 결정)
```

### 12.3 전체 플러그인 목록

| 행동 | 플러그인 | 방식 |
|------|---------|------|
| Takeoff | `takeoff_plugin_speed` | 속도 명령 |
| Takeoff | `takeoff_plugin_position` | 위치 명령 |
| Takeoff | `takeoff_plugin_platform` | 플랫폼 자체 제어 |
| Takeoff | `takeoff_plugin_trajectory` | 궤적 기반 |
| Land | `land_plugin_speed` | 속도 명령 |
| Land | `land_plugin_platform` | 플랫폼 자체 제어 |
| Land | `land_plugin_trajectory` | 궤적 기반 |
| GoTo | `go_to_plugin_position` | 위치 명령 |
| GoTo | `go_to_plugin_trajectory` | 궤적 기반 |
| FollowPath | `follow_path_plugin_position` | 위치 명령 |
| FollowPath | `follow_path_plugin_trajectory` | 궤적 기반 |
| Gripper | `dc_servo` | DC 서보 |
| Gripper | `two_fingers` | 2축 구동 |
| PathPlanning | `a_star` | A* 알고리즘 |
| PathPlanning | `voronoi` | Voronoi 기반 |

---

## 13. ROS2 통신 패턴

### 13.1 전체 통신 지도

```
사용자/BehaviorTree
    │
    │ Action: /drone0/TakeoffBehavior
    ▼
TakeoffBehavior Node (/drone0/bt_manager)
    │
    ├─ Service: /drone0/set_platform_state_machine_event
    │   └─► PlatformFSM (TAKE_OFF 이벤트)
    │
    ├─ Subscribe: /drone0/self_localization/twist
    │   └─► 드론 현재 속도/위치 수신
    │
    └─ (플러그인을 통해)
        │
        ├─ Publish: /drone0/motion_reference/speed
        │   또는
        ├─ Publish: /drone0/motion_reference/pose
        │   또는
        └─ Publish: /drone0/motion_reference/trajectory
            └─► 플랫폼이 수신하여 모터 제어
```

### 13.2 Action 메시지 흐름

```
클라이언트                        TakeoffBehavior
    │                                  │
    │── GoalRequest ─────────────────►│
    │                                  │─── on_activate() 호출
    │◄─ GoalAccepted ─────────────────│
    │                                  │─── (별도 스레드)
    │                                  │    on_run() 루프 시작
    │◄─ Feedback (현재 고도) ──────────│
    │◄─ Feedback (현재 고도) ──────────│
    │◄─ Feedback (현재 고도) ──────────│
    │                                  │─── 목표 고도 도달
    │◄─ Result (success=true) ─────────│
    │                                  │─── on_execution_end() 호출
```

---

## 14. 실행 흐름 — Takeoff 예시

### 14.1 전체 시퀀스 다이어그램

```
사용자앱   BehaviorTree   TakeoffBehavior   Plugin(speed)   Platform   모터
  │            │                │               │             │         │
  │─Goal──────►│                │               │             │         │
  │            │──GoalReq──────►│               │             │         │
  │            │                │──arm──────────────────────►│         │
  │            │                │──offboard─────────────────►│         │
  │            │                │──TAKEOFF_EVENT─────────────►│         │
  │            │◄─Accepted──────│               │             │         │
  │            │                │──activate()──►│             │         │
  │            │                │               │─────────────────────►│
  │            │                │               │  speed_cmd           │
  │            │                │──run()────────►│             │         │
  │            │◄─Feedback──────│               │             │         │
  │            │◄─Feedback──────│               │             │         │
  │            │                │               │   (고도 상승 중)       │
  │            │◄─Feedback──────│               │             │         │
  │            │                │──run()────────►│             │         │
  │            │                │               │ 목표고도 도달│         │
  │            │                │◄─SUCCESS──────│             │         │
  │            │                │──HOVER────────────────────────────► │
  │            │◄─Result────────│               │             │         │
  │◄─Result───│                │               │             │         │
```

### 14.2 launch 파일 사용 예시

```bash
# 1. 플랫폼 행동들 실행 (arm, offboard)
ros2 launch as2_behaviors_platform as2_platform_behaviors_launch.py \
    namespace:=drone0 \
    use_sim_time:=true

# 2. 이륙 행동 실행 (속도 제어 플러그인 사용)
ros2 launch as2_behaviors_motion takeoff_behavior_launch.py \
    namespace:=drone0 \
    plugin_name:=as2_behaviors_motion/TakeoffPluginSpeed \
    use_sim_time:=true

# 3. 모든 모션 행동 한번에 실행
ros2 launch as2_behaviors_motion motion_behaviors_launch.py \
    namespace:=drone0 \
    use_sim_time:=true \
    config_file:=/path/to/config.yaml

# 4. 경로 계획 (A* 플러그인)
ros2 launch as2_behaviors_path_planning path_planner_behavior_launch.py \
    namespace:=drone0 \
    plugin_name:=as2_behaviors_path_planning/AStarPlanner

# 5. 궤적 생성 행동
ros2 launch as2_behaviors_trajectory_generation \
    generate_polynomial_trajectory_behavior_launch.py \
    namespace:=drone0
```

---

## 15. 패키지 의존성 지도

```
                    ┌─────────────────────────────────────────┐
                    │            외부 의존성                   │
                    │  ROS2(rclcpp, rclcpp_action)            │
                    │  pluginlib  tf2  Eigen  OpenCV  yaml-cpp │
                    └──────────────────┬──────────────────────┘
                                       │
                    ┌──────────────────▼──────────────────────┐
                    │              as2_core                    │
                    │  (as2::Node, TfHandler, MotionReference) │
                    └──┬────────────────────────────────────┬─┘
                       │                                    │
          ┌────────────▼────────────┐    ┌─────────────────▼────────────┐
          │       as2_msgs          │    │         as2_behavior          │
          │  (Action/Msg 정의)      │    │   (BehaviorServer 기반 클래스) │
          └────────────┬────────────┘    └──────────────┬───────────────┘
                       │                                │
                       └──────────────┬─────────────────┘
                                      │ (모든 행동 패키지가 이 두 패키지에 의존)
              ┌───────────────────────┼────────────────────────────┐
              │                       │                            │
  ┌───────────▼─────┐  ┌─────────────▼──────┐  ┌─────────────────▼──────┐
  │as2_behaviors_   │  │as2_behaviors_       │  │as2_behaviors_          │
  │  motion         │  │  platform           │  │  path_planning         │
  │(이륙/착륙/이동)  │  │(arm/offboard)       │  │(A*, Voronoi)           │
  └─────────────────┘  └────────────────────┘  └────────────────────────┘
              │
  ┌───────────┼────────────────────────────────────────────┐
  │           │                                            │
  ▼           ▼                                            ▼
as2_behaviors_  as2_behaviors_trajectory_  as2_behaviors_swarm_
  payload        generation                  flocking
(gripper,gimbal) (poly trajectory)           (multi-drone)
              │
              ▼
as2_behaviors_perception   as2_behaviors_param_estimation
(ArUco markers)            (mass, force)
```

---

## 부록 — 핵심 파일 위치 요약

| 파일 | 경로 | 역할 |
|------|------|------|
| `behavior_server.hpp` | `as2_behavior/include/as2_behavior/` | 모든 행동의 기반 클래스 |
| `behavior_utils.hpp` | `as2_behavior/include/as2_behavior/` | ExecutionStatus 정의 |
| `takeoff_behavior.hpp` | `as2_behaviors_motion/takeoff_behavior/include/` | 이륙 행동 서버 |
| `takeoff_base.hpp` | `as2_behaviors_motion/takeoff_behavior/include/` | 이륙 플러그인 인터페이스 |
| `takeoff_behavior.cpp` | `as2_behaviors_motion/takeoff_behavior/src/` | 이륙 행동 구현 |
| `path_planner_plugin_base.hpp` | `as2_behaviors_path_planning/include/` | 경로 계획 플러그인 인터페이스 |
| `motion_behaviors_launch.py` | `as2_behaviors_motion/launch/` | 모션 행동 통합 실행 |
| `plugins.xml` | `as2_behaviors_motion/` | 플러그인 등록 선언 |
