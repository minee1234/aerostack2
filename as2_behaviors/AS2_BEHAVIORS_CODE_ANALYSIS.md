# as2_behaviors 패키지 코드 분석

> 전체 코드베이스 기반 상세 분석 | Aerostack2 v1.1.3

---

## 목차

1. [패키지 전체 구조](#1-패키지-전체-구조)
2. [as2_behavior — BehaviorServer 기반 클래스](#2-as2_behavior--behaviorserver-기반-클래스)
3. [as2_behaviors_motion — 이동 행동](#3-as2_behaviors_motion--이동-행동)
   - [Takeoff Behavior](#31-takeoff-behavior)
   - [Land Behavior](#32-land-behavior)
   - [GoTo Behavior](#33-goto-behavior)
   - [FollowPath Behavior](#34-followpath-behavior)
   - [FollowReference Behavior](#35-followreference-behavior)
4. [as2_behaviors_platform — 플랫폼 제어](#4-as2_behaviors_platform--플랫폼-제어)
5. [as2_behaviors_path_planning — 경로 계획](#5-as2_behaviors_path_planning--경로-계획)
6. [as2_behaviors_trajectory_generation — 궤적 생성](#6-as2_behaviors_trajectory_generation--궤적-생성)
7. [as2_behaviors_swarm_flocking — 군집 비행](#7-as2_behaviors_swarm_flocking--군집-비행)
8. [as2_behaviors_payload — 페이로드 제어](#8-as2_behaviors_payload--페이로드-제어)
9. [as2_behaviors_perception — 인식](#9-as2_behaviors_perception--인식)
10. [as2_behaviors_param_estimation — 파라미터 추정](#10-as2_behaviors_param_estimation--파라미터-추정)
11. [플러그인 시스템 전체 지도](#11-플러그인-시스템-전체-지도)
12. [행동 간 의존 관계](#12-행동-간-의존-관계)
13. [설정 파라미터 전체 목록](#13-설정-파라미터-전체-목록)

---

## 1. 패키지 전체 구조

```
as2_behaviors/
│
├── as2_behavior/                          ← [기반] BehaviorServer 템플릿
│   └── include/as2_behavior/
│       ├── behavior_server.hpp            ← 진입 헤더
│       ├── behavior_utils.hpp             ← ExecutionStatus 정의
│       ├── __detail/
│       │   └── behavior_server__class.hpp ← 클래스 선언
│       └── __impl/
│           └── behavior_server__impl.hpp  ← 템플릿 구현
│
├── as2_behaviors_motion/                  ← [이동] 5종 비행 행동
│   ├── takeoff_behavior/                  ← 이륙 (4 플러그인)
│   ├── land_behavior/                     ← 착륙 (3 플러그인)
│   ├── go_to_behavior/                    ← 이동 (2 플러그인)
│   ├── follow_path_behavior/              ← 경로추종 (2 플러그인)
│   ├── follow_reference_behavior/         ← 참조추종 (플러그인 없음)
│   └── launch/
│
├── as2_behaviors_platform/                ← [플랫폼] 기본 상태 제어
├── as2_behaviors_path_planning/           ← [경로] A*/Voronoi 알고리즘
├── as2_behaviors_trajectory_generation/   ← [궤적] 다항식 궤적 생성
├── as2_behaviors_swarm_flocking/          ← [군집] 편대 비행
├── as2_behaviors_payload/                 ← [페이로드] 그리퍼/짐벌
├── as2_behaviors_perception/              ← [인식] ArUco 마커
└── as2_behaviors_param_estimation/        ← [추정] 질량/힘 추정
```

### 패키지별 행동-액션 매핑

```
패키지                       행동 클래스                       ROS2 Action
─────────────────────────────────────────────────────────────────────────
as2_behaviors_platform      SetArmingStateBehavior            SetArmingState
                            SetOffboardModeBehavior            SetOffboardMode
as2_behaviors_motion        TakeoffBehavior                   Takeoff
                            LandBehavior                      Land
                            GoToBehavior                      GoToWaypoint
                            FollowPathBehavior                FollowPath
                            FollowReferenceBehavior           FollowReference
as2_behaviors_path_planning PathPlannerBehavior               NavigateToPoint
as2_behaviors_traj_gen      DynamicPolynomialTrajGenerator    GeneratePolynomialTrajectory
as2_behaviors_swarm         SwarmFlockingBehavior             SwarmFlocking
as2_behaviors_payload       GripperHandlerBehavior            GripperHandler
                            PointGimbalBehavior               PointGimbal
as2_behaviors_perception    DetectArucoMarkersBehavior        DetectArucoMarkers
as2_behaviors_param_est     MassEstimationBehavior            MassEstimation
                            ForceEstimationBehavior           ForceEstimation
```

---

## 2. as2_behavior — BehaviorServer 기반 클래스

### 2.1 ExecutionStatus 정의

```cpp
// behavior_utils.hpp
namespace as2_behavior {
  enum class ExecutionStatus {
    SUCCESS,   // on_run()이 이 값을 반환하면 행동 성공 완료
    RUNNING,   // on_run()이 이 값을 반환하면 계속 실행
    FAILURE,   // on_run()이 이 값을 반환하면 행동 실패
    ABORTED    // 외부에서 강제 중단됨
  };
}
```

### 2.2 BehaviorServer 클래스 구조

```
BehaviorServer<ActionT> 핵심 멤버:

  [ROS2 Action]
  action_server_           : rclcpp_action::Server<ActionT>
  goal_handle_             : GoalHandleAction (현재 실행 중인 목표)

  [서비스 서버]
  stop_srv_                : std_srvs::srv::Trigger   → deactivate
  pause_srv_               : std_srvs::srv::Trigger   → pause
  resume_srv_              : std_srvs::srv::Trigger   → resume
  modify_srv_              : as2_msgs::srv::ModifySrv → modify

  [발행자]
  behavior_status_pub_     : as2_msgs::msg::BehaviorStatus
                             IDLE(0) / RUNNING(1) / PAUSED(2)

  [타이머]
  run_timer_               : 주기적 on_run() 호출 (run_frequency Hz)
  behavior_status_timer_   : 상태 발행 (10Hz)
```

### 2.3 BehaviorServer 생명주기 상세

```
┌─────────────────────────────────────────────────────────────────────┐
│                    BehaviorServer 생명주기                           │
│                                                                     │
│  ┌────────┐                                                         │
│  │  IDLE  │◄──────────────────────────────────────────────────┐    │
│  └───┬────┘                                                   │    │
│      │ Goal 수신 → handleGoal()                                │    │
│      │   └─ on_activate(goal) 호출                            │    │
│      │       ├─ true  → ACCEPT_AND_EXECUTE                    │    │
│      │       └─ false → REJECT                                │    │
│      ▼                                                         │    │
│  ┌──────────┐                                                  │    │
│  │ RUNNING  │ ◄──── on_modify(new_goal) ── /modify 서비스      │    │
│  │          │                                                  │    │
│  │ run_timer│──► on_run(goal, feedback, result) 호출           │    │
│  │  (N Hz)  │        ├─ RUNNING  → publish_feedback()         │    │
│  │          │        ├─ SUCCESS  → goal_handle_->succeed()     │    │
│  └─────┬────┘        ├─ FAILURE  → goal_handle_->abort()      │    │
│        │             └─ ABORTED  → goal_handle_->abort()      │    │
│        │                                                       │    │
│        │ /pause 서비스                                          │    │
│        ▼                                                       │    │
│  ┌────────┐   on_pause()                                       │    │
│  │ PAUSED │   run_timer_ 중지                                  │    │
│  └───┬────┘                                                   │    │
│      │ /resume 서비스                                           │    │
│      │   on_resume()                                           │    │
│      │   run_timer_ 재시작                                      │    │
│      └──────────────────► RUNNING 복귀                         │    │
│                                                                │    │
│  SUCCESS/FAILURE/ABORTED 시:                                   │    │
│    on_execution_end(state) 호출                                │    │
│    run_timer_ 중지                                             │    │
│    behavior_status = IDLE ───────────────────────────────────►┘    │
└─────────────────────────────────────────────────────────────────────┘
```

### 2.4 BehaviorServer 구현 코드 핵심 (impl.hpp)

```cpp
// 생성자 — 4가지 등록 작업
BehaviorServer<ActionT>::BehaviorServer(const std::string & name, ...)
  : as2::Node(name, options)
{
  this->declare_parameter<float>("run_frequency", 10.0);
  register_action();          // ROS2 Action Server 등록
  register_service_servers(); // stop/pause/resume/modify 서비스 등록
  register_publishers();      // behavior_status 발행자 등록
  register_timers();          // status 발행 타이머 등록
}

// 액션 서버 콜백 체인
handleGoal(goal)
  → on_activate(goal)
      → ACCEPT_AND_EXECUTE 또는 REJECT

handleAccepted(goal_handle)
  → activate(goal)
      → register_run_timer()   ← run_timer_ 시작
      → behavior_status = RUNNING

run(goal_handle)                ← run_timer_ 콜백
  → on_run(goal, feedback, result)
      → SUCCESS  : goal_handle->succeed(result)
                   on_execution_end(SUCCESS)
                   cancel_run_timer()
      → RUNNING  : goal_handle->publish_feedback(feedback)
      → FAILURE  : goal_handle->abort(result)
                   on_execution_end(FAILURE)
                   cancel_run_timer()

// 서비스 콜백
pause_callback()
  → on_pause(message)
  → cancel_run_timer()
  → behavior_status = PAUSED

resume_callback()
  → on_resume(message)
  → register_run_timer()
  → behavior_status = RUNNING
```

### 2.5 서브클래스가 반드시 구현해야 하는 메서드

```cpp
template<typename ActionT>
class MyBehavior : public BehaviorServer<ActionT> {
  // [필수] 행동 시작 — false 반환 시 Goal 거부
  bool on_activate(std::shared_ptr<const Goal> goal) override;

  // [필수] 반복 실행 — run_frequency Hz로 호출됨
  ExecutionStatus on_run(
    const std::shared_ptr<const Goal>& goal,
    std::shared_ptr<Feedback>& feedback,
    std::shared_ptr<Result>& result) override;

  // [선택] 목표 수정 (실행 중 /modify 서비스 수신 시)
  bool on_modify(std::shared_ptr<const Goal> goal) override;

  // [선택] 행동 중단
  bool on_deactivate(const std::shared_ptr<std::string>& message) override;

  // [선택] 일시정지
  bool on_pause(const std::shared_ptr<std::string>& message) override;

  // [선택] 재개
  bool on_resume(const std::shared_ptr<std::string>& message) override;

  // [선택] 종료 후 처리 (FSM 이벤트 발송 등)
  void on_execution_end(const ExecutionStatus& state) override;
};
```

---

## 3. as2_behaviors_motion — 이동 행동

### 3.1 Takeoff Behavior

#### 클래스 다이어그램

```
TakeoffBehavior
  상속: BehaviorServer<as2_msgs::action::Takeoff>
  │
  멤버:
  ├── loader_          : ClassLoader<TakeoffBase>
  ├── takeoff_plugin_  : shared_ptr<TakeoffBase>   ← 런타임 결정
  ├── tf_handler_      : shared_ptr<TfHandler>
  ├── twist_sub_       : Subscription<TwistStamped>
  └── platform_cli_    : SyncServiceClient<SetPlatformStateMachineEvent>

TakeoffBase (플러그인 인터페이스)
  │
  멤버:
  ├── params_          : takeoff_plugin_params
  │   ├── takeoff_height   : double
  │   ├── takeoff_speed    : double
  │   └── takeoff_threshold: double
  ├── actual_pose_     : PoseStamped    ← state_callback 갱신
  ├── goal_            : Takeoff::Goal
  ├── feedback_        : Takeoff::Feedback
  └── result_          : Takeoff::Result

  [순수 가상]
  ├── own_activate(Goal&) → bool
  ├── own_run()           → ExecutionStatus
  ├── own_deactivate(msg) → bool
  ├── own_pause(msg)      → bool
  ├── own_resume(msg)     → bool
  └── own_execution_end(ExecutionStatus)
```

#### 4가지 플러그인 동작 비교

```
┌─────────────────────────────────────────────────────────────────────┐
│  takeoff_plugin_speed                                               │
│                                                                     │
│  activate:  즉시 true 반환                                          │
│  run():                                                             │
│    ┌──────────────────────────────────────────────────┐            │
│    │ if |actual_height - target_height| < threshold   │            │
│    │   → return SUCCESS                               │            │
│    │ else                                             │            │
│    │   speed_motion_handler_->                        │            │
│    │     sendSpeedCommandWithYawSpeed(                │            │
│    │       "earth", 0, 0, takeoff_speed, 0)          │            │
│    │   → return RUNNING                              │            │
│    └──────────────────────────────────────────────────┘            │
│  deactivate/end: hover_motion_handler_->sendHover()                │
└─────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────┐
│  takeoff_plugin_position                                            │
│                                                                     │
│  activate:                                                          │
│    target_pose = {current_x, current_y, takeoff_height}           │
│  run():                                                             │
│    position_motion_handler_->                                       │
│      sendPositionCommandWithYawAngle(                               │
│        "earth", tx, ty, tz, current_yaw,                          │
│        "earth", 0, 0, 0)                                           │
│    if distance_to_target < threshold → return SUCCESS              │
└─────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────┐
│  takeoff_plugin_platform                                            │
│                                                                     │
│  activate:                                                          │
│    request = make_shared<SetBool::Request>()                       │
│    request->data = true                                            │
│    platform_takeoff_future_ =                                      │
│      platform_takeoff_cli_->async_send_request(request)           │
│  run():                                                             │
│    if future_.wait_for(0s) == ready                               │
│      result = future_.get()                                        │
│      → result->success ? SUCCESS : FAILURE                        │
│    else → RUNNING                                                  │
└─────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────┐
│  takeoff_plugin_trajectory                                          │
│                                                                     │
│  activate:                                                          │
│    GeneratePolynomialTrajectory 액션 목표 생성                      │
│    목표: {current_pos → takeoff_height}                            │
│    trajectory_gen_client_->async_send_goal(traj_goal)             │
│  run():                                                             │
│    궤적 생성기 진행 상태 모니터링                                    │
│    if 목표 고도 도달 → SUCCESS                                      │
└─────────────────────────────────────────────────────────────────────┘
```

#### process_goal() 흐름 (takeoff_behavior.cpp)

```
process_goal(goal) 호출 시:

  1. 목표 유효성 검사
     if goal.takeoff_height <= 0 → reject

  2. 플랫폼 FSM 이벤트 전송
     sendEventFSME(PlatformStateMachineEvent::TAKE_OFF)
     → /drone0/set_platform_state_machine_event 서비스 호출
     → 플랫폼: LANDED → TAKING_OFF 전이

  3. TF 변환 확인
     tf_handler_->getState() 호출

  4. 플러그인에 목표 전달
     takeoff_plugin_->on_activate(new_goal)
```

#### on_execution_end() 이벤트 처리

```cpp
void TakeoffBehavior::on_execution_end(const ExecutionStatus& state) {
  if (state == ExecutionStatus::SUCCESS) {
    sendEventFSME(TOOK_OFF);   // → TAKING_OFF → FLYING
  } else {
    sendEventFSME(EMERGENCY);  // → EMERGENCY
  }
  takeoff_plugin_->on_execution_end(state);
}
```

### 3.2 Land Behavior

#### Land vs Takeoff 차이점

```
공통점:
  - BehaviorServer<ActionT> 상속
  - pluginlib 플러그인 구조
  - TF 기반 상태 콜백
  - FSM 이벤트 전송

차이점:
  ┌─────────────────────────────────────────────────┐
  │  Land 전용 멤버                                  │
  │  platform_disarm_cli_: SetBool 서비스 클라이언트  │
  │  → 착륙 성공 후 자동 무장 해제                    │
  └─────────────────────────────────────────────────┘

  process_goal()에서:
    land_speed를 음수로 강제 (하강 방향 보장)
    new_goal.land_speed = -fabs(goal.land_speed)

  on_execution_end():
    SUCCESS → sendEventFSME(LANDED)    // → LANDING → LANDED
              sendDisarm()             // 무장 해제
    FAILURE → sendEventFSME(EMERGENCY)
```

#### 3가지 착륙 플러그인

```
land_plugin_speed:
  멤버: speed_motion_handler_
        land_speed_condition_percentage_ (기본: 0.2 = 20%)
        land_speed_condition_height_     (기본: 0.2m)

  activate():
    initial_height_ = actual_height_  ← 이륙 높이 기록

  checkGoalCondition():
    height_dropped = initial_height_ - actual_height_
    speed_ok = actual_speed_z < params_.land_speed * percentage_
    return height_dropped > condition_height_ && speed_ok

─────────────────────────────────────────────────────────
land_plugin_trajectory:
  멤버: traj_gen_client_         ← 궤적 생성 액션 클라이언트
        traj_gen_pause_client_   ← 일시정지 서비스
        traj_gen_resume_client_  ← 재개 서비스

  activate():
    traj_goal = landGoalToTrajectoryGeneratorGoal(goal)
    → target.z = current_z + land_trajectory_height (-10.0m 기본)
    → 충분히 낮은 목표로 궤적 생성
    traj_gen_client_->async_send_goal(traj_goal)

  on_pause():
    traj_gen_pause_client_->async_send_request(...)

─────────────────────────────────────────────────────────
land_plugin_platform:
  activate(): platform_land_cli_->async_send_request(...)
  run():      future 응답 대기 → SUCCESS/FAILURE
```

### 3.3 GoTo Behavior

#### 클래스 구조

```
GoToBehavior
  상속: BehaviorServer<as2_msgs::action::GoToWaypoint>
  │
  추가 멤버 (takeoff 대비):
  └── platform_info_sub_   : Subscription<PlatformInfo>
       → platform_info_callback() → 플랫폼 상태 업데이트

GoToBase
  추가 기능:
  ├── processGoal():
  │   ├── FLYING 상태 확인 (아니면 거부)
  │   ├── localization_flag_ 확인
  │   └── earth 좌표계로 target 변환
  │
  └── actual_dist_to_goal_ 계산
      = euclidean_distance(actual_pose_, goal_.target_pose)
```

#### GoTo 처리 흐름

```
on_activate(goal):
  ├── process_goal() 호출
  │   ├── platform_info_.status == FLYING 확인
  │   ├── localization_flag_ 확인
  │   ├── frame_id 확인 → earth 좌표계로 변환
  │   └── yaw 처리
  │       ├── YAW_ANGLE: 절대값 사용
  │       ├── PATH_FACING: 방향 자동 계산
  │       ├── KEEP_YAW: 현재 yaw 유지
  │       └── FIXED_YAW: 고정값 사용
  └── go_to_plugin_->on_activate(goal)

on_run():
  └── go_to_plugin_->on_run()
       ├── actual_dist_to_goal_ 계산
       ├── if dist < threshold → SUCCESS
       └── 이동 명령 발행 (position/trajectory)
```

### 3.4 FollowPath Behavior

#### 경로 처리 설계

```
FollowPathBase:
  path_ids_           : deque<string>  ← 전체 웨이포인트 ID 순서
  path_ids_remaining_ : deque<string>  ← 남은 웨이포인트 ID

follow_path_plugin_position 동작:

  activate():
    path_ids_ 초기화
    path_ids_remaining_ 초기화
    첫 번째 웨이포인트로 이동 명령 시작

  run():
    current_wp = path_.at(path_ids_remaining_.front())
    dist_to_wp = euclidean_distance(actual_pose_, current_wp)

    if dist_to_wp < threshold:
      path_ids_remaining_.pop_front()  ← 다음 웨이포인트로
      if path_ids_remaining_.empty():
        return SUCCESS

    Yaw 모드 처리:
      PATH_FACING: atan2(next.y-cur.y, next.x-cur.x)
      FIXED_YAW:   고정 각도
      KEEP_YAW:    현재 유지

    position_motion_handler_->sendPositionCommand(target_wp)
    feedback:
      remaining_waypoints = path_ids_remaining_.size()
      next_waypoint_id    = path_ids_remaining_.front()
      actual_distance     = dist_to_wp
```

#### 궤적 기반 FollowPath 플러그인

```
follow_path_plugin_trajectory:

  activate():
    goal_ = follow_path_goal
    traj_goal = followPathGoalToTrajectoryGeneratorGoal(goal_)
    traj_gen_client_->async_send_goal(traj_goal)

  run():
    모니터링: RUNNING/SUCCESS/FAILURE 상태 추적

  on_pause():
    traj_gen_pause_client_->call() ← 궤적 생성기도 함께 정지

  on_resume():
    traj_gen_resume_client_->call() ← 궤적 생성기도 함께 재개

  Trajectory 변환:
    FollowPath::Goal (path[])
      → GeneratePolynomialTrajectory::Goal (waypoints[])
    YawMode 변환:
      KEEP_YAW   → KEEP_YAW
      PATH_FACING→ PATH_FACING
      FIXED_YAW  → FIXED_YAW
```

### 3.5 FollowReference Behavior

#### 특징 (플러그인 없음)

```
FollowReferenceBehavior:
  플러그인 없음 — 직접 구현
  상속: BehaviorServer<as2_msgs::action::FollowReference>

  멤버:
  ├── position_motion_handler_ : PositionMotion
  ├── hover_motion_handler_    : HoverMotion
  └── tf_handler_              : TfHandler

  파라미터:
  ├── follow_reference_max_speed_x : float
  ├── follow_reference_max_speed_y : float
  └── follow_reference_max_speed_z : float

  on_run():
    ① 참조점을 earth 좌표계로 변환
    ② Yaw 계산:
       ├── PATH_FACING:  atan2(target.y-cur.y, target.x-cur.x)
       ├── YAW_TO_FRAME: TF 프레임 방향에서 추출
       ├── FIXED_YAW:    goal.yaw 값 사용
       └── KEEP_YAW:     현재 yaw 유지
    ③ position_motion_handler_->sendPositionCommand(
         target_pose, max_speed_x/y/z, yaw)
    ④ feedback 업데이트
    ⑤ return RUNNING  ← 목표 도달 없음! 외부에서 cancel 필요

  on_deactivate():
    hover_motion_handler_->sendHover()  ← 현재 위치 유지
```

---

## 4. as2_behaviors_platform — 플랫폼 제어

### 4.1 SetArmingStateBehavior

```
SetArmingStateBehavior
  상속: BehaviorServer<as2_msgs::action::SetArmingState>

  멤버:
  ├── client_  : rclcpp::Client<SetBool>
  │   → /drone0/set_arming_state 서비스
  └── future_  : rclcpp::Client<SetBool>::SharedFuture

  on_activate(goal):
    request->data = goal->request  ← true=arm, false=disarm
    future_ = client_->async_send_request(request).share()
    return future_.valid()

  on_run():
    if future_.wait_for(0s) == ready:
      result = future_.get()
      result_msg->success = result->success
      return result->success ? SUCCESS : FAILURE
    return RUNNING

  동작 시퀀스:
    GoalRequest(request=true)
        │
        ▼
    on_activate() → /set_arming_state 서비스 비동기 호출
        │
        ▼
    on_run() 반복 → future 완료 대기
        │
        ▼
    결과 반환 → Result(success=true/false)
```

### 4.2 비행 준비 시퀀스

```
완전한 비행 준비 순서:

  [1] SetArmingStateBehavior(request=true)
       → /set_arming_state(SetBool, data=true)
       → 플랫폼: 모터 스핀업, armed=true
       → FSM: DISARMED → LANDED

  [2] SetOffboardModeBehavior(request=true)
       → /set_offboard_mode(SetBool, data=true)
       → 플랫폼: ROS2 명령 수락 모드 전환, offboard=true

  [3] TakeoffBehavior(takeoff_height=2.0)
       → FSM TAKE_OFF 이벤트
       → 플러그인 실행
       → 목표 고도 도달
       → FSM TOOK_OFF 이벤트
       → FSM: LANDED → TAKING_OFF → FLYING
```

---

## 5. as2_behaviors_path_planning — 경로 계획

### 5.1 전체 아키텍처

```
사용자 앱
    │  NavigateToPoint Action
    │  { point: (5,3,2), yaw: KEEP_YAW, speed: 1.0 }
    ▼
PathPlannerBehavior
    │
    ├── [activate] 플러그인으로 경로 계획 시작
    │     plugin_->on_activate(drone_pose, goal)
    │
    ├── [run] 계획 완료 확인
    │     status = plugin_->on_run()
    │     if status == SUCCESS:
    │       path_ = plugin_->path_  ← 계산된 경로
    │       FollowPath 액션 전송
    │
    └── [run] FollowPath 완료 모니터링
          follow_path_result → NavigateToPoint Result 반환

플러그인:
    ├── AStarPlanner    ← 최단 경로 (격자 기반)
    └── VoronoiPlanner  ← 최대 안전 마진 경로
```

### 5.2 A* 알고리즘 구현

```
AStarPlanner (a_star.cpp):

  initialize():
    safety_distance_    : float  ← 장애물 팽창 거리
    enable_path_optimizer: bool  ← 경로 최적화 여부

  on_activate(drone_pose, goal):
    ① Occupancy Grid 수신 (map 토픽)
    ② grid → OpenCV Mat 변환
    ③ 안전 거리 팽창 적용 (update_grid())
    ④ drone_pose → cell 좌표 변환
    ⑤ goal      → cell 좌표 변환
    ⑥ A* 실행: a_star_.solveGraph(start_cell, goal_cell)
    ⑦ cell 경로 → 3D 포인트 변환
    ⑧ 경로 시각화 마커 발행

─────────────────────────────────────────────────────────
AStarAlgorithm (a_star_algorithm.hpp):

  Node 구조:
    position  : Point2i (격자 x, y)
    g_cost_   : float  ← 시작에서의 실제 비용
    h_cost_   : float  ← 목표까지의 휴리스틱 비용
    f_cost_   : g + h  ← 탐색 우선순위
    parent_ptr_: Node* ← 경로 역추적용

  solveGraph(start, goal):
    open_list  = { start_node }
    closed_set = {}

    while open_list not empty:
      current = min(open_list, key=f_cost)

      if current == goal:
        return reconstruct_path(current)

      for neighbor in get_8_neighbors(current):
        if neighbor in closed_set: skip
        if cell_occupied(neighbor): skip

        tentative_g = current.g + step_cost
        if tentative_g < neighbor.g:
          neighbor.parent = current
          neighbor.g = tentative_g
          neighbor.h = calc_h_cost(neighbor, goal)
          open_list.add(neighbor)

      closed_set.add(current)

  calc_h_cost(node, goal):
    return euclidean_distance(node.position, goal.position)

  8방향 이동:
    ↑ ↗ → ↘ ↓ ↙ ← ↖
    비용: 직선=1.0, 대각선=√2
```

### 5.3 격자 좌표 변환

```
공간 좌표 ↔ 격자 좌표 변환 (utils.hpp):

  pointToCell(point, origin, resolution):
    cell.x = (point.x - origin.x) / resolution
    cell.y = (point.y - origin.y) / resolution

  cellToPoint(cell, origin, resolution):
    point.x = cell.x * resolution + origin.x
    point.y = cell.y * resolution + origin.y

  장애물 팽창 (a_star_searcher.cpp):
    safety_pixels = safety_distance_ / resolution
    erode(occupancy_map, kernel_size=safety_pixels)
    → 장애물 주변 safety_distance_ 반경을 점유 공간으로 확장

  맵 → 이미지 변환:
    occupancy > threshold(50) → 흰색(255) 장애물
    occupancy < 0 (미탐색)   → 회색(128)
    점유 없음               → 검정(0)
```

---

## 6. as2_behaviors_trajectory_generation — 궤적 생성

### 6.1 DynamicPolynomialTrajectoryGenerator 구조

```
DynamicPolynomialTrajectoryGenerator
  상속: BehaviorServer<as2_msgs::action::GeneratePolynomialTrajectory>

  핵심 멤버:
  ├── trajectory_generator_ : DynamicTrajectory  ← 외부 라이브러리
  ├── trajectory_motion_handler_: TrajectoryMotion
  ├── hover_motion_handler_     : HoverMotion
  ├── tf_handler_               : TfHandler
  │
  ├── 파라미터:
  │   ├── sampling_n_              : int    ← 평가 샘플 수
  │   ├── sampling_dt_             : double ← 샘플 시간 간격
  │   ├── yaw_threshold_           : float  ← yaw 전환 임계값
  │   ├── wp_close_threshold_      : float  ← 웨이포인트 근접 판정
  │   └── frequency_update_frame_  : double ← 프레임 업데이트 주파수
  │
  └── 타이머:
      └── update_frame_timer_ ← TF 오류 감지 및 보정
```

### 6.2 궤적 생성 흐름

```
on_activate(goal):
  ┌─────────────────────────────────────────────────────┐
  │  1. 오도메트리 확인 (localization_flag_)              │
  │  2. 웨이포인트 변환                                   │
  │     goal.path[] → DynamicWaypoint[]                 │
  │     각 웨이포인트: position + yaw_angle              │
  │  3. trajectory_generator_->setTrajectory(waypoints) │
  │  4. update_frame_timer_ 시작                         │
  │     → updateFrame() 주기적 호출                      │
  └─────────────────────────────────────────────────────┘

on_run():
  ┌─────────────────────────────────────────────────────┐
  │  1. evaluateTrajectory(sampling_n_, sampling_dt_)   │
  │     → N개 setpoint 계산                             │
  │                                                     │
  │  2. 각 setpoint:                                    │
  │     trajectory_generator_->evaluateSetpoint(t)      │
  │     → position, velocity, acceleration              │
  │                                                     │
  │  3. Yaw 계산:                                       │
  │     FACE_REFERENCE: atan2(next-cur)                 │
  │     KEEP_YAW:       initial_yaw 유지                │
  │     PATH_FACING:    atan2(vx, vy)                   │
  │     FIXED_YAW:      goal.yaw 값                     │
  │     YAW_FROM_TOPIC: 외부 토픽                        │
  │                                                     │
  │  4. trajectory_motion_handler_->                    │
  │       sendTrajectoryCommandWithYawAngle(             │
  │         pos, vel, acc, yaw)                         │
  │                                                     │
  │  5. 피드백:                                          │
  │     next_waypoint_id, remaining_waypoints           │
  │                                                     │
  │  6. 종료 조건:                                       │
  │     마지막 웨이포인트 < wp_close_threshold_ → SUCCESS│
  └─────────────────────────────────────────────────────┘

on_pause():
  hover_motion_handler_->sendHover()
  update_frame_timer_ 취소

on_resume():
  현재 웨이포인트부터 궤적 재시작
  update_frame_timer_ 재시작

updateFrame() (TF 오류 감지):
  error = computeErrorFrames(map↔odom 변환)
  if error > transform_threshold_:
    웨이포인트 재조정  ← 드리프트 보정
```

### 6.3 궤적 시각화

```
디버그 시각화 마커:
  plotTrajectory():
    → /trajectory 토픽 (전체 경로 선)
  plotRefTrajPoint(BLUE):
    → 현재 참조점
  plotRefTrajPoint(GREEN):
    → 최종 목표점
  웨이포인트 마커 배열:
    → /waypoints 토픽
```

---

## 7. as2_behaviors_swarm_flocking — 군집 비행

### 7.1 설계 개요

```
SwarmFlockingBehavior — 리더 드론에서 실행

  목표 (Action Goal):
  ├── virtual_centroid  : PoseStamped  ← 무리 가상 중심점
  ├── swarm_formation   : PoseWithID[] ← 각 드론 상대 오프셋
  │   [{"id":"drone0", offset:(0,0,0)},
  │    {"id":"drone1", offset:(1,0,0)},
  │    {"id":"drone2", offset:(-1,0,0)}]
  └── drones_namespace  : string[]     ← 제어할 드론 목록

  내부 구조:
  drones_ : unordered_map<string, shared_ptr<DroneSwarm>>
    key  = drone namespace (e.g., "drone1")
    value= DroneSwarm 객체 (개별 드론 관리)
```

### 7.2 DroneSwarm 클래스

```
DroneSwarm (drone_swarm.hpp)
  개별 드론 관리 클래스

  멤버:
  ├── follow_ref_client_ : rclcpp_action::Client<FollowReference>
  ├── pose_sub_          : Subscription<PoseStamped>
  ├── static_tf_broadcaster_: StaticTransformBroadcaster
  │   → 이 드론의 참조 위치를 TF로 브로드캐스트
  └── current_pose_      : PoseStamped

  initFollowReference():
    FollowReference::Goal goal
    goal.target_pose = {0, 0, 0}  ← 참조 프레임 원점
    goal.reference_frame = 드론 참조 TF 프레임
    goal.max_speed_x/y/z = 설정값
    follow_ref_client_->async_send_goal(goal)

  checkPosition():
    피드백의 actual_distance_to_goal < 0.3m 확인

  updateStaticTf(new_offset):
    static_tf_broadcaster_->sendTransform(
      {centroid_frame → drone_ref_frame, offset})
    → 드론 참조 위치 실시간 업데이트

  stopFollowReference():
    stop 서비스 호출 → FollowReference 중지
```

### 7.3 편대 비행 시퀀스

```
on_activate(goal):
  ┌──────────────────────────────────────────────────────────┐
  │  1. setUpVirtualCentroid(goal.virtual_centroid)          │
  │     → TF: earth → swarm_centroid 브로드캐스트            │
  │                                                          │
  │  2. setUpDronesFormation(goal.swarm_formation)           │
  │     for each drone in formation:                        │
  │       DroneSwarm 객체 생성                               │
  │       updateStaticTf(drone.offset)                      │
  │       → TF: swarm_centroid → drone_ref 브로드캐스트       │
  │                                                          │
  │  3. initDroneReferences()                               │
  │     for each drone:                                     │
  │       drone.initFollowReference()                       │
  │       최대 5초 대기: drone.checkPosition() 확인          │
  └──────────────────────────────────────────────────────────┘

on_run():
  ┌──────────────────────────────────────────────────────────┐
  │  monitoring():                                           │
  │    for each drone:                                      │
  │      if follow_ref 액션 완료/실패 → FAILURE 반환         │
  │    return RUNNING                                        │
  └──────────────────────────────────────────────────────────┘

on_modify(new_goal):
  ┌──────────────────────────────────────────────────────────┐
  │  가상 중심점 위치 업데이트                                │
  │  setUpVirtualCentroid(new_goal.virtual_centroid)        │
  │  → TF 변환만 업데이트                                    │
  │  → 드론들은 자동으로 새 위치로 이동                       │
  └──────────────────────────────────────────────────────────┘

TF 구조 (편대 비행 중):
  earth
    └── swarm_centroid  (동적, 중심점 이동)
          ├── drone0_ref  (정적 오프셋 0,0,0)
          ├── drone1_ref  (정적 오프셋 +1,0,0)
          └── drone2_ref  (정적 오프셋 -1,0,0)

  각 드론의 FollowReference는 자신의 drone_N_ref 프레임 원점을 추종
  → 가상 중심점 이동 시 모든 드론이 오프셋 유지하며 이동
```

---

## 8. as2_behaviors_payload — 페이로드 제어

### 8.1 GripperHandlerBehavior

```
GripperHandlerBehavior
  상속: BehaviorServer<as2_msgs::action::GripperHandler>
  플러그인 기반 (pluginlib)

  Goal:
    request_gripper : bool  (true=닫기, false=열기)

  플러그인 인터페이스 (GripperBase):
    own_activate()
    own_run()      → ExecutionStatus
    own_modify()
    own_deactivate()
    own_pause()
    own_resume()
    own_execution_end()
```

#### 2가지 그리퍼 플러그인

```
┌─────────────────────────────────────────────────────────────┐
│  dc_servo 플러그인                                           │
│                                                             │
│  파라미터:                                                   │
│    angle_to_open  : 45.0°   (열기 각도)                     │
│    angle_to_close : 62.0°   (닫기 각도)                     │
│    max_angle      : 180.0°  (최대 각도)                     │
│    duty_min       : PWM 최소 듀티                           │
│    duty_max       : PWM 최대 듀티                           │
│                                                             │
│  own_run():                                                 │
│    target_angle = goal_->request_gripper ?                 │
│                   angle_to_close : angle_to_open            │
│    pwm = angle_to_pwm(target_angle)                        │
│    → PWM 신호 발행                                          │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│  two_fingers 플러그인                                        │
│                                                             │
│  파라미터:                                                   │
│    l_finger_open, l_finger_close  ← 왼쪽 손가락 위치        │
│    r_finger_open, r_finger_close  ← 오른쪽 손가락 위치      │
│    topic_l_finger                 ← 왼쪽 명령 토픽          │
│    topic_r_finger                 ← 오른쪽 명령 토픽        │
│                                                             │
│  own_run():                                                 │
│    l_pos = goal_->request ? l_close : l_open               │
│    r_pos = goal_->request ? r_close : r_open               │
│    left_pub_->publish(Float64(l_pos))                      │
│    right_pub_->publish(Float64(r_pos))                     │
└─────────────────────────────────────────────────────────────┘
```

### 8.2 PointGimbalBehavior

```
PointGimbalBehavior
  상속: BehaviorServer<as2_msgs::action::PointGimbal>
  플러그인 없음 — 직접 구현

  파라미터:
    gimbal_name          : string
    gimbal_base_frame_id : string
    gimbal_frame_id      : string
    gimbal_threshold     : 0.01 (rad)
    roll/pitch/yaw_range : ±4π (rad)
    behavior_timeout     : 10.0 (s)

  두 가지 모드:

  POINT_MODE (3D 점을 바라봄):
    activate():
      방향 벡터 = target - current_pos
      방향 = 정규화(방향 벡터)
      roll  = 0
      pitch = -asin(direction.z)
      yaw   = atan2(direction.y, direction.x)
      → GimbalControl 명령 발행

  MOVE_MODE (각도로 이동):
    activate():
      ENU 프레임에서 base_link로 변환
      각도 오프셋 계산
      → GimbalControl 명령 발행

  run():
    update_gimbal_state():
      TF에서 현재 gimbal 자세 조회
      quaternion → roll/pitch/yaw 변환

    point_check_finished():  (POINT_MODE)
      desired_vector vs current_vector 사이 각도 < threshold

    move_check_finished():  (MOVE_MODE)
      각 축별 |원하는각 - 현재각| < threshold

    check_gimbal_limits():
      각 축이 range 내인지 확인 → 범위 초과 시 FAILURE
```

---

## 9. as2_behaviors_perception — 인식

### 9.1 DetectArucoMarkersBehavior

```
DetectArucoMarkersBehavior
  상속: BehaviorServer<as2_msgs::action::DetectArucoMarkers>

  멤버:
  ├── aruco_dict_    : cv::Ptr<cv::aruco::Dictionary>
  ├── camera_matrix_ : cv::Mat  (3×3 내부 행렬)
  ├── dist_coeffs_   : cv::Mat  (왜곡 계수)
  ├── target_ids_    : vector<uint16_t>
  └── aruco_size_    : float  (마커 실제 크기, m)

  Goal:
    target_ids[] : 감지할 마커 ID 목록 (비어있으면 전체)
```

#### ArUco 감지 파이프라인

```
카메라 이미지 수신
        │
        ▼
setCameraParameters(CameraInfo):
  camera_matrix_(0,0) = fx  (초점거리 x)
  camera_matrix_(1,1) = fy  (초점거리 y)
  camera_matrix_(0,2) = cx  (주점 x)
  camera_matrix_(1,2) = cy  (주점 y)
  dist_coeffs_ = [k1, k2, p1, p2, k3, ...]

        │
        ▼
imageCallback(Image):
  ① 이미지 모델 확인
     Pinhole: undistort(img, camera_matrix_, dist_coeffs_)
     Fisheye: fisheye::undistortImage(...)

  ② ArUco 감지
     cv::aruco::detectMarkers(
       img, aruco_dict_,
       corners, ids)  ← 감지된 마커 코너 + ID

  ③ ID 필터링
     checkIdIsTarget(id):
       target_ids_.empty() → 모두 반환
       id in target_ids_   → 반환

  ④ 3D 자세 추정
     cv::aruco::estimatePoseSingleMarkers(
       corners, aruco_size_,
       camera_matrix_, dist_coeffs_,
       rvecs, tvecs)
     → 각 마커의 rotation/translation 벡터

  ⑤ 카메라 → earth 좌표계 변환
     tf_handler_->transform(marker_pose, "earth")

  ⑥ 결과 발행
     PoseStampedWithIDArray 발행
     → id: 마커 ID (string)
     → pose: earth 좌표계의 마커 위치/자세

  ⑦ Feedback
     detected: 현재 감지 성공 여부
```

---

## 10. as2_behaviors_param_estimation — 파라미터 추정

### 10.1 MassEstimationBehavior

```
MassEstimationBehavior
  상속: BehaviorServer<as2_msgs::action::MassEstimation>

  파라미터:
    mass_threshold       : 질량 변화 임계값
    thrust_threshold     : 추력 임계값
    alpha                : 저역 필터 계수 (0~1)
    n_samples            : 평균 계산 샘플 수
    minimum_mass         : 최소 허용 질량
    maximum_mass         : 최대 허용 질량
    controler_node       : 제어기 파라미터 서버 노드명
    mass_param_name      : 업데이트할 파라미터명

  입력:
    command_thrust_      : Thrust 메시지 구독
    imu_                 : IMU 메시지 구독

  출력:
    estimated_mass_pub_  : Float32 (추정 질량)
    filtered_mass_pub_   : Float32 (필터링된 질량)
```

#### 질량 추정 수식

```
ParamEstimation 클래스:

  computeMass(thrust, a_z):
    g = 9.81 m/s²
    mass = thrust / (a_z + g)
    → 호버 시 thrust = mass * g, a_z = 0 가정

  lowPassFilter(current, last, alpha):
    filtered = alpha * current + (1 - alpha) * last
    alpha = 1   → 필터 없음 (즉시 반응)
    alpha = 0.1 → 강한 필터 (느린 변화)

  computeMeanFromNSamples(samples, n):
    return mean(samples[-n:])  ← 최근 n개 평균

샘플링 조건:
  thrust > thrust_threshold 일 때만 샘플 수집
  (저추력 = 불확실한 상황)
```

### 10.2 ForceEstimationBehavior

```
ForceEstimationBehavior
  상속: BehaviorServer<as2_msgs::action::ForceEstimation>

  목적: 외부 교란력 추정
    바람, 페이로드 무게, 충돌 등

  ForceEstimation 클래스:
    computeThrustError(mass, a_z_mean, u_thrust):
      F_measured = mass * a_z_mean  (실제 가속도로 역산)
      F_error = F_measured - u_thrust  (명령 추력과 차이)
      return F_error
      → 양수: 예상보다 더 큰 힘 (상승 저항)
      → 음수: 예상보다 작은 힘 (추가 하중)

  출력:
    thrust_error_pub_         : Float32 (추력 오차)
    filtered_thrust_error_pub_: Float32 (필터링)
    limited_thrust_error_pub_ : Float32 (제한됨)
    → 제어기 force_param 파라미터 업데이트
```

---

## 11. 플러그인 시스템 전체 지도

```
패키지                    기반 클래스              플러그인 (구현)
────────────────────────────────────────────────────────────────────
as2_behaviors_motion      takeoff_base::TakeoffBase
                            ├── TakeoffPluginSpeed
                            ├── TakeoffPluginPosition
                            ├── TakeoffPluginPlatform
                            └── TakeoffPluginTrajectory

                          land_base::LandBase
                            ├── LandPluginSpeed
                            ├── LandPluginPlatform
                            └── LandPluginTrajectory

                          go_to_base::GoToBase
                            ├── GoToPluginPosition
                            └── GoToPluginTrajectory

                          follow_path_base::FollowPathBase
                            ├── FollowPathPluginPosition
                            └── FollowPathPluginTrajectory

as2_behaviors_path_planning as2_behaviors_path_planning::PluginBase
                            ├── AStarPlanner
                            └── VoronoiPlanner

as2_behaviors_payload      gripper_behavior_plugin_base::GripperBase
                            ├── DcServoGripper
                            └── TwoFingersGripper

플러그인 선택 방법 (YAML):
  plugin_name: "as2_behaviors_motion/TakeoffPluginSpeed"
```

### 플러그인 로딩 패턴 (모든 행동에서 동일)

```cpp
// 1. 파라미터에서 플러그인 이름 읽기
std::string plugin_name = this->get_parameter("plugin_name").as_string();
plugin_name += "::Plugin";  // "패키지/클래스::Plugin"

// 2. ClassLoader 생성
loader_ = std::make_shared<pluginlib::ClassLoader<TakeoffBase>>(
  "as2_behaviors_motion",      // 패키지 이름 (plugins.xml 위치)
  "takeoff_base::TakeoffBase"  // 기반 클래스 전체 이름
);

// 3. 인스턴스 생성
takeoff_plugin_ = loader_->createSharedInstance(plugin_name);

// 4. 초기화
takeoff_plugin_->initialize(this, tf_handler_, params);
```

---

## 12. 행동 간 의존 관계

```
행동 내부에서 다른 행동을 액션 클라이언트로 호출하는 관계:

  PathPlannerBehavior
    └──[Action Client]──► FollowPathBehavior
         (NavigateToPoint)   (FollowPath)

  LandBehavior (trajectory 플러그인)
    └──[Action Client]──► DynamicPolynomialTrajectoryGenerator
         (Land)               (GeneratePolynomialTrajectory)

  FollowPathBehavior (trajectory 플러그인)
    └──[Action Client]──► DynamicPolynomialTrajectoryGenerator
         (FollowPath)          (GeneratePolynomialTrajectory)

  GoToBehavior (trajectory 플러그인)
    └──[Action Client]──► DynamicPolynomialTrajectoryGenerator
         (GoToWaypoint)        (GeneratePolynomialTrajectory)

  TakeoffBehavior (trajectory 플러그인)
    └──[Action Client]──► DynamicPolynomialTrajectoryGenerator
         (Takeoff)             (GeneratePolynomialTrajectory)

  SwarmFlockingBehavior
    └──[N × Action Client]──► FollowReferenceBehavior
         (SwarmFlocking)         (FollowReference, 드론별 1개)

행동과 플랫폼 간 서비스 의존:
  TakeoffBehavior, LandBehavior, GoToBehavior, FollowPathBehavior
    └──[Service]──► /set_platform_state_machine_event
                     (FSM: TAKE_OFF/TOOK_OFF/LAND/LANDED/EMERGENCY)

  LandBehavior
    └──[Service]──► /set_arming_state (착륙 후 무장 해제)
```

---

## 13. 설정 파라미터 전체 목록

### 이동 행동 파라미터

```yaml
# takeoff_behavior config_default.yaml
/**:
  ros__parameters:
    plugin_name:         "as2_behaviors_motion/TakeoffPluginSpeed"
    takeoff_height:      1.0     # 기본 이륙 고도 (m)
    takeoff_speed:       0.5     # 이륙 속도 (m/s)
    takeoff_threshold:   0.2     # 완료 판정 오차 (m)
    tf_timeout_threshold: 0.05   # TF 타임아웃 (s)

# land_behavior config_default.yaml
/**:
  ros__parameters:
    plugin_name:                    "as2_behaviors_motion/LandPluginSpeed"
    land_speed:                     0.5   # 착륙 속도 (m/s)
    land_speed_condition_percentage: 0.2  # 완료 속도 비율
    land_speed_condition_height:    0.2   # 완료 높이 (m)
    land_trajectory_height:         -10.0 # 궤적 목표 높이 (m)
    tf_timeout_threshold:           0.05

# go_to_behavior config_default.yaml
/**:
  ros__parameters:
    plugin_name:         "as2_behaviors_motion/GoToPluginPosition"
    go_to_speed:         0.5   # 이동 속도 (m/s)
    go_to_threshold:     0.2   # 완료 판정 오차 (m)
    tf_timeout_threshold: 0.05
```

### 경로 계획 파라미터

```yaml
# path_planner_behavior
/**:
  ros__parameters:
    plugin_name:          "as2_behaviors_path_planning/AStarPlanner"
    enable_visualization: false
    safety_distance:      0.5   # 장애물 팽창 거리 (m)
    enable_path_optimizer: false
```

### 궤적 생성 파라미터

```yaml
# generate_polynomial_trajectory_behavior
/**:
  ros__parameters:
    sampling_n:              1      # 평가 샘플 수
    sampling_dt:             0.0    # 샘플 시간 간격 (s)
    yaw_threshold:           0.4    # yaw 전환 임계값 (rad)
    wp_close_threshold:      0.5    # 웨이포인트 근접 판정 (m)
    frequency_update_frame:  10.0   # 프레임 업데이트 빈도 (Hz)
    transform_threshold:     0.1    # TF 오류 임계값 (m)
```

### 짐벌 파라미터

```yaml
# point_gimbal_behavior
/**:
  ros__parameters:
    gimbal_name:           "gimbal"
    gimbal_base_frame_id:  "gimbal_base"
    gimbal_frame_id:       "gimbal"
    gimbal_threshold:      0.01   # 정렬 임계값 (rad)
    roll_range:            12.57  # ±4π (rad)
    pitch_range:           12.57
    yaw_range:             12.57
    behavior_timeout:      10.0   # 타임아웃 (s)
```

### 질량 추정 파라미터

```yaml
# mass_estimation_behavior
/**:
  ros__parameters:
    mass_threshold:     0.1     # 질량 변화 임계값 (kg)
    thrust_threshold:   1.0     # 샘플링 추력 임계값 (N)
    alpha:              0.1     # 저역 필터 계수
    n_samples:          100     # 평균 샘플 수
    minimum_mass:       0.1     # 최소 질량 (kg)
    maximum_mass:       10.0    # 최대 질량 (kg)
    controler_node:     "motion_controller"
    mass_param_name:    "mass"
```

---

## 부록 — 핵심 파일 인덱스

| 파일 | 경로 |
|------|------|
| BehaviorServer 구현 | `as2_behavior/include/as2_behavior/__impl/behavior_server__impl.hpp` |
| BehaviorServer 클래스 | `as2_behavior/include/as2_behavior/__detail/behavior_server__class.hpp` |
| ExecutionStatus | `as2_behavior/include/as2_behavior/behavior_utils.hpp` |
| Takeoff 행동 | `as2_behaviors_motion/takeoff_behavior/src/takeoff_behavior.cpp` |
| Takeoff 플러그인 인터페이스 | `as2_behaviors_motion/takeoff_behavior/include/takeoff_behavior/takeoff_base.hpp` |
| Takeoff Speed 플러그인 | `as2_behaviors_motion/takeoff_behavior/plugins/takeoff_plugin_speed.cpp` |
| Land 행동 | `as2_behaviors_motion/land_behavior/src/land_behavior.cpp` |
| GoTo 행동 | `as2_behaviors_motion/go_to_behavior/src/go_to_behavior.cpp` |
| FollowPath 위치 플러그인 | `as2_behaviors_motion/follow_path_behavior/plugins/follow_path_plugin_position.cpp` |
| FollowReference 행동 | `as2_behaviors_motion/follow_reference_behavior/src/follow_reference_behavior.cpp` |
| Arm 행동 | `as2_behaviors_platform/include/as2_behaviors_platform/set_arming_state_behavior.hpp` |
| 경로 계획 행동 | `as2_behaviors_path_planning/src/path_planner_behavior.cpp` |
| A* 알고리즘 | `as2_behaviors_path_planning/plugins/a_star/include/a_star_algorithm.hpp` |
| 궤적 생성 행동 | `as2_behaviors_trajectory_generation/generate_polynomial_trajectory_behavior/src/generate_polynomial_trajectory_behavior.cpp` |
| 군집 행동 | `as2_behaviors_swarm_flocking/src/swarm_flocking_behavior.cpp` |
| DroneSwarm | `as2_behaviors_swarm_flocking/src/drone_swarm.cpp` |
| 그리퍼 행동 | `as2_behaviors_payload/gripper_behavior/src/gripper_behavior.cpp` |
| 짐벌 행동 | `as2_behaviors_payload/point_gimbal_behavior/src/point_gimbal_behavior.cpp` |
| ArUco 감지 | `as2_behaviors_perception/detect_aruco_markers_behavior/src/detect_aruco_markers_behavior.cpp` |
| 질량 추정 | `as2_behaviors_param_estimation/mass_estimation_behavior/include/mass_estimation_behavior/param_estimation.hpp` |
