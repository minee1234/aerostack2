# as2_behavior_tree 패키지 전문가 심층 분석

> **버전:** 1.1.3 | **ROS2:** Humble / Jazzy | **C++ 표준:** C++17
> **라이선스:** Apache-2.0 (bt_action_node/bt_service_node) + BSD-3-Clause (나머지)
> **분석 기준:** 실제 소스 코드 직접 정독 (2026-03-26)

---

## 목차

1. [패키지 개요 및 위치](#1-패키지-개요-및-위치)
2. [전체 파일 구조](#2-전체-파일-구조)
3. [외부 의존성](#3-외부-의존성)
4. [아키텍처 개요](#4-아키텍처-개요)
5. [핵심 프레임워크: BtActionNode](#5-핵심-프레임워크-btactionnode)
6. [핵심 프레임워크: BtServiceNode](#6-핵심-프레임워크-btservicenode)
7. [포트 특수화 시스템](#7-포트-특수화-시스템)
8. [등록된 BT 노드 전체 목록](#8-등록된-bt-노드-전체-목록)
9. [Action 노드 상세 분석](#9-action-노드-상세-분석)
10. [Condition 노드 상세 분석](#10-condition-노드-상세-분석)
11. [Decorator 노드 상세 분석](#11-decorator-노드-상세-분석)
12. [메인 실행기: as2_behavior_tree_node](#12-메인-실행기-as2_behavior_tree_node)
13. [블랙보드 설계](#13-블랙보드-설계)
14. [XML 트리 정의 예제 분석](#14-xml-트리-정의-예제-분석)
15. [내부 동작 메커니즘](#15-내부-동작-메커니즘)
16. [설계 패턴 분석](#16-설계-패턴-분석)
17. [주의사항 및 Known Issues](#17-주의사항-및-known-issues)
18. [새 BT 노드 구현 가이드](#18-새-bt-노드-구현-가이드)

---

## 1. 패키지 개요 및 위치

`as2_behavior_tree`는 BehaviorTree.CPP v4 라이브러리를 기반으로 Aerostack2 드론 제어 시스템에 **Behavior Tree 기반 임무 시퀀싱**을 제공하는 패키지다.

### 프레임워크 내 위치

```
[사용자 임무 XML 파일]
        ↓
[as2_behavior_tree_node] ← 이 패키지의 진입점
        ↓ tick()
[BT 노드들: Action, Condition, Decorator]
        ↓ ROS2 Action/Service 호출
[as2_behaviors: TakeoffBehavior, GoToBehavior, LandBehavior ...]
        ↓
[as2_motion_controller → as2_aerial_platform → 하드웨어]
```

**핵심 역할:**
- XML로 정의된 임무 시퀀스를 파싱하고 실행
- ROS2 Action/Service를 BT 노드로 감싸는 어댑터 제공
- Groot2 시각화 도구와의 연동 지원
- 이벤트/알림 기반 반응형 행동 지원

---

## 2. 전체 파일 구조

```
as2_behavior_tree/
├── CMakeLists.txt
├── package.xml
├── README.md
│
├── include/as2_behavior_tree/
│   ├── bt_action_node.hpp          # ★★★ ROS2 Action → BT 노드 템플릿 기반 클래스
│   ├── bt_service_node.hpp         # ★★★ ROS2 Service → BT 노드 템플릿 기반 클래스
│   ├── port_specialization.hpp     # ★★  BT 포트 타입 변환 (문자열 → ROS 메시지)
│   │
│   ├── action/
│   │   ├── arm_service.hpp         # ArmService, DisarmService
│   │   ├── offboard_service.hpp    # OffboardService
│   │   ├── takeoff_action.hpp      # TakeoffAction
│   │   ├── land_action.hpp         # LandAction
│   │   ├── go_to_action.hpp        # GoToAction
│   │   ├── go_to_gps_action.hpp    # GoToGpsAction (GPS→Cartesian 내장)
│   │   ├── follow_path.hpp         # FollowPathAction
│   │   ├── echo.hpp                # Echo (디버그용)
│   │   ├── send_event.hpp          # SendEvent (토픽 발행)
│   │   ├── set_origin.hpp          # SetOrigin (GPS 원점 설정)
│   │   ├── get_origin.hpp          # GetOrigin (GPS 원점 조회)
│   │   └── gps_to_cartesian.hpp    # GpsToCartesian (좌표 변환)
│   │
│   ├── condition/
│   │   └── is_flying_condition.hpp # IsFlyingCondition
│   │
│   └── decorator/
│       ├── wait_for_event.hpp      # WaitForEvent (String 토픽 대기)
│       └── wait_for_alert.hpp      # WaitForAlert (AlertEvent 대기)
│
├── plugins/
│   ├── action/
│   │   ├── takeoff_action.cpp
│   │   ├── land_action.cpp
│   │   ├── go_to_action.cpp
│   │   ├── arm_service.cpp
│   │   ├── offboard_service.cpp
│   │   ├── echo.cpp
│   │   ├── send_event.cpp
│   │   ├── set_origin.cpp
│   │   ├── get_origin.cpp
│   │   ├── gps_to_cartesian.cpp
│   │   └── go_to_gps_action.cpp
│   ├── condition/
│   │   └── is_flying_condition.cpp
│   └── decorator/
│       ├── wait_for_event.cpp
│       └── wait_for_alert.cpp
│
├── src/
│   └── as2_behavior_tree_node.cpp  # ★★★ 메인 실행 진입점
│
├── resource/                        # 예제 XML 트리 파일들
│   ├── basic_mission.xml            # TakeOff → GoTo → Land
│   ├── takeoff.xml                  # WaitForEvent → IsFlying? → ArmTakeoff
│   ├── arm_test.xml                 # Arm 단독 테스트
│   ├── test.xml                     # Arm → Offboard → TakeOff
│   ├── event_test.xml               # SendEvent + WaitForEvent 테스트
│   ├── event_test_2.xml             # 다중 이벤트 시퀀스
│   ├── follow_path_test.xml         # FollowPath 테스트
│   ├── RTL_test_2.xml               # GPS + RTL 복합 임무
│   └── actions.xml                  # 전체 노드 포트 레퍼런스
│
├── launch/                          # 런치 파일
├── tests/                           # 포트 특수화 단위 테스트
│   ├── port_test.cpp
│   └── port_pose_test.cpp
└── docs/
```

---

## 3. 외부 의존성

```
as2_behavior_tree 의존성
│
├── behaviortree_cpp          ★★★ BehaviorTree.CPP v4 — BT 엔진
│   ├── BT::ActionNodeBase    — 액션 노드 기반 클래스
│   ├── BT::ConditionNode     — 조건 노드 기반 클래스
│   ├── BT::DecoratorNode     — 데코레이터 노드 기반 클래스
│   ├── BT::BehaviorTreeFactory — 노드 등록 및 트리 생성
│   ├── BT::Blackboard        — 노드 간 데이터 공유
│   └── BT::Groot2Publisher   — Groot2 시각화 연동
│
├── rclcpp                    ROS2 노드, 토픽, 서비스
├── rclcpp_action             ROS2 Action 클라이언트
├── as2_core                  as2_names (토픽/액션 이름 상수)
├── as2_msgs                  Takeoff, Land, GoToWaypoint 등 AS2 액션/서비스
├── std_srvs                  SetBool (Arm, Offboard 서비스)
└── geometry_msgs             Pose, PointStamped
```

---

## 4. 아키텍처 개요

```
┌────────────────────────────────────────────────────────────────┐
│                  as2_behavior_tree                             │
│                                                                │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │              BT::BehaviorTreeFactory                     │  │
│  │   registerNodeType<TakeoffAction>("TakeOff")             │  │
│  │   registerNodeType<ArmService>("Arm")  ...16개 등록      │  │
│  └──────────────────────────────────────────────────────────┘  │
│                          ↓ createTreeFromFile(xml)             │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │                    BT::Tree                              │  │
│  │  ┌─────────────┐  ┌─────────────┐  ┌────────────────┐  │  │
│  │  │ BtActionNode│  │BtServiceNode│  │  BT 내장 노드  │  │  │
│  │  │  <ActionT>  │  │ <ServiceT>  │  │ Sequence       │  │  │
│  │  │             │  │             │  │ Fallback       │  │  │
│  │  │ Action      │  │ Service     │  │ Parallel       │  │  │
│  │  │ Client      │  │ Client      │  │ SubTree        │  │  │
│  │  └──────┬──────┘  └──────┬──────┘  └────────────────┘  │  │
│  └─────────┼───────────────┼──────────────────────────────┘  │
│            ↓               ↓                                   │
│  ┌─────────────────────────────────────────────────────────┐  │
│  │              공유 블랙보드 (BT::Blackboard)              │  │
│  │  "node"              → rclcpp::Node::SharedPtr          │  │
│  │  "server_timeout"    → chrono::milliseconds             │  │
│  │  "bt_loop_duration"  → chrono::milliseconds             │  │
│  │  "wait_for_service_timeout" → chrono::milliseconds      │  │
│  │  "{var}", "{lat}", "{lon}" → 사용자 정의 변수           │  │
│  └─────────────────────────────────────────────────────────┘  │
│                                                                │
│  메인 루프: tickWhileRunning() @ bt_loop_duration Hz          │
└────────────────────────────────────────────────────────────────┘
                          ↓ ROS2 Action/Service
┌────────────────────────────────────────────────────────────────┐
│  as2_behaviors: TakeoffBehavior, LandBehavior, GoToBehavior.. │
└────────────────────────────────────────────────────────────────┘
```

---

## 5. 핵심 프레임워크: BtActionNode

**파일:** `include/as2_behavior_tree/bt_action_node.hpp`
**원작자:** Intel Corporation (Apache-2.0)

### 클래스 구조

```cpp
template<class ActionT>
class BtActionNode : public BT::ActionNodeBase
{
protected:
    std::string action_name_;
    std::shared_ptr<rclcpp_action::Client<ActionT>> action_client_;

    typename ActionT::Goal goal_;          // 전송할 목표
    bool goal_updated_{false};             // 목표 업데이트 플래그
    bool goal_result_available_{false};    // 결과 수신 플래그
    typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr goal_handle_;
    typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult result_;
    std::shared_ptr<const typename ActionT::Feedback> feedback_;

    rclcpp::Node::SharedPtr node_;         // 블랙보드에서 가져온 공유 노드
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

    std::chrono::milliseconds server_timeout_;    // 서버 응답 대기 타임아웃
    std::chrono::milliseconds bt_loop_duration_;  // BT 틱 주기
};
```

### 생성자 초기화 순서

```
BtActionNode 생성자
│
├── 블랙보드에서 공유 노드 가져오기
│   └── node_ = blackboard->get<rclcpp::Node::SharedPtr>("node")
│
├── 독립 CallbackGroup + Executor 생성
│   ├── create_callback_group(MutuallyExclusive, false)
│   └── callback_group_executor_.add_callback_group(...)
│
├── 블랙보드에서 타임아웃 값 가져오기
│   ├── bt_loop_duration_ = blackboard->get("bt_loop_duration")
│   └── server_timeout_   = blackboard->get("server_timeout")
│
├── "server_name" 포트로 액션 이름 오버라이드 가능
│   └── getInput("server_name", remapped_action_name)
│
└── createActionClient(action_name_)
    └── wait_for_action_server(1s) — 서버 없으면 예외
```

### tick() 상태 머신

```
첫 번째 tick (status == IDLE):
    setStatus(RUNNING)
    on_tick()          ← 서브클래스가 goal_ 채움
    send_new_goal()    ← async_send_goal() 전송

이후 tick (status == RUNNING):
    [Phase 1] future_goal_handle_ 처리
        ├── is_future_goal_handle_complete() 호출
        │   └── spin_until_future_complete(bt_loop_duration_)
        ├── 완료 → goal_handle_ 획득
        └── server_timeout_ 초과 → FAILURE 반환

    [Phase 2] 결과 대기 루프
        on_wait_for_result(feedback_)   ← 피드백 처리 훅
        goal_updated_ 이면 새 goal 재전송 (동적 목표 업데이트)
        callback_group_executor_.spin_some()
        goal_result_available_ 아직 false → RUNNING 반환

결과 수신 시:
    SUCCEEDED → on_success() (기본: SUCCESS)
    ABORTED   → on_aborted() (기본: FAILURE)
    CANCELED  → on_cancelled() (기본: SUCCESS)
```

### halt() — 강제 종료

```cpp
void halt() override {
    if (should_cancel_goal()) {
        // async_cancel_goal() → spin_until_future_complete(server_timeout_)
        action_client_->async_cancel_goal(goal_handle_);
    }
    setStatus(BT::NodeStatus::IDLE);
}
```

트리가 중단되거나 다른 브랜치로 전환될 때 실행 중인 ROS2 Action을 자동으로 취소한다.

### 제공되는 기본 포트

```cpp
static BT::PortsList providedBasicPorts(BT::PortsList addition) {
    BT::PortsList basic = {
        BT::InputPort<std::string>("server_name"),         // 액션 서버 이름 오버라이드
        BT::InputPort<std::chrono::milliseconds>("server_timeout")
    };
    basic.insert(addition.begin(), addition.end());
    return basic;
}
```

서브클래스는 `providedBasicPorts({...추가 포트...})`를 호출해 기본 포트에 노드별 포트를 추가한다.

---

## 6. 핵심 프레임워크: BtServiceNode

**파일:** `include/as2_behavior_tree/bt_service_node.hpp`
**원작자:** Samsung Research America (Apache-2.0)

### 클래스 구조

```cpp
template<class ServiceT>
class BtServiceNode : public BT::ActionNodeBase
{
protected:
    std::string service_name_, service_node_name_;
    std::shared_ptr<rclcpp::Client<ServiceT>> service_client_;
    std::shared_ptr<typename ServiceT::Request> request_;

    rclcpp::Node::SharedPtr node_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

    std::chrono::milliseconds server_timeout_;
    std::chrono::milliseconds bt_loop_duration_;
    std::shared_future<typename ServiceT::Response::SharedPtr> future_result_;
    bool request_sent_{false};
    rclcpp::Time sent_time_;
};
```

### tick() 흐름

```
첫 번째 tick (request_sent_ == false):
    on_tick()                      ← request_ 데이터 채우기
    async_send_request(request_)   ← 비동기 요청 전송
    request_sent_ = true

check_future() 호출:
    spin_until_future_complete(min(remaining, bt_loop_duration_))
    SUCCESS  → request_sent_=false, on_completion(response) 호출
    TIMEOUT  → on_wait_for_result() 호출
               elapsed < server_timeout_ → RUNNING 반환
    최종 타임아웃 → FAILURE 반환
```

**BtActionNode vs BtServiceNode 비교:**

| 항목 | BtActionNode | BtServiceNode |
|------|-------------|---------------|
| 기반 클래스 | `BT::ActionNodeBase` | `BT::ActionNodeBase` |
| ROS2 인터페이스 | `rclcpp_action::Client<T>` | `rclcpp::Client<T>` |
| 비동기 핵심 | `async_send_goal()` | `async_send_request()` |
| 생성자 대기 | `wait_for_action_server(1s)` | `wait_for_service()` (무한) |
| 결과 콜백 | `on_success/aborted/cancelled` | `on_completion(response)` |
| 진행 피드백 | `on_wait_for_result(feedback)` | `on_wait_for_result()` |
| 중단 처리 | `async_cancel_goal()` | `request_sent_=false` |

---

## 7. 포트 특수화 시스템

**파일:** `include/as2_behavior_tree/port_specialization.hpp`

BT 포트는 기본적으로 문자열로 전달된다. ROS2 메시지 타입을 직접 포트로 사용하려면 `BT::convertFromString<T>()` 템플릿을 특수화해야 한다.

### 구현된 특수화

#### 1. `geometry_msgs::msg::Pose` — 세미콜론 구분 "x;y;z"

```cpp
template<>
inline geometry_msgs::msg::Pose convertFromString(BT::StringView str)
{
    auto parts = splitString(str, ';');  // "5;3;2" → ["5","3","2"]
    if (parts.size() != 3) throw RuntimeError("invalid input");
    geometry_msgs::msg::Pose output;
    output.position.x = convertFromString<double>(parts[0]);
    output.position.y = convertFromString<double>(parts[1]);
    output.position.z = convertFromString<double>(parts[2]);
    return output;
}
```

**XML 사용 예:**
```xml
<Action ID="GoTo" pose="5;5;2" .../>
```

#### 2. `geometry_msgs::msg::PointStamped` — "x;y;z" + frame_id="earth" 고정

```cpp
template<>
inline geometry_msgs::msg::PointStamped convertFromString(BT::StringView str)
{
    auto parts = splitString(str, ';');
    geometry_msgs::msg::PointStamped output;
    output.header.frame_id = "earth";   // ← 항상 지구 기준 프레임
    output.point.x = convertFromString<double>(parts[0]);
    output.point.y = convertFromString<double>(parts[1]);
    output.point.z = convertFromString<double>(parts[2]);
    return output;
}
```

#### 3. `std::vector<as2_msgs::msg::PoseWithID>` — 파이프(`|`) 구분 다중 웨이포인트

```cpp
template<>
inline std::vector<as2_msgs::msg::PoseWithID>
convertFromString(BT::StringView str)
{
    auto points = splitString(str, '|');   // "1;2;3|4;5;6" → 2개 포인트
    // 현재 구현은 정확히 2개 포인트만 지원 (TODO 있음)
    // points[0] → id="0", points[1] → id="1"
}
```

**XML 사용 예:**
```xml
<Action ID="FollowPath" path="1;2;3|4;5;6" speed="1.0"/>
```

**한계:** 현재 구현은 정확히 2개 웨이포인트만 파싱한다. 코드 주석에 `// TODO(pariaspe): generalize`가 명시되어 있다.

---

## 8. 등록된 BT 노드 전체 목록

`as2_behavior_tree_node.cpp`에서 팩토리에 등록되는 16개 노드:

| XML ID | C++ 클래스 | 타입 | 기반 ROS2 |
|--------|-----------|------|----------|
| `Arm` | `ArmService` | Action(Service) | `set_arming_state` (SetBool) |
| `Disarm` | `DisarmService` | Action(Service) | `set_arming_state` (SetBool) |
| `Offboard` | `OffboardService` | Action(Service) | `set_offboard_mode` (SetBool) |
| `TakeOff` | `TakeoffAction` | Action(Action) | `TakeoffBehavior` |
| `GoTo` | `GoToAction` | Action(Action) | `GoToBehavior` |
| `Land` | `LandAction` | Action(Action) | `LandBehavior` |
| `FollowPath` | `FollowPathAction` | Action(Action) | `FollowPathBehavior` |
| `Echo` | `Echo` | SyncAction | - |
| `SendEvent` | `SendEvent` | SyncAction | std_msgs/String 발행 |
| `SetOrigin` | `SetOrigin` | Action(Service) | `set_origin` |
| `GetOrigin` | `GetOrigin` | Action(Service) | `get_origin` |
| `GpsToCartesian` | `GpsToCartesian` | Action(Service) | `geopath_to_path` |
| `GoToGps` | `GoToGpsAction` | Action(Action) | `GoToBehavior` + `geopath_to_path` |
| `IsFlying` | `IsFlyingCondition` | Condition | `platform/info` 구독 |
| `WaitForEvent` | `WaitForEvent` | Decorator | String 토픽 구독 |
| `WaitForAlert` | `WaitForAlert` | Decorator | `alert_event` 구독 |

---

## 9. Action 노드 상세 분석

### 9.1 TakeoffAction

```cpp
class TakeoffAction : public BtActionNode<as2_msgs::action::Takeoff>
```

**포트:**

| 포트 | 방향 | 타입 | 기본값 | 설명 |
|------|------|------|--------|------|
| `height` | Input | double | 2.0 | 이륙 목표 고도 (m) |
| `speed` | Input | double | 0.5 | 이륙 속도 (m/s) |
| `server_name` | Input | string | - | 액션 서버 이름 오버라이드 |
| `server_timeout` | Input | ms | - | 서버 타임아웃 |

**특이사항:** `on_success()`에서 `std::this_thread::sleep_for(500ms)`를 호출한다. 이륙 완료 후 자세 안정화를 위한 대기 시간이지만, BT 틱 루프를 블로킹한다.

```cpp
BT::NodeStatus on_success() {
    std::this_thread::sleep_for(std::chrono::milliseconds(500));  // ← 블로킹!
    return BT::NodeStatus::SUCCESS;
}
```

### 9.2 LandAction

```cpp
class LandAction : public BtActionNode<as2_msgs::action::Land>
```

**포트:**

| 포트 | 방향 | 타입 | 기본값 |
|------|------|------|--------|
| `speed` | Input | double | 0.5 |

### 9.3 GoToAction

```cpp
class GoToAction : public BtActionNode<as2_msgs::action::GoToWaypoint>
```

**포트:**

| 포트 | 방향 | 타입 | 기본값 | 설명 |
|------|------|------|--------|------|
| `pose` | Input | `PointStamped` | - | 목표 위치 "x;y;z" |
| `max_speed` | Input | double | - | 최대 속도 (m/s) |
| `yaw_angle` | Input | double | 0.0 | 목표 요 각도 (rad) |
| `yaw_mode` | Input | int | 0 | 0=KEEP_YAW, 1=PATH_FACING, 2=FIXED_YAW |

**on_tick() 핵심 로직:**
```cpp
void on_tick() {
    getInput("pose", pose_);         // "x;y;z" → PointStamped (frame_id="earth")
    getInput("max_speed", max_speed_);
    getInput("yaw_angle", yaw_angle_);
    getInput("yaw_mode", yaw_mode_);

    goal_.target_pose.header = pose_.header;
    goal_.target_pose.point  = pose_.point;
    goal_.max_speed          = max_speed_;
    goal_.yaw.angle          = yaw_angle_;
    goal_.yaw.mode           = yaw_mode_;
}
```

### 9.4 FollowPathAction

```cpp
class FollowPathAction : public BtActionNode<as2_msgs::action::FollowPath>
```

**포트:**

| 포트 | 방향 | 타입 | 설명 |
|------|------|------|------|
| `path` | Input | `vector<PoseWithID>` | 웨이포인트 목록 "x1;y1;z1\|x2;y2;z2" |
| `speed` | Input | double | 최대 속도 |
| `yaw_mode` | Output | int | 요 모드 (현재 KEEP_YAW 고정) |

**주의:** `yaw_mode`가 Output 포트로 선언되어 있지만 실제로는 KEEP_YAW로 하드코딩되어 있다.

### 9.5 ArmService / DisarmService

```cpp
class ArmService    : public BtServiceNode<std_srvs::srv::SetBool>
class DisarmService : public BtServiceNode<std_srvs::srv::SetBool>
```

**동작:**
- `ArmService::on_tick()`: `request_->data = true`
- `DisarmService::on_tick()`: `request_->data = false`
- `on_completion()`: `response->success` 가 true면 SUCCESS, false면 FAILURE

**포트:** 없음 (XML에서 `service_name`만 설정)

### 9.6 OffboardService

```cpp
class OffboardService : public BtServiceNode<std_srvs::srv::SetBool>
```

`on_tick()`에서 `request_->data = true` 고정. DisarmService와 구조 동일.

### 9.7 Echo (동기 액션)

```cpp
class Echo : public BT::SyncActionNode
```

BtServiceNode나 BtActionNode를 상속하지 않는 **순수 동기 노드**다.

```cpp
BT::NodeStatus tick() override {
    std::string data;
    getInput("data", data);
    RCLCPP_INFO(node_->get_logger(), "[Echo] %s", data.c_str());
    return BT::NodeStatus::SUCCESS;
}
```

### 9.8 SendEvent (동기 액션)

```cpp
class SendEvent : public BT::SyncActionNode
```

생성자에서 퍼블리셔를 생성하고 tick마다 메시지를 발행한다.

```cpp
// 생성자: topic_name 포트를 읽어 publisher 생성
std::string topic_name;
getInput("topic_name", topic_name);
pub_ = node_->create_publisher<std_msgs::msg::String>(topic_name, 10);

// tick(): data 포트의 내용을 발행
BT::NodeStatus tick() override {
    std::string data;
    getInput("data", data);
    std_msgs::msg::String msg;
    msg.data = data;
    pub_->publish(msg);
    return BT::NodeStatus::SUCCESS;
}
```

### 9.9 SetOrigin / GetOrigin

**SetOrigin:**
```cpp
class SetOrigin : public BtServiceNode<as2_msgs::srv::SetOrigin>
```
- `on_tick()`: latitude, longitude, altitude 포트 읽어 request 구성
- `on_completion()`: response.success 기반 SUCCESS/FAILURE

**GetOrigin:**
```cpp
class GetOrigin : public BtServiceNode<as2_msgs::srv::GetOrigin>
```
- `on_tick()`: 빈 request (필드 없음)
- `on_completion()`: response에서 lat/lon/alt 읽어 Output 포트에 쓰기

### 9.10 GpsToCartesian

```cpp
class GpsToCartesian : public BtServiceNode<as2_msgs::srv::GeopathToPath>
```

GPS 좌표를 로컬 직교좌표(Cartesian)로 변환하는 서비스 래퍼.

```cpp
// on_tick(): GeoPath 요청 구성
void on_tick() {
    getInput("latitude", lat_);
    getInput("longitude", lon_);
    getInput("z", z_);
    // GeoPoseStamped 메시지 구성 후 request 설정
}

// on_completion(): 응답에서 x, y 추출 + z는 그대로 유지
BT::NodeStatus on_completion(response) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = response->path.poses[0].pose.position.x;
    pose.position.y = response->path.poses[0].pose.position.y;
    pose.position.z = z_;   // ← 원래 z 값 사용
    setOutput("out_pose", pose);
    return SUCCESS;
}
```

### 9.11 GoToGpsAction

```cpp
class GoToGpsAction : public BtActionNode<as2_msgs::action::GoToWaypoint>
```

GPS 좌표로의 이동을 **하나의 BT 노드**에서 처리하는 복합 노드. 내부적으로 GPS→Cartesian 변환 서비스 클라이언트를 직접 보유한다.

```cpp
// 생성자: GeopathToPath 서비스 클라이언트 추가 생성
client = node_->create_client<as2_msgs::srv::GeopathToPath>(service_name_);

// on_tick():
void on_tick() {
    // 1. GPS 포트 읽기
    getInput("latitude", lat);  getInput("longitude", lon);  getInput("altitude", alt);

    // 2. GPS → Cartesian 변환 서비스 동기 호출
    client->wait_for_service(chrono::seconds(1));
    auto future = client->async_send_request(request);
    // spin_until_future_complete(future, 1s)

    // 3. 변환된 좌표로 goal_ 구성
    goal_.target_pose.point.x = response->path.poses[0].pose.position.x;
    goal_.target_pose.point.y = response->path.poses[0].pose.position.y;
    goal_.target_pose.point.z = alt;
}
```

---

## 10. Condition 노드 상세 분석

### IsFlyingCondition

```cpp
class IsFlyingCondition : public BT::ConditionNode
```

**동작 원리:**

```cpp
// 생성자: platform/info 토픽 구독 (독립 CallbackGroup)
state_sub_ = node_->create_subscription<as2_msgs::msg::PlatformInfo>(
    as2_names::topics::platform::info,
    as2_names::topics::platform::qos,
    std::bind(&IsFlyingCondition::stateCallback, this, _1),
    sub_options  // callback_group_ 지정
);

// stateCallback: 상태 업데이트
void stateCallback(PlatformInfo::SharedPtr msg) {
    is_flying_ = (msg->status.state == PlatformStatus::FLYING);
}

// tick(): spin_some() 후 플래그 확인
BT::NodeStatus tick() override {
    callback_group_executor_.spin_some();
    return is_flying_ ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}
```

**포트:** 없음 (토픽 이름이 `as2_names::topics::platform::info`로 고정)

**한계:** 토픽 이름이 하드코딩되어 있어 멀티 드론 환경에서는 네임스페이스 설정에 의존한다.

---

## 11. Decorator 노드 상세 분석

### 11.1 WaitForEvent

```cpp
class WaitForEvent : public BT::DecoratorNode
```

지정한 토픽에서 `std_msgs/String` 메시지가 수신될 때까지 `RUNNING`을 반환하다가, 수신되면 자식 노드를 한 번 실행한다.

```cpp
// 포트
static BT::PortsList providedPorts() {
    return {
        BT::InputPort<std::string>("topic_name"),  // 대기할 토픽 이름
        BT::OutputPort("result")                   // 수신된 메시지 내용
    };
}

// tick() 핵심 로직
BT::NodeStatus tick() override {
    callback_group_executor_.spin_some();
    if (!flag_) {
        return BT::NodeStatus::RUNNING;  // 이벤트 미수신 → 대기
    }
    flag_ = false;  // 플래그 리셋 (다음 이벤트 대기 가능)
    return child()->executeTick();  // 자식 노드 실행
}

void callback(std_msgs::msg::String::SharedPtr msg) {
    setOutput("result", msg->data);  // 메시지를 블랙보드에 저장
    flag_ = true;
}
```

### 11.2 WaitForAlert

```cpp
class WaitForAlert : public BT::DecoratorNode
```

AS2의 `AlertEvent` 메시지를 대기한다. WaitForEvent와 동일한 패턴이지만 `as2_msgs::msg::AlertEvent` 타입을 사용한다.

```cpp
void callback(as2_msgs::msg::AlertEvent::SharedPtr msg) {
    std::string alert_str = std::to_string(msg->alert);  // alert 코드를 문자열로 변환
    setOutput("alert", alert_str);
    flag_ = true;
}
```

**WaitForEvent vs WaitForAlert 비교:**

| 항목 | WaitForEvent | WaitForAlert |
|------|-------------|-------------|
| 메시지 타입 | `std_msgs/String` | `as2_msgs/AlertEvent` |
| 용도 | 일반 이벤트 (임무 시작 등) | 비상/경고 이벤트 |
| 출력 포트 | `result` (원본 문자열) | `alert` (alert 코드 → 문자열) |
| 토픽 이름 | 포트로 지정 | 포트로 지정 |

---

## 12. 메인 실행기: as2_behavior_tree_node

**파일:** `src/as2_behavior_tree_node.cpp`

### 전체 실행 흐름

```
main()
│
├── rclcpp::init(argc, argv)
├── auto node = make_shared<rclcpp::Node>("bt_manager")
│
├── ROS2 파라미터 선언 및 읽기
│   ├── "tree"                   (필수: XML 파일 경로)
│   ├── "use_groot"              (기본: false)
│   ├── "groot_client_port"      (기본: 1666)
│   ├── "groot_server_port"      (기본: 1667)
│   ├── "server_timeout"         (기본: 10000 ms)
│   ├── "bt_loop_duration"       (기본: 10 ms → 100Hz)
│   └── "wait_for_service_timeout" (기본: 5000 ms)
│
├── BT::BehaviorTreeFactory 생성
│   └── registerNodeType() 16번 호출
│
├── 공유 블랙보드 생성 및 초기화
│   ├── "node"              → node (공유 ROS2 노드)
│   ├── "server_timeout"    → chrono::milliseconds(10000)
│   ├── "bt_loop_duration"  → chrono::milliseconds(10)
│   └── "wait_for_service_timeout" → chrono::milliseconds(5000)
│
├── factory.createTreeFromFile(tree_description, blackboard)
│
├── 로거 설정
│   ├── BT::StdCoutLogger(tree)          (항상 활성)
│   └── BT::Groot2Publisher(tree, port)  (use_groot=true 일 때)
│
└── 메인 루프
    ├── rclcpp::WallRate loopRate(bt_loop_duration_)
    └── while (rclcpp::ok() && result == RUNNING):
        ├── result = tree.tickWhileRunning()
        ├── ticks++
        └── loopRate.sleep()
```

### ROS2 파라미터 정리

| 파라미터 | 타입 | 기본값 | 설명 |
|---------|------|--------|------|
| `tree` | string | `""` | XML 트리 파일 절대 경로 (**필수**) |
| `use_groot` | bool | `false` | Groot2 시각화 활성화 |
| `groot_client_port` | int | 1666 | Groot2 클라이언트 포트 |
| `groot_server_port` | int | 1667 | Groot2 서버 포트 (미사용) |
| `server_timeout` | int (ms) | 10000 | ROS2 서버 응답 대기 최대 시간 |
| `bt_loop_duration` | int (ms) | 10 | BT 틱 주기 (= 100Hz) |
| `wait_for_service_timeout` | int (ms) | 5000 | 서비스 가용성 대기 시간 |

---

## 13. 블랙보드 설계

BehaviorTree.CPP의 블랙보드는 노드 간 데이터를 공유하는 키-값 저장소다.

### 시스템 예약 키

| 키 | 타입 | 설정 위치 | 용도 |
|----|------|----------|------|
| `"node"` | `rclcpp::Node::SharedPtr` | 메인 실행기 | 모든 BT 노드가 공유하는 ROS2 노드 |
| `"server_timeout"` | `chrono::milliseconds` | 메인 실행기 | Action/Service 서버 타임아웃 |
| `"bt_loop_duration"` | `chrono::milliseconds` | 메인 실행기 | tick 주기 |
| `"wait_for_service_timeout"` | `chrono::milliseconds` | 메인 실행기 | 서비스 가용 대기 시간 |
| `"number_recoveries"` | `int` | BT 노드 (선택) | 복구 시도 횟수 추적 |

### 사용자 정의 변수 (XML에서 `{}` 문법)

```xml
<!-- 쓰기: GpsToCartesian이 out_pose 포트로 블랙보드에 저장 -->
<Action ID="GpsToCartesian" latitude="52.17" longitude="4.41" out_pose="{var}" z="2.0"/>

<!-- 읽기: GoTo가 pose 포트로 블랙보드에서 읽기 -->
<Action ID="GoTo" pose="{var}" max_speed="1.0"/>
```

### SubTree 블랙보드 격리

```xml
<!-- __shared_blackboard="true": 부모 블랙보드 공유 -->
<SubTree ID="ArmTakeoff" __shared_blackboard="true" tk_height="2" tk_speed="0.5"/>

<!-- __shared_blackboard="false" (기본): 격리된 블랙보드, 포트 리매핑 필요 -->
<SubTree ID="RTL" __shared_blackboard="false"/>
```

---

## 14. XML 트리 정의 예제 분석

### 14.1 basic_mission.xml — 가장 단순한 임무

```xml
<BehaviorTree ID="BehaviorTree">
    <SequenceStar>
        <Action ID="TakeOff" height="2" speed="0.5"/>
        <Action ID="GoTo" max_speed="2" pose="5;5;2" yaw_angle="0.0" yaw_mode="0"/>
        <Action ID="Land" speed="0.5"/>
    </SequenceStar>
</BehaviorTree>
```

**`SequenceStar`:** 자식이 SUCCESS를 반환하면 다음으로 진행. 한 번 SUCCESS한 자식은 재실행하지 않음(메모리 있음). 자식이 FAILURE면 전체 FAILURE.

**실행 흐름:**
```
TakeOff(2m, 0.5m/s) → RUNNING(n번) → SUCCESS
    └→ GoTo(5,5,2, 2m/s) → RUNNING(m번) → SUCCESS
        └→ Land(0.5m/s) → RUNNING(k번) → SUCCESS → 임무 완료
```

### 14.2 takeoff.xml — 이벤트 기반 이륙

```xml
<BehaviorTree ID="BehaviorTree">
    <Decorator ID="WaitForEvent" topic_name="mission/start">
        <Fallback>
            <Condition ID="IsFlying"/>
            <SubTree ID="ArmTakeoff" __shared_blackboard="true" tk_height="2" tk_speed="0.5"/>
        </Fallback>
    </Decorator>
</BehaviorTree>

<BehaviorTree ID="ArmTakeoff">
    <SequenceStar>
        <Action ID="Arm" service_name="set_arming_state"/>
        <Action ID="Offboard" service_name="set_offboard_mode"/>
        <Action ID="TakeOff" height="{tk_height}" speed="{tk_speed}"/>
    </SequenceStar>
</BehaviorTree>
```

**실행 흐름:**
```
WaitForEvent("mission/start") → RUNNING (메시지 대기)
    ↓ (String 메시지 수신)
Fallback:
    IsFlying? → SUCCESS (이미 비행 중) → 완료
    IsFlying? → FAILURE (지상)
        ArmTakeoff:
            Arm → SUCCESS
            Offboard → SUCCESS
            TakeOff(2m, 0.5m/s) → ... → SUCCESS
        → 완료
```

### 14.3 RTL_test_2.xml — GPS 복합 임무

```xml
<Sequence>
    <Action ID="SetOrigin" latitude="52.171776" longitude="4.416351" altitude="0.0"/>
    <Action ID="Arm"/>
    <Action ID="Offboard"/>
    <Action ID="TakeOff" height="2"/>
    <Parallel failure_threshold="1" success_threshold="1">
        <!-- 비상 알림을 기다리는 동시에 GPS 경유지로 비행 -->
        <Decorator ID="WaitForAlert" topic_name="/drone_sim/alert">
            <AlwaysSuccess/>
        </Decorator>
        <Sequence>
            <Action ID="GpsToCartesian" latitude="52.172046" longitude="4.416790" out_pose="{var}" z="2.0"/>
            <Action ID="GoTo" pose="{var}" max_speed="1.0"/>
        </Sequence>
    </Parallel>
    <!-- RTL: 출발지로 복귀 후 착륙 -->
    <Action ID="GetOrigin" latitude="{lat}" longitude="{lon}" altitude="{alt}"/>
    <Action ID="GpsToCartesian" latitude="{lat}" longitude="{lon}" out_pose="{pose}" z="2.0"/>
    <Action ID="GoTo" pose="{pose}" max_speed="1.0"/>
    <Action ID="Land"/>
</Sequence>
```

**`Parallel` 노드 설정:**
- `failure_threshold="1"`: 자식 중 1개라도 FAILURE면 전체 FAILURE
- `success_threshold="1"`: 자식 중 1개라도 SUCCESS면 전체 SUCCESS

→ WaitForAlert(알림 수신 시 SUCCESS)와 GoTo(도착 시 SUCCESS) 중 **먼저 완료되는 쪽**이 Parallel을 종료.

---

## 15. 내부 동작 메커니즘

### 15.1 CallbackGroup 패턴 (BT-ROS2 통합 핵심)

모든 BT 노드(Action, Service, Condition, Decorator)는 동일한 패턴을 사용한다:

```cpp
// 1. 독립 CallbackGroup 생성 (false = 주 executor에 자동 추가 안 함)
callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);

// 2. 독립 SingleThreadedExecutor에만 등록
callback_group_executor_.add_callback_group(
    callback_group_, node_->get_node_base_interface());

// 3. tick() 내에서 수동으로 spin
callback_group_executor_.spin_some();           // 비블로킹
callback_group_executor_.spin_until_future_complete(future, timeout);  // 타임아웃 있는 대기
```

**왜 이 패턴이 필요한가:**
BT의 `tick()`은 BT 엔진이 호출하는 동기 함수다. ROS2 콜백(Action 결과, 서비스 응답, 토픽 메시지)을 받으려면 executor를 spin해야 한다. 주 executor는 BT 루프 바깥에서 동작하므로, 각 BT 노드가 자신만의 독립 executor로 필요할 때만 spin한다.

### 15.2 Goal 재전송 메커니즘 (BtActionNode)

`goal_updated_` 플래그를 true로 설정하면 현재 실행 중인 Action을 취소하지 않고 새 Goal을 재전송할 수 있다:

```cpp
// on_wait_for_result()에서 동적으로 목표를 업데이트하는 경우:
void on_wait_for_result(feedback) {
    if (조건) {
        goal_.target_pose = 새_위치;
        goal_updated_ = true;  // tick()에서 send_new_goal() 재호출
    }
}
```

이를 통해 비행 중 목표 위치를 실시간으로 변경하는 동적 타겟 추종이 가능하다.

### 15.3 `tickWhileRunning()` vs `tickOnce()`

메인 루프에서 사용하는 `tickWhileRunning()`은 내부적으로 RUNNING 상태인 동안 계속 tick을 반복한다. 이로 인해 BT 노드의 `tick()`이 `bt_loop_duration`마다 호출된다.

```
BT 틱 타이밍:
[10ms] tickWhileRunning()
    ├── TakeoffAction.tick() → spin_some() → 아직 완료 안 됨 → RUNNING
    └── sleep(loopRate)
[20ms] tickWhileRunning()
    ├── TakeoffAction.tick() → spin_some() → 완료 → SUCCESS
    └── LandAction.tick() → ...
```

---

## 16. 설계 패턴 분석

### 16.1 Template Method Pattern (BtActionNode/BtServiceNode)

```
BtActionNode::tick()          ← 알고리즘 골격 정의
    ├── on_tick()             ← 훅: 서브클래스가 goal_ 채움
    ├── on_wait_for_result()  ← 훅: 피드백 처리
    ├── on_success()          ← 훅: 성공 시 처리
    ├── on_aborted()          ← 훅: 중단 시 처리
    └── on_cancelled()        ← 훅: 취소 시 처리
```

### 16.2 Adapter Pattern

ROS2의 비동기 Action/Service 인터페이스를 BehaviorTree.CPP의 동기 tick() 인터페이스로 변환:

```
[BT tick() - 동기]
    ↓ 변환
[ROS2 Action/Service - 비동기 + spin]
```

### 16.3 Bridge Pattern (port_specialization.hpp)

BT의 문자열 기반 포트 시스템과 ROS2 메시지 타입 사이의 변환 브리지:

```
XML 문자열 "5;3;2"
    ↓ BT::convertFromString<geometry_msgs::msg::Pose>()
geometry_msgs::msg::Pose{x=5, y=3, z=2}
```

### 16.4 Blackboard Pattern

공유 메모리를 통한 BT 노드 간 데이터 교환. 특히 ROS2 노드 공유에 핵심적으로 사용:

```
모든 BT 노드
    └── config().blackboard->get<rclcpp::Node::SharedPtr>("node")
            → 동일한 ROS2 노드 인스턴스 공유
```

### 16.5 Decorator Pattern (GoToGpsAction)

GoToGpsAction은 GoToAction에 GPS→Cartesian 변환 기능을 추가한 데코레이터 패턴의 실용적 적용:

```
GoToGpsAction = GPS 좌표 변환 + GoToAction의 기능
```

---

## 17. 주의사항 및 Known Issues

### 17.1 TakeoffAction의 블로킹 sleep

```cpp
BT::NodeStatus on_success() {
    std::this_thread::sleep_for(std::chrono::milliseconds(500));  // ← BT 루프 블로킹
    return BT::NodeStatus::SUCCESS;
}
```

이 500ms sleep은 BT 틱 루프 전체를 블로킹한다. Parallel 노드와 함께 사용 시 다른 브랜치도 500ms 지연된다.

### 17.2 FollowPathAction의 경로 파싱 한계

`std::vector<as2_msgs::msg::PoseWithID>`의 port specialization은 정확히 2개 포인트만 지원한다:

```cpp
// 코드에 TODO 명시
// TODO(pariaspe): generalize
auto points = splitString(str, '|');
// points[0]와 points[1]만 처리 (3개 이상 → 무시)
```

### 17.3 GoToGpsAction의 동기 서비스 호출

`on_tick()` 내에서 GPS→Cartesian 서비스를 동기적으로 호출한다. 서비스 응답 지연이 BT 루프를 블로킹한다.

### 17.4 BtServiceNode의 무한 wait_for_service()

```cpp
// 생성자에서 서비스가 나타날 때까지 무한 대기
service_client_->wait_for_service();  // 타임아웃 없음
```

해당 서비스가 절대 나타나지 않으면 노드 초기화가 영구적으로 블로킹된다.

### 17.5 IsFlyingCondition의 토픽 이름 하드코딩

`platform/info` 토픽이 `as2_names::topics::platform::info`로 하드코딩되어 있어, 멀티 드론 환경에서는 ROS2 네임스페이스로만 구분해야 한다. 포트로 설정 가능하면 더 유연했을 것이다.

### 17.6 Groot2Publisher 포트 불일치

```cpp
// 코드: client_port만 사용
groot_pub = make_shared<BT::Groot2Publisher>(tree, groot_client_port);
// server_port 파라미터는 선언되지만 실제로 사용되지 않음
```

### 17.7 action_client_ 생성 실패 시 예외

```cpp
if (!action_client_->wait_for_action_server(1s)) {
    throw std::runtime_error(
        std::string("Action server %s not available", action_name.c_str()));
    // std::string 생성자에 format 인수를 잘못 사용 - 실제로는 "Action server %s not available" 그대로 출력
}
```

format 문자열이 실제로 `%s`를 치환하지 않는 버그가 있다 (`std::string(const char*, ...)` 생성자는 format 함수가 아님).

---

## 18. 새 BT 노드 구현 가이드

### ROS2 Action 기반 노드 (BtActionNode 상속)

```cpp
// 1. 헤더 파일
#include "as2_behavior_tree/bt_action_node.hpp"
#include "as2_msgs/action/my_action.hpp"

class MyAction : public as2_behavior_tree::BtActionNode<as2_msgs::action::MyAction>
{
public:
    MyAction(const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
    : BtActionNode<as2_msgs::action::MyAction>(
        xml_tag_name,
        as2_names::actions::behaviors::my_action,  // 액션 서버 이름
        conf) {}

    // 필수: goal_ 채우기
    void on_tick() override {
        double param;
        getInput("param", param);
        goal_.param = param;
    }

    // 선택: 성공 후 처리
    BT::NodeStatus on_success() override {
        setOutput("result", result_.result->value);
        return BT::NodeStatus::SUCCESS;
    }

    // 포트 정의
    static BT::PortsList providedPorts() {
        return providedBasicPorts({
            BT::InputPort<double>("param"),
            BT::OutputPort<double>("result")
        });
    }
};
```

### ROS2 Service 기반 노드 (BtServiceNode 상속)

```cpp
class MyService : public as2_behavior_tree::BtServiceNode<my_pkg::srv::MyService>
{
public:
    MyService(const std::string & name, const BT::NodeConfiguration & conf)
    : BtServiceNode<my_pkg::srv::MyService>(name, conf) {}

    // 필수: request_ 채우기
    void on_tick() override {
        getInput("input", request_->field);
    }

    // 필수: response 처리
    BT::NodeStatus on_completion(
        std::shared_ptr<my_pkg::srv::MyService::Response> response) override
    {
        if (response->success) {
            setOutput("output", response->data);
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }

    static BT::PortsList providedPorts() {
        return providedBasicPorts({
            BT::InputPort<std::string>("service_name", "my_service"),
            BT::InputPort<std::string>("input"),
            BT::OutputPort<std::string>("output")
        });
    }
};
```

### 노드 등록 (`as2_behavior_tree_node.cpp` 수정)

```cpp
factory.registerNodeType<MyAction>("MyAction");
factory.registerNodeType<MyService>("MyService");
```

### XML 사용

```xml
<Action ID="MyAction" param="1.5" result="{my_result}" server_name="MyActionBehavior"/>
<Action ID="MyService" service_name="my_srv" input="hello" output="{srv_result}"/>
```

---

*작성: 2026-03-26 | 기반: as2_behavior_tree v1.1.3 실제 소스 코드 직접 분석*
