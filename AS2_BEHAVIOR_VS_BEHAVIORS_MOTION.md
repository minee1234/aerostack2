# as2_behavior vs as2_behaviors_motion 차이점 분석

> 분석 기준 브랜치: `main`
> 분석 일자: 2026-04-02

---

## 목차

1. [한 줄 요약](#1-한-줄-요약)
2. [계층 관계](#2-계층-관계)
3. [as2_behavior — 무엇을 제공하는가](#3-as2_behavior--무엇을-제공하는가)
4. [as2_behaviors_motion — 무엇을 제공하는가](#4-as2_behaviors_motion--무엇을-제공하는가)
5. [구조 비교 한눈에 보기](#5-구조-비교-한눈에-보기)
6. [새로운 Behavior를 만들 때](#6-새로운-behavior를-만들-때)

---

## 1. 한 줄 요약

| 패키지 | 역할 |
|---|---|
| `as2_behavior` | **프레임워크** — Behavior를 만들기 위한 베이스 클래스와 인터페이스 제공 |
| `as2_behaviors_motion` | **구현체** — `as2_behavior`를 상속해서 실제 동작하는 Behavior 노드들 구현 |

---

## 2. 계층 관계

```
as2_behavior                          ← 틀 (framework)
    └─ BehaviorServer<ActionT>        ← 템플릿 베이스 클래스
    └─ BasicBehavior<MessageT>        ← 구형 베이스 클래스 (legacy)
    └─ ExecutionStatus                ← 공통 열거형

         ↑ #include "as2_behavior/behavior_server.hpp"
         │ 상속

as2_behaviors_motion                  ← 살 (implementation)
    ├─ TakeoffBehavior      : BehaviorServer<as2_msgs::action::Takeoff>
    ├─ LandBehavior         : BehaviorServer<as2_msgs::action::Land>
    ├─ GoToBehavior         : BehaviorServer<as2_msgs::action::GoToWaypoint>
    ├─ FollowPathBehavior   : BehaviorServer<as2_msgs::action::FollowPath>
    └─ FollowReferenceBehavior : BehaviorServer<as2_msgs::action::FollowReference>
```

---

## 3. as2_behavior — 무엇을 제공하는가

**빌드 결과물: 공유 라이브러리 (`libas2_behavior.so`)**

실행 가능한 노드가 없습니다. 다른 패키지가 `#include`해서 쓰는 헤더 + 라이브러리입니다.

### 파일 구조

```
as2_behavior/include/as2_behavior/
├── behavior_server.hpp              ← 진입 헤더 (이것만 include하면 됨)
├── behavior_utils.hpp               ← ExecutionStatus 열거형 정의
├── as2_basic_behavior.hpp           ← 구형 베이스 (std::thread 방식)
├── __detail/
│   └── behavior_server__class.hpp  ← BehaviorServer 클래스 선언
└── __impl/
    └── behavior_server__impl.hpp   ← BehaviorServer 메서드 구현 (템플릿)
```

### 핵심 제공 요소

**`behavior_utils.hpp` — 상태 열거형**

```cpp
namespace as2_behavior {
    enum class ExecutionStatus { SUCCESS, RUNNING, FAILURE, ABORTED };
}
```

**`behavior_server__class.hpp` — BehaviorServer 클래스**

```cpp
template<typename actionT>
class BehaviorServer : public as2::Node {
    // ROS2 Action 서버 자동 생성
    // 타이머 기반 run() 루프 (10Hz 기본)
    // pause / resume / stop 서비스 자동 등록
    // behavior_status 토픽 자동 발행 (10Hz)

    // 파생 클래스가 오버라이드하는 메서드들
    virtual bool on_activate(goal)    { return true; }
    virtual ExecutionStatus on_run(goal, feedback, result);
    virtual void on_execution_end(state) {}
    virtual bool on_modify(goal)      { return true; }
    virtual bool on_deactivate(msg)   { return true; }
    virtual bool on_pause(msg)        { return true; }
    virtual bool on_resume(msg)       { return true; }
};
```

**`as2_basic_behavior.hpp` — BasicBehavior (구형)**

```cpp
template<class MessageT>
class BasicBehavior : public as2::Node {
    // std::thread로 onExecute() 실행 (블로킹 방식)
    virtual void onExecute(goal_handle) = 0;
};
```

### BehaviorServer vs BasicBehavior 차이

| | `BehaviorServer` | `BasicBehavior` |
|---|---|---|
| 실행 방식 | **타이머** (10Hz 주기 반복) | **std::thread** (블로킹) |
| 상태 관리 | `ExecutionStatus` 열거형 반환 | 직접 구현 |
| 부가 서비스 | pause/resume/stop 자동 등록 | 없음 |
| 피드백 발행 | `publish_feedback()` 자동 호출 | 직접 구현 |
| 현재 사용 | 모든 신규 Behavior | legacy |

### 패키지 의존성 (순수 인터페이스)

```xml
<!-- as2_behavior/package.xml -->
<depend>rclcpp</depend>
<depend>as2_core</depend>
<depend>as2_msgs</depend>
<depend>std_srvs</depend>
<!-- pluginlib, rclcpp_action 없음 → 구현 세부사항에 비의존 -->
```

---

## 4. as2_behaviors_motion — 무엇을 제공하는가

**빌드 결과물: 실행 노드 + 컴포넌트 라이브러리 + 플러그인 라이브러리**

### 빌드 산출물

```
빌드 산출물:
├── takeoff_behavior_node             ← 실행 가능한 ROS2 노드
├── land_behavior_node
├── go_to_behavior_node
├── follow_path_behavior_node
├── follow_reference_behavior_node
│
├── takeoff_component (shared lib)   ← rclcpp_components 컨테이너용
├── land_component
├── go_to_component
├── follow_path_component
├── follow_reference_component
│
└── libas2_behaviors_motion.so       ← pluginlib 플러그인 라이브러리
    ├─ takeoff_plugin_position
    ├─ takeoff_plugin_speed
    ├─ takeoff_plugin_trajectory
    ├─ takeoff_plugin_platform
    ├─ land_plugin_speed
    ├─ land_plugin_trajectory
    ├─ land_plugin_platform
    ├─ go_to_plugin_position
    ├─ go_to_plugin_trajectory
    ├─ follow_path_plugin_position
    └─ follow_path_plugin_trajectory
```

### 파일 구조

```
as2_behaviors_motion/
├── takeoff_behavior/
│   ├── include/takeoff_behavior/
│   │   ├── takeoff_behavior.hpp     ← BehaviorServer 상속 클래스
│   │   └── takeoff_base.hpp         ← 플러그인 추상 인터페이스
│   ├── src/
│   │   ├── takeoff_behavior.cpp
│   │   └── takeoff_behavior_node.cpp ← main()
│   ├── plugins/
│   │   ├── takeoff_plugin_position.cpp
│   │   ├── takeoff_plugin_speed.cpp
│   │   ├── takeoff_plugin_trajectory.cpp
│   │   └── takeoff_plugin_platform.cpp
│   └── config/config_default.yaml
├── land_behavior/     (동일 구조)
├── go_to_behavior/    (동일 구조)
├── follow_path_behavior/    (동일 구조)
├── follow_reference_behavior/ (동일 구조)
└── launch/
    ├── motion_behaviors_launch.py   ← 5개 노드 한번에 실행
    ├── takeoff_behavior_launch.py
    └── ...
```

### 패키지 의존성 (구현체)

```xml
<!-- as2_behaviors_motion/package.xml -->
<depend>as2_behavior</depend>                    ← 베이스 클래스 사용
<depend>pluginlib</depend>                        ← 플러그인 동적 로딩
<depend>rclcpp_action</depend>                    ← Action 서버/클라이언트
<depend>as2_motion_reference_handlers</depend>    ← 드론 제어 명령 전송
<depend>rclcpp_components</depend>                ← 컴포넌트 컨테이너 지원
```

### TakeoffBehavior 구현 예시

```cpp
// takeoff_behavior.hpp
class TakeoffBehavior
  : public as2_behavior::BehaviorServer<as2_msgs::action::Takeoff>
{
    // pluginlib으로 동적 로딩
    std::shared_ptr<pluginlib::ClassLoader<TakeoffBase>> loader_;
    std::shared_ptr<TakeoffBase> takeoff_plugin_;

    bool on_activate(goal) override {
        return takeoff_plugin_->on_activate(goal);
    }
    ExecutionStatus on_run(goal, feedback, result) override {
        return takeoff_plugin_->on_run(goal, feedback, result);
    }
    void on_execution_end(state) override {
        sendEventFSME(PSME::TOOK_OFF);   // Platform FSM 상태 전환
        takeoff_plugin_->on_execution_end(state);
    }
};
```

---

## 5. 구조 비교 한눈에 보기

```
as2_behavior (라이브러리)
─────────────────────────────────────────────────
소스 파일     헤더 전용 (템플릿 클래스)
실행 노드     없음
역할          "Behavior를 어떻게 만드는가" 정의
의존성        rclcpp, as2_core, as2_msgs만
외부 노출     BehaviorServer<T>, BasicBehavior<T>, ExecutionStatus
빌드 결과     libas2_behavior.so (헤더+라이브러리)
─────────────────────────────────────────────────

           ↑ 상속 (as2_behaviors_motion이 사용)

as2_behaviors_motion (실행 패키지)
─────────────────────────────────────────────────
소스 파일     takeoff_behavior.cpp, land_behavior.cpp ...
실행 노드     *_behavior_node (5개)
역할          "드론이 실제로 어떻게 움직이는가" 구현
의존성        as2_behavior + pluginlib + motion_handlers
외부 노출     실행 노드, 플러그인(position/speed/trajectory)
빌드 결과     *_behavior_node + *_component + libas2_behaviors_motion.so
─────────────────────────────────────────────────
```

### ROS2 통신 관점 비교

| 관점 | `as2_behavior` | `as2_behaviors_motion` |
|---|---|---|
| Action 서버 이름 | 파생 클래스 생성자에서 결정 | `as2_names::actions::behaviors::*` |
| 서비스 | pause/resume/stop 자동 등록 | 추가 없음 (베이스에서 상속) |
| 토픽 발행 | `behavior_status` (10Hz) | 없음 (베이스에서 상속) |
| 토픽 구독 | 없음 | twist, platform_info (상태 모니터링) |

---

## 6. 새로운 Behavior를 만들 때

`as2_behavior`는 **건드리지 않고**, `as2_behaviors_motion`처럼
`BehaviorServer`를 상속한 새 패키지를 만들면 됩니다.

```cpp
// 새 패키지: as2_behaviors_my_custom
#include "as2_behavior/behavior_server.hpp"   // as2_behavior에서 가져옴

class MyCustomBehavior
  : public as2_behavior::BehaviorServer<as2_msgs::action::MyAction>
{
public:
    MyCustomBehavior(const rclcpp::NodeOptions& options)
    : BehaviorServer(as2_names::actions::behaviors::my_action, options)
    {
        // 플러그인 로딩 등 초기화
    }

    bool on_activate(
        std::shared_ptr<const as2_msgs::action::MyAction::Goal> goal) override
    {
        // Goal 수신 시 1회 호출
        return true;
    }

    as2_behavior::ExecutionStatus on_run(
        const std::shared_ptr<const Goal>& goal,
        std::shared_ptr<Feedback>& feedback,
        std::shared_ptr<Result>& result) override
    {
        // 10Hz 주기로 반복 호출
        // return SUCCESS / RUNNING / FAILURE / ABORTED
        return as2_behavior::ExecutionStatus::SUCCESS;
    }

    void on_execution_end(
        const as2_behavior::ExecutionStatus& state) override
    {
        // 종료 시 1회 호출 (정리 작업)
    }
};
```

### 의존성 설정 (package.xml)

```xml
<depend>as2_behavior</depend>       ← 반드시 필요
<depend>pluginlib</depend>           ← 플러그인 구조 사용 시
<depend>as2_motion_reference_handlers</depend>  ← 드론 제어 시
```
