# behavior_trees.launch.py → Behavior 실행 전체 흐름 분석

> 분석 기준 브랜치: `main`
> 분석 일자: 2026-04-01
> 예제 XML: `as2_behavior_tree/resource/takeoff.xml`

---

## 목차

1. [전체 구조 개요](#1-전체-구조-개요)
2. [STEP 1 — launch 파일 실행](#step-1--launch-파일-실행)
3. [STEP 2 — main() 진입 및 초기화](#step-2--main-진입-및-초기화)
4. [STEP 3 — 노드 타입 등록](#step-3--노드-타입-등록)
5. [STEP 4 — Blackboard 생성](#step-4--blackboard-생성)
6. [STEP 5 — XML 파싱 및 트리 객체 생성](#step-5--xml-파싱-및-트리-객체-생성)
7. [STEP 6 — BT 실행 루프 시작](#step-6--bt-실행-루프-시작)
8. [STEP 7 — tick() 전파: WaitForEvent](#step-7--tick-전파-waitforevent)
9. [STEP 8 — tick() 전파: IsFlyingCondition](#step-8--tick-전파-isflyingcondition)
10. [STEP 9 — tick() 전파: ArmService](#step-9--tick-전파-armservice)
11. [STEP 10 — tick() 전파: TakeoffAction](#step-10--tick-전파-takeoffaction)
12. [STEP 11 — ROS2 Action: BehaviorServer 수신](#step-11--ros2-action-behaviorserver-수신)
13. [STEP 12 — TakeoffBehavior::on_activate() + 플러그인 위임](#step-12--takeoffbehavioron_activate--플러그인-위임)
14. [STEP 13 — 10Hz 타이머 루프: run() 반복](#step-13--10hz-타이머-루프-run-반복)
15. [STEP 14 — BT로 결과 반환](#step-14--bt로-결과-반환)
16. [전체 흐름 한눈에 보기](#전체-흐름-한눈에-보기)
17. [핵심 파일 참조 표](#핵심-파일-참조-표)

---

## 1. 전체 구조 개요

```
┌──────────────────────────────────────────────────────────────┐
│  [프로세스 A] as2_behavior_tree_node                          │
│                                                              │
│  behavior_trees.launch.py                                    │
│       ↓ executable                                           │
│  as2_behavior_tree_node.cpp::main()                          │
│       ↓ createTreeFromFile(takeoff.xml)                      │
│  BT 노드 인스턴스들 (WaitForEvent, TakeoffAction, ...)        │
│       ↓ tick() 100Hz                                         │
│  TakeoffAction::tick()                                       │
│       ↓ async_send_goal()                                    │
└──────────────────────┬───────────────────────────────────────┘
                       │ ROS2 Action (/drone0/TakeoffBehavior)
                       │ Goal / Feedback / Result
┌──────────────────────┴───────────────────────────────────────┐
│  [프로세스 B] takeoff_behavior_node                           │
│                                                              │
│  TakeoffBehavior (BehaviorServer<Takeoff>)                   │
│       ↓ on_activate() → register_run_timer()                 │
│  10Hz 타이머 루프                                             │
│       ↓ on_run() → plugin->own_run()                         │
│  takeoff_plugin_position::Plugin                             │
│       ↓ sendPositionCommand()                                │
│  MotionController → Platform → 드론 물리                     │
└──────────────────────────────────────────────────────────────┘
```

---

## STEP 1 — launch 파일 실행

**파일:** `as2_behavior_tree/launch/behavior_trees.launch.py`

```python
# :58~72
Node(
    package='as2_behavior_tree',
    executable='as2_behavior_tree_node',
    namespace=LaunchConfiguration('drone_id'),   # 예: drone0
    parameters=[{
        'tree':             LaunchConfiguration('tree'),  # XML 파일 경로
        'use_groot':        LaunchConfiguration('groot_logger'),
        'groot_client_port': LaunchConfiguration('groot_client_port'),  # 1666
        'groot_server_port': LaunchConfiguration('groot_server_port'),  # 1667
        'server_timeout':    LaunchConfiguration('server_timeout'),     # 10000ms
        'bt_loop_duration':  LaunchConfiguration('bt_loop_duration'),   # 10ms
    }]
)
```

**실행 명령:**

```bash
ros2 launch as2_behavior_tree behavior_trees.launch.py \
  drone_id:=drone0 \
  tree:=$(ros2 pkg prefix as2_behavior_tree)/share/as2_behavior_tree/resource/takeoff.xml
```

---

## STEP 2 — main() 진입 및 초기화

**파일:** `as2_behavior_tree/src/as2_behavior_tree_node.cpp`

```cpp
// :66~84
rclcpp::init(argc, argv);
auto node = std::make_shared<rclcpp::Node>("bt_manager");
// → ROS2 노드: /drone0/bt_manager

node->declare_parameter<std::string>("tree", "");
node->declare_parameter<bool>("use_groot", false);
node->declare_parameter<int>("server_timeout", 10000);   // ms
node->declare_parameter<int>("bt_loop_duration", 10);    // ms = 100Hz

std::string tree_description = node->get_parameter("tree").as_string();
// → ".../resource/takeoff.xml"
int server_timeout   = node->get_parameter("server_timeout").as_int();
int bt_loop_duration = node->get_parameter("bt_loop_duration").as_int();
```

**파라미터 정리:**

| 파라미터 | 기본값 | 역할 |
|---|---|---|
| `tree` | `""` | 실행할 BT XML 파일 경로 (필수) |
| `use_groot` | `false` | Groot2 시각화 연결 여부 |
| `server_timeout` | `10000` ms | Action/Service 서버 응답 대기 최대 시간 |
| `bt_loop_duration` | `10` ms | BT tick 주기 (100Hz) |
| `wait_for_service_timeout` | `5000` ms | 서비스 연결 대기 시간 |

---

## STEP 3 — 노드 타입 등록

**파일:** `as2_behavior_tree/src/as2_behavior_tree_node.cpp:86~103`

```cpp
BT::BehaviorTreeFactory factory;

//  XML ID          →  C++ 클래스
factory.registerNodeType<ArmService>       ("Arm");
factory.registerNodeType<DisarmService>    ("Disarm");
factory.registerNodeType<OffboardService>  ("Offboard");
factory.registerNodeType<TakeoffAction>    ("TakeOff");
factory.registerNodeType<GoToAction>       ("GoTo");
factory.registerNodeType<LandAction>       ("Land");
factory.registerNodeType<FollowPathAction> ("FollowPath");
factory.registerNodeType<GoToGpsAction>    ("GoToGps");
factory.registerNodeType<IsFlyingCondition>("IsFlying");
factory.registerNodeType<WaitForEvent>     ("WaitForEvent");
factory.registerNodeType<WaitForAlert>     ("WaitForAlert");
factory.registerNodeType<SendEvent>        ("SendEvent");
factory.registerNodeType<Echo>             ("Echo");
factory.registerNodeType<SetOrigin>        ("SetOrigin");
factory.registerNodeType<GetOrigin>        ("GetOrigin");
factory.registerNodeType<GpsToCartesian>   ("GpsToCartesian");
```

> XML의 `<Action ID="TakeOff">` 문자열이 `TakeoffAction` 클래스와 연결됩니다.

---

## STEP 4 — Blackboard 생성

**파일:** `as2_behavior_tree/src/as2_behavior_tree_node.cpp:105~115`

```cpp
BT::NodeConfiguration* config = new BT::NodeConfiguration();
config->blackboard = BT::Blackboard::create();

// 모든 BT 노드가 공유하는 전역 저장소에 공통 데이터 주입
config->blackboard->set<rclcpp::Node::SharedPtr>("node", node);
config->blackboard->set<std::chrono::milliseconds>(
    "server_timeout",           std::chrono::milliseconds(10000));
config->blackboard->set<std::chrono::milliseconds>(
    "bt_loop_duration",         std::chrono::milliseconds(10));
config->blackboard->set<std::chrono::milliseconds>(
    "wait_for_service_timeout", std::chrono::milliseconds(5000));
```

**Blackboard 사용처:**

각 BT 노드 생성자에서 `"node"` 포인터를 꺼내 ROS2 통신에 사용합니다.

```cpp
// bt_action_node.hpp:52
node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
```

---

## STEP 5 — XML 파싱 및 트리 객체 생성

**파일:** `as2_behavior_tree/src/as2_behavior_tree_node.cpp:116`

```cpp
auto tree = factory.createTreeFromFile(tree_description, config->blackboard);
```

**takeoff.xml 구조와 생성되는 객체:**

```xml
<root main_tree_to_execute="BehaviorTree">

  <BehaviorTree ID="ArmTakeoff">          <!-- 서브트리 정의 -->
    <SequenceStar>
      <Action ID="Arm"     service_name="set_arming_state"/>
      <Action ID="Offboard" service_name="set_offboard_mode"/>
      <Action ID="TakeOff" height="{tk_height}" speed="{tk_speed}"/>
    </SequenceStar>
  </BehaviorTree>

  <BehaviorTree ID="BehaviorTree">        <!-- 진입 트리 -->
    <Decorator ID="WaitForEvent" topic_name="mission/start">
      <Fallback>
        <Condition ID="IsFlying"/>
        <SubTree ID="ArmTakeoff" __shared_blackboard="true"
                 tk_height="2" tk_speed="0.5"/>
      </Fallback>
    </Decorator>
  </BehaviorTree>

</root>
```

**createTreeFromFile() 시 각 노드 생성자에서 일어나는 일:**

| 생성되는 객체 | 생성자에서 하는 일 |
|---|---|
| `WaitForEvent` | `/drone0/mission/start` 토픽 구독 등록 |
| `IsFlyingCondition` | `/drone0/platform/info` 토픽 구독 등록 |
| `ArmService` | `/drone0/set_arming_state` 서비스 클라이언트 생성 + 서버 연결 대기 |
| `OffboardService` | `/drone0/set_offboard_mode` 서비스 클라이언트 생성 |
| `TakeoffAction` | `/drone0/TakeoffBehavior` action 클라이언트 생성 + 서버 연결 대기 (1s) |

**BtActionNode 생성자 핵심 코드:**

```cpp
// bt_action_node.hpp:47~79
BtActionNode(xml_tag_name, action_name, conf)
{
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    callback_group_ = node_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive, false);
    callback_group_executor_.add_callback_group(
        callback_group_, node_->get_node_base_interface());

    server_timeout_   = blackboard->get<milliseconds>("server_timeout");
    bt_loop_duration_ = blackboard->get<milliseconds>("bt_loop_duration");

    createActionClient(action_name_);
    // → rclcpp_action::create_client<Takeoff>(node_, "TakeoffBehavior")
    // → action_client_->wait_for_action_server(1s)
}
```

> `TakeoffBehavior` 노드가 먼저 실행되어 있어야 합니다.
> 서버가 없으면 `std::runtime_error` 발생.

---

## STEP 6 — BT 실행 루프 시작

**파일:** `as2_behavior_tree/src/as2_behavior_tree_node.cpp:129~139`

```cpp
BT::NodeStatus result = BT::NodeStatus::RUNNING;
rclcpp::WallRate loopRate(std::chrono::milliseconds(10)); // 100Hz

while (rclcpp::ok() && result == BT::NodeStatus::RUNNING) {
    result = tree.tickWhileRunning();  // 루트부터 재귀적으로 tick()
    ticks++;
    loopRate.sleep();                  // 10ms 대기
}

rclcpp::shutdown();
// result가 SUCCESS 또는 FAILURE가 되면 프로세스 종료
```

---

## STEP 7 — tick() 전파: WaitForEvent

**파일:** `as2_behavior_tree/plugins/decorator/wait_for_event.cpp`

```cpp
BT::NodeStatus WaitForEvent::tick()
{
    callback_group_executor_.spin_some();   // 구독 콜백 처리

    if (flag_) {                            // 이벤트 수신 후
        return child_node_->executeTick();  // 자식 노드(Fallback) 실행
    }
    return BT::NodeStatus::RUNNING;         // 수신 전: 매 tick마다 RUNNING 반환
}

void WaitForEvent::callback(std_msgs::msg::String::SharedPtr msg)
{
    setOutput("result", msg->data);         // Blackboard에 결과 저장
    flag_ = true;                           // 다음 tick부터 자식 실행 허용
}
```

**이벤트 발행 (외부에서):**

```bash
ros2 topic pub --once /drone0/mission/start std_msgs/msg/String '{data: "start"}'
```

---

## STEP 8 — tick() 전파: IsFlyingCondition

**파일:** `as2_behavior_tree/include/as2_behavior_tree/condition/is_flying_condition.hpp`

```cpp
BT::NodeStatus tick() override
{
    callback_group_executor_.spin_some();  // PlatformInfo 구독 처리
    return is_flying_ ? BT::NodeStatus::SUCCESS
                      : BT::NodeStatus::FAILURE;
}

void stateCallback(as2_msgs::msg::PlatformInfo::SharedPtr msg)
{
    is_flying_ = (msg->status.state == as2_msgs::msg::PlatformStatus::FLYING);
}
```

- 이륙 전이므로 `is_flying_ == false` → **FAILURE**
- Fallback 노드가 FAILURE를 받아 다음 자식(SubTree ArmTakeoff) 실행

---

## STEP 9 — tick() 전파: ArmService

**파일:** `as2_behavior_tree/plugins/action/arm_service.cpp`
**베이스:** `as2_behavior_tree/include/as2_behavior_tree/bt_service_node.hpp`

```cpp
// BtServiceNode<std_srvs::srv::SetBool>::tick()
BT::NodeStatus tick() override
{
    if (!request_sent_) {
        on_tick();   // ArmService::on_tick() → request_->data = true
        future_result_ = service_client_->async_send_request(request_).share();
        // → /drone0/set_arming_state 서비스 호출 (SetBool: data=true)
        sent_time_ = node_->now();
        request_sent_ = true;
    }
    return check_future();
    // spin_until_future_complete() → 응답 대기
    // response->success == true → BT::NodeStatus::SUCCESS
}
```

**OffboardService도 동일한 패턴:**

```cpp
// offboard_service.cpp
void OffboardService::on_tick() { this->request_->data = true; }
// → /drone0/set_offboard_mode 서비스 호출
```

---

## STEP 10 — tick() 전파: TakeoffAction

**파일:** `as2_behavior_tree/plugins/action/takeoff_action.cpp`
**베이스:** `as2_behavior_tree/include/as2_behavior_tree/bt_action_node.hpp`

### 첫 번째 tick (status == IDLE)

```cpp
// bt_action_node.hpp:166~182
BT::NodeStatus tick() override
{
    if (status() == BT::NodeStatus::IDLE) {
        setStatus(BT::NodeStatus::RUNNING);

        on_tick();  // takeoff_action.cpp:55~58
        // getInput("height", goal_.takeoff_height);  → 2.0
        // getInput("speed",  goal_.takeoff_speed);   → 0.5

        send_new_goal();  // bt_action_node.hpp:307
    }
    ...
}
```

**send_new_goal() 내부:**

```cpp
// bt_action_node.hpp:307~340
void send_new_goal()
{
    goal_result_available_ = false;

    auto send_goal_options = Client<ActionT>::SendGoalOptions();
    send_goal_options.result_callback =
        std::bind(&BtActionNode::result_callback, this, _1);
    send_goal_options.feedback_callback =
        [this](auto, auto feedback) { feedback_ = feedback; };

    future_goal_handle_ = make_shared<shared_future<...>>(
        action_client_->async_send_goal(goal_, send_goal_options));
    //  goal_.takeoff_height = 2.0
    //  goal_.takeoff_speed  = 0.5

    time_goal_sent_ = node_->now();
}
```

### Goal 승인 대기 tick

```cpp
// bt_action_node.hpp:184~203
if (future_goal_handle_) {
    auto elapsed = (node_->now() - time_goal_sent_).to_chrono<milliseconds>();
    if (!is_future_goal_handle_complete(elapsed)) {
        if (elapsed < server_timeout_) {
            return BT::NodeStatus::RUNNING;  // 아직 승인 안 됨
        }
        return BT::NodeStatus::FAILURE;      // 타임아웃
    }
    // 승인 완료 → goal_handle_ 저장
}
```

### 실행 중 tick (RUNNING)

```cpp
// bt_action_node.hpp:206~240
if (!goal_result_available_) {
    on_wait_for_result(feedback_);       // 피드백 처리 (현재 비어있음)
    feedback_.reset();
    callback_group_executor_.spin_some(); // result_callback 이벤트 처리
    return BT::NodeStatus::RUNNING;
}
```

### 결과 처리 tick

```cpp
// bt_action_node.hpp:248~265
switch (result_.code) {
    case SUCCEEDED: return on_success();
    // TakeoffAction::on_success():
    //   std::this_thread::sleep_for(500ms);
    //   return BT::NodeStatus::SUCCESS;

    case ABORTED:   return on_aborted();   // FAILURE
    case CANCELED:  return on_cancelled(); // SUCCESS
}
```

---

## STEP 11 — ROS2 Action: BehaviorServer 수신

**파일:** `as2_behavior/include/as2_behavior/__impl/behavior_server__impl.hpp`

```cpp
// :68~77  Goal 수신 콜백
rclcpp_action::GoalResponse BehaviorServer::handleGoal(uuid, goal)
{
    if (this->activate(goal)) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    return rclcpp_action::GoalResponse::REJECT;
}

// :82~84  Goal 핸들 저장
void BehaviorServer::handleAccepted(goal_handle)
{
    goal_handle_ = goal_handle;  // → BT 노드에 승인 신호 전달
}

// :130~141  activate() 내부
bool BehaviorServer::activate(goal)
{
    RCLCPP_INFO(this->get_logger(), "START");
    if (on_activate(goal)) {          // 파생 클래스 구현 호출
        register_run_timer();         // 10Hz 타이머 등록
        behavior_status_.status = BehaviorStatus::RUNNING;
        return true;
    }
    return false;
}
```

**register_run_timer() 내부:**

```cpp
// :120~127
void BehaviorServer::register_run_timer()
{
    float run_frequency;
    this->get_parameter("run_frequency", run_frequency);  // 기본값: 10.0 Hz
    run_timer_ = this->create_timer(
        std::chrono::duration<double>(1.0f / run_frequency),
        std::bind(&BehaviorServer::timer_callback, this));
    // → 100ms마다 timer_callback() → run() 호출
}
```

---

## STEP 12 — TakeoffBehavior::on_activate() + 플러그인 위임

**파일:** `as2_behaviors_motion/takeoff_behavior/src/takeoff_behavior.cpp`

```cpp
bool TakeoffBehavior::on_activate(
    std::shared_ptr<const as2_msgs::action::Takeoff::Goal> goal)
{
    as2_msgs::action::Takeoff::Goal new_goal = *goal;
    if (!process_goal(goal, new_goal)) return false;

    // pluginlib로 로드된 플러그인에 위임
    return takeoff_plugin_->on_activate(
        std::make_shared<const Takeoff::Goal>(new_goal));
}
```

**takeoff_plugin_position::Plugin::own_activate():**

```cpp
// takeoff_behavior/plugins/takeoff_plugin_position.cpp
bool own_activate(as2_msgs::action::Takeoff::Goal& _goal) override
{
    RCLCPP_INFO(node_ptr_->get_logger(), "Takeoff accepted");
    // 목표 좌표 계산
    takeoff_position_.x = actual_pose_.pose.position.x;
    takeoff_position_.y = actual_pose_.pose.position.y;
    takeoff_position_.z = actual_pose_.pose.position.z + _goal.takeoff_height;
    // 현재 z + 2.0m
    takeoff_angle_ = as2::frame::getYawFromQuaternion(
        actual_pose_.pose.orientation);
    return true;
}
```

---

## STEP 13 — 10Hz 타이머 루프: run() 반복

**파일:** `as2_behavior/include/as2_behavior/__impl/behavior_server__impl.hpp:173~213`

```cpp
void BehaviorServer::run(goal_handle_action)
{
    if (behavior_status_.status != BehaviorStatus::RUNNING) return;

    auto feedback = make_shared<Takeoff::Feedback>();
    auto result   = make_shared<Takeoff::Result>();

    ExecutionStatus status = on_run(goal, feedback, result);
    // → TakeoffBehavior::on_run()
    //   → takeoff_plugin_->on_run()
    //     → takeoff_plugin_position::Plugin::own_run()

    switch (status) {
        case ExecutionStatus::RUNNING:
            goal_handle_action->publish_feedback(feedback);  // BT에 피드백 전송
            behavior_status_.status = BehaviorStatus::RUNNING;
            break;

        case ExecutionStatus::SUCCESS:
            RCLCPP_INFO(this->get_logger(), "SUCCESS");
            behavior_status_.status = BehaviorStatus::IDLE;
            goal_handle_->succeed(result);  // ← BT result_callback() 트리거
            cleanup_run_timer(ExecutionStatus::SUCCESS);
            // → on_execution_end(SUCCESS)
            // → goal_handle_.reset()
            // → run_timer_.reset()
            break;

        case ExecutionStatus::FAILURE:
            goal_handle_->abort(result);
            cleanup_run_timer(ExecutionStatus::FAILURE);
            break;
    }
}
```

**takeoff_plugin_position::Plugin::own_run():**

```cpp
as2_behavior::ExecutionStatus own_run() override
{
    if (checkGoalCondition()) {   // 현재 z >= 목표 z - threshold
        result_.takeoff_success = true;
        return as2_behavior::ExecutionStatus::SUCCESS;
    }

    if (!position_motion_handler_->sendPositionCommandWithYawAngle(
            "earth",
            takeoff_position_.x, takeoff_position_.y, takeoff_position_.z,
            takeoff_angle_, "earth",
            goal_.takeoff_speed, goal_.takeoff_speed, goal_.takeoff_speed)) {
        result_.takeoff_success = false;
        return as2_behavior::ExecutionStatus::FAILURE;
    }

    return as2_behavior::ExecutionStatus::RUNNING;  // 아직 상승 중
}
```

---

## STEP 14 — BT로 결과 반환

**파일:** `as2_behavior_tree/include/as2_behavior_tree/bt_action_node.hpp:354~371`

```cpp
// goal_handle_->succeed() 호출 시 ROS2가 이 콜백 실행
void result_callback(const WrappedResult& result)
{
    if (future_goal_handle_) return;  // 아직 goal 승인 안 된 경우 무시

    if (this->goal_handle_->get_goal_id() == result.goal_id) {
        goal_result_available_ = true;  // ← tick()의 RUNNING 루프 탈출 조건
        result_ = result;
    }
}

// 다음 tick()에서
switch (result_.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
        return on_success();
        // TakeoffAction::on_success():
        //   std::this_thread::sleep_for(std::chrono::milliseconds(500));
        //   return BT::NodeStatus::SUCCESS;
}

goal_handle_.reset();
return status;  // BT::NodeStatus::SUCCESS
```

---

## 전체 흐름 한눈에 보기

```
behavior_trees.launch.py
  │  package: as2_behavior_tree
  │  executable: as2_behavior_tree_node
  ▼
as2_behavior_tree_node.cpp::main()
  ├─ rclcpp::init()
  ├─ Node("bt_manager")  →  /drone0/bt_manager
  ├─ factory.registerNodeType<TakeoffAction>("TakeOff")  ← 14개 등록
  ├─ Blackboard::create() + set("node", node)
  ├─ factory.createTreeFromFile("takeoff.xml")
  │     ├─ WaitForEvent 생성  →  /drone0/mission/start 구독
  │     ├─ IsFlyingCondition  →  /drone0/platform/info 구독
  │     ├─ ArmService         →  /drone0/set_arming_state 서비스 클라이언트
  │     ├─ OffboardService    →  /drone0/set_offboard_mode 서비스 클라이언트
  │     └─ TakeoffAction      →  /drone0/TakeoffBehavior action 클라이언트
  │                                wait_for_action_server(1s) ← 반드시 먼저 기동
  │
  └─ while(RUNNING) { tree.tickWhileRunning(); loopRate.sleep(10ms); }

       [매 10ms tick]
       WaitForEvent::tick()                    wait_for_event.cpp
         ├─ spin_some()
         ├─ flag_==false  →  RUNNING (mission/start 대기)
         │
         └─ [ros2 topic pub /drone0/mission/start ...]
            flag_==true  →  child_node_->executeTick()

               Fallback::tick()
                 ├─ IsFlyingCondition::tick()        is_flying_condition.hpp
                 │    spin_some()
                 │    is_flying_==false  →  FAILURE
                 │
                 └─ SubTree(ArmTakeoff)::tick()
                      SequenceStar::tick()
                        ├─ ArmService::tick()         bt_service_node.hpp
                        │    on_tick() → request_->data=true
                        │    async_send_request(/drone0/set_arming_state)
                        │    check_future()  →  SUCCESS
                        │
                        ├─ OffboardService::tick()
                        │    async_send_request(/drone0/set_offboard_mode)
                        │    →  SUCCESS
                        │
                        └─ TakeoffAction::tick()      bt_action_node.hpp
                             [1st tick]
                             on_tick()                takeoff_action.cpp
                               getInput("height") → goal_.takeoff_height=2.0
                               getInput("speed")  → goal_.takeoff_speed=0.5
                             send_new_goal()
                               async_send_goal(goal_)
                                      │
                                      │ ROS2 Action: /drone0/TakeoffBehavior
                                      ▼
                             BehaviorServer::handleGoal()    behavior_server__impl.hpp:68
                               activate(goal)
                                 on_activate()              takeoff_behavior.cpp
                                   takeoff_plugin_->on_activate()
                                     takeoff_position_.z = current_z + 2.0
                                 register_run_timer()  ←  10Hz 시작
                               return ACCEPT_AND_EXECUTE
                             handleAccepted() → goal_handle_ 저장

                             [이후 tick - RUNNING]
                             is_future_goal_handle_complete()
                               spin_until_future_complete()  → goal_handle_ 확보
                             spin_some()  ← feedback 수신 처리
                             return RUNNING

                                      [10Hz 타이머 - BehaviorServer]
                                      timer_callback() → run()    :173
                                        on_run()                   takeoff_behavior.cpp
                                          takeoff_plugin_->on_run()
                                            own_run()              takeoff_plugin_position.cpp
                                              sendPositionCommand(목표좌표)
                                              if 도달 → SUCCESS
                                              else    → RUNNING
                                        SUCCESS 시:
                                          goal_handle_->succeed(result)
                                            → result_callback() 트리거  bt_action_node.hpp:354
                                               goal_result_available_ = true
                                          cleanup_run_timer()
                                          on_execution_end()  → FSM 상태 FLYING

                             [결과 tick]
                             goal_result_available_==true
                             result_.code==SUCCEEDED
                             on_success()
                               sleep_for(500ms)
                               return BT::NodeStatus::SUCCESS

              SequenceStar  →  SUCCESS
            SubTree         →  SUCCESS
          Fallback          →  SUCCESS
        WaitForEvent        →  SUCCESS

  result == SUCCESS
  while 루프 탈출
  rclcpp::shutdown()
  프로세스 종료
```

---

## 핵심 파일 참조 표

| 역할 | 파일 경로 |
|---|---|
| launch 파일 | `as2_behavior_tree/launch/behavior_trees.launch.py` |
| 진입점 (main) | `as2_behavior_tree/src/as2_behavior_tree_node.cpp` |
| BT Action 베이스 | `as2_behavior_tree/include/as2_behavior_tree/bt_action_node.hpp` |
| BT Service 베이스 | `as2_behavior_tree/include/as2_behavior_tree/bt_service_node.hpp` |
| TakeoffAction (BT) | `as2_behavior_tree/plugins/action/takeoff_action.cpp` |
| ArmService (BT) | `as2_behavior_tree/plugins/action/arm_service.cpp` |
| WaitForEvent (BT) | `as2_behavior_tree/plugins/decorator/wait_for_event.cpp` |
| IsFlyingCondition (BT) | `as2_behavior_tree/include/as2_behavior_tree/condition/is_flying_condition.hpp` |
| BehaviorServer 클래스 선언 | `as2_behaviors/as2_behavior/include/as2_behavior/__detail/behavior_server__class.hpp` |
| BehaviorServer 구현 | `as2_behaviors/as2_behavior/include/as2_behavior/__impl/behavior_server__impl.hpp` |
| TakeoffBehavior | `as2_behaviors/as2_behaviors_motion/takeoff_behavior/src/takeoff_behavior.cpp` |
| TakeoffBase (플러그인 인터페이스) | `as2_behaviors/as2_behaviors_motion/takeoff_behavior/include/takeoff_behavior/takeoff_base.hpp` |
| Position 플러그인 구현 | `as2_behaviors/as2_behaviors_motion/takeoff_behavior/plugins/takeoff_plugin_position.cpp` |
| 예제 XML | `as2_behavior_tree/resource/takeoff.xml` |
