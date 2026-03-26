# as2_core 패키지 전문가 심층 분석

> **버전:** 1.1.3 | **ROS2:** Galactic / Humble / Jazzy | **C++ 표준:** C++17
> **라이선스:** BSD-3-Clause | **관리:** CVAR-UPM (Universidad Politécnica de Madrid)
> **분석 기준:** 실제 소스 코드 직접 정독 (2026-03-26)

---

## 목차

1. [패키지 역할 및 위치](#1-패키지-역할-및-위치)
2. [전체 파일 구조](#2-전체-파일-구조)
3. [외부 의존성](#3-외부-의존성)
4. [핵심 클래스 계층](#4-핵심-클래스-계층)
5. [as2::Node — 기반 노드](#5-as2node--기반-노드)
6. [as2::AerialPlatform — 플랫폼 추상화](#6-as2aerialplatform--플랫폼-추상화)
7. [as2::PlatformStateMachine — 상태 기계](#7-as2platformstatemachine--상태-기계)
8. [as2::sensors — 센서 시스템](#8-as2sensors--센서-시스템)
9. [as2::BasicBehavior — 행동 기반 클래스](#9-as2basicbehavior--행동-기반-클래스)
10. [as2::SynchronousServiceClient — 동기 서비스 클라이언트](#10-as2synchronousserviceclient--동기-서비스-클라이언트)
11. [as2_names — 네이밍 표준화](#11-as2_names--네이밍-표준화)
12. [유틸리티 라이브러리 상세](#12-유틸리티-라이브러리-상세)
13. [Python 바인딩 및 런치 유틸리티](#13-python-바인딩-및-런치-유틸리티)
14. [설계 패턴 분석](#14-설계-패턴-분석)
15. [내부 구현 핵심 로직](#15-내부-구현-핵심-로직)
16. [주의사항 및 Known Issues](#16-주의사항-및-known-issues)
17. [소스 분석 로드맵](#17-소스-분석-로드맵)

---

## 1. 패키지 역할 및 위치

`as2_core`는 Aerostack2 전체 프레임워크의 **기반 패키지**다. 다른 모든 AS2 패키지(`as2_aerial_platforms`, `as2_motion_controller`, `as2_behaviors`, `as2_python_api` 등)는 이 패키지에 의존한다.

### 프레임워크 내 위치

```
[Python API / Behavior Tree]
         ↓
[Behaviors: Takeoff, Land, GoTo, FollowPath ...]
         ↓
[Motion Reference Handlers]
         ↓
[Motion Controller (pluginlib)]
         ↓
[State Estimator (pluginlib)]
         ↓
[AerialPlatform] ←── as2_core가 정의하는 추상 계층
         ↓
[Platform 구현체: Gazebo, PX4, DJI, ...]
         ↓
[실제 하드웨어 / 시뮬레이터]
```

`as2_core`는 이 스택 전체가 의존하는 **공통 언어(토픽명, 서비스명, 타입)**와 **기반 클래스(Node, AerialPlatform, BasicBehavior)**를 제공한다.

---

## 2. 전체 파일 구조

```
as2_core/
├── CMakeLists.txt                    # C++17, STATIC lib, pybind11, ament_cmake_gtest
├── package.xml                       # 메타데이터 v1.1.3
├── Doxyfile                          # Doxygen API 문서화 설정
│
├── include/as2_core/
│   ├── node.hpp                      # ★★★ as2::Node (전체 기반)
│   ├── aerial_platform.hpp           # ★★★ as2::AerialPlatform
│   ├── as2_basic_behavior.hpp        # ★★★ as2::BasicBehavior<MessageT>
│   ├── platform_state_machine.hpp    # ★★  FSM (6 상태, 7 이벤트)
│   ├── sensor.hpp                    # ★★★ Sensor<T>, Camera, GroundTruth, Gimbal
│   ├── synchronous_service_client.hpp# ★★  동기 서비스 클라이언트
│   ├── rate.hpp                      # ★   GenericRate<Clock>
│   ├── core_functions.hpp            # ★   spinLoop()
│   │
│   ├── custom/                       # ROS distro 별 헤더 (CMake configure 시 생성)
│   │   ├── cv_bridge.hpp.in
│   │   └── tf2_geometry_msgs.hpp.in
│   │
│   ├── names/                        # 모든 ROS2 이름 상수 (중앙 관리)
│   │   ├── topics.hpp                # ★★★ 토픽명 + QoS
│   │   ├── services.hpp              # ★★  서비스명
│   │   └── actions.hpp              # ★★  액션명
│   │
│   └── utils/
│       ├── control_mode_utils.hpp    # ★★★ 8비트 제어 모드 인코딩
│       ├── tf_utils.hpp              # ★★★ TfHandler (tf2 래퍼)
│       ├── frame_utils.hpp           # ★★  쿼터니언/오일러 변환
│       ├── gps_utils.hpp             # ★★  GpsHandler (LLA↔ENU↔ECEF)
│       └── yaml_utils.hpp            # ★   YAML 파싱
│
├── src/
│   ├── node.cpp
│   ├── aerial_platform.cpp           # initialize(), sendCommand(), alertEvent() 핵심
│   ├── platform_state_machine.cpp    # defineTransitions() FSM 정의
│   ├── sensor.cpp
│   ├── rate.cpp
│   ├── core_functions.cpp
│   ├── _as2_core_pybind11.cpp        # pybind11 Python 바인딩
│   └── utils/
│       ├── control_mode_utils.cpp
│       ├── tf_utils.cpp
│       ├── frame_utils.cpp
│       ├── gps_utils.cpp
│       └── yaml_utils.cpp
│
├── as2_core/                         # Python 패키지
│   ├── __init__.py
│   ├── launch_param_utils.py         # YAML → DeclareLaunchArgument
│   ├── declare_launch_arguments_from_config_file.py
│   ├── launch_configuration_from_config_file.py
│   └── launch_plugin_utils.py
│
├── tests/
│   ├── platform_state_machine_test.cpp
│   ├── sensor_test.cpp
│   ├── frame_test.cpp
│   ├── tf_utils_gtest.cpp
│   ├── tf2_namespace_test.cpp
│   └── mocks/
│       ├── aerial_platform/
│       │   ├── mock_aerial_platform.hpp
│       │   └── mock_aerial_platform.cpp
│       └── executor_thread_util/
│           ├── executor_thread_util.hpp
│           └── executor_thread_util.cpp
│
└── cmake/
    ├── yaml-cpp-extras.cmake
    └── GeographicLib-extras.cmake
```

---

## 3. 외부 의존성

```
as2_core 의존성 트리
│
├── ROS2 실행 계층
│   ├── rclcpp                 Node, Publisher, Subscriber, Timer, Service, CallbackGroup
│   ├── rclcpp_lifecycle       LifecycleNode (조건부 컴파일)
│   ├── rclcpp_action          ActionServer (BasicBehavior)
│   └── rcl_interfaces
│
├── ROS2 메시지 타입
│   ├── std_msgs / std_srvs    SetBool, Bool
│   ├── nav_msgs               Odometry, Path
│   ├── sensor_msgs            Imu, NavSatFix, Image, LaserScan, BatteryState, ...
│   ├── geometry_msgs          Pose, Twist, Transform, Quaternion, ...
│   ├── geographic_msgs        GeoPoseStamped (GPS)
│   └── as2_msgs               ★ AS2 전용 - ControlMode, PlatformInfo, Thrust, ...
│
├── 좌표 변환
│   ├── tf2 / tf2_ros          TransformBroadcaster, Buffer, Listener
│   ├── tf2_geometry_msgs      ROS 메시지 ↔ tf2 변환
│   └── Eigen3                 선형대수 (벡터/행렬/쿼터니언)
│
├── 비전
│   ├── image_transport        카메라 이미지 (압축 포함)
│   └── cv_bridge              ROS Image ↔ OpenCV Mat
│
├── 설정 / 지리
│   ├── yaml-cpp               YAML 파라미터 파싱
│   └── GeographicLib          고정밀 WGS84 좌표 변환 (LocalCartesian, Geocentric)
│
└── Python 바인딩
    └── pybind11               C++ → Python 모듈 생성
```

---

## 4. 핵심 클래스 계층

```
rclcpp::Node  (또는 rclcpp_lifecycle::LifecycleNode)
└── as2::Node
    ├── as2::AerialPlatform           ← 하드웨어 추상화
    │   └── [구현체들]
    │       예) as2_platform_gazebo, as2_platform_multirotor_simulator
    │
    └── as2::BasicBehavior<MessageT>  ← 행동 추상화
        └── [행동 구현체들]
            예) TakeoffBehavior, LandBehavior, GoToBehavior

─────────────────────────────────────────────────────
as2::PlatformStateMachine             ← FSM (AerialPlatform 내부 보유)

─────────────────────────────────────────────────────
as2::sensors
    ├── TFStatic                      ← 정적 TF 발행
    ├── TFDynamic                     ← 동적 TF 발행
    ├── SensorData<T>                 ← 타입별 토픽 발행
    ├── GenericSensor                 ← 주파수 제어 기반 클래스
    └── Sensor<T>                     ← TFStatic + GenericSensor + SensorData<T>
        ├── Odometry = Sensor<nav_msgs::msg::Odometry>
        ├── Imu      = Sensor<sensor_msgs::msg::Imu>
        ├── GPS      = Sensor<sensor_msgs::msg::NavSatFix>
        ├── Lidar    = Sensor<sensor_msgs::msg::LaserScan>
        ├── Battery  = Sensor<sensor_msgs::msg::BatteryState>
        ├── Barometer= Sensor<sensor_msgs::msg::FluidPressure>
        ├── Compass  = Sensor<sensor_msgs::msg::MagneticField>
        └── RangeFinder = Sensor<sensor_msgs::msg::Range>
    ├── Camera                        ← TFStatic + GenericSensor + image_transport
    ├── GroundTruth                   ← GenericSensor + 2개 SensorData
    └── Gimbal                        ← TFStatic + TFDynamic + GenericSensor + SensorData

─────────────────────────────────────────────────────
as2::SynchronousServiceClient<ServiceT>  ← 동기 서비스 호출 래퍼

─────────────────────────────────────────────────────
as2::tf::TfHandler                    ← tf2_ros::Buffer 래퍼
as2::gps::GpsHandler                  ← GeographicLib::LocalCartesian 래퍼

─────────────────────────────────────────────────────
GenericRate<Clock>
├── as2::Rate      = GenericRate<system_clock>
└── as2::WallRate  = GenericRate<steady_clock>
```

---

## 5. as2::Node — 기반 노드

**파일:** `include/as2_core/node.hpp`, `src/node.cpp`

### 컴파일 타임 스위치

```cpp
// node.hpp 상단 매크로
#define AS2_RCLCPP_NODE 1
#define AS2_LIFECYLCE_NODE 2
#define AS2_NODE_FATHER AS2_RCLCPP_NODE       // 현재 기본값
// #define AS2_NODE_FATHER AS2_LIFECYLCE_NODE // LifecycleNode로 전환 가능

#if AS2_NODE_FATHER == AS2_RCLCPP_NODE
#define AS2_NODE_FATHER_TYPE rclcpp::Node
#elif AS2_NODE_FATHER == AS2_LIFECYLCE_NODE
#define AS2_NODE_FATHER_TYPE rclcpp_lifecycle::LifecycleNode
#endif
```

**핵심 설계 선택:** rclcpp::Node를 기본으로 하되, 매크로 한 줄로 LifecycleNode로 전환 가능하도록 설계했다. 현재는 `AS2_RCLCPP_NODE`가 기본값이므로 LifecycleNode 특화 기능은 `rclcpp_lifecycle`을 include하면서도 표준 Node처럼 동작한다.

### 초기화 흐름 (`init()`)

```
Node 생성자 호출
    └── init()
        ├── has_parameter("node_frequency") 확인
        ├── declare_parameter<float>("node_frequency", -1.0) 선언
        ├── get_parameter("node_frequency", loop_frequency_)
        └── loop_frequency_ > 0 이면:
            └── loop_rate_ptr_ = make_shared<Rate>(loop_frequency_)
```

### 핵심 메서드

| 메서드 | 설명 | 구현 위치 |
|--------|------|-----------|
| `generate_local_name(name)` | `node_ns/node_name/name` | src/node.cpp |
| `generate_global_name(name)` | 드론 네임스페이스/name (leading '/' 제거) | src/node.cpp |
| `sleep()` | loop_rate_ptr_->sleep() 또는 예외 throw | node.hpp (inline) |
| `preset_loop_frequency(freq)` | 이미 파라미터로 설정되지 않은 경우만 주파수 설정 | node.hpp (inline) |
| `get_loop_frequency()` | 현재 루프 주파수 반환 | node.hpp (inline) |
| `create_timer(period, callback)` | 노드 클럭 기반 타이머 생성 | node.hpp (template) |
| `configure/activate/deactivate/cleanup/shutdown/error` | LifecycleNode 호환 콜백 | node.hpp (virtual) |

### ROS2 파라미터

| 파라미터명 | 타입 | 기본값 | 의미 |
|-----------|------|--------|------|
| `node_frequency` | float | -1.0 | 메인 루프 Hz (-1이면 sleep() 호출 시 예외) |

---

## 6. as2::AerialPlatform — 플랫폼 추상화

**파일:** `include/as2_core/aerial_platform.hpp`, `src/aerial_platform.cpp`

### 클래스 구조 (멤버 변수)

```cpp
class AerialPlatform : public as2::Node {
private:
    // 내부 상태
    bool sending_commands_ = false;
    rclcpp::TimerBase::SharedPtr platform_cmd_timer_;   // cmd_freq_ Hz
    rclcpp::TimerBase::SharedPtr platform_info_timer_;  // info_freq_ Hz
    as2::PlatformStateMachine state_machine_;           // FSM 보유
    std::vector<uint8_t> available_control_modes_;      // YAML에서 로드

protected:
    float cmd_freq_;     // ROS2 파라미터 "cmd_freq" (기본 100Hz)
    float info_freq_;    // ROS2 파라미터 "info_freq" (기본 10Hz)

    // 수신 명령 버퍼 (마지막 값 보존)
    as2_msgs::msg::TrajectorySetpoints command_trajectory_msg_;
    geometry_msgs::msg::PoseStamped    command_pose_msg_;
    geometry_msgs::msg::TwistStamped   command_twist_msg_;
    as2_msgs::msg::Thrust              command_thrust_msg_;
    as2_msgs::msg::PlatformInfo        platform_info_msg_;
    bool has_new_references_ = false;
};
```

### initialize() 상세 흐름

```
AerialPlatform::initialize()
│
├── resetPlatform()
│   ├── platform_info_msg_.armed = false
│   ├── platform_info_msg_.offboard = false
│   ├── platform_info_msg_.connected = true
│   ├── state_machine_.setState(DISARMED)
│   └── resetActuatorCommandMsgs()
│
├── ROS2 파라미터 선언
│   ├── "cmd_freq"           (기본 100.0)
│   ├── "info_freq"          (기본 10.0)
│   └── "control_modes_file" (필수 - 없으면 FATAL 후 종료)
│
├── loadControlModes(control_modes_file)
│   └── YAML에서 "available_modes" 태그를 찾아
│       uint8_t로 변환하여 available_control_modes_ 구성
│
├── 구독 생성 (람다로 간단히 구현)
│   ├── actuator_command/trajectory  → command_trajectory_msg_ + has_new_references_=true
│   ├── actuator_command/pose        → command_pose_msg_ + has_new_references_=true
│   ├── actuator_command/twist       → command_twist_msg_ + has_new_references_=true
│   ├── actuator_command/thrust      → command_thrust_msg_ + has_new_references_=true
│   └── alert_event                  → alertEventCallback()
│
├── 서비스 서버 생성
│   ├── "set_platform_control_mode"  → setPlatformControlModeSrvCall()
│   ├── "set_arming_state"           → setArmingStateSrvCall()
│   ├── "set_offboard_mode"          → setOffboardModeSrvCall()
│   ├── "platform_takeoff"           → platformTakeoffSrvCall()
│   ├── "platform_land"              → platformLandSrvCall()
│   └── "platform/list_control_modes"→ listControlModesSrvCall()
│
├── platform_info_pub_ 생성
│   └── platform/info (QoS 10)
│
├── platform_info_timer_ 생성
│   └── info_freq_ Hz → publishPlatformInfo()
│
└── platform_cmd_timer_ 생성
    └── cmd_freq_ Hz → sendCommand()
```

### 순수 가상 메서드 (구현 계약)

플랫폼 구현체는 반드시 아래 7개를 구현해야 한다:

```cpp
// 센서 초기화
virtual void configureSensors() = 0;

// 명령 전송 (매 cmd_freq_ Hz마다 호출됨)
virtual bool ownSendCommand() = 0;

// 암/디암 (서비스 콜백에서 호출됨)
virtual bool ownSetArmingState(bool state) = 0;

// 오프보드 모드 전환
virtual bool ownSetOffboardControl(bool offboard) = 0;

// 제어 모드 설정
virtual bool ownSetPlatformControlMode(const as2_msgs::msg::ControlMode & msg) = 0;

// 긴급 모터 정지 (복구 불가)
virtual void ownKillSwitch() = 0;

// 긴급 호버링 (복구 가능)
virtual void ownStopPlatform() = 0;
```

선택적 구현 (기본값 false 반환):
```cpp
virtual bool ownTakeoff() {return false;}
virtual bool ownLand()    {return false;}
```

### sendCommand() — 핵심 제어 루프

```cpp
void AerialPlatform::sendCommand()
{
    // 전제조건 검사 (모두 충족 시에만 명령 전송)
    if (!isControlModeSettled()) return;   // 제어 모드 미설정
    if (!getConnectedStatus())   return;   // 연결 미확인
    if (!getArmingState())       return;   // 미암
    if (!getOffboardMode())      return;   // 오프보드 모드 미활성

    if (state_machine_.getState() == EMERGENCY) {
        ownStopPlatform();   // 비상 시 반복 호버링 명령
    } else if (has_new_references_) {
        ownSendCommand();    // 새 명령이 있을 때만 전송
    }
}
```

**설계 요점:** `has_new_references_` 플래그를 통해 동일 명령의 반복 전송을 방지한다. 단, HOVER/UNSET 모드 설정 시에는 `has_new_references_=true`로 강제 설정되어 지속 호버링 명령이 전송된다.

### alertEvent() — 비상 처리

```cpp
void AerialPlatform::alertEvent(const as2_msgs::msg::AlertEvent & msg)
{
    if (msg.alert > 0) return;  // 양수 값은 무시 (경고 레벨)

    switch (msg.alert) {
        case AlertEvent::KILL_SWITCH:       // -1
            state_machine_.processEvent(EMERGENCY);
            ownKillSwitch();    // 모터 즉시 정지
            break;
        case AlertEvent::EMERGENCY_HOVER:   // -2
            state_machine_.processEvent(EMERGENCY);
            ownStopPlatform();  // 최선의 호버링
            break;
    }
}
```

### ROS2 파라미터

| 파라미터명 | 타입 | 기본값 | 필수 |
|-----------|------|--------|------|
| `cmd_freq` | float | 100.0 | 선택 |
| `info_freq` | float | 10.0 | 선택 |
| `control_modes_file` | string | - | **필수** |

---

## 7. as2::PlatformStateMachine — 상태 기계

**파일:** `include/as2_core/platform_state_machine.hpp`, `src/platform_state_machine.cpp`

### 상태 전이 테이블 (defineTransitions()에서 직접 구현)

```
상태             이벤트         다음 상태
─────────────────────────────────────────────
DISARMED    + ARM        → LANDED
LANDED      + DISARM     → DISARMED
LANDED      + TAKE_OFF   → TAKING_OFF
TAKING_OFF  + TOOK_OFF   → FLYING
FLYING      + LAND       → LANDING
LANDING     + LANDED     → LANDED

[모든 상태]  + EMERGENCY  → EMERGENCY
  └── DISARMED    + EMERGENCY → EMERGENCY
  └── LANDED      + EMERGENCY → EMERGENCY
  └── TAKING_OFF  + EMERGENCY → EMERGENCY
  └── FLYING      + EMERGENCY → EMERGENCY
  └── LANDING     + EMERGENCY → EMERGENCY
```

**주의:** EMERGENCY 상태에서 빠져나오는 전이는 **정의되지 않음**. 비상 상태 후 복구는 플랫폼 재시작을 의미한다.

### 전이 실패 처리

```cpp
bool PlatformStateMachine::processEvent(const int8_t & event)
{
    StateMachineTransition transition = getTransition(current_state, event);

    if (transition.transition_id == -11) {  // 유효하지 않은 전이
        RCLCPP_WARN(..., "Invalid transition: %s -> %s", ...);
        return false;
    }
    // 유효한 전이면 상태 변경
    state_.state = transition.to_state_id;
    return true;
}
```

### 외부 상태 변경 서비스

생성자에서 자동으로 `<node_name>/state_machine_event` 서비스를 등록한다:
```cpp
state_machine_event_srv_ = node_ptr_->create_service<SetPlatformStateMachineEvent>(
    node_ptr_->generate_local_name("state_machine_event"),
    ...);
```

이 서비스를 통해 외부(테스트 코드 등)에서 FSM 이벤트를 강제로 주입할 수 있다.

---

## 8. as2::sensors — 센서 시스템

**파일:** `include/as2_core/sensor.hpp`, `src/sensor.cpp`

### 센서 클래스 계층 설계 원리

센서 시스템은 4개의 독립 구성요소를 조합(mixin)하는 방식으로 설계되었다:

```
TFStatic     → 정적 좌표 변환 발행 (tf2_ros::StaticTransformBroadcaster)
TFDynamic    → 동적 좌표 변환 발행 (tf2_ros::TransformBroadcaster)
SensorData<T>→ 특정 메시지 타입의 토픽 발행 (rclcpp::Publisher<T>)
GenericSensor→ 주파수 기반 타이머 발행 제어
```

### SensorData<T> — 토픽 발행 코어

```cpp
template<typename T>
class SensorData {
public:
    SensorData(const std::string & topic_name, rclcpp::Node * node_ptr,
               bool add_sensor_measurements_base = true)
    {
        // "sensor_measurements/" 접두사 자동 처리
        topic_name_ = processTopicName(topic_name, add_sensor_measurements_base);
        sensor_publisher_ = node_ptr->create_publisher<T>(
            topic_name_, as2_names::topics::sensor_measurements::qos);
    }

    void setData(const T & msg);          // 데이터만 저장
    void publish();                        // 저장된 데이터 발행
    void updateAndPublish(const T & msg);  // 저장 + 즉시 발행
    const T & getData() const;
    T & getDataRef();
};
```

### GenericSensor — 주파수 제어

```cpp
class GenericSensor {
public:
    // pub_freq = -1: updateData 호출 즉시 발행
    // pub_freq > 0:  타이머로 해당 주파수에 맞춰 발행
    explicit GenericSensor(as2::Node * node_ptr, const float pub_freq = -1.0f);

    void dataUpdated();          // 하위 클래스가 새 데이터 수신 시 호출
    virtual void publishData() = 0;  // 하위 클래스가 실제 발행 구현
};
```

**동작 원리:**
- `pub_freq == -1`: `dataUpdated()` 호출 시 즉시 `publishData()` 호출
- `pub_freq > 0`: 타이머가 해당 Hz로 `publishData()` 주기적 호출

### Sensor<T> — 조합 클래스

```cpp
template<typename T>
class Sensor : public TFStatic, protected GenericSensor, public SensorData<T>
{
public:
    Sensor(const std::string & id, as2::Node * node_ptr,
           float pub_freq = -1.0f, bool add_sensor_measurements_base = true)
    : TFStatic(node_ptr),
      GenericSensor(node_ptr, pub_freq),
      SensorData<T>(id, node_ptr, add_sensor_measurements_base) {}

    void updateData(const T & msg) {
        SensorData<T>::setData(msg);
        dataUpdated();  // GenericSensor에게 알림
    }

protected:
    void publishData() override {
        SensorData<T>::publish();  // 실제 발행
    }
};
```

### Camera — 특수화 카메라 센서

```cpp
class Camera : public TFStatic, protected GenericSensor {
    // image_transport::CameraPublisher 사용 (이미지 + camera_info 동시 발행)
    std::shared_ptr<image_transport::CameraPublisher> it_camera_publisher_ptr_;
    sensor_msgs::msg::Image image_data_;
    sensor_msgs::msg::CameraInfo camera_info_;

    // 두 가지 입력 형식 지원
    void updateData(const sensor_msgs::msg::Image & img);
    void updateData(const cv::Mat & img);  // OpenCV 직접 지원

    // 카메라 캘리브레이션 파라미터 ROS param에서 읽기
    void readCameraInfoFromROSParameters(const std::string & prefix = "");
};
```

### GroundTruth — 시뮬레이션 진실값

```cpp
class GroundTruth : protected GenericSensor {
    // 두 개의 독립 SensorData로 pose와 twist 분리 발행
    shared_ptr<SensorData<PoseStamped>>  pose_sensor_;
    shared_ptr<SensorData<TwistStamped>> twist_sensor_;

    void updateData(const PoseStamped & pose_msg);
    void updateData(const TwistStamped & twist_msg);
    void updateData(const PoseStamped &, const TwistStamped &);  // 동시 업데이트
};
```

### Gimbal — 짐벌 센서

```cpp
class Gimbal : public TFStatic, protected TFDynamic,
               protected GenericSensor,
               protected SensorData<geometry_msgs::msg::PoseStamped>
{
    // gimbal_base: 정적 TF (드론 본체 → 짐벌 마운트)
    // gimbal dynamic TF: 동적 TF (짐벌 마운트 → 짐벌 현재 방향)
    // pose 토픽: 현재 짐벌 방향 발행

    void setGimbalBaseTransform(const Transform &, const string & parent = "base_link");
    void updateData(const PoseStamped & pose_msg);
    void updateData(const QuaternionStamped & orientation_msg);
};
```

### 타입 별칭 (편의 API)

```cpp
using Odometry   = Sensor<nav_msgs::msg::Odometry>;
using Imu        = Sensor<sensor_msgs::msg::Imu>;
using GPS        = Sensor<sensor_msgs::msg::NavSatFix>;
using Lidar      = Sensor<sensor_msgs::msg::LaserScan>;
using Battery    = Sensor<sensor_msgs::msg::BatteryState>;
using Barometer  = Sensor<sensor_msgs::msg::FluidPressure>;
using Compass    = Sensor<sensor_msgs::msg::MagneticField>;
using RangeFinder= Sensor<sensor_msgs::msg::Range>;
```

### 플랫폼 구현체에서 사용 예시

```cpp
void MyPlatform::configureSensors() {
    // IMU: sensor_measurements/imu 에 100Hz 발행
    imu_sensor_ = std::make_shared<as2::sensors::Imu>("imu", this, 100.0);
    imu_sensor_->setStaticTransform("imu_frame", "base_link", 0.0, 0.0, 0.1, 0, 0, 0);

    // GPS: sensor_measurements/gps 에 즉시 발행 (pub_freq=-1)
    gps_sensor_ = std::make_shared<as2::sensors::GPS>("gps", this);
}

// 루프에서:
void MyPlatform::publishSensorData(const ImuData & data) {
    sensor_msgs::msg::Imu msg;
    // ... 데이터 채우기 ...
    imu_sensor_->updateData(msg);  // 내부적으로 100Hz 타이머가 발행
}
```

---

## 9. as2::BasicBehavior — 행동 기반 클래스

**파일:** `include/as2_core/as2_basic_behavior.hpp`

### 구조 분석

```cpp
template<class MessageT>
class BasicBehavior : public as2::Node {
public:
    using GoalHandleAction = rclcpp_action::ServerGoalHandle<MessageT>;

    explicit BasicBehavior(const std::string & name) : Node(name)
    {
        // 액션 서버를 글로벌 이름으로 등록
        // generate_global_name(name)을 사용하므로 드론 네임스페이스 하위에 등록됨
        action_server_ = rclcpp_action::create_server<MessageT>(
            this, this->generate_global_name(name),
            std::bind(&BasicBehavior::handleGoal, ...),
            std::bind(&BasicBehavior::handleCancel, ...),
            std::bind(&BasicBehavior::handleAccepted, ...));
    }

    // 하위 클래스가 구현해야 할 인터페이스
    virtual GoalResponse onAccepted(const shared_ptr<const Goal> goal) = 0;
    virtual CancelResponse onCancel(const shared_ptr<GoalHandleAction>) = 0;
    virtual void onExecute(const shared_ptr<GoalHandleAction>) = 0;

private:
    // handleAccepted: 블로킹을 피하기 위해 별도 스레드에서 onExecute 실행
    void handleAccepted(const shared_ptr<GoalHandleAction> goal_handle)
    {
        if (execution_thread_.joinable()) execution_thread_.join();  // 이전 실행 완료 대기
        execution_thread_ = std::thread(
            std::bind(&BasicBehavior::onExecute, this, _1), goal_handle);
    }

    std::thread execution_thread_;
    typename rclcpp_action::Server<MessageT>::SharedPtr action_server_;
};
```

### 중요 설계 특징

1. **스레드 분리:** `handleAccepted`는 executor를 블로킹하지 않도록 즉시 반환. `onExecute`는 별도 스레드에서 실행.
2. **순차 실행 보장:** 새 Goal 수락 전 이전 실행 스레드 join. 동시에 2개의 행동이 실행되지 않음.
3. **TODO 존재:** 코드에 `// TODO(miferco97): explore the use of Timers instead std::thread` 주석이 있어 스레드 대신 타이머 기반으로 리팩터링이 검토 중임.

---

## 10. as2::SynchronousServiceClient — 동기 서비스 클라이언트

**파일:** `include/as2_core/synchronous_service_client.hpp`

### 문제와 해법

ROS2의 서비스 클라이언트는 기본적으로 비동기(async)다. 행동 실행 중 서비스 응답을 기다리려면 별도 spin이 필요하다. `SynchronousServiceClient`는 이 문제를 별도 `CallbackGroup + SingleThreadedExecutor`로 해결한다.

```cpp
template<class ServiceT>
class SynchronousServiceClient {
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
    shared_ptr<rclcpp::Client<ServiceT>> service_client_;

public:
    SynchronousServiceClient(string service_name, as2::Node * node)
    {
        // MutuallyExclusive + not automatically added = 독립 executor 가능
        callback_group_ = node_->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive, false);
        callback_group_executor_.add_callback_group(
            callback_group_, node_->get_node_base_interface());
        service_client_ = node_->create_client<ServiceT>(
            service_name, rmw_qos_profile_services_default, callback_group_);
    }

    bool sendRequest(const RequestT & req, ResponseT & resp, int wait_time = 0)
    {
        // wait_time=0: 서비스가 나타날 때까지 무한 대기
        // wait_time>0: 해당 초만큼 대기 후 실패 반환
        auto result = service_client_->async_send_request(req);
        callback_group_executor_.spin_until_future_complete(result);
        resp = *result.get();
        return true;
    }
};
```

---

## 11. as2_names — 네이밍 표준화

**파일:** `include/as2_core/names/topics.hpp`, `services.hpp`, `actions.hpp`

### 토픽 네임스페이스 체계

```
[드론 네임스페이스]/
│
├── alert_event                          QoS 10     전역 비상 이벤트
│
├── sensor_measurements/                 SensorDataQoS
│   ├── imu
│   ├── lidar
│   ├── gps
│   ├── camera
│   ├── battery
│   └── odom
│
├── ground_truth/                        SensorDataQoS (시뮬레이션)
│   ├── pose
│   └── twist
│
├── self_localization/                   SensorDataQoS (추정 상태)
│   ├── odom
│   ├── pose
│   └── twist
│
├── motion_reference/                    SensorDataQoS (Behavior → Controller)
│   ├── thrust
│   ├── pose
│   ├── twist
│   ├── trajectory
│   ├── modify_waypoint
│   └── traj_gen_info
│
├── actuator_command/                    SensorDataQoS (Controller → Platform)
│   ├── pose
│   ├── twist
│   ├── thrust
│   └── trajectory
│
├── platform/                            QoS 10
│   └── info
│
├── controller/                          QoS 10
│   └── info
│
└── follow_target/                       QoS 10
    └── info
```

### QoS 정책 분류

| QoS | 사용 토픽 | 특성 |
|-----|---------|------|
| `rclcpp::SensorDataQoS()` | 센서/제어 토픽 | Best Effort, Volatile, depth=10 |
| `rclcpp::QoS(10)` | 상태/정보 토픽 | Reliable, depth=10 |
| `rclcpp::QoS(10)` (qos_waypoint) | 웨이포인트 | Reliable, depth=10 |

### 서비스 이름 체계

```
[로컬 노드 네임스페이스]/              (generate_local_name 사용)
├── set_arming_state                   std_srvs/srv/SetBool
├── set_offboard_mode                  std_srvs/srv/SetBool
├── set_platform_control_mode          as2_msgs/srv/SetControlMode
├── platform_takeoff                   std_srvs/srv/SetBool
├── platform_land                      std_srvs/srv/SetBool
│
├── platform/
│   ├── state_machine_event            as2_msgs/srv/SetPlatformStateMachineEvent
│   └── list_control_modes             as2_msgs/srv/ListControlModes
│
├── controller/
│   ├── set_control_mode               as2_msgs/srv/SetControlMode
│   └── list_control_modes             as2_msgs/srv/ListControlModes
│
├── traj_gen/
│   ├── send_traj_wayp
│   ├── add_traj_wayp
│   └── set_traj_speed
│
├── get_origin / set_origin            GPS 원점 관련
│
└── behavior/
    ├── package_pickup
    ├── package_unpick
    ├── dynamic_land
    └── dynamic_follower
```

### 액션 이름

```
[드론 네임스페이스]/              (generate_global_name 사용)
├── TakeoffBehavior
├── GoToBehavior
├── FollowReferenceBehavior
├── FollowPathBehavior
├── LandBehavior
└── TrajectoryGeneratorBehavior
```

---

## 12. 유틸리티 라이브러리 상세

### 12.1 control_mode_utils — 8비트 제어 모드 인코딩

**파일:** `include/as2_core/utils/control_mode_utils.hpp`

```
8비트 인코딩 구조
─────────────────────────────────────────────────────────────────
Bit 7  6  5  4   |  Bit 3  2   |  Bit 1  0
[  Control Mode  ] [ Yaw Mode  ] [ Reference Frame ]
─────────────────────────────────────────────────────────────────
Control Mode (상위 4비트):
  0000 = 0x00 = UNSET           : 미설정
  0001 = 0x10 = HOVER           : 호버링 유지
  0010 = 0x20 = ACRO            : Acrobatic (속도/각속도 직접 제어)
  0011 = 0x30 = ATTITUDE        : 자세 제어
  0100 = 0x40 = SPEED           : 속도 제어
  0101 = 0x50 = SPEED_IN_A_PLANE: 수평면 속도 + 고도 제어
  0110 = 0x60 = POSITION        : 위치 제어
  0111 = 0x70 = TRAJECTORY      : 궤적 추종

Yaw Mode (비트 3-2):
  00 = ANGLE  : 요 각도 제어
  01 = SPEED  : 요 각속도 제어
  10 = NONE   : 요 제어 안함

Reference Frame (비트 1-0):
  00 = LOCAL_FLU   : 로컬 드론 기준 (Forward-Left-Up)
  01 = GLOBAL_ENU  : 전역 ENU (East-North-Up)
  10 = GLOBAL_LLA  : GPS 좌표 (Latitude-Longitude-Altitude)
  11 = UNDEFINED   : 미정의
─────────────────────────────────────────────────────────────────
```

**인코딩 공식:**
```cpp
constexpr uint8_t convertToUint8t(const ControlMode & mode) {
    return (mode.control_mode << 4) | (mode.yaw_mode << 2) | mode.reference_frame;
}
```

**비트마스크 상수:**
```cpp
#define MATCH_ALL             0b11111111  // 모든 필드 비교
#define MATCH_CONTROL_MODE    0b11110000  // 상위 4비트만 비교
#define MATCH_YAW_MODE        0b00001100  // 비트 3-2만 비교
#define MATCH_REFERENCE_FRAME 0b00000011  // 비트 1-0만 비교
#define UNSET_MODE_MASK       0b00000000
#define HOVER_MODE_MASK       0b00010000
```

**활용 예시:**
```cpp
// 제어 모드만 같은지 비교 (yaw, frame은 무시)
bool same_mode = as2::control_mode::compareModes(mode1, mode2, MATCH_CONTROL_MODE);

// POSITION 모드인지 확인
uint8_t encoded = as2::control_mode::convertAS2ControlModeToUint8t(mode);
bool is_position = as2::control_mode::compareModes(
    encoded, 0x60, MATCH_CONTROL_MODE);
```

### 12.2 TfHandler — TF2 래퍼

**파일:** `include/as2_core/utils/tf_utils.hpp`

```cpp
class TfHandler {
    shared_ptr<tf2_ros::Buffer>            tf_buffer_;
    shared_ptr<tf2_ros::TransformListener> tf_listener_;
    as2::Node * node_;
    chrono::nanoseconds tf_timeout_threshold_ = chrono::nanoseconds::zero();

public:
    // 제네릭 변환 (PoseStamped, PointStamped, Vector3Stamped, ...)
    template<typename T>
    T convert(const T & input, const string & target_frame, chrono::nanoseconds timeout);

    // TwistStamped 특수화 (linear만 변환, angular는 그대로)
    TwistStamped convert(const TwistStamped &, const string &, chrono::nanoseconds);

    // Path 변환
    nav_msgs::msg::Path convert(const nav_msgs::msg::Path &, const string &, chrono::nanoseconds);

    // 안전한 변환 (예외 대신 bool 반환)
    template<typename T>
    bool tryConvert(T & input, const string & target_frame);

    // 특정 프레임 간 포즈 조회
    PoseStamped getPoseStamped(const string & target, const string & source,
                                const tf2::TimePoint & time = tf2::TimePointZero);

    // pose + twist 동시 조회
    pair<PoseStamped, TwistStamped> getState(
        const TwistStamped & twist, const string & twist_target,
        const string & pose_target, const string & pose_source);
};
```

**중요:** `convert()` 내부에서 `"earth"` 프레임을 고정 참조 프레임으로 사용한다 (REP105 표준의 `earth` = WGS84 지구 중심 프레임).

### 12.3 GpsHandler — GPS 좌표 변환

**파일:** `include/as2_core/utils/gps_utils.hpp`

```cpp
class GpsHandler : private GeographicLib::LocalCartesian {
    bool is_origin_set_ = false;
    const string local_frame_ = "map";    // REP105 지역 프레임명
    // global_frame은 네임스페이스 상수: "earth"

public:
    // 원점 설정 (한 번만 설정 가능 - OriginAlreadySet 예외)
    void setOrigin(double lat0, double lon0, double h0 = 0);
    void setOrigin(const NavSatFix & fix);

    // LLA → 지역 직교좌표 (ENU)
    void LatLon2Local(double lat, double lon, double h, double & x, double & y, double & z);
    void LatLon2Local(const NavSatFix & fix, PoseStamped & ps);

    // 지역 직교좌표 (ENU) → LLA
    void Local2LatLon(double x, double y, double z, double & lat, double & lon, double & h);
    void Local2LatLon(const PoseStamped & ps, GeoPoseStamped & gps);

    // LLA ↔ ECEF (정적 메서드 - 원점 불필요)
    static void LatLon2Ecef(double lat, double lon, double h, double & x, double & y, double & z);
    static void Ecef2LatLon(double x, double y, double z, double & lat, double & lon, double & h);
};
```

**예외 처리:**
- `OriginNonSet`: 원점 미설정 상태에서 Local↔LLA 변환 시도
- `OriginAlreadySet`: 이미 설정된 원점을 다시 설정 시도

### 12.4 frame_utils — 좌표 변환 수학

**파일:** `include/as2_core/utils/frame_utils.hpp`

**입력 타입 다형성:** 모든 변환 함수는 tf2::Quaternion, geometry_msgs::Quaternion, Eigen::Quaterniond, 또는 roll/pitch/yaw(float) 등 여러 입력 형식을 지원한다.

```cpp
namespace as2::frame {

// 벡터 회전
Eigen::Vector3d transform(const tf2::Quaternion & q, const Eigen::Vector3d & v);
Eigen::Vector3d transform(float roll, float pitch, float yaw, const Eigen::Vector3d & v);
Eigen::Vector3d transform(const geometry_msgs::msg::Quaternion & q, const Eigen::Vector3d & v);
Eigen::Vector3d transform(const Eigen::Quaterniond & q, const Eigen::Vector3d & v);

// 역방향 회전 (q^{-1} * v)
Eigen::Vector3d transformInverse(const tf2::Quaternion & q, const Eigen::Vector3d & v);
// ... (동일한 다형성)

// 쿼터니언 ↔ 오일러
void quaternionToEuler(const tf2::Quaternion & q, double & roll, double & pitch, double & yaw);
void eulerToQuaternion(double roll, double pitch, double yaw, tf2::Quaternion & q);
void eulerToQuaternion(double roll, double pitch, double yaw, geometry_msgs::msg::Quaternion & q);
void eulerToQuaternion(double roll, double pitch, double yaw, Eigen::Quaterniond & q);

// 요 각도 추출
double getYawFromQuaternion(const tf2::Quaternion & q);
double getYawFromQuaternion(const geometry_msgs::msg::Quaternion & q);
double getYawFromQuaternion(const Eigen::Quaterniond & q);

// 벡터 방향각
double getVector2DAngle(double x, double y);  // atan2(y, x)

// 각도 래핑
double wrapAngle0To2Pi(double theta);   // [0, 2π]
double wrapAnglePiToPi(double theta);   // [-π, π]
double angleMinError(double theta1, double theta2);  // 최소 각도 차이
}
```

---

## 13. Python 바인딩 및 런치 유틸리티

### 13.1 pybind11 바인딩

**파일:** `src/_as2_core_pybind11.cpp`

`as2_names` 네임스페이스의 모든 토픽/서비스/액션 이름 상수를 Python에서 접근 가능하게 한다.

```python
# Python에서 사용
from as2_core._as2_core_pybind11 import as2_names

topic = as2_names.topics.self_localization.pose  # "self_localization/pose"
svc   = as2_names.services.platform.set_arming_state
action = as2_names.actions.behaviors.takeoff
```

### 13.2 launch_param_utils.py

```python
# YAML 파일에서 파라미터와 설명(description)을 함께 파싱
def read_complete_yaml_text(yaml_file: str) -> dict:
    """YAML 파일을 파싱하여 {name: (value, description)} 딕셔너리 반환"""

def _dict_to_declare_launch_argument(param_dict) -> list[DeclareLaunchArgument]:
    """딕셔너리를 ROS2 DeclareLaunchArgument 리스트로 변환"""
```

### 13.3 declare_launch_arguments_from_config_file.py

```python
class DeclareLaunchArgumentsFromConfigFile(GroupAction):
    """
    YAML 설정 파일에서 자동으로 DeclareLaunchArgument를 생성하는 런치 액션.
    플러그인 기반 런치 파일에서 파라미터 선언을 자동화한다.
    """
```

---

## 14. 설계 패턴 분석

### 14.1 Template Method Pattern

`AerialPlatform`은 Template Method 패턴의 교과서적 구현이다:
- **템플릿 메서드:** `sendCommand()`, `setArmingState()`, `alertEvent()` — 프레임워크가 제어 흐름을 정의
- **훅 메서드:** `ownSendCommand()`, `ownSetArmingState()` 등 — 서브클래스가 세부 구현

```
[프레임워크 호출]        [플랫폼 구현체]
sendCommand()
  ├── 전제조건 검사
  ├── EMERGENCY 처리
  └── ownSendCommand()  ← 서브클래스 구현 (하드웨어 제어)
```

### 14.2 Mixin (다중 상속 구성)

`Sensor<T>`, `Gimbal` 등이 독립적인 기능 클래스들을 다중 상속으로 조합:

```cpp
class Sensor<T> : public TFStatic,          // TF 발행 능력
                  protected GenericSensor,  // 주파수 제어 능력
                  public SensorData<T>      // 타입별 토픽 발행 능력
```

다이아몬드 상속 문제를 피하기 위해 각 기반 클래스가 독립적인 멤버를 가지도록 설계되었다.

### 14.3 Type Alias as Factory

```cpp
using Odometry = Sensor<nav_msgs::msg::Odometry>;
// 사실상 Sensor<Odometry>를 Odometry라는 이름으로 팩토리처럼 사용
auto odom = std::make_shared<as2::sensors::Odometry>("odom", this, 50.0);
```

### 14.4 Strategy Pattern (제어 모드)

8비트 인코딩 + 비트마스크로 다양한 제어 전략을 런타임에 선택:
```cpp
// "POSITION 모드이기만 하면 yaw/frame은 상관없다" 형태의 유연한 매칭
compareModes(platform_mode, required_mode, MATCH_CONTROL_MODE);
```

### 14.5 Proxy Pattern (SynchronousServiceClient)

비동기 ROS2 서비스를 동기 인터페이스로 감싸는 프록시:
```cpp
SynchronousServiceClient<SetBool> arming_client("set_arming_state", node);
SetBool::Response resp;
arming_client.sendRequest(req, resp);  // 내부적으로 async + spin
```

### 14.6 Observer Pattern (FSM + Topics)

- 플랫폼 상태 변화 → `platform/info` 토픽 발행 (Observer)
- `alert_event` 토픽 구독 → FSM 이벤트 처리 (Observable)

### 14.7 Compile-Time Polymorphism (노드 기반 클래스)

```cpp
// 매크로로 컴파일 타임에 부모 클래스 결정
#define AS2_NODE_FATHER AS2_RCLCPP_NODE
class Node : public AS2_NODE_FATHER_TYPE { ... }
```

---

## 15. 내부 구현 핵심 로직

### 15.1 제어 모드 변경 시 has_new_references_ 처리

```cpp
bool AerialPlatform::setPlatformControlMode(const ControlMode & msg)
{
    if (ownSetPlatformControlMode(msg)) {
        if (msg.control_mode == ControlMode::HOVER ||
            msg.control_mode == ControlMode::UNSET) {
            has_new_references_ = true;  // HOVER/UNSET은 즉시 명령 전송 시작
        } else {
            has_new_references_ = false; // 새 명령이 구독 콜백으로 도착할 때까지 대기
        }
        platform_info_msg_.current_control_mode = msg;
        return true;
    }
}
```

**설계 의도:** POSITION/SPEED 등 명시적 목표 모드는 새 명령 수신 전까지 sendCommand를 억제한다. HOVER 모드는 별도 명령 없이도 즉시 호버링 명령이 나가야 하므로 예외 처리.

### 15.2 FSM 전이 검색 알고리즘

```cpp
StateMachineTransition PlatformStateMachine::getTransition(
    const int8_t & current_state, const int8_t & event)
{
    StateMachineTransition transition;
    transition.transition_id = -11;  // 유효하지 않음을 나타내는 sentinel 값

    for (const auto & t : transitions_) {
        if (t.from_state_id == current_state && t.transition_id == event) {
            return t;  // 첫 번째 매칭 반환 (중복 없음)
        }
    }
    return transition;  // sentinel 반환 = 유효하지 않은 전이
}
```

순차 검색(O(n))이지만 전이 수가 11개로 고정되어 있어 성능 문제 없음.

### 15.3 SynchronousServiceClient 스레드 안전성

```cpp
// MutuallyExclusive + false(자동 추가 안 함) = 주 executor와 독립
callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);  // ← false가 핵심

// 이 executor만 해당 callback_group을 spin
callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
```

`false` 파라미터 없이 생성하면 주 executor도 이 콜백 그룹을 spin하려 해서 데드락 발생 가능.

### 15.4 GenericSensor 발행 타이밍

```
pub_freq = -1 (즉시 발행 모드):
    updateData() → setData() → dataUpdated() → publishData() [즉시 호출]

pub_freq > 0 (타이머 모드):
    updateData() → setData() → dataUpdated() → [아무것도 하지 않음]
    Timer(1/pub_freq Hz) → timerCallback() → publishData() [주기적 호출]
```

타이머 모드에서는 `dataUpdated()` 호출 간격이 타이머 주기보다 빠르면 가장 최신 데이터만 발행된다.

---

## 16. 주의사항 및 Known Issues

### 16.1 EMERGENCY 상태 복구 불가

FSM에서 EMERGENCY에서 나오는 전이가 정의되지 않았다. 비상 상태 후 복구하려면 노드를 재시작해야 한다.

### 16.2 control_modes_file 필수

`AerialPlatform`은 `control_modes_file` ROS2 파라미터가 없으면 **FATAL 로그 후 소멸자 직접 호출**이라는 비정상적인 방식으로 종료된다:
```cpp
} catch (const rclcpp::ParameterTypeException & e) {
    RCLCPP_FATAL(...);
    this->~AerialPlatform();  // ← 이 패턴은 UB 가능성 있음
}
```

### 16.3 TODO 주석들

코드 내 미완성 부분:
- `aerial_platform.cpp:183`: `// TODO: Check if connected` — connected 상태가 항상 true로 고정
- `aerial_platform.cpp:253,265`: `// TODO: Implement STATE MACHINE check` — takeoff/land 시 FSM 상태 검증 미구현
- `as2_basic_behavior.hpp:99`: `// TODO: explore Timers instead std::thread` — 스레드 기반 실행 개선 예정
- `services.hpp:72,73`: GPS 경로 변환 서비스 이름이 빈 문자열 (`""`) — 미구현

### 16.4 has_new_references_ 동기화

`has_new_references_` 플래그는 구독 콜백(ROS2 executor 스레드)과 `sendCommand()` 타이머 콜백(동일 executor 스레드)에서 접근한다. 단일 스레드 executor 사용 시 문제없지만, 멀티스레드 executor 사용 시 atomic 처리가 필요할 수 있다.

---

## 17. 소스 분석 로드맵

as2_core 이해 후 전체 프레임워크를 분석하는 권장 순서:

```
Phase 0: as2_core (현재 문서) — 완료
         ↓
Phase 1: as2_platform_multirotor_simulator — AerialPlatform 구현 패턴 확인
         ↓
Phase 2: as2_state_estimator — 센서 → 상태 추정 파이프라인
         ↓
Phase 3: as2_motion_reference_handlers — 명령 추상화 계층
         ↓
Phase 4: as2_motion_controller — 제어 알고리즘 (ControllerBase 플러그인)
         ↓
Phase 5: as2_behaviors — 행동 구현 (BasicBehavior 상속 패턴)
         ↓
Phase 6: as2_behavior_tree — BehaviorTree.CPP 기반 임무 시퀀싱
         ↓
Phase 7: as2_python_api — 사용자 인터페이스 (DroneInterface)
```

### 빠른 탐색 팁

```bash
# 특정 토픽의 발행자 찾기
grep -r "self_localization/pose" --include="*.cpp" .

# 특정 서비스의 서버 찾기
grep -r "set_arming_state" --include="*.cpp" .

# 제어 모드 관련 코드 찾기
grep -r "control_mode_utils" --include="*.cpp" .

# 새 플랫폼 구현 시 필요한 순수 가상 메서드 목록
grep -r "= 0;" as2_core/include/as2_core/aerial_platform.hpp
```

---

*작성: 2026-03-26 | 기반: as2_core v1.1.3 실제 소스 코드 직접 분석*
