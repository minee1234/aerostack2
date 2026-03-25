# as2_core 패키지 전문가 분석 및 소스 분석 로드맵

> **패키지 버전:** 1.1.3
> **ROS2 지원:** Galactic / Humble / Jazzy
> **C++ 표준:** C++17
> **라이선스:** BSD-3-Clause
> **관리:** CVAR-UPM (Universidad Politécnica de Madrid)

---

## 목차

1. [패키지 개요](#1-패키지-개요)
2. [아키텍처 다이어그램](#2-아키텍처-다이어그램)
3. [전체 파일 구조](#3-전체-파일-구조)
4. [의존성 그래프](#4-의존성-그래프)
5. [핵심 클래스 계층구조](#5-핵심-클래스-계층구조)
6. [클래스별 상세 분석](#6-클래스별-상세-분석)
7. [네이밍 컨벤션 전체 목록](#7-네이밍-컨벤션-전체-목록)
8. [제어 모드 인코딩 시스템](#8-제어-모드-인코딩-시스템)
9. [설계 패턴 분석](#9-설계-패턴-분석)
10. [소스 분석 로드맵](#10-소스-분석-로드맵)
11. [확장 포인트 및 커스터마이징 가이드](#11-확장-포인트-및-커스터마이징-가이드)

---

## 1. 패키지 개요

`as2_core`는 Aerostack2 무인항공기(UAV) 프레임워크의 **기반 핵심 패키지**로, 모든 다른 Aerostack2 패키지가 의존하는 공통 추상화 계층을 제공한다.

### 핵심 역할

| 역할 | 설명 |
|------|------|
| **노드 추상화** | ROS2 Node/LifecycleNode 위에 AS2 특화 기능 추가 |
| **플랫폼 추상화** | 하드웨어 독립적인 드론 플랫폼 인터페이스 정의 |
| **센서 추상화** | 다양한 센서 타입에 대한 통일된 발행 메커니즘 |
| **행동 추상화** | ROS2 Action 기반 비행 행동(Behavior) 기반 클래스 |
| **상태 기계** | 플랫폼 생명주기(DISARMED→FLYING) 관리 |
| **유틸리티 라이브러리** | TF, GPS, 프레임, 제어 모드 변환 등 |
| **네이밍 표준화** | 토픽/서비스/액션 이름 중앙 관리 |

---

## 2. 아키텍처 다이어그램

```
┌─────────────────────────────────────────────────────────────────┐
│                         as2_core                                │
│                                                                 │
│  ┌──────────────┐  ┌──────────────────┐  ┌─────────────────┐  │
│  │  as2::Node   │  │ as2::BasicBehavior│  │   as2_names     │  │
│  │  (rclcpp 래퍼)│  │  <ActionMsgT>    │  │ (토픽/서비스/액션│  │
│  └──────┬───────┘  └──────────────────┘  │  이름 상수)     │  │
│         │                                └─────────────────┘  │
│  ┌──────▼───────────────────────────────────────────────────┐  │
│  │              as2::AerialPlatform                         │  │
│  │  ┌───────────────────┐  ┌──────────────────────────┐    │  │
│  │  │PlatformStateMachine│  │   Sensor<T> 계층         │    │  │
│  │  │  DISARMED          │  │  Odometry, IMU, GPS ...  │    │  │
│  │  │  LANDED            │  │  Camera, GroundTruth     │    │  │
│  │  │  TAKING_OFF        │  │  Gimbal, RangeFinder     │    │  │
│  │  │  FLYING            │  └──────────────────────────┘    │  │
│  │  │  LANDING           │                                   │  │
│  │  │  EMERGENCY         │  ┌──────────────────────────┐    │  │
│  │  └───────────────────┘  │   Command 구독            │    │  │
│  │                          │  Trajectory/Pose/Twist    │    │  │
│  │                          │  Thrust (cmd_freq Hz)     │    │  │
│  │                          └──────────────────────────┘    │  │
│  └──────────────────────────────────────────────────────────┘  │
│                                                                 │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │                    유틸리티 레이어                       │   │
│  │  TfHandler │ GpsHandler │ frame_utils │ yaml_utils       │   │
│  │  control_mode_utils │ SynchronousServiceClient<T>        │   │
│  │  as2::Rate / as2::WallRate                               │   │
│  └─────────────────────────────────────────────────────────┘   │
│                                                                 │
│  ┌──────────────────────────────────────┐                      │
│  │  Python 바인딩 (pybind11)            │                      │
│  │  as2_names 모듈 (토픽/서비스/액션)   │                      │
│  └──────────────────────────────────────┘                      │
└─────────────────────────────────────────────────────────────────┘
```

---

## 3. 전체 파일 구조

```
as2_core/
├── CMakeLists.txt                          # 빌드 시스템 (as2_core STATIC 라이브러리)
├── package.xml                             # ROS2 패키지 메타데이터 v1.1.3
├── README.md
├── Doxyfile                                # API 문서화 설정
├── CHANGELOG.rst
│
├── include/as2_core/
│   ├── node.hpp                            # ★★★ as2::Node (최상위 기반 클래스)
│   ├── aerial_platform.hpp                 # ★★★ as2::AerialPlatform
│   ├── as2_basic_behavior.hpp              # ★★★ as2::BasicBehavior<MessageT>
│   ├── platform_state_machine.hpp          # ★★  as2::PlatformStateMachine
│   ├── sensor.hpp                          # ★★★ Sensor<T>, Camera, GroundTruth, Gimbal
│   ├── rate.hpp                            # ★   GenericRate<Clock>
│   ├── core_functions.hpp                  # ★   spinLoop()
│   ├── synchronous_service_client.hpp      # ★★  SynchronousServiceClient<T>
│   │
│   ├── custom/
│   │   ├── cv_bridge.hpp.in               # OpenCV 브리지 헤더 (빌드 시 생성)
│   │   └── tf2_geometry_msgs.hpp.in       # TF2 geometry 헤더 (빌드 시 생성)
│   │
│   ├── names/
│   │   ├── topics.hpp                     # ★★★ 모든 토픽 이름/QoS 상수
│   │   ├── services.hpp                   # ★★  모든 서비스 이름 상수
│   │   └── actions.hpp                    # ★★  모든 액션 이름 상수
│   │
│   └── utils/
│       ├── control_mode_utils.hpp         # ★★★ 제어 모드 uint8 인코딩/변환
│       ├── tf_utils.hpp                   # ★★★ TfHandler (tf2 래퍼)
│       ├── frame_utils.hpp                # ★★  쿼터니언/오일러, 벡터 변환
│       ├── gps_utils.hpp                  # ★★  GpsHandler (LLA↔ENU)
│       └── yaml_utils.hpp                 # ★   YAML 파싱 유틸리티
│
├── src/
│   ├── node.cpp                           # Node 구현
│   ├── aerial_platform.cpp                # AerialPlatform 구현 (핵심 로직)
│   ├── platform_state_machine.cpp         # FSM 전이 테이블
│   ├── sensor.cpp                         # Sensor 구현
│   ├── rate.cpp                           # Rate 구현
│   ├── core_functions.cpp                 # spinLoop 구현
│   ├── _as2_core_pybind11.cpp             # Python 바인딩 소스
│   └── utils/
│       ├── control_mode_utils.cpp
│       ├── tf_utils.cpp
│       ├── frame_utils.cpp
│       ├── gps_utils.cpp
│       └── yaml_utils.cpp
│
├── as2_core/                              # Python 패키지
│   ├── __init__.py
│   ├── launch_configuration_from_config_file.py
│   ├── declare_launch_arguments_from_config_file.py
│   ├── launch_param_utils.py
│   └── launch_plugin_utils.py
│
├── tests/
│   ├── CMakeLists.txt
│   ├── platform_state_machine_test.cpp    # FSM 단위 테스트
│   ├── sensor_test.cpp                    # Sensor 단위 테스트
│   ├── frame_test.cpp                     # frame_utils 테스트
│   ├── tf_utils_gtest.cpp                 # TF 변환 테스트
│   ├── tf2_namespace_test.cpp             # TF2 네임스페이스 테스트
│   └── mocks/
│       ├── aerial_platform/
│       │   ├── mock_aerial_platform.hpp   # 테스트용 가상 플랫폼
│       │   └── mock_aerial_platform.cpp
│       └── executor_thread_util/
│           ├── executor_thread_util.hpp   # 테스트 실행자 유틸리티
│           └── executor_thread_util.cpp
│
└── cmake/
    ├── yaml-cpp-extras.cmake              # yaml-cpp 설정 헬퍼
    └── GeographicLib-extras.cmake         # GeographicLib 설정 헬퍼
```

**★ 중요도:** ★★★ 필수 이해 / ★★ 주요 / ★ 참고

---

## 4. 의존성 그래프

```
as2_core
│
├── ROS2 실행 계층
│   ├── rclcpp              # Node, Publisher, Subscriber, Timer, Service
│   ├── rclcpp_lifecycle    # LifecycleNode (선택적)
│   ├── rclcpp_action       # ActionServer (BasicBehavior)
│   └── rcl_interfaces
│
├── ROS2 메시지 타입
│   ├── std_msgs            # Header, Bool 등 기본 타입
│   ├── nav_msgs            # Odometry, Path
│   ├── sensor_msgs         # Imu, NavSatFix, Image, LaserScan 등
│   ├── geometry_msgs       # Pose, Twist, Transform 등
│   ├── as2_msgs            # ★ AS2 전용 메시지/서비스/액션 정의
│   ├── geographic_msgs     # GeoPoint (GPS 관련)
│   └── std_srvs            # SetBool, Trigger
│
├── 변환 라이브러리
│   ├── tf2                 # 변환 데이터 타입
│   ├── tf2_ros             # TransformBroadcaster, Buffer, Listener
│   ├── tf2_geometry_msgs   # ROS 메시지 ↔ tf2 변환
│   └── Eigen3              # 선형 대수 (벡터/행렬 연산)
│
├── 비전/이미지
│   ├── image_transport     # 카메라 이미지 발행 (압축 지원)
│   └── cv_bridge           # ROS Image ↔ OpenCV Mat 변환
│
├── 설정/지리
│   ├── yaml-cpp            # YAML 파라미터 파일 파싱
│   └── GeographicLib       # 고정밀 WGS84 좌표 변환
│
└── 파이썬 바인딩
    └── pybind11            # C++ → Python 모듈 생성
```

---

## 5. 핵심 클래스 계층구조

```
rclcpp::Node
└── as2::Node
    └── as2::AerialPlatform
        └── [각 플랫폼 구현체]
            예) DJIMatrice, ArduPilot, PX4, Gazebo 시뮬레이터

rclcpp_action::ServerBase
└── as2::BasicBehavior<MessageT>
    └── [각 행동 구현체]
        예) TakeoffBehavior, LandBehavior, GoToBehavior

as2::PlatformStateMachine
    (AerialPlatform이 내부적으로 보유)

TFStatic
TFDynamic
SensorData<T>
GenericSensor
├── Sensor<T>               (TFStatic + GenericSensor + SensorData<T>)
│   ├── Odometry            = Sensor<nav_msgs::msg::Odometry>
│   ├── Imu                 = Sensor<sensor_msgs::msg::Imu>
│   ├── GPS                 = Sensor<sensor_msgs::msg::NavSatFix>
│   ├── Lidar               = Sensor<sensor_msgs::msg::LaserScan>
│   ├── Battery             = Sensor<sensor_msgs::msg::BatteryState>
│   ├── Barometer           = Sensor<sensor_msgs::msg::FluidPressure>
│   ├── Compass             = Sensor<sensor_msgs::msg::MagneticField>
│   └── RangeFinder         = Sensor<sensor_msgs::msg::Range>
├── Camera                  (TFStatic + GenericSensor)
├── GroundTruth             (GenericSensor)
└── Gimbal                  (TFStatic + TFDynamic + GenericSensor + SensorData<PoseStamped>)

GenericRate<Clock>
├── as2::Rate               = GenericRate<system_clock>
└── as2::WallRate           = GenericRate<steady_clock>

SynchronousServiceClient<ServiceT>

TfHandler
GpsHandler                  (: GeographicLib::LocalCartesian)
```

---

## 6. 클래스별 상세 분석

### 6.1 as2::Node

**파일:** `include/as2_core/node.hpp`, `src/node.cpp`

**역할:** 모든 AS2 노드의 기반 클래스. `rclcpp::Node`를 확장하여 AS2 특화 기능 추가.

**핵심 기능:**
```cpp
class Node : public rclcpp::Node {
public:
    // 노드 이름 생성 (로컬/글로벌 스코프)
    std::string generate_local_name(const std::string & name);
    // → "node_name/topic_name"

    std::string generate_global_name(const std::string & name);
    // → "topic_name" (앞의 '/' 제거)

    // 주파수 기반 루프 슬립
    void sleep();

    // 타이머 생성
    rclcpp::TimerBase::SharedPtr create_timer(
        double rate_hz,
        std::function<void()> callback);

    // 라이프사이클 콜백 (LifecycleNode 모드 시)
    virtual CallbackReturn on_configure(...);
    virtual CallbackReturn on_activate(...);
    virtual CallbackReturn on_deactivate(...);
    virtual CallbackReturn on_cleanup(...);
    virtual CallbackReturn on_shutdown(...);
    virtual CallbackReturn on_error(...);

private:
    double loop_frequency_;               // ROS2 파라미터 "node_frequency"
    std::shared_ptr<as2::Rate> loop_rate_ptr_;
};
```

**ROS2 파라미터:**
| 이름 | 타입 | 기본값 | 설명 |
|------|------|--------|------|
| `node_frequency` | double | -1.0 | 실행 주파수 (Hz), -1 = 제한 없음 |

---

### 6.2 as2::AerialPlatform

**파일:** `include/as2_core/aerial_platform.hpp`, `src/aerial_platform.cpp`

**역할:** 모든 UAV 플랫폼 드라이버의 추상 기반 클래스.

**순수 가상 메서드 (반드시 구현):**
```cpp
virtual void configureSensors() = 0;
// → 플랫폼의 센서 초기화

virtual bool ownSendCommand() = 0;
// → 제어 명령을 실제 하드웨어로 전송

virtual bool ownSetArmingState(bool state) = 0;
// → ARM/DISARM 명령 처리

virtual bool ownSetOffboardControl(bool offboard) = 0;
// → Offboard 모드 전환 처리

virtual bool ownSetPlatformControlMode(const as2_msgs::msg::ControlMode & msg) = 0;
// → 제어 모드 변경 처리 (POSITION, SPEED, ATTITUDE 등)

virtual void ownKillSwitch() = 0;
// → 긴급 정지

virtual void ownStopPlatform() = 0;
// → 플랫폼 정지
```

**명령 처리 흐름:**
```
[외부 컨트롤러]
    │
    ▼ ROS2 Subscribe
┌─────────────────────────────────────┐
│  command_trajectory_msg_            │  ← "actuator_command/trajectory"
│  command_pose_msg_                  │  ← "actuator_command/pose"
│  command_twist_msg_                 │  ← "actuator_command/twist"
│  command_thrust_msg_                │  ← "actuator_command/thrust"
│           has_new_references_ = true│
└──────────────────┬──────────────────┘
                   │
    Timer (cmd_freq Hz, 기본 100Hz)
                   │
                   ▼
         sendCommand()  [virtual, 오버라이드 가능]
                   │
                   ▼
         ownSendCommand()  [순수 가상]
                   │
                   ▼
         [하드웨어 인터페이스]
```

**제공 서비스:**
```
"set_arming_state"             → std_srvs::srv::SetBool
"set_offboard_mode"            → std_srvs::srv::SetBool
"set_platform_control_mode"    → as2_msgs::srv::SetControlMode
"platform_takeoff"             → std_srvs::srv::SetBool
"platform_land"                → std_srvs::srv::SetBool
"platform/list_control_modes"  → as2_msgs::srv::ListControlModes
```

**ROS2 파라미터:**
| 이름 | 타입 | 기본값 | 설명 |
|------|------|--------|------|
| `cmd_freq` | double | 100.0 | 명령 발행 주파수 (Hz) |
| `info_freq` | double | 10.0 | 플랫폼 정보 발행 주파수 (Hz) |
| `control_modes_file` | string | (필수) | 지원 제어 모드 YAML 파일 경로 |

---

### 6.3 as2::PlatformStateMachine

**파일:** `include/as2_core/platform_state_machine.hpp`, `src/platform_state_machine.cpp`

**상태 전이 다이어그램:**
```
          ARM                    TAKE_OFF
DISARMED ──────► LANDED ──────────────────► TAKING_OFF
   ▲               │  ◄─── LANDED ──────────────────── LANDING
   │DISARM          │                                    │
   │               │           TOOK_OFF                  │LAND
   │               └────────────────────► FLYING ────────┘
   │                                       │
   │                                       │ EMERGENCY
   └───────────────────────────────── EMERGENCY ◄──────── (모든 상태)
```

**상태 열거:**
```cpp
// as2_msgs::msg::PlatformStatus
DISARMED   = 0  // 비무장 상태
LANDED     = 1  // 착지 (무장)
TAKING_OFF = 2  // 이륙 중
FLYING     = 3  // 비행 중
LANDING    = 4  // 착지 중
EMERGENCY  = 5  // 비상 모드
```

**이벤트 열거:**
```cpp
// as2_msgs::msg::PlatformStateMachineEvent
ARM        // 무장
DISARM     // 비무장
TAKE_OFF   // 이륙 시작
TOOK_OFF   // 이륙 완료
LAND       // 착지 시작
LANDED     // 착지 완료
EMERGENCY  // 비상 발생
```

**제공 서비스:**
```
"platform/state_machine_event"  → as2_msgs::srv::SetPlatformStateMachineEvent
```

---

### 6.4 Sensor 계층

**파일:** `include/as2_core/sensor.hpp`, `src/sensor.cpp`

**기반 클래스들:**

```cpp
// TF 정적 변환 발행
class TFStatic {
    void setStaticTransform(const geometry_msgs::msg::TransformStamped & transform);
};

// TF 동적 변환 발행
class TFDynamic {
    void setDynamicTransform(const geometry_msgs::msg::TransformStamped & transform);
};

// 센서 데이터 발행자
template<typename T>
class SensorData {
    void publish(const T & msg);
    rclcpp::Publisher<T>::SharedPtr publisher_;
};

// 주파수 기반 발행 제어
class GenericSensor {
    // pub_freq > 0  → 고정 주파수 발행
    // pub_freq = -1 → updateData() 즉시 발행
};
```

**Sensor<T> API:**
```cpp
template<typename T>
class Sensor : public TFStatic, public GenericSensor, public SensorData<T> {
    void updateData(const T & msg);      // 데이터 업데이트
    void updateAndPublish(const T & msg); // 업데이트 + 즉시 발행
    void publish();                       // 주기적 발행 (타이머에서 호출)
};
```

**Camera 특수 API:**
```cpp
class Camera : public TFStatic, public GenericSensor {
    void updateData(const sensor_msgs::msg::Image & img);
    void updateData(const cv::Mat & img);
    void setCameraInfo(const sensor_msgs::msg::CameraInfo & info);
    void setCameraLinkTransform(
        const std::string & base_link,
        const std::string & sensor_link,
        const geometry_msgs::msg::TransformStamped & transform);
    void readCameraInfoFromROSParameters();
};
```

---

### 6.5 as2::BasicBehavior<MessageT>

**파일:** `include/as2_core/as2_basic_behavior.hpp`

**역할:** ROS2 Action 서버 기반의 행동(Behavior) 구현을 위한 추상 기반 클래스.

```cpp
template<typename MessageT>
class BasicBehavior : public as2::Node {
    using GoalHandleAction = rclcpp_action::ServerGoalHandle<MessageT>;

    // 순수 가상 메서드
    virtual rclcpp_action::GoalResponse onAccepted(
        const std::shared_ptr<const typename MessageT::Goal> goal) = 0;

    virtual rclcpp_action::CancelResponse onCancel(
        const std::shared_ptr<GoalHandleAction> goal_handle) = 0;

    virtual void onExecute(
        const std::shared_ptr<GoalHandleAction> goal_handle) = 0;
    // → 별도 스레드에서 실행 (executor 차단 없음)

private:
    rclcpp_action::Server<MessageT>::SharedPtr action_server_;
};
```

---

### 6.6 TfHandler

**파일:** `include/as2_core/utils/tf_utils.hpp`, `src/utils/tf_utils.cpp`

```cpp
class TfHandler {
public:
    explicit TfHandler(as2::Node * node);

    void setTfTimeoutThreshold(double seconds);

    // 좌표 변환 (예외 발생)
    template<typename T>
    T convert(const T & input, const std::string & target_frame,
              const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(100));

    // 좌표 변환 (bool 반환)
    template<typename T>
    bool tryConvert(T & input, const std::string & target_frame,
                    const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(100));

    // 특정 시점의 포즈 조회
    geometry_msgs::msg::PoseStamped getPoseStamped(
        const std::string & target_frame,
        const std::string & source_frame,
        const tf2::TimePoint & time = tf2::TimePointZero);

    geometry_msgs::msg::TransformStamped getTransform(
        const std::string & target_frame,
        const std::string & source_frame);
};
```

**지원 타입:** `PoseStamped`, `PointStamped`, `Vector3Stamped`, `TwistStamped`, `nav_msgs::msg::Path`, `QuaternionStamped`

---

### 6.7 GpsHandler

**파일:** `include/as2_core/utils/gps_utils.hpp`, `src/utils/gps_utils.cpp`

```cpp
class GpsHandler : private GeographicLib::LocalCartesian {
public:
    // 원점 설정 (최초 1회)
    void setOrigin(double lat0, double lon0, double h0 = 0.0);
    void setOrigin(const sensor_msgs::msg::NavSatFix & fix);
    void getOrigin(double & lat, double & lon, double & h);

    // LLA → Local ENU 변환
    void LatLon2Local(double lat, double lon, double h,
                      double & x, double & y, double & z);
    void LatLon2Local(const sensor_msgs::msg::NavSatFix & fix,
                      geometry_msgs::msg::PoseStamped & ps);

    // Local ENU → LLA 변환
    void Local2LatLon(double x, double y, double z,
                      double & lat, double & lon, double & h);

    // ECEF 변환 (정적 메서드)
    static void LatLon2Ecef(double lat, double lon, double alt,
                             double & x, double & y, double & z);
    static void Ecef2LatLon(double x, double y, double z,
                             double & lat, double & lon, double & alt);

    // 예외
    // OriginNonSet    → setOrigin() 전에 변환 시도
    // OriginAlreadySet → 원점 중복 설정
};
```

---

### 6.8 SynchronousServiceClient<ServiceT>

**파일:** `include/as2_core/synchronous_service_client.hpp`

```cpp
template<class ServiceT>
class SynchronousServiceClient {
public:
    SynchronousServiceClient(const std::string & service_name, as2::Node * node);

    // 동기 서비스 호출 (대기 포함)
    bool sendRequest(const RequestT & req, ResponseT & resp,
                     int wait_time = 0);

    bool sendRequest(const std::shared_ptr<RequestT> & req,
                     std::shared_ptr<ResponseT> & resp,
                     int wait_time = 0);
};
```

**내부 구현:**
- `rclcpp::executors::SingleThreadedExecutor` 사용
- 전용 콜백 그룹 (`MutuallyExclusive`) 으로 스레드 안전성 확보
- `wait_for_service()` 로 서비스 가용성 대기

---

## 7. 네이밍 컨벤션 전체 목록

### 7.1 토픽 (as2_names::topics)

```cpp
// ─── 글로벌 ───────────────────────────────────────────────────────────
topics::global::alert_event         = "alert_event"
topics::global::qos                 = QoS(10)

// ─── 센서 측정값 ───────────────────────────────────────────────────────
topics::sensor_measurements::imu    = "sensor_measurements/imu"
topics::sensor_measurements::lidar  = "sensor_measurements/lidar"
topics::sensor_measurements::gps    = "sensor_measurements/gps"
topics::sensor_measurements::camera = "sensor_measurements/camera"
topics::sensor_measurements::battery= "sensor_measurements/battery"
topics::sensor_measurements::odom   = "sensor_measurements/odom"
topics::sensor_measurements::qos    = SensorDataQoS()

// ─── 그라운드 트루스 ───────────────────────────────────────────────────
topics::ground_truth::pose          = "ground_truth/pose"
topics::ground_truth::twist         = "ground_truth/twist"
topics::ground_truth::qos           = SensorDataQoS()

// ─── 자기 위치추정 ─────────────────────────────────────────────────────
topics::self_localization::odom     = "self_localization/odom"
topics::self_localization::pose     = "self_localization/pose"
topics::self_localization::twist    = "self_localization/twist"
topics::self_localization::qos      = SensorDataQoS()

// ─── 모션 레퍼런스 ─────────────────────────────────────────────────────
topics::motion_reference::pose          = "motion_reference/pose"
topics::motion_reference::twist         = "motion_reference/twist"
topics::motion_reference::trajectory    = "motion_reference/trajectory"
topics::motion_reference::thrust        = "motion_reference/thrust"
topics::motion_reference::modify_waypoint = "motion_reference/modify_waypoint"
topics::motion_reference::traj_gen_info = "motion_reference/traj_gen_info"
topics::motion_reference::qos           = SensorDataQoS()
topics::motion_reference::qos_waypoint  = QoS(10)
topics::motion_reference::qos_trajectory= QoS(10)

// ─── 액추에이터 명령 ───────────────────────────────────────────────────
topics::actuator_command::pose       = "actuator_command/pose"
topics::actuator_command::twist      = "actuator_command/twist"
topics::actuator_command::thrust     = "actuator_command/thrust"
topics::actuator_command::trajectory = "actuator_command/trajectory"
topics::actuator_command::qos        = SensorDataQoS()

// ─── 플랫폼 ───────────────────────────────────────────────────────────
topics::platform::info              = "platform/info"
topics::platform::qos               = QoS(10)

// ─── 컨트롤러 ─────────────────────────────────────────────────────────
topics::controller::info            = "controller/info"
topics::controller::qos_info        = QoS(10)

// ─── 팔로우 타겟 ───────────────────────────────────────────────────────
topics::follow_target::info         = "follow_target/info"
```

### 7.2 서비스 (as2_names::services)

```cpp
// ─── 플랫폼 서비스 ─────────────────────────────────────────────────────
services::platform::set_arming_state            = "set_arming_state"
services::platform::set_offboard_mode           = "set_offboard_mode"
services::platform::set_platform_control_mode   = "set_platform_control_mode"
services::platform::takeoff                     = "platform_takeoff"
services::platform::land                        = "platform_land"
services::platform::set_platform_state_machine_event = "platform/state_machine_event"
services::platform::list_control_modes          = "platform/list_control_modes"

// ─── 컨트롤러 서비스 ───────────────────────────────────────────────────
services::controller::set_control_mode          = "controller/set_control_mode"
services::controller::list_control_modes        = "controller/list_control_modes"

// ─── 모션 레퍼런스 서비스 ─────────────────────────────────────────────
services::motion_reference::send_traj_wayp      = "traj_gen/send_traj_wayp"
services::motion_reference::add_traj_wayp       = "traj_gen/add_traj_wayp"
services::motion_reference::set_traj_speed      = "traj_gen/set_traj_speed"
```

### 7.3 액션 (as2_names::actions)

```cpp
actions::behaviors::takeoff             = "TakeoffBehavior"
actions::behaviors::gotowaypoint        = "GoToBehavior"
actions::behaviors::followreference     = "FollowReferenceBehavior"
actions::behaviors::followpath          = "FollowPathBehavior"
actions::behaviors::land                = "LandBehavior"
actions::behaviors::trajectorygenerator = "TrajectoryGeneratorBehavior"
```

---

## 8. 제어 모드 인코딩 시스템

### 8비트 인코딩 구조

```
비트:  7 6 5 4  |  3 2  |  1 0
       ─────────   ────    ────
       MODE(4)    YAW(2)  FRAME(2)
```

### MODE (상위 4비트)

| 값 | 이진 | 상수 | 설명 |
|----|------|------|------|
| 0 | 0000xxxx | `UNSET` | 미설정 |
| 1 | 0001xxxx | `HOVER` | 호버링 |
| 2 | 0010xxxx | `ACRO` | 아크로배틱 |
| 3 | 0011xxxx | `ATTITUDE` | 자세 제어 |
| 4 | 0100xxxx | `SPEED` | 속도 제어 |
| 5 | 0101xxxx | `SPEED_IN_A_PLANE` | 수평면 속도 제어 |
| 6 | 0110xxxx | `POSITION` | 위치 제어 |
| 7 | 0111xxxx | `TRAJECTORY` | 궤적 추종 |

### YAW (비트 3-2)

| 값 | 이진 | 상수 | 설명 |
|----|------|------|------|
| 0 | xx00xx | `ANGLE` | 각도 기반 |
| 1 | xx01xx | `SPEED` | 속도 기반 |
| 2 | xx10xx | `NONE` | Yaw 제어 없음 |

### FRAME (하위 2비트)

| 값 | 이진 | 상수 | 설명 |
|----|------|------|------|
| 0 | xxxx00 | `LOCAL_FRAME_FLU` | 로컬 FLU 프레임 |
| 1 | xxxx01 | `GLOBAL_FRAME_ENU` | 글로벌 ENU 프레임 |
| 2 | xxxx10 | `GLOBAL_FRAME_LLA` | GPS LLA 좌표 |
| 3 | xxxx11 | `UNDEFINED_FRAME` | 미정의 |

### 주요 유틸리티 함수

```cpp
// 변환
uint8_t convertAS2ControlModeToUint8t(const as2_msgs::msg::ControlMode & mode);
as2_msgs::msg::ControlMode convertUint8tToAS2ControlMode(uint8_t mode_uint8);

// 비교 (마스크 사용)
bool compareModes(uint8_t mode1, uint8_t mode2, uint8_t mask);
// mask: 0b11110000 → MODE만 비교
// mask: 0b11111100 → MODE+YAW 비교
// mask: 0b11111111 → 전체 비교

// 판별
bool isUnsetMode(const as2_msgs::msg::ControlMode & mode);
bool isHoverMode(const as2_msgs::msg::ControlMode & mode);

// 디버그
std::string controlModeToString(const uint8_t & mode);
void printControlMode(const as2_msgs::msg::ControlMode & mode);
```

---

## 9. 설계 패턴 분석

### 9.1 템플릿 메서드 패턴 (Template Method)

`AerialPlatform`이 알고리즘 골격 정의, 서브클래스가 세부 구현:

```
AerialPlatform::sendCommand()    ← 알고리즘 골격 (타이머에서 호출)
    └── ownSendCommand()         ← 서브클래스 구현 (순수 가상)

AerialPlatform::setArmingState() ← 상태 머신 처리
    └── ownSetArmingState()      ← 서브클래스 구현 (순수 가상)
```

### 9.2 전략 패턴 (Strategy)

`GenericRate<Clock>` 에서 시계 타입을 템플릿 파라미터로 주입:
```cpp
as2::Rate      = GenericRate<std::chrono::system_clock>
as2::WallRate  = GenericRate<std::chrono::steady_clock>
```

### 9.3 파사드 패턴 (Facade)

복잡한 라이브러리를 단순화:
```
TfHandler    → tf2_ros 복잡성 추상화
GpsHandler   → GeographicLib 복잡성 추상화
```

### 9.4 타입 안전성 (Type Safety via Templates)

```cpp
Sensor<nav_msgs::msg::Odometry>        // 컴파일 타임 타입 체크
BasicBehavior<as2_msgs::action::TakeOff>
SynchronousServiceClient<as2_msgs::srv::SetControlMode>
```

### 9.5 상태 머신 패턴 (State Machine)

`PlatformStateMachine`에서 전이 테이블 기반 명시적 상태 관리.

### 9.6 다중 상속 믹스인 (Mixin via Multiple Inheritance)

```cpp
class Sensor<T> : public TFStatic, public GenericSensor, public SensorData<T>
class Camera    : public TFStatic, public GenericSensor
class Gimbal    : public TFStatic, public TFDynamic, public GenericSensor, public SensorData<PoseStamped>
```

---

## 10. 소스 분석 로드맵

### Phase 1: 기반 이해 (1-2일)

**목표:** AS2 노드 시스템과 핵심 추상화 파악

**순서:**
```
1. include/as2_core/names/topics.hpp        → 토픽 이름 체계 파악
2. include/as2_core/names/services.hpp      → 서비스 이름 체계 파악
3. include/as2_core/names/actions.hpp       → 액션 이름 체계 파악
4. include/as2_core/node.hpp               → as2::Node 인터페이스 이해
5. src/node.cpp                            → Node 구현 세부 사항
6. include/as2_core/rate.hpp               → 주파수 제어 메커니즘
7. include/as2_core/core_functions.hpp     → spinLoop() 패턴
```

**핵심 질문:**
- `generate_local_name()` vs `generate_global_name()`의 차이는?
- `node_frequency` 파라미터가 어떻게 spinLoop에 영향을 미치는가?

---

### Phase 2: 플랫폼 추상화 (2-3일)

**목표:** 드론 플랫폼 드라이버 구현 방법 이해

**순서:**
```
1. include/as2_core/platform_state_machine.hpp  → FSM 구조 파악
2. src/platform_state_machine.cpp               → 전이 테이블 구현
3. tests/platform_state_machine_test.cpp        → 테스트로 동작 확인
4. include/as2_core/aerial_platform.hpp         → 플랫폼 인터페이스
5. src/aerial_platform.cpp                      → 명령 처리 흐름 분석
6. tests/mocks/aerial_platform/                 → 목업 구현 참조
```

**실습:** Mock 플랫폼 구현 코드(`mock_aerial_platform.cpp`) 분석으로 실제 구현 방법 학습

---

### Phase 3: 센서 시스템 (1-2일)

**목표:** 센서 데이터 발행 메커니즘 이해

**순서:**
```
1. include/as2_core/sensor.hpp     → 전체 Sensor 계층 이해
2. src/sensor.cpp                  → 발행 타이머 로직 분석
3. tests/sensor_test.cpp           → 테스트로 동작 확인
```

**핵심 질문:**
- `pub_freq = -1` vs `pub_freq > 0` 시 발행 시점 차이는?
- `Camera::readCameraInfoFromROSParameters()`는 어떤 파라미터를 읽는가?

---

### Phase 4: 유틸리티 라이브러리 (2-3일)

**목표:** 좌표 변환, GPS, 제어 모드 시스템 이해

**순서:**
```
1. include/as2_core/utils/control_mode_utils.hpp  → 8비트 인코딩 체계
2. src/utils/control_mode_utils.cpp               → 변환 로직
3. include/as2_core/utils/frame_utils.hpp         → 쿼터니언/오일러 변환
4. src/utils/frame_utils.cpp
5. tests/frame_test.cpp                           → 수치 검증
6. include/as2_core/utils/tf_utils.hpp            → TfHandler API
7. src/utils/tf_utils.cpp                         → tf2_ros 래핑 방식
8. tests/tf_utils_gtest.cpp                       → TF 변환 테스트
9. include/as2_core/utils/gps_utils.hpp           → GpsHandler
10. src/utils/gps_utils.cpp                       → GeographicLib 활용
```

---

### Phase 5: 행동 시스템 (1일)

**목표:** ROS2 Action 기반 행동 구현 패턴 이해

**순서:**
```
1. include/as2_core/as2_basic_behavior.hpp  → 템플릿 기반 액션 서버
2. src/_as2_core_pybind11.cpp              → Python 바인딩 확인
```

---

### Phase 6: 빌드 시스템 & 테스트 (1일)

**목표:** CMake 빌드 구성 및 테스트 인프라 이해

**순서:**
```
1. CMakeLists.txt                → 전체 빌드 구성 분석
2. cmake/yaml-cpp-extras.cmake   → 외부 의존성 설정 방식
3. cmake/GeographicLib-extras.cmake
4. tests/CMakeLists.txt          → 테스트 빌드 구성
5. tests/tf2_namespace_test.cpp  → 네임스페이스 처리 테스트
```

---

### Phase 7: Python 레이어 (선택, 0.5일)

**목표:** Python에서 AS2 사용 방법 이해

**순서:**
```
1. as2_core/__init__.py
2. as2_core/launch_param_utils.py
3. as2_core/launch_plugin_utils.py
4. as2_core/launch_configuration_from_config_file.py
```

---

### 분석 우선순위 요약

| 우선순위 | 파일 | 이유 |
|---------|------|------|
| 1 | `names/topics.hpp` | 시스템 전체 토픽 구조 이해 |
| 2 | `node.hpp` + `node.cpp` | 모든 AS2 노드의 기반 |
| 3 | `aerial_platform.hpp` + `.cpp` | 플랫폼 드라이버 인터페이스 |
| 4 | `platform_state_machine.hpp` + `.cpp` | 드론 상태 관리 |
| 5 | `sensor.hpp` + `.cpp` | 센서 데이터 파이프라인 |
| 6 | `utils/control_mode_utils.hpp` | 제어 모드 시스템 |
| 7 | `utils/tf_utils.hpp` | 좌표 변환 인프라 |
| 8 | `as2_basic_behavior.hpp` | 행동 구현 패턴 |
| 9 | `utils/gps_utils.hpp` | GPS 좌표 변환 |
| 10 | `utils/frame_utils.hpp` | 수학 유틸리티 |

---

## 11. 확장 포인트 및 커스터마이징 가이드

### 새 플랫폼 드라이버 구현

```cpp
class MyDronePlatform : public as2::AerialPlatform {
public:
    MyDronePlatform() : as2::AerialPlatform() {
        // 1. 가용 제어 모드 등록
        // control_modes_file 파라미터로 YAML 파일 제공
    }

    // 필수 구현
    void configureSensors() override {
        imu_sensor_ = std::make_unique<as2::sensors::Imu>("imu", this);
        gps_sensor_ = std::make_unique<as2::sensors::GPS>("gps", this);
    }

    bool ownSendCommand() override {
        // command_pose_msg_ 또는 command_twist_msg_ 읽어서 하드웨어 전송
        return true;
    }

    bool ownSetArmingState(bool state) override {
        // 하드웨어 ARM/DISARM
        return true;
    }

    bool ownSetOffboardControl(bool offboard) override {
        // Offboard 모드 전환
        return true;
    }

    bool ownSetPlatformControlMode(const as2_msgs::msg::ControlMode & msg) override {
        // 제어 모드 변경 (supported_modes와 비교 필수)
        return true;
    }

    void ownKillSwitch() override { /* 모터 즉시 정지 */ }
    void ownStopPlatform() override { /* 안전 정지 */ }

private:
    std::unique_ptr<as2::sensors::Imu> imu_sensor_;
    std::unique_ptr<as2::sensors::GPS> gps_sensor_;
};
```

### 새 행동(Behavior) 구현

```cpp
class MyBehavior : public as2::BasicBehavior<as2_msgs::action::MyAction> {
    rclcpp_action::GoalResponse onAccepted(
        const std::shared_ptr<const Goal> goal) override {
        // 목표 유효성 검사
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse onCancel(
        const std::shared_ptr<GoalHandleAction> handle) override {
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void onExecute(
        const std::shared_ptr<GoalHandleAction> handle) override {
        // 별도 스레드에서 실행
        // 주기적으로 feedback 발행
        // 완료 시 result 설정
    }
};
```

### TF 변환 활용

```cpp
as2::TfHandler tf_handler(this);

// 포즈 변환
auto pose_in_world = tf_handler.convert(
    pose_in_body,
    "world",
    std::chrono::milliseconds(100));

// 안전한 변환 (예외 없음)
geometry_msgs::msg::PoseStamped converted_pose = pose_in_body;
if (tf_handler.tryConvert(converted_pose, "world")) {
    // 변환 성공
}
```

### GPS 좌표 변환

```cpp
as2::GPS::GpsHandler gps;

// 원점 설정 (최초 GPS 수신 시)
gps.setOrigin(37.5665, 126.9780, 30.0);  // 위도, 경도, 고도

// 이후 변환
double x, y, z;
gps.LatLon2Local(37.5666, 126.9781, 30.5, x, y, z);
// → 로컬 ENU 좌표 (미터)
```

---

## 부록: 빠른 참조 카드

### 핵심 헤더 인클루드 순서

```cpp
// 1. 기반 노드
#include "as2_core/node.hpp"

// 2. 플랫폼 (드라이버 개발 시)
#include "as2_core/aerial_platform.hpp"

// 3. 행동 (Behavior 개발 시)
#include "as2_core/as2_basic_behavior.hpp"

// 4. 센서
#include "as2_core/sensor.hpp"

// 5. 이름 상수
#include "as2_core/names/topics.hpp"
#include "as2_core/names/services.hpp"
#include "as2_core/names/actions.hpp"

// 6. 유틸리티
#include "as2_core/utils/tf_utils.hpp"
#include "as2_core/utils/frame_utils.hpp"
#include "as2_core/utils/gps_utils.hpp"
#include "as2_core/utils/control_mode_utils.hpp"
```

### 자주 사용하는 타입 별칭

```cpp
as2::sensors::Odometry      = as2::sensors::Sensor<nav_msgs::msg::Odometry>
as2::sensors::Imu           = as2::sensors::Sensor<sensor_msgs::msg::Imu>
as2::sensors::GPS           = as2::sensors::Sensor<sensor_msgs::msg::NavSatFix>
as2::sensors::Lidar         = as2::sensors::Sensor<sensor_msgs::msg::LaserScan>
as2::sensors::Battery       = as2::sensors::Sensor<sensor_msgs::msg::BatteryState>
as2::Rate                   = as2::GenericRate<std::chrono::system_clock>
as2::WallRate               = as2::GenericRate<std::chrono::steady_clock>
```

---

*이 문서는 as2_core v1.1.3 기준으로 작성되었습니다.*
