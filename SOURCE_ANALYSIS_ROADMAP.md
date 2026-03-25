# Aerostack2 소스 분석 로드맵

> 전문가 권장 순서로 Aerostack2 코드베이스를 체계적으로 탐색하는 가이드입니다.
> 각 Phase는 이전 Phase의 이해를 전제로 설계되었습니다.

---

## 전체 순서 한눈에 보기

```
Phase 0   메시지·토픽 이름 파악          ← 언어 학습         (1일)
    ↓
Phase 1   as2_core 코어 프레임워크       ← 기반 이해         (2~3일)
    ↓
Phase 2   AerialPlatform 구현체         ← "하드웨어 쪽"     (1~2일)
    ↓
Phase 3   State Estimator               ← "센서 → 상태"     (1일)
    ↓
Phase 4   Motion Reference Handlers     ← "명령 추상화"     (1일)
    ↓
Phase 5   Motion Controller             ← "핵심 제어 로직"  (2일)
    ↓
Phase 6   Behaviors                     ← "고수준 동작"     (2~3일)
    ↓
Phase 7   Behavior Tree                 ← "조합·시퀀싱"     (1일)
    ↓
Phase 8   Python API                    ← "사용자 인터페이스" (1일)
    ↓
Phase 9   Test Code                     ← "이해 검증"       (수시)
```

---

## Phase 0 — 전체 맥락 파악 (1일)

코드를 열기 전에 개념 지도를 먼저 잡습니다.

> **왜?** 메시지·토픽·서비스 이름을 먼저 알면, 이후 코드에서
> `self_localization/pose`가 보일 때 즉시 의미를 파악할 수 있습니다.

### 읽을 파일

```
aerostack2/package.xml
```
- 메타패키지: 전체 패키지 목록 한눈에 파악

```
ARCHITECTURE_DIAGRAMS.md
```
- 전체 구조 다이어그램으로 구조 숙지

```
as2_msgs/msg/*.msg
as2_msgs/action/*.action
as2_msgs/srv/*.srv
```
- 전체 메시지 타입 훑기 (시스템이 주고받는 데이터 언어 파악)
- 15개 액션 정의 (시스템이 할 수 있는 것)

```
as2_core/include/as2_core/names/topics.hpp
as2_core/include/as2_core/names/services.hpp
as2_core/include/as2_core/names/actions.hpp
```
- 모든 토픽·서비스·액션 이름을 상수로 정의한 파일
- 코드 탐색 시 grep 기준점으로 계속 활용

### 체크포인트

- [ ] 토픽 이름 구조(`self_localization/`, `motion_reference/`, `actuator_command/`)를 외울 것
- [ ] ControlMode 메시지의 3가지 필드(control_mode, yaw_mode, reference_frame) 이해
- [ ] 15개 Action의 목적을 한 줄씩 설명할 수 있을 것

---

## Phase 1 — 코어 프레임워크 (2~3일)

가장 먼저 깊이 이해해야 할 기반입니다.
이 Phase를 충분히 이해하지 않으면 이후 모든 코드가 맥락 없이 보입니다.

### 1-1. 노드 추상화

```
as2_core/include/as2_core/node.hpp
as2_core/src/node.cpp
```

**파악할 것:**
- `as2::Node`가 `rclcpp::Node`를 어떻게 감싸는지
- `setup_timer(freq, callback)` 주파수 관리 방식
- 파라미터 선언 패턴 (`node_frequency` 파라미터)

### 1-2. 제어 모드 시스템 ← **핵심 중 핵심**

```
as2_msgs/msg/ControlMode.msg
as2_core/include/as2_core/utils/control_mode_utils/control_mode_utils.hpp
as2_core/src/utils/control_mode_utils/control_mode_utils.cpp
```

**8비트 인코딩 구조:**

```
[ Bit 7-4 : Control Mode ] [ Bit 3-2 : Yaw Mode ] [ Bit 1-0 : Reference Frame ]

Control Mode:
  0000 = UNSET           0001 = HOVER
  0010 = POSITION        0011 = SPEED
  0100 = SPEED_IN_PLANE  0101 = ATTITUDE
  0110 = ACRO            0111 = TRAJECTORY

Yaw Mode:
  00 = NONE   01 = YAW_ANGLE   10 = YAW_SPEED

Reference Frame:
  00 = UNDEFINED   01 = LOCAL_ENU   10 = BODY_FLU   11 = GLOBAL_GPS
```

**파악할 것:**
- `matchControlMode(a, b, mask)` 비트마스크 로직
- `MATCH_ALL`, `MATCH_CONTROL_MODE` 등 마스크 상수
- 이것을 모르면 컨트롤러·플랫폼 코드가 전혀 읽히지 않음

### 1-3. 플랫폼 상태 머신

```
as2_core/include/as2_core/platform_state_machine.hpp
as2_core/src/platform_state_machine.cpp
as2_msgs/msg/PlatformStatus.msg
as2_msgs/msg/PlatformStateMachineEvent.msg
```

**상태 전이 테이블:**

```
DISARMED    + ARM       → LANDED
LANDED      + DISARM    → DISARMED
LANDED      + TAKE_OFF  → TAKING_OFF
TAKING_OFF  + TOOK_OFF  → FLYING
FLYING      + LAND      → LANDING
LANDING     + LANDED    → LANDED
(모든 상태) + EMERGENCY  → EMERGENCY
```

**파악할 것:**
- 이벤트 트리거 방식 (`process_event()`)
- 잘못된 상태에서 명령이 오면 어떻게 거부하는지

### 1-4. AerialPlatform 베이스

```
as2_core/include/as2_core/aerial_platform.hpp
as2_core/src/aerial_platform.cpp
```

**파악할 것:**
- 순수 가상 메서드 목록 → 구현 계약 파악
  ```cpp
  virtual void configureSensors() = 0;
  virtual bool ownSendCommand() = 0;
  virtual bool ownSetArmingState(bool state) = 0;
  virtual bool ownSetOffboardControl(bool offboard) = 0;
  virtual bool ownSetPlatformControlMode(const ControlMode &mode) = 0;
  virtual void ownKillSwitch() = 0;
  virtual void ownStopPlatform() = 0;
  ```
- 구독 토픽: `motion_reference/trajectory`, `pose`, `twist`, `thrust`
- 발행 토픽: `platform/info`
- 서비스: `set_arming_state`, `set_offboard_mode`, `set_platform_control_mode`

### 1-5. 센서 추상화

```
as2_core/include/as2_core/sensor.hpp
as2_core/src/sensor.cpp
```

**파악할 것:**
- `Sensor<T>` 템플릿 패턴 — 어떤 메시지 타입도 감쌀 수 있는 구조
- TF(좌표 변환) 발행 메커니즘
- 카메라 센서 특수화 (`image_transport` 사용)

### 1-6. Behavior 베이스

```
as2_core/include/as2_core/as2_basic_behavior.hpp
```

**파악할 것:**
- `BasicBehavior<MessageT>` 템플릿 구조
- `onAccepted` / `onExecute` / `onCancel` 생명주기
- Action Server 래핑 방식

### 1-7. 유틸리티 (필요시 참조)

```
as2_core/include/as2_core/utils/tf_utils/tf_utils.hpp
as2_core/include/as2_core/utils/gps_utils/gps_utils.hpp
as2_core/include/as2_core/utils/frame_utils/frame_utils.hpp
as2_core/include/as2_core/utils/yaml_utils/yaml_utils.hpp
```

---

## Phase 2 — 플랫폼 구현체 (1~2일)

추상 베이스가 실제로 어떻게 구현되는지 확인합니다.

> **전략:** Gazebo보다 단순한 Multirotor Simulator를 먼저 읽습니다.
> 외부 시뮬레이터 연동 없이 순수 ROS 2 코드만으로 구성되어 있어 구조 파악이 쉽습니다.

### 2-1. Multirotor Simulator (먼저)

```
as2_aerial_platforms/as2_platform_multirotor_simulator/
  include/as2_platform_multirotor_simulator/as2_platform_multirotor_simulator.hpp
  src/as2_platform_multirotor_simulator.cpp
```

**파악할 것:**
- Phase 1-4에서 파악한 순수 가상 메서드들의 실제 구현
- `ownSendCommand()` → 물리 모델에 명령 전달 경로
- `configureSensors()` — `Sensor<T>` 인스턴스 생성 및 등록

### 2-2. Gazebo 플랫폼 (이후)

```
as2_aerial_platforms/as2_platform_gazebo/
  include/as2_platform_gazebo/as2_platform_gazebo.hpp
  src/as2_platform_gazebo.cpp
```

**파악할 것:**
- Gazebo 플러그인과 ROS 2 브리지 방식
- Multirotor Simulator 대비 추가 복잡도

### 체크포인트

- [ ] `ownSendCommand()`에서 `actuator_command/*` 토픽을 발행하는 지점 확인
- [ ] Phase 1-4의 AerialPlatform과 완전히 연결되었는지 확인

---

## Phase 3 — 상태 추정기 (1일)

센서 → 추정 상태 파이프라인을 이해합니다.

### 읽을 파일

```
as2_state_estimator/
  include/as2_state_estimator/state_estimator_base.hpp   ← 플러그인 인터페이스
  src/state_estimator.cpp                                ← 매니저

  plugins/raw_odometry/       ← 가장 단순: 외부 오도메트리 직접 사용
  plugins/ground_truth/       ← 두 번째: 시뮬레이터 ground truth 사용
  plugins/mocap_pose/         ← 세 번째: OptiTrack 모션캡처
  plugins/ground_truth_odometry_fuse/  ← 융합 예시
```

**파악할 것:**
- 입력: `sensor_measurements/*`, `ground_truth/*`
- 출력: `self_localization/pose`, `self_localization/twist`
- TF 브로드캐스트 (`odom` → `base_link` 프레임)
- 플러그인 등록 방식 (`plugins.xml` 함께 확인)

### 체크포인트

- [ ] `raw_odometry` 플러그인 전체 흐름 추적
- [ ] `self_localization/pose`가 발행되는 정확한 라인 찾기

---

## Phase 4 — 모션 레퍼런스 핸들러 (1일)

Behavior → Controller 사이의 제어 명령 추상화 레이어입니다.

### 읽을 파일

```
as2_motion_reference_handlers/include/as2_motion_reference_handlers/
  basic_motion_reference_handler.hpp   ← 베이스 클래스 먼저
  position_motion.hpp                  ← POSITION 모드
  speed_motion.hpp                     ← SPEED 모드
  trajectory_motion.hpp               ← TRAJECTORY 모드
  hover_motion.hpp                    ← HOVER 모드
  acro_motion.hpp                     ← ACRO 모드
  speed_in_a_plane_motion.hpp         ← SPEED_IN_PLANE 모드

as2_motion_reference_handlers/src/
  basic_motion_reference_handler.cpp
  position_motion.cpp
  speed_motion.cpp
  ...
```

**파악할 것:**

| 핸들러 | 발행 토픽 | 메시지 타입 |
|--------|-----------|-------------|
| PositionMotion | `motion_reference/pose` | PoseStamped |
| SpeedMotion | `motion_reference/twist` | TwistStamped |
| TrajectoryMotion | `motion_reference/trajectory` | TrajectorySetpoints |
| HoverMotion | (없음, 상태 유지) | — |
| AcroMotion | `motion_reference/thrust` | Thrust |

- 정적 공유 퍼블리셔(`static shared_ptr`) 패턴과 그 이유
  (여러 핸들러 인스턴스가 동일 토픽에 중복 발행 방지)

### 체크포인트

- [ ] `SpeedMotion::sendSpeedCommandWithYawAngle()` 호출 → 토픽 발행까지 추적

---

## Phase 5 — 모션 컨트롤러 (2일)

시스템에서 가장 복잡한 부분입니다.
제어 이론 배경지식이 있으면 플러그인 이해에 도움이 됩니다.

### 5-1. 인터페이스 먼저

```
as2_motion_controller/include/as2_motion_controller/controller_base.hpp
```

**순수 가상 메서드 (플러그인 계약):**
```cpp
virtual bool ownInitialize() = 0;
virtual void updateState(const PoseStamped &pose, const TwistStamped &twist) = 0;
virtual void updateReference(const PoseStamped &ref) = 0;
virtual void updateReference(const TwistStamped &ref) = 0;
virtual void updateReference(const TrajectorySetpoints &ref) = 0;
virtual void updateReference(const Thrust &ref) = 0;
virtual bool computeOutput(double dt, PoseStamped &pose, TwistStamped &twist, Thrust &thrust) = 0;
virtual bool setMode(const ControlMode &mode_in, const ControlMode &mode_out) = 0;
virtual bool updateParams(const std::vector<rclcpp::Parameter> &params) = 0;
virtual void reset() = 0;
```

**입력/출력 토픽:**
- 입력: `self_localization/pose`, `self_localization/twist`
- 입력: `motion_reference/pose`, `twist`, `trajectory`, `thrust`
- 출력: `actuator_command/pose`, `twist`, `trajectory`, `thrust`

### 5-2. 핸들러 (구독~발행 배선)

```
as2_motion_controller/include/as2_motion_controller/controller_handler.hpp
as2_motion_controller/src/controller_handler.cpp
```

**파악할 것:**
- 모드 협상 로직 (플랫폼 지원 모드 vs 컨트롤러 지원 모드)
- 프레임 변환 (ENU ↔ FLU) 자동 처리
- `platform/info` 구독으로 플랫폼 상태 감시

### 5-3. 매니저

```
as2_motion_controller/src/controller_manager.cpp
```

**파악할 것:**
- pluginlib `ClassLoader`로 플러그인 로딩 과정
- `controller_plugin` 파라미터로 런타임 선택

### 5-4. 실제 플러그인 구현

```
as2_motion_controller/plugins/differential_flatness_controller/
as2_motion_controller/plugins/pid_speed_controller/
```

- `computeOutput()` 구현 — 제어 알고리즘 핵심
- `setMode()` — 지원 가능한 입출력 모드 조합 선언

### 체크포인트

- [ ] `motion_reference/twist` 수신 → `computeOutput()` → `actuator_command/thrust` 발행까지 전체 흐름 추적

---

## Phase 6 — 동작(Behavior) 시스템 (2~3일)

가장 많은 코드가 있지만, Phase 1~5를 이해했다면 패턴이 반복됩니다.

### 6-1. Behavior 베이스

```
as2_behaviors/as2_behavior/include/as2_behavior/behavior_server.hpp
as2_behaviors/as2_behavior/src/behavior_server.cpp
```

**파악할 것:**
- `BehaviorServer<ActionT>` — `BasicBehavior`의 확장
- Goal 수락/거부 정책
- 피드백 발행 주기

### 6-2. 가장 단순한 동작부터 — Takeoff

```
as2_behaviors/as2_behaviors_motion/takeoff_behavior/
  include/takeoff_behavior/takeoff_behavior.hpp
  src/takeoff_behavior.cpp                      ← Behavior Server (플러그인 로더)

  plugins/takeoff_plugin_speed/                 ← 가장 단순: SpeedMotion 사용
    src/takeoff_plugin_speed.cpp
  plugins/takeoff_plugin_position/              ← PositionMotion 사용
  plugins/takeoff_plugin_trajectory/            ← TrajectoryMotion 사용
  plugins/takeoff_plugin_platform/              ← 플랫폼 서비스 호출
```

**읽기 순서:**
1. `takeoff_behavior.cpp` — 플러그인을 어떻게 로드하는지
2. `takeoff_plugin_speed.cpp` — `onExecute()` 루프에서 SpeedMotion 사용 패턴
3. `plugins.xml` — 플러그인 등록 선언

### 6-3. 복잡한 동작으로 확장

```
as2_behaviors/as2_behaviors_motion/go_to_behavior/
  plugins/go_to_plugin_position/
  plugins/go_to_plugin_trajectory/

as2_behaviors/as2_behaviors_motion/follow_path_behavior/
  plugins/follow_path_plugin_position/
  plugins/follow_path_plugin_trajectory/

as2_behaviors/as2_behaviors_motion/land_behavior/
  plugins/land_plugin_speed/
  plugins/land_plugin_platform/
  plugins/land_plugin_trajectory/

as2_behaviors/as2_behaviors_motion/follow_reference_behavior/
```

### 6-4. 파라미터 추정 동작

```
as2_behaviors/as2_behaviors_param_estimation/
  mass_estimation/
  force_estimation/
```

### 6-5. 경로 계획 (선택)

```
as2_behaviors/as2_behaviors_path_planning/
  include/as2_behaviors_path_planning/path_planner_behavior.hpp
  plugins/a_star/
  plugins/voronoi/
```

### 6-6. 페이로드·군집 (선택)

```
as2_behaviors/as2_behaviors_payload/
  gripper_behavior/plugins/two_fingers/
  gripper_behavior/plugins/dc_servo/
  gimbal_behavior/

as2_behaviors/as2_behaviors_swarm/
  flocking_behavior/
```

### 체크포인트

- [ ] `takeoff_plugin_speed`의 `onExecute()` 루프에서 `self_localization/pose`를 읽고 목표 고도 비교 후 SpeedMotion 발행까지 추적
- [ ] `plugins.xml` → CMakeLists.txt → pluginlib 로딩 연결 확인

---

## Phase 7 — Behavior Tree (1일)

개별 동작을 조합하여 복잡한 임무를 구성하는 방법을 이해합니다.

### 읽을 파일

```
as2_behavior_tree/include/as2_behavior_tree/
  action/takeoff_action.hpp         ← BT Action 노드 구조 (가장 단순)
  action/go_to_action.hpp
  action/follow_path_action.hpp
  action/land_action.hpp
  condition/is_flying_condition.hpp ← BT Condition 노드
  decorator/wait_for_event_decorator.hpp  ← 이벤트 대기

as2_behavior_tree/src/
  as2_behavior_tree_node.cpp        ← 트리 실행 진입점

as2_behavior_tree/trees/
  *.xml                             ← 트리 정의 XML 예시
```

**파악할 것:**
- BT Action 노드가 ROS 2 Action Client를 어떻게 래핑하는지
- `tick()` 메서드 → `RUNNING` / `SUCCESS` / `FAILURE` 반환 로직
- Groot XML 형식으로 트리 정의하는 방법
- `WaitForEvent` — `alert_event` 토픽을 BT에서 사용하는 방식

### 체크포인트

- [ ] XML 트리에서 `<TakeoffAction>` 노드가 실행되어 `TakeoffBehavior` Action Server에 Goal을 보내는 흐름 추적

---

## Phase 8 — Python API (1일)

C++ 구현 위에 올라간 고수준 인터페이스를 이해합니다.

### 읽을 파일

```
as2_python_api/as2_python_api/
  drone_interface_base.py              ← 구독/발행 설정, spin 스레드
  drone_interface.py                   ← 최종 사용자 인터페이스

  behavior_actions/
    takeoff_behavior.py                ← Action 클라이언트 래퍼
    go_to_behavior.py
    follow_path_behavior.py
    land_behavior.py
    trajectory_generation_behavior.py

  modules/
    takeoff_module.py                  ← Mixin 패턴 (drone.takeoff() 구현)
    go_to_module.py
    follow_path_module.py
    land_module.py

  mission/
    mission_interpreter.py
    mission_item.py
    behavior_manager.py
```

**파악할 것:**
- `DroneInterfaceBase` — 단일 스레드 Executor + spin 스레드 구조
- `DroneInterface` — 모든 모듈을 Mixin으로 합성
- `TakeoffModule.__call__()` → `TakeoffBehavior.start_behavior()` → ROS 2 Action Goal
- `MissionInterpreter` — JSON/YAML 미션 파일 파싱 및 실행

**사용 예시 패턴:**
```python
drone = DroneInterface(drone_id='drone0', use_sim_time=True)
drone.arm()
drone.offboard()
drone.takeoff(height=1.0, speed=0.5)          # TakeoffBehavior Action
drone.go_to([1.0, 0.0, 1.0], max_speed=1.0)  # GoToBehavior Action
drone.land(speed=0.3)                          # LandBehavior Action
drone.shutdown()
```

### 체크포인트

- [ ] `drone.takeoff(height=1.0)` 호출 → Python Action Client → C++ TakeoffBehavior → Plugin → SpeedMotion → Controller → Platform까지 전체 체인 추적

---

## Phase 9 — 테스트 코드 (수시로)

> **테스트는 "이 모듈이 어떻게 사용되어야 하는가"의 공식 예시입니다.**
> 각 Phase 후 해당 패키지의 테스트 코드를 읽으면 이해를 검증할 수 있습니다.

### 테스트 파일 위치

```
as2_core/tests/
as2_behaviors/as2_behaviors_motion/tests/
as2_behaviors/as2_behaviors_path_planning/tests/
as2_aerial_platforms/as2_platform_multirotor_simulator/tests/
as2_hardware_drivers/usb_camera_driver/tests/
as2_hardware_drivers/realsense2_camera_driver/tests/
```

### 활용 방법

1. 각 Phase 완료 후 → 해당 패키지 테스트 코드 읽기
2. 테스트가 모킹(mocking)하는 부분 → 모듈 경계를 확인
3. 테스트 파라미터 → 실제 운용 값 참고

---

## 핵심 원칙 3가지

### 1. 인터페이스(`.hpp`) → 구현(`.cpp`) 순으로

헤더에서 계약(메서드 시그니처, 구독/발행 토픽)을 먼저 파악한 뒤
구현 세부사항을 봅니다.
헤더만 읽어도 모듈의 역할을 80% 이해할 수 있습니다.

### 2. 플러그인은 베이스 클래스 먼저

```
ControllerBase → DifferentialFlatnessController (X)
ControllerBase → DifferentialFlatnessController (O, 순서 지킴)
```

베이스를 모르면 플러그인 코드는 맥락 없는 함수 모음입니다.
반드시 `*_base.hpp` → `plugins.xml` → 구체 구현 순으로 읽으세요.

### 3. 토픽을 기준점으로 코드 탐색

토픽 이름으로 grep 검색하면 발행자(Publisher)와 구독자(Subscriber)를 즉시 찾을 수 있습니다.

```bash
# self_localization/pose를 발행하는 쪽 찾기
grep -r "self_localization/pose" --include="*.cpp" .

# self_localization/pose를 구독하는 쪽 찾기
grep -r "self_localization/pose" --include="*.hpp" .
```

발행자 → 구독자를 연결하면 모듈 간 경계와 데이터 흐름이 자연스럽게 보입니다.

---

## 추가 참고 자료

| 자료 | 위치 |
|------|------|
| 전체 아키텍처 다이어그램 | `ARCHITECTURE_DIAGRAMS.md` |
| 각 패키지 README | `{package}/README.md` |
| 플러그인 등록 선언 | `{package}/plugins.xml` |
| 빌드 의존성 | `{package}/CMakeLists.txt`, `package.xml` |
| 제어 모드 설정 예시 | `{package}/config/control_modes.yaml` |
| 런치 파일 예시 | `{package}/launch/*.launch.py` |

---

*Generated from Aerostack2 v1.1.3 source analysis — 2026-03-25*
