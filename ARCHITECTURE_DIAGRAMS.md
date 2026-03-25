# Aerostack2 아키텍처 다이어그램

> 이 문서는 Aerostack2 코드베이스의 구조, 메시지 흐름, 모듈 연계를 다이어그램으로 시각화합니다.
> Mermaid 렌더링을 지원하는 뷰어(GitHub, VS Code + Mermaid 플러그인, Obsidian 등)에서 보세요.

---

## 1. 전체 시스템 계층 아키텍처

```mermaid
block-beta
  columns 1

  block:UI["🖥️ 사용자 인터페이스 레이어"]
    A["as2_user_interfaces\n키보드 텔레옵 / RViz 플러그인 / 알파뉴메릭 뷰어"]
  end

  space

  block:API["🐍 Python API 레이어"]
    B["as2_python_api\nDroneInterface / MissionInterpreter / BehaviorManager"]
  end

  space

  block:BT["🌳 Behavior Tree 레이어"]
    C["as2_behavior_tree\nArm · Disarm · Takeoff · Land · GoTo · FollowPath"]
  end

  space

  block:BEH["🎯 동작(Behavior) 레이어"]
    D1["as2_behaviors_motion\nTakeoff · Land · GoTo · FollowPath · FollowReference"]
    D2["as2_behaviors_path_planning\nA* · Voronoi"]
    D3["as2_behaviors_payload\n그리퍼 · 짐벌"]
    D4["as2_behaviors_swarm\nFlocking"]
  end

  space

  block:CTRL["⚙️ 제어 레이어"]
    E1["as2_motion_controller\nControllerManager + Plugin\n(DifferentialFlatness / PID)"]
    E2["as2_motion_reference_handlers\nPosition / Speed / Trajectory / Hover / Acro"]
  end

  space

  block:EST["📡 상태 추정 레이어"]
    F1["as2_state_estimator\n(RawOdom / GroundTruth / MoCap)"]
    F2["as2_map_server\n(Scan2Grid / Depth2Grid / PC2Grid)"]
  end

  space

  block:CORE["🔧 코어 레이어"]
    G["as2_core\nNode · AerialPlatform · Sensor<T> · PlatformStateMachine"]
  end

  space

  block:MSG["📨 메시지 레이어"]
    H["as2_msgs\n15개 Action · 30개+ Message · Services"]
  end

  space

  block:PLAT["🚁 플랫폼 레이어"]
    I1["as2_platform_gazebo\nGazebo 시뮬레이터"]
    I2["as2_platform_multirotor_simulator\n자체 물리 시뮬레이터"]
    I3["실제 드론 플랫폼\n(외부 드라이버)"]
  end
```

---

## 2. 핵심 모듈 블록도 & 연계 구조

```mermaid
graph TB
    subgraph USER["사용자 레이어"]
        PY["🐍 Python DroneInterface<br/>drone.takeoff() / go_to() / land()"]
        BT["🌳 BehaviorTree<br/>XML 트리 정의"]
        KB["⌨️ 키보드 텔레옵"]
    end

    subgraph BEHAV["동작 레이어 (as2_behaviors)"]
        BTO["TakeoffBehavior<br/>Action Server"]
        BGT["GoToBehavior<br/>Action Server"]
        BFP["FollowPathBehavior<br/>Action Server"]
        BLD["LandBehavior<br/>Action Server"]
    end

    subgraph CTRL["제어 레이어"]
        MRH["as2_motion_reference_handlers<br/>PositionMotion / SpeedMotion<br/>TrajectoryMotion / HoverMotion"]
        CM["as2_motion_controller<br/>ControllerManager<br/>↓ pluginlib ↓<br/>DiffFlatness / PID"]
    end

    subgraph EST["추정 레이어"]
        SE["as2_state_estimator<br/>↓ pluginlib ↓<br/>MoCap / GroundTruth / RawOdom"]
        MS["as2_map_server<br/>occupancy grid"]
    end

    subgraph PLAT["플랫폼 레이어"]
        AP["as2_aerial_platforms<br/>Gazebo / MultirotorSim"]
    end

    PY -->|"ROS2 Action Client"| BTO & BGT & BFP & BLD
    BT -->|"BT Action Node"| BTO & BGT & BFP & BLD
    KB -->|"motion_reference/twist"| MRH

    BTO & BGT & BFP & BLD -->|"motion_reference/pose<br/>motion_reference/twist<br/>motion_reference/trajectory"| MRH
    MRH -->|"motion_reference/* topics"| CM
    CM -->|"actuator_command/* topics"| AP
    SE -->|"self_localization/pose<br/>self_localization/twist"| CM
    SE -->|"self_localization/pose"| BTO & BGT & BFP & BLD
    AP -->|"sensor_measurements/*<br/>ground_truth/*"| SE
    MS -->|"occupancy_grid"| BGT & BFP

    style USER fill:#e8f4f8
    style BEHAV fill:#fff3e0
    style CTRL fill:#f3e5f5
    style EST fill:#e8f5e9
    style PLAT fill:#fce4ec
```

---

## 3. 메시지(토픽) 흐름 상세도

```mermaid
flowchart LR
    subgraph PLATFORM["🚁 Aerial Platform"]
        HW["하드웨어 / Gazebo"]
    end

    subgraph SENSORS["📡 센서 & 추정"]
        IMU["sensor_measurements/imu"]
        GPS["sensor_measurements/gps"]
        CAM["sensor_measurements/camera"]
        BATT["sensor_measurements/battery"]
        ODOM["sensor_measurements/odom"]
        GT_P["ground_truth/pose"]
        GT_T["ground_truth/twist"]
        SE["as2_state_estimator"]
        SL_P["self_localization/pose ✅"]
        SL_T["self_localization/twist ✅"]
    end

    subgraph CONTROL["⚙️ 제어 파이프라인"]
        MR_PS["motion_reference/pose"]
        MR_TW["motion_reference/twist"]
        MR_TR["motion_reference/trajectory"]
        MR_TH["motion_reference/thrust"]
        CTRL["as2_motion_controller\n(Plugin)"]
        AC_PS["actuator_command/pose"]
        AC_TW["actuator_command/twist"]
        AC_TR["actuator_command/trajectory"]
        AC_TH["actuator_command/thrust ✅"]
    end

    subgraph BEHAVIOR["🎯 동작 / 참조"]
        MRH["motion_reference_handlers"]
        BEHS["Behaviors\n(Takeoff/Land/GoTo...)"]
    end

    HW -->|"publish"| IMU & GPS & CAM & BATT & ODOM & GT_P & GT_T
    IMU & GPS & ODOM & GT_P & GT_T --> SE
    SE --> SL_P & SL_T

    BEHS --> MRH
    MRH --> MR_PS & MR_TW & MR_TR & MR_TH

    SL_P & SL_T --> CTRL
    MR_PS & MR_TW & MR_TR & MR_TH --> CTRL
    CTRL --> AC_PS & AC_TW & AC_TR & AC_TH

    AC_PS & AC_TW & AC_TR & AC_TH --> HW

    SL_P & SL_T --> BEHS

    style PLATFORM fill:#fce4ec
    style SENSORS fill:#e8f5e9
    style CONTROL fill:#f3e5f5
    style BEHAVIOR fill:#fff3e0
```

---

## 4. 플랫폼 상태 머신 (PlatformStateMachine)

```mermaid
stateDiagram-v2
    [*] --> DISARMED : 시스템 시작

    DISARMED --> LANDED : ARM 이벤트
    LANDED --> DISARMED : DISARM 이벤트

    LANDED --> TAKING_OFF : TAKE_OFF 이벤트
    TAKING_OFF --> FLYING : TOOK_OFF 이벤트
    FLYING --> LANDING : LAND 이벤트
    LANDING --> LANDED : LANDED 이벤트

    DISARMED --> EMERGENCY : EMERGENCY 이벤트
    LANDED --> EMERGENCY : EMERGENCY 이벤트
    TAKING_OFF --> EMERGENCY : EMERGENCY 이벤트
    FLYING --> EMERGENCY : EMERGENCY 이벤트
    LANDING --> EMERGENCY : EMERGENCY 이벤트

    EMERGENCY --> [*] : 시스템 재시작 필요

    note right of FLYING
        모든 제어 명령 수신 가능
        motion_reference/* 토픽 활성
    end note

    note right of EMERGENCY
        킬스위치 / 즉시 정지
        모든 명령 거부
    end note
```

---

## 5. 제어 모드 인코딩 & 파이프라인

```mermaid
graph TB
    subgraph ENCODING["🔢 ControlMode 8비트 인코딩"]
        direction LR
        B7["Bit 7-4\nControl Mode\n0000=UNSET\n0001=HOVER\n0010=POSITION\n0011=SPEED\n0100=SPEED_IN_PLANE\n0101=ATTITUDE\n0110=ACRO\n0111=TRAJECTORY"]
        B3["Bit 3-2\nYaw Mode\n00=NONE\n01=YAW_ANGLE\n10=YAW_SPEED"]
        B1["Bit 1-0\nReference Frame\n00=UNDEFINED\n01=LOCAL_ENU\n10=BODY_FLU\n11=GLOBAL_GPS"]
    end

    subgraph PIPELINE["⚙️ 제어 모드 협상 파이프라인"]
        REQ["사용자 요청\nControlMode"]
        BEH_CHK["Behavior\n지원 모드 확인"]
        CTRL_CHK["Controller\n지원 모드 확인"]
        PLAT_CHK["Platform\n지원 모드 확인"]
        NEG["모드 협상\n(SetControlMode 서비스)"]
        ACT["활성 제어 모드\n적용"]
    end

    REQ --> BEH_CHK --> CTRL_CHK --> PLAT_CHK --> NEG --> ACT

    subgraph TOPICS["📨 모드별 토픽 매핑"]
        TM1["POSITION → motion_reference/pose (PoseStamped)"]
        TM2["SPEED → motion_reference/twist (TwistStamped)"]
        TM3["TRAJECTORY → motion_reference/trajectory (TrajectorySetpoints)"]
        TM4["ATTITUDE/ACRO → motion_reference/thrust (Thrust)"]
        TM5["HOVER → (명령 없음, 유지)"]
    end

    ACT --> TM1 & TM2 & TM3 & TM4 & TM5

    style ENCODING fill:#e3f2fd
    style PIPELINE fill:#f3e5f5
    style TOPICS fill:#e8f5e9
```

---

## 6. 동작(Behavior) 실행 흐름 — 시퀀스 다이어그램

```mermaid
sequenceDiagram
    actor User as 사용자 / Python API
    participant BEH as Behavior Server<br/>(e.g. TakeoffBehavior)
    participant PLUG as Behavior Plugin<br/>(e.g. takeoff_plugin_speed)
    participant MRH as MotionRefHandler<br/>(SpeedMotion)
    participant CTRL as ControllerManager<br/>(DiffFlatness Plugin)
    participant PLAT as AerialPlatform<br/>(Gazebo)
    participant SE as StateEstimator

    User->>BEH: GoalRequest (height=1.0, speed=0.5)
    BEH->>BEH: onAccepted(goal)
    BEH->>PLUG: initialize() via pluginlib

    Note over PLAT,SE: 센서 데이터 지속 발행
    PLAT-->>SE: sensor_measurements/imu, gps, odom
    SE-->>CTRL: self_localization/pose, twist
    SE-->>BEH: self_localization/pose

    loop 제어 루프 (예: 20Hz)
        BEH->>PLUG: onExecute(feedback)
        PLUG->>PLUG: 목표 달성 여부 확인<br/>(현재 고도 vs 목표 고도)
        PLUG->>MRH: sendSpeedCommandWithYawAngle(vz, yaw)
        MRH->>CTRL: motion_reference/twist (TwistStamped)
        CTRL->>CTRL: computeOutput(dt, pose, twist)
        CTRL->>PLAT: actuator_command/thrust (Thrust)
        PLAT->>PLAT: 실제 추력 적용
        BEH-->>User: ActionFeedback (current_height=0.3)
    end

    PLUG->>BEH: SUCCEEDED (height reached)
    BEH-->>User: ActionResult (SUCCESS)
```

---

## 7. 플러그인 아키텍처 구조도

```mermaid
graph TB
    subgraph BEHAVIORS["🎯 Behavior Plugins (pluginlib)"]
        direction TB
        BS["BehaviorServer\n(as2_behaviors_motion)"]
        TP1["takeoff_plugin_speed"]
        TP2["takeoff_plugin_position"]
        TP3["takeoff_plugin_platform"]
        TP4["takeoff_plugin_trajectory"]
        GP1["go_to_plugin_position"]
        GP2["go_to_plugin_trajectory"]
        FP1["follow_path_plugin_position"]
        FP2["follow_path_plugin_trajectory"]
        LP1["land_plugin_speed"]
        LP2["land_plugin_platform"]
        LP3["land_plugin_trajectory"]
        BS -- "ClassLoader\n(plugin_name param)" --> TP1 & TP2 & TP3 & TP4
        BS -- "ClassLoader" --> GP1 & GP2
        BS -- "ClassLoader" --> FP1 & FP2
        BS -- "ClassLoader" --> LP1 & LP2 & LP3
    end

    subgraph CONTROLLERS["⚙️ Controller Plugins (pluginlib)"]
        direction TB
        CM["ControllerManager"]
        CP1["differential_flatness_controller::Plugin\n(비선형 제어)"]
        CP2["pid_speed_controller::Plugin\n(PID 제어)"]
        CM -- "ClassLoader\n(controller_plugin param)" --> CP1 & CP2
    end

    subgraph ESTIMATORS["📡 State Estimator Plugins (pluginlib)"]
        direction TB
        SE["StateEstimator"]
        SEP1["raw_odometry::Plugin"]
        SEP2["ground_truth::Plugin"]
        SEP3["mocap_pose::Plugin\n(OptiTrack)"]
        SEP4["ground_truth_odometry_fuse::Plugin"]
        SE -- "ClassLoader" --> SEP1 & SEP2 & SEP3 & SEP4
    end

    subgraph PLANNERS["🗺️ Path Planning Plugins (pluginlib)"]
        direction TB
        PP["PathPlannerBehavior"]
        PP1["a_star::Plugin"]
        PP2["voronoi::Plugin"]
        PP -- "ClassLoader" --> PP1 & PP2
    end

    subgraph MAPPERS["🗺️ Map Server Plugins (pluginlib)"]
        direction TB
        MP["MapServer"]
        MP1["scan2occ_grid::Plugin"]
        MP2["depth2occ_grid::Plugin"]
        MP3["point_cloud2occ_grid::Plugin"]
        MP -- "ClassLoader" --> MP1 & MP2 & MP3
    end

    style BEHAVIORS fill:#fff3e0
    style CONTROLLERS fill:#f3e5f5
    style ESTIMATORS fill:#e8f5e9
    style PLANNERS fill:#e3f2fd
    style MAPPERS fill:#fce4ec
```

---

## 8. Python API 모듈 구조

```mermaid
classDiagram
    class DroneInterface {
        +drone_id: str
        +verbose: bool
        +use_sim_time: bool
        +spin_rate: float
        +position: [x, y, z]
        +orientation: [roll, pitch, yaw]
        +speed: [vx, vy, vz]
        +info: dict
        +arm() bool
        +disarm() bool
        +offboard() bool
        +manual() bool
        +load_module(pkg) None
        +shutdown() None
    }

    class DroneInterfaceBase {
        -_executor: SingleThreadedExecutor
        -_spin_thread: Thread
        +__subscribers__: list
        +__publishers__: list
        -_platform_info_callback()
        -_pose_callback()
        -_twist_callback()
    }

    class TakeoffModule {
        +__alias__: "takeoff"
        +__deps__: []
        +__call__(height, speed, wait) bool
        +takeoff(height, speed, wait) bool
    }

    class GoToModule {
        +__alias__: "go_to"
        +__deps__: []
        +__call__(position, yaw, speed) bool
        +go_to_point(position, speed) bool
        +go_to_gps(lat, lon, alt) bool
    }

    class FollowPathModule {
        +__alias__: "follow_path"
        +__deps__: []
        +__call__(path, speed) bool
        +follow_path_with_gps(gps_path) bool
    }

    class LandModule {
        +__alias__: "land"
        +__deps__: []
        +__call__(speed, wait) bool
    }

    class MissionInterpreter {
        +drone: DroneInterface
        +plan: MissionPlan
        +execute() None
        +add_item(item: MissionItem) None
    }

    class MissionItem {
        +behavior: str
        +args: dict
        +to_json() str
    }

    DroneInterface --|> DroneInterfaceBase
    DroneInterface *-- TakeoffModule
    DroneInterface *-- GoToModule
    DroneInterface *-- FollowPathModule
    DroneInterface *-- LandModule
    MissionInterpreter --> DroneInterface
    MissionInterpreter *-- MissionItem
```

---

## 9. ROS 2 서비스 & 액션 인터페이스 맵

```mermaid
graph LR
    subgraph SERVICES["🔧 ROS 2 Services (/namespace/...)"]
        S1["set_arming_state\n(std_srvs/SetBool)"]
        S2["set_offboard_mode\n(std_srvs/SetBool)"]
        S3["set_platform_control_mode\n(as2_msgs/SetControlMode)"]
        S4["platform_takeoff\n(std_srvs/SetBool)"]
        S5["platform_land\n(std_srvs/SetBool)"]
        S6["platform/list_control_modes\n(as2_msgs/ListControlModes)"]
        S7["controller/set_control_mode\n(as2_msgs/SetControlMode)"]
        S8["gps/set_origin\ngps/get_origin"]
        S9["traj_gen/send_traj_wayp\ntraj_gen/add_traj_wayp"]
    end

    subgraph ACTIONS["🎬 ROS 2 Actions (/namespace/behaviors/...)"]
        A1["TakeoffBehavior\n(as2_msgs/action/Takeoff)"]
        A2["LandBehavior\n(as2_msgs/action/Land)"]
        A3["GoToBehavior\n(as2_msgs/action/GoToWaypoint)"]
        A4["FollowPathBehavior\n(as2_msgs/action/FollowPath)"]
        A5["FollowReferenceBehavior\n(as2_msgs/action/FollowReference)"]
        A6["TrajectoryGeneratorBehavior\n(as2_msgs/action/TrajectoryGenerator)"]
    end

    subgraph CLIENT["클라이언트"]
        PY["Python API\nDroneInterface"]
        BT2["BehaviorTree\nBT Action Nodes"]
    end

    PY --> S1 & S2 & S3
    PY --> A1 & A2 & A3 & A4 & A5 & A6
    BT2 --> A1 & A2 & A3 & A4

    style SERVICES fill:#e3f2fd
    style ACTIONS fill:#fff3e0
    style CLIENT fill:#e8f5e9
```

---

## 10. 다중 드론(Multi-Robot) 네임스페이스 구조

```mermaid
graph TB
    subgraph NS0["/drone0 네임스페이스"]
        D0_SE["state_estimator"]
        D0_CM["controller_manager"]
        D0_BEH["behaviors\n(takeoff/land/goto...)"]
        D0_PLAT["aerial_platform\n(gazebo)"]
        D0_API["DroneInterface\n(drone_id='drone0')"]
        D0_API --> D0_BEH --> D0_CM --> D0_PLAT
        D0_PLAT --> D0_SE --> D0_CM
    end

    subgraph NS1["/drone1 네임스페이스"]
        D1_SE["state_estimator"]
        D1_CM["controller_manager"]
        D1_BEH["behaviors"]
        D1_PLAT["aerial_platform"]
        D1_API["DroneInterface\n(drone_id='drone1')"]
        D1_API --> D1_BEH --> D1_CM --> D1_PLAT
        D1_PLAT --> D1_SE --> D1_CM
    end

    subgraph SWARM["군집 조정 레이어"]
        SW["as2_behaviors_swarm\nFlockingBehavior"]
        MAPSVR["as2_map_server\n공유 지도"]
    end

    subgraph MISSION["미션 레이어"]
        MI["MissionInterpreter"]
        MI --> D0_API & D1_API
    end

    SW --> D0_API & D1_API
    MAPSVR --> D0_BEH & D1_BEH

    style NS0 fill:#e8f4f8
    style NS1 fill:#f0f4e8
    style SWARM fill:#fff3e0
    style MISSION fill:#fce4ec
```

---

## 11. as2_core 핵심 클래스 다이어그램

```mermaid
classDiagram
    class Node {
        +node_frequency_: double
        +setup_timer(freq, callback)
        +declare_parameter(name, value)
        +get_parameter(name)
    }

    class AerialPlatform {
        +state_machine_: PlatformStateMachine
        +cmd_freq_: double
        +info_freq_: double
        +configureSensors()*
        +ownSendCommand()*
        +ownSetArmingState(bool)*
        +ownSetOffboardControl(bool)*
        +ownSetPlatformControlMode(mode)*
        +ownKillSwitch()*
        +ownStopPlatform()*
        +ownTakeoff() bool
        +ownLand() bool
        -publishPlatformInfo()
        -handleCommandSubscriptions()
    }

    class PlatformStateMachine {
        +current_state_: PlatformStatus
        +process_event(event) bool
        +getState() PlatformStatus
        -state_transitions_: map
    }

    class SensorBase {
        +sensor_ptr_: SharedPtr
        +tf_broadcaster_
        +setStaticTransform(frame, tf)
        +setDynamicTransform(frame, tf)
        +publishTransform()
    }

    class Sensor~T~ {
        +data_: T
        +publisher_: Publisher~T~
        +updateData(T)
        +publishData()
    }

    class BasicBehavior~MessageT~ {
        +action_server_: ActionServer
        +onAccepted(goal)*
        +onCancel(handle)*
        +onExecute(handle)*
    }

    class ControlModeUtils {
        +controlModeToString(mode) string
        +controlModeFromString(str) ControlMode
        +matchControlMode(a, b, mask) bool
        +hover() ControlMode
        +position() ControlMode
        +speed() ControlMode
        +trajectory() ControlMode
    }

    class GpsUtils {
        +lla2enu(lat, lon, alt) [x,y,z]
        +enu2lla(x, y, z) [lat,lon,alt]
        +setOrigin(lat, lon, alt)
    }

    Node <|-- AerialPlatform
    AerialPlatform *-- PlatformStateMachine
    AerialPlatform *-- SensorBase
    Node <|-- BasicBehavior
    SensorBase <|-- Sensor
```

---

## 12. 빌드 의존성 그래프

```mermaid
graph BT
    STD["ROS 2 표준 패키지\nstd_msgs / geometry_msgs\nnav_msgs / sensor_msgs"]
    EXT["외부 라이브러리\nEigen3 / yaml-cpp\ngeographiclib / OpenCV\nBehaviorTree.CPP / pluginlib"]

    MSGS["as2_msgs"]
    CORE["as2_core"]
    MRH["as2_motion_reference_handlers"]
    MC["as2_motion_controller"]
    SE["as2_state_estimator"]
    MS["as2_map_server"]
    BEH["as2_behavior\n(base)"]
    BEHM["as2_behaviors_motion"]
    BEHPP["as2_behaviors_path_planning"]
    BEHPAY["as2_behaviors_payload"]
    BEHS["as2_behaviors_swarm"]
    BT["as2_behavior_tree"]
    PYAPI["as2_python_api"]
    PLAT["as2_aerial_platforms\n(Gazebo / Simulator)"]
    UI["as2_user_interfaces"]
    META["aerostack2\n(메타패키지)"]

    STD --> MSGS
    EXT --> CORE
    MSGS --> CORE
    CORE --> MRH
    CORE --> SE
    CORE --> MS
    CORE --> BEH
    MRH --> MC
    BEH --> BEHM
    BEH --> BEHPP
    BEH --> BEHPAY
    BEH --> BEHS
    MC --> BEHM
    BEHM --> BT
    BEHPP --> BT
    CORE --> PYAPI
    CORE --> PLAT
    BT --> META
    PYAPI --> META
    PLAT --> META
    UI --> META
    SE --> META
    MS --> META

    style STD fill:#e3f2fd
    style EXT fill:#fce4ec
    style META fill:#e8f5e9
    style CORE fill:#fff9c4
```

---

*Generated from Aerostack2 v1.1.3 source analysis — 2026-03-25*
