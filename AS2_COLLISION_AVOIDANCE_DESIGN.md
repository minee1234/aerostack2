# 충돌 회피를 위한 실시간 상호 위치 공유 — 설계 문서

> Aerostack2 코드베이스 분석 기반 · 2026-04-14

---

## 1. 설계 전제 조건 분석

### 기존 코드베이스에서 재활용 가능한 요소

| 요소 | 파일 | 재활용 방법 |
|---|---|---|
| `PoseStampedWithIDArray` | `as2_msgs/msg/` | Fleet 상태 전송 메시지 |
| `AlertEvent` (FORCE_HOVER / EMERGENCY_HOVER) | `as2_msgs/msg/AlertEvent.msg` | 충돌 위험 시 비상 명령 발행 |
| `SensorDataQoS` | `as2_core/names/topics.hpp:54` | 실시간 위치 토픽 QoS |
| `as2::Node` 기반 클래스 | `as2_core/node.hpp` | 신규 노드 통합 기반 |
| 크로스 네임스페이스 구독 패턴 | `drone_swarm.cpp:47-50` | 검증된 타 드론 구독 방법 |
| `alert_event` 토픽 | `topics.hpp:50` | 비상 신호 채널 |

### 설계 제약 조건

```
실시간 요구사항:
  위치 갱신 주기: ≥ 20 Hz (50ms 이내 반응)
  충돌 감지 레이턴시: < 100ms (경보 → 회피 명령)
  안전 이격 거리: 설정값 (예: warning=2.0m, critical=1.0m)

확장성 요구사항:
  드론 수 증가 시 O(N²) 토픽 연결 방지
  드론 추가/이탈이 런타임에 가능해야 함

내결함성 요구사항:
  FleetAggregator 노드 장애 시 → 개별 드론 독립 동작 유지
  타 드론 위치 데이터 유실 시 → last-known 위치 기반 보수적 판단
```

---

## 2. 아키텍처 비교 및 선택

### 방식 A: 완전 분산 (P2P 직접 구독)

```
drone0 ──► /drone0/self_localization/pose ──► drone1, drone2, drone3
drone1 ──► /drone1/self_localization/pose ──► drone0, drone2, drone3
drone2 ──► /drone2/self_localization/pose ──► drone0, drone1, drone3
drone3 ──► /drone3/self_localization/pose ──► drone0, drone1, drone2

토픽 연결 수: N × (N-1) = O(N²)
N=10 → 90개 연결
N=20 → 380개 연결
```

**장점:** SPOF 없음  
**단점:** N² 연결, 런타임 드론 추가 시 전체 재구성 필요

---

### 방식 B: 중앙 집중형 Aggregator

```
모든 드론 → FleetAggregator → /fleet/state → 모든 드론
토픽 연결 수: 2N = O(N)
```

**장점:** O(N) 연결, 단순  
**단점:** Aggregator 장애 = 전체 fleet 상태 블라인드

---

### 방식 C: 계층형 하이브리드 ★ 권장

```
1차: FleetStateAggregator (집중 수집, O(N))
2차: 각 드론의 CollisionMonitor (분산 판단, 로컬 실행)
3차: AlertEvent 채널 (기존 비상 인프라 재활용)
백업: 직접 구독 폴백 (Aggregator 장애 시 자동 전환)
```

**선택 근거:**
- 연결 수 O(N) — 효율적
- CollisionMonitor가 드론 로컬에서 실행 — 네트워크 단절 시에도 동작
- 기존 `AlertEvent` 인프라 재활용 — 플랫폼 FSM이 자동 처리
- as2 패턴(Node, BehaviorServer) 그대로 사용 — 통합 비용 최소

---

## 3. 전체 시스템 블럭도

```
┌──────────────────────────────────────────────────────────────────────────────┐
│                        COLLISION AVOIDANCE SYSTEM                            │
│                                                                              │
│  ┌──────────────────────────────────────────────────────────────────────┐   │
│  │                    FleetStateAggregator  (신규, 전역 1개)             │   │
│  │                                                                      │   │
│  │  /drone0/self_localization/pose ──►┐                                 │   │
│  │  /drone1/self_localization/pose ──►┤                                 │   │
│  │  /drone2/self_localization/pose ──►┼──► PoseStampedWithIDArray      │   │
│  │  /droneN/self_localization/pose ──►┘    발행: /fleet/state  (20Hz+) │   │
│  │                                                                      │   │
│  │  /drone0/platform/info ──►┐                                          │   │
│  │  /drone1/platform/info ──►┤──► FleetStatusArray                     │   │
│  │  /droneN/platform/info ──►┘    발행: /fleet/status                  │   │
│  └────────────────────────────────────┬─────────────────────────────────┘   │
│                                       │ /fleet/state  (SensorDataQoS)       │
│                                       │ /fleet/status (QoS reliable)        │
│                    ┌──────────────────┼──────────────────┐                  │
│                    │                  │                  │                  │
│          ┌─────────▼──────┐ ┌─────────▼──────┐ ┌────────▼───────┐          │
│          │    drone0      │ │    drone1      │ │    drone2      │          │
│          │                │ │                │ │                │          │
│          │ ┌────────────┐ │ │ ┌────────────┐ │ │ ┌────────────┐ │          │
│          │ │Collision   │ │ │ │Collision   │ │ │ │Collision   │ │          │
│          │ │Monitor     │ │ │ │Monitor     │ │ │ │Monitor     │ │          │
│          │ │(신규 노드)  │ │ │ │(신규 노드)  │ │ │ │(신규 노드)  │ │          │
│          │ │            │ │ │ │            │ │ │ │            │ │          │
│          │ │sub: /fleet/│ │ │ │sub: /fleet/│ │ │ │sub: /fleet/│ │          │
│          │ │state       │ │ │ │state       │ │ │ │state       │ │          │
│          │ │            │ │ │ │            │ │ │ │            │ │          │
│          │ │→ 거리 계산  │ │ │ │→ 거리 계산  │ │ │ │→ 거리 계산  │ │          │
│          │ │→ 속도 예측  │ │ │ │→ 속도 예측  │ │ │ │→ 속도 예측  │ │          │
│          │ │→ 위험 판단  │ │ │ │→ 위험 판단  │ │ │ │→ 위험 판단  │ │          │
│          │ └─────┬──────┘ │ │ └─────┬──────┘ │ │ └─────┬──────┘ │          │
│          │       │        │ │       │        │ │       │        │          │
│          │  alert_event   │ │  alert_event   │ │  alert_event   │          │
│          │  FORCE_HOVER / │ │  FORCE_HOVER / │ │  FORCE_HOVER / │          │
│          │  EMERGENCY_    │ │  EMERGENCY_    │ │  EMERGENCY_    │          │
│          │  HOVER         │ │  HOVER         │ │  HOVER         │          │
│          │       │        │ │       │        │ │       │        │          │
│          │       ▼        │ │       ▼        │ │       ▼        │          │
│          │  AerialPlatform│ │  AerialPlatform│ │  AerialPlatform│          │
│          │  (FSM 처리)    │ │  (FSM 처리)    │ │  (FSM 처리)    │          │
│          └────────────────┘ └────────────────┘ └────────────────┘          │
└──────────────────────────────────────────────────────────────────────────────┘
```

---

## 4. 토픽 및 메시지 설계

### 신규 토픽 정의

```
/fleet/state        (PoseStampedWithIDArray, SensorDataQoS)
  └─ 전체 드론 위치·시각 배열, 20~50Hz 갱신

/fleet/status       (FleetStatusArray [신규 메시지], QoS reliable)
  └─ 각 드론 PlatformStatus + armed/offboard 상태, 5Hz

/fleet/proximity    (ProximityWarning [신규 메시지], QoS reliable)
  └─ CollisionMonitor → 외부 로깅/시각화용 (경보 기록)
```

### 신규 메시지 정의

**`as2_msgs/msg/FleetAgentState.msg`** — Aggregator 내부용

```
# 단일 드론의 실시간 상태 (위치+속도+플랫폼 상태)
string              drone_id
geometry_msgs/PoseStamped   pose          # 위치 + 시각
geometry_msgs/TwistStamped  twist         # 속도 (예측용)
as2_msgs/PlatformStatus     status        # FLYING/LANDED 등
builtin_interfaces/Time     last_update   # 데이터 신선도 판단
```

**`as2_msgs/msg/ProximityWarning.msg`** — 경보 이벤트 기록

```
# 충돌 위험 경보
string  self_drone_id
string  threat_drone_id
float32 distance_m           # 현재 거리 (m)
float32 predicted_distance_m # 1초 후 예측 거리 (m)
int8    WARNING  = 1         # warning_radius 이내
int8    CRITICAL = 2         # critical_radius 이내
int8    level
builtin_interfaces/Time stamp
```

---

## 5. FleetStateAggregator 구현 설계

```
파일: as2_utilities/as2_fleet_state_aggregator/
  ├── include/as2_fleet_state_aggregator/fleet_state_aggregator.hpp
  ├── src/fleet_state_aggregator.cpp
  ├── src/fleet_state_aggregator_node.cpp
  ├── launch/fleet_state_aggregator.launch.py
  └── package.xml
```

### 클래스 구조

```cpp
class FleetStateAggregator : public as2::Node
{
public:
  FleetStateAggregator();

private:
  // 파라미터
  std::vector<std::string> drone_namespaces_;  // ["drone0","drone1","drone2"]
  double publish_rate_hz_;                     // 기본 50Hz
  double stale_timeout_sec_;                   // 0.5s 이상 갱신 없으면 stale

  // 각 드론별 구독자
  std::unordered_map<std::string,
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> pose_subs_;
  std::unordered_map<std::string,
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr> twist_subs_;
  std::unordered_map<std::string,
    rclcpp::Subscription<as2_msgs::msg::PlatformInfo>::SharedPtr> info_subs_;

  // 캐시 (최신 상태 유지)
  std::unordered_map<std::string, as2_msgs::msg::FleetAgentState> fleet_cache_;
  std::mutex cache_mutex_;

  // 발행자
  rclcpp::Publisher<as2_msgs::msg::PoseStampedWithIDArray>::SharedPtr fleet_state_pub_;
  rclcpp::Publisher<as2_msgs::msg::FleetStatusArray>::SharedPtr fleet_status_pub_;

  // 타이머
  rclcpp::TimerBase::SharedPtr publish_timer_;

  // 메서드
  void setupSubscriptions();
  void poseCallback(const std::string & drone_id,
                    const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void twistCallback(const std::string & drone_id,
                     const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void publishFleetState();
  bool isStale(const std::string & drone_id) const;
};
```

### 구독 설정 패턴 (drone_swarm.cpp 패턴 확장)

```cpp
void FleetStateAggregator::setupSubscriptions()
{
  for (const auto & drone_id : drone_namespaces_) {
    // 위치 구독 (cross-namespace, SensorDataQoS)
    auto pose_sub = create_subscription<geometry_msgs::msg::PoseStamped>(
      drone_id + "/" + as2_names::topics::self_localization::pose,
      as2_names::topics::self_localization::qos,
      [this, drone_id](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        poseCallback(drone_id, msg);
      });
    pose_subs_[drone_id] = pose_sub;

    // 속도 구독 (충돌 예측용)
    auto twist_sub = create_subscription<geometry_msgs::msg::TwistStamped>(
      drone_id + "/" + as2_names::topics::self_localization::twist,
      as2_names::topics::self_localization::qos,
      [this, drone_id](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        twistCallback(drone_id, msg);
      });
    twist_subs_[drone_id] = twist_sub;
  }
}
```

---

## 6. CollisionMonitor 구현 설계

```
파일: as2_behaviors/as2_behaviors_safety/
  ├── collision_monitor/
  │   ├── include/collision_monitor/collision_monitor.hpp
  │   ├── src/collision_monitor.cpp
  │   └── src/collision_monitor_node.cpp
  └── package.xml
```

### 클래스 구조

```cpp
class CollisionMonitor : public as2::Node
{
public:
  CollisionMonitor();

private:
  // 파라미터
  std::string self_drone_id_;      // 이 노드가 속한 드론 네임스페이스
  float warning_radius_m_;         // 기본 2.0m
  float critical_radius_m_;        // 기본 1.0m
  float prediction_horizon_sec_;   // 1.0s 예측
  float stale_timeout_sec_;        // 0.5s 신선도 임계값
  float check_rate_hz_;            // 20Hz

  // 구독
  rclcpp::Subscription<as2_msgs::msg::PoseStampedWithIDArray>::SharedPtr fleet_state_sub_;

  // 발행
  rclcpp::Publisher<as2_msgs::msg::AlertEvent>::SharedPtr alert_pub_;
  rclcpp::Publisher<as2_msgs::msg::ProximityWarning>::SharedPtr proximity_pub_;

  // 상태 캐시
  std::unordered_map<std::string, AgentState> fleet_cache_;  // id → {pose, twist, timestamp}

  // 타이머 기반 주기 판단
  rclcpp::TimerBase::SharedPtr check_timer_;

  // 핵심 메서드
  void fleetStateCallback(const as2_msgs::msg::PoseStampedWithIDArray::SharedPtr msg);
  void checkCollisions();
  float computeDistance(const std::string & other_id) const;
  float predictDistance(const std::string & other_id, float horizon_sec) const;
  void handleWarning(const std::string & threat_id, float dist, float pred_dist);
  void handleCritical(const std::string & threat_id, float dist, float pred_dist);
  void publishAlert(int8_t alert_type, const std::string & description);
};
```

### 충돌 판단 로직

```cpp
void CollisionMonitor::checkCollisions()
{
  for (auto & [drone_id, state] : fleet_cache_) {
    if (drone_id == self_drone_id_) continue;

    // 신선도 확인
    auto age = (now() - state.timestamp).seconds();
    if (age > stale_timeout_sec_) {
      // 0.5초 이상 데이터 없음 → 보수적으로 마지막 위치 유지
      RCLCPP_WARN(get_logger(), "Stale data for %s (%.2fs)", drone_id.c_str(), age);
      continue;
    }

    float dist_now  = computeDistance(drone_id);
    float dist_pred = predictDistance(drone_id, prediction_horizon_sec_);

    if (dist_now < critical_radius_m_ || dist_pred < critical_radius_m_) {
      handleCritical(drone_id, dist_now, dist_pred);       // EMERGENCY_HOVER 발행
    } else if (dist_now < warning_radius_m_ || dist_pred < warning_radius_m_) {
      handleWarning(drone_id, dist_now, dist_pred);        // FORCE_HOVER 발행
    }
  }
}

float CollisionMonitor::predictDistance(
  const std::string & other_id, float horizon) const
{
  // 현재 위치 + 속도 × 시간 → 미래 위치 예측
  auto & self  = fleet_cache_.at(self_drone_id_);
  auto & other = fleet_cache_.at(other_id);

  // 선형 예측 (단순, 저레이턴시)
  float sx = self.pose.pose.position.x  + self.twist.twist.linear.x  * horizon;
  float sy = self.pose.pose.position.y  + self.twist.twist.linear.y  * horizon;
  float sz = self.pose.pose.position.z  + self.twist.twist.linear.z  * horizon;
  float ox = other.pose.pose.position.x + other.twist.twist.linear.x * horizon;
  float oy = other.pose.pose.position.y + other.twist.twist.linear.y * horizon;
  float oz = other.pose.pose.position.z + other.twist.twist.linear.z * horizon;

  return std::sqrt((sx-ox)*(sx-ox) + (sy-oy)*(sy-oy) + (sz-oz)*(sz-oz));
}
```

### 경보 발행 (기존 AlertEvent 인프라 연결)

```cpp
void CollisionMonitor::handleCritical(
  const std::string & threat_id, float dist, float pred_dist)
{
  // 기존 as2 AlertEvent 채널 사용 → AerialPlatform FSM이 자동 처리
  as2_msgs::msg::AlertEvent alert;
  alert.alert       = as2_msgs::msg::AlertEvent::EMERGENCY_HOVER;  // = -2
  alert.description = "CRITICAL: " + threat_id +
                      " dist=" + std::to_string(dist) + "m" +
                      " pred=" + std::to_string(pred_dist) + "m";
  alert_pub_->publish(alert);

  // 로깅/시각화용 ProximityWarning도 발행
  as2_msgs::msg::ProximityWarning warn;
  warn.self_drone_id        = self_drone_id_;
  warn.threat_drone_id      = threat_id;
  warn.distance_m           = dist;
  warn.predicted_distance_m = pred_dist;
  warn.level                = as2_msgs::msg::ProximityWarning::CRITICAL;
  warn.stamp                = now();
  proximity_pub_->publish(warn);
}
```

---

## 7. 데이터 플로우 상세

```
  주기별 타임라인 (50Hz 기준 = 20ms 주기)

  T+0ms   /droneN/self_localization/pose 발행 (각 드론)
          │
  T+1ms   FleetStateAggregator pose_callback 호출 → cache 갱신
          │
  T+2ms   FleetStateAggregator publish_timer 발동
          │
          └─► /fleet/state 발행 (PoseStampedWithIDArray)
                │
  T+3ms         └─► 각 drone의 CollisionMonitor fleet_state_callback 수신
                │
  T+4ms         └─► check_timer 발동 (20Hz) → 거리 계산
                │   ├─ dist(drone0, drone1) = 1.5m → CRITICAL
                │   └─ dist(drone0, drone2) = 3.2m → OK
                │
  T+5ms         └─► alert_event 발행 (EMERGENCY_HOVER)
                │
  T+6ms         └─► AerialPlatform.alertEventCallback → FSM 전환
                │
  T+7ms         └─► 드론 호버링 시작

  총 레이턴시: ~7ms (목표 100ms 대비 충분한 여유)
```

---

## 8. Aggregator 장애 시 폴백 설계

```
┌─────────────────────────────────────────────────────────────────┐
│                    FALLBACK 메커니즘                              │
│                                                                 │
│  정상 모드:                                                       │
│  [droneN] ──► FleetAggregator ──► /fleet/state ──► [CollisionMonitor]
│                                                                 │
│  장애 감지 (watchdog 타이머):                                     │
│  CollisionMonitor가 /fleet/state를 N초 동안 수신 못하면 →         │
│  fallback_mode = true                                           │
│                                                                 │
│  폴백 모드 (P2P 직접 구독):                                       │
│  [drone0 CollisionMonitor]                                      │
│    ──► /drone1/self_localization/pose (직접 구독)               │
│    ──► /drone2/self_localization/pose (직접 구독)               │
│                                                                 │
│  폴백 모드 비용:                                                   │
│  - Aggregator 장애는 드문 이벤트이므로 일시적 O(N²) 허용          │
│  - 장애 복구 시 자동 정상 모드 복귀                               │
└─────────────────────────────────────────────────────────────────┘
```

### CollisionMonitor 내 Watchdog 구현

```cpp
// 초기화 시
fallback_watchdog_timer_ = create_timer(
  std::chrono::seconds(2),    // 2초 미수신 시 폴백
  [this]() { activateFallbackMode(); });

// fleet_state 수신 시마다 리셋
void fleetStateCallback(...) {
  fallback_watchdog_timer_->reset();   // watchdog 리셋
  if (fallback_mode_) {
    deactivateFallbackMode();           // 정상 복귀
  }
  // ... 데이터 처리
}
```

---

## 9. 런타임 드론 추가/이탈 처리

```
방법: 파라미터 서버 + ROS 2 노드 발견 (DDS)

FleetStateAggregator 옵션 A (정적):
  launch 파라미터로 drone_namespaces 고정
  → 단순하지만 런타임 변경 불가

FleetStateAggregator 옵션 B (동적) ★ 권장:
  /fleet/register  (Service: RegisterDrone.srv)
  /fleet/unregister (Service: UnregisterDrone.srv)

  새 드론 기동 시:
  1. 드론 노드가 /fleet/register 서비스 호출
  2. FleetStateAggregator가 새 구독자 동적 생성
  3. FleetStatusArray에 자동 포함

  드론 이탈 시:
  1. 드론 노드 종료 전 /fleet/unregister 호출
  2. 또는 stale_timeout 초과 시 자동 제거 (watchdog)
```

**`as2_msgs/srv/RegisterDrone.srv`** (신규)

```
string drone_id          # 등록할 드론 네임스페이스
float32 safety_radius_m  # 이 드론의 물리적 크기 기반 안전 반경
---
bool success
string message
```

---

## 10. 파라미터 설계

### FleetStateAggregator

```yaml
# fleet_state_aggregator_params.yaml
fleet_state_aggregator:
  ros__parameters:
    drone_namespaces: ["drone0", "drone1", "drone2"]
    publish_rate_hz: 50.0          # 50Hz 발행
    stale_timeout_sec: 0.5         # 0.5s 이상 갱신 없으면 stale 표시
    use_twist: true                # twist 데이터 수집 여부 (예측 정확도 향상)
```

### CollisionMonitor (드론별)

```yaml
# drone0_collision_monitor_params.yaml
collision_monitor:
  ros__parameters:
    warning_radius_m: 2.0          # 경고 반경 (FORCE_HOVER)
    critical_radius_m: 1.0         # 위험 반경 (EMERGENCY_HOVER)
    prediction_horizon_sec: 1.0    # 충돌 예측 시간 (1초 후 위치)
    check_rate_hz: 20.0            # 거리 검사 주기
    fleet_state_timeout_sec: 2.0   # Aggregator watchdog 임계값
    enable_prediction: true        # 속도 기반 예측 사용
    alert_cooldown_sec: 0.5        # 동일 드론 경보 쿨다운 (중복 방지)
```

---

## 11. 전체 패키지 배치도

```
as2_utilities/
└── as2_fleet_state_aggregator/         ← 신규 패키지
    ├── include/
    │   └── fleet_state_aggregator.hpp
    ├── src/
    │   ├── fleet_state_aggregator.cpp
    │   └── fleet_state_aggregator_node.cpp
    ├── launch/
    │   └── fleet_state_aggregator.launch.py
    ├── config/
    │   └── fleet_state_aggregator_params.yaml
    └── package.xml
        └── depends: as2_core, as2_msgs, geometry_msgs

as2_behaviors/
└── as2_behaviors_safety/               ← 신규 패키지
    ├── collision_monitor/
    │   ├── include/
    │   │   └── collision_monitor.hpp
    │   ├── src/
    │   │   ├── collision_monitor.cpp
    │   │   └── collision_monitor_node.cpp
    │   ├── launch/
    │   │   └── collision_monitor.launch.py
    │   └── config/
    │       └── collision_monitor_params.yaml
    └── package.xml
        └── depends: as2_core, as2_msgs, as2_behavior

as2_msgs/
├── msg/
│   ├── FleetAgentState.msg             ← 신규
│   └── ProximityWarning.msg            ← 신규
└── srv/
    └── RegisterDrone.srv               ← 신규 (동적 등록용)
```

---

## 12. 시스템 시작 순서

```
1. FleetStateAggregator 기동
   ros2 launch as2_fleet_state_aggregator fleet_state_aggregator.launch.py
   drone_namespaces:="[drone0,drone1,drone2]"

2. 각 드론 스택 기동 (기존 as2 기동과 동일)
   - Platform → StateEstimator → MotionController → Behaviors

3. CollisionMonitor 기동 (드론별)
   ros2 launch as2_behaviors_safety collision_monitor.launch.py
   namespace:=drone0

4. 미션 실행
   python3 mission.py  ← 기존 Python API 그대로 사용
```

---

## 13. 구현 우선순위 및 단계

```
Phase 1 — 핵심 경보 (1~2주)
  [x] FleetAgentState.msg / ProximityWarning.msg 메시지 정의
  [x] FleetStateAggregator 기본 구현 (정적 드론 목록, 위치만)
  [x] CollisionMonitor 기본 구현 (거리 계산 + AlertEvent 발행)
  [x] 검증: 3대 드론 시뮬레이터 환경

Phase 2 — 예측 + 안정성 (2~3주)
  [ ] CollisionMonitor에 속도 기반 충돌 예측 추가
  [ ] Aggregator watchdog + P2P 폴백 모드
  [ ] alert_cooldown 로직 (경보 중복 방지)

Phase 3 — 동적 관리 (1~2주)
  [ ] RegisterDrone 서비스 기반 런타임 드론 추가/이탈
  [ ] /fleet/status 토픽 (플랫폼 상태 집계)
  [ ] RViz 시각화 플러그인 (proximity sphere 표시)
```

---

## 14. 기존 인프라 재활용 요약

```
재활용 항목                    파일                           재활용 방식
─────────────────────────────  ─────────────────────────────  ──────────────────────────────
AlertEvent 메시지              as2_msgs/msg/AlertEvent.msg    EMERGENCY_HOVER 코드 그대로 사용
PoseStampedWithIDArray         as2_msgs/msg/                  /fleet/state 발행 타입
SensorDataQoS                  as2_core/names/topics.hpp:54   /fleet/state QoS 재사용
AerialPlatform.alertCallback   as2_core/aerial_platform.cpp   신규 코드 불필요, 자동 처리
cross-namespace 구독 패턴      drone_swarm.cpp:47-50          FleetAggregator에 동일 패턴 적용
as2::Node 기반 클래스          as2_core/node.hpp              신규 노드 모두 이걸로 상속
```

---

*이 설계는 Aerostack2 코드베이스(`drone_swarm.cpp`, `AlertEvent.msg`, `topics.hpp` 등) 직접 분석에 기반하며, 기존 패턴을 최대한 재활용하여 통합 비용을 최소화한다.*
