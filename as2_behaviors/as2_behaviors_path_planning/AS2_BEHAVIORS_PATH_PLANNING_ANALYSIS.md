# as2_behaviors_path_planning 패키지 상세 분석

**버전**: 1.1.3
**라이선스**: BSD-3-Clause

---

## 1. 패키지 개요

`as2_behaviors_path_planning`은 **경로 계획(Path Planning)** 기능을 제공하는 behavior 패키지다.
Occupancy Grid Map을 기반으로 장애물 회피 경로를 계산하고, `FollowPathBehavior`에 위임하여 실행하는 2계층 구조를 갖는다.

### 핵심 특징
- `pluginlib` 기반 경로 계획 알고리즘 교체 가능
- A* 알고리즘과 Voronoi Diagram 2가지 플러그인 제공
- FollowPathBehavior에 대한 Action Client 내장
- Pause/Resume 시 FollowPathBehavior도 연동하여 일시정지

---

## 2. 파일 구조

```
as2_behaviors_path_planning/
├── include/as2_behaviors_path_planning/
│   ├── path_planner_behavior.hpp        # 메인 behavior 클래스
│   └── path_planner_plugin_base.hpp     # 플러그인 추상 기반 클래스
├── src/
│   ├── path_planner_behavior.cpp        # 구현
│   └── path_planner_behavior_node.cpp   # main 진입점
├── plugins/
│   ├── a_star/
│   │   ├── include/
│   │   │   ├── a_star.hpp              # A* 플러그인 클래스
│   │   │   ├── a_star_algorithm.hpp    # A* 알고리즘 구현
│   │   │   └── a_star_searcher.hpp     # 탐색 상태 관리
│   │   ├── src/
│   │   │   ├── a_star.cpp
│   │   │   └── a_star_searcher.cpp
│   │   ├── launch/
│   │   │   └── a_star-path_planner_behavior.launch.py
│   │   └── CMakeLists.txt
│   └── voronoi/
│       ├── include/
│       │   ├── voronoi.hpp             # Voronoi 플러그인 클래스
│       │   └── voronoi_searcher.hpp    # Voronoi 탐색
│       ├── src/
│       │   ├── voronoi.cpp
│       │   └── voronoi_searcher.cpp
│       ├── thirdparty/dynamicvoronoi/  # 오픈소스 Voronoi 라이브러리
│       │   ├── dynamicvoronoi.h
│       │   ├── dynamicvoronoi.cpp
│       │   ├── bucketedqueue.h
│       │   └── bucketedqueue.hxx
│       ├── launch/
│       │   └── voronoi-path_planner_behavior.launch.py
│       └── CMakeLists.txt
├── common/
│   └── include/
│       ├── cell_node.hpp               # 그리드 셀 노드 (f, g, h 비용)
│       ├── graph_searcher.hpp          # 템플릿 기반 그래프 탐색 기반
│       └── utils.hpp                  # 유틸리티 함수
├── config/
│   └── behavior_default.yaml
├── launch/
│   ├── path_planner_behavior_launch.py
│   └── composable_path_planner_behavior.launch.py
├── tests/
│   └── behavior_test.py
├── plugins.xml
├── CMakeLists.txt
└── package.xml
```

---

## 3. 의존성

```xml
<depend>rclcpp</depend>
<depend>rclcpp_action</depend>
<depend>pluginlib</depend>
<depend>as2_behavior</depend>
<depend>as2_core</depend>
<depend>as2_msgs</depend>
<depend>geometry_msgs</depend>
<depend>nav_msgs</depend>        <!-- OccupancyGrid -->
<depend>visualization_msgs</depend>
<depend>tf2_ros</depend>
<depend>std_srvs</depend>
```

---

## 4. `PathPlannerBehavior` 클래스

**파일**: `include/as2_behaviors_path_planning/path_planner_behavior.hpp`

```cpp
class PathPlannerBehavior
  : public as2_behavior::BehaviorServer<as2_msgs::action::NavigateToPoint>
{
  // ── Behavior 상태 변수 ───────────────────────────────────────
  as2_msgs::action::NavigateToPoint::Goal goal_;
  as2_msgs::action::NavigateToPoint::Feedback feedback_;
  as2_msgs::action::NavigateToPoint::Result result_;
  as2_msgs::msg::YawMode yaw_mode_;

  // ── 경로 계획 변수 ──────────────────────────────────────────
  bool enable_visualization_ = false;
  bool enable_path_optimizer_ = false;
  geometry_msgs::msg::PoseStamped drone_pose_;
  double safety_distance_ = 1.0;           // 드론 크기 근사값 [m]
  std::vector<geometry_msgs::msg::Point> path_;  // 플러그인이 채우는 경로

  // ── 플러그인 시스템 ──────────────────────────────────────────
  std::filesystem::path plugin_name_;
  std::shared_ptr<pluginlib::ClassLoader<as2_behaviors_path_planning::PluginBase>> loader_;
  std::shared_ptr<as2_behaviors_path_planning::PluginBase> path_planner_plugin_;

  // ── FollowPath 연동 ─────────────────────────────────────────
  rclcpp_action::Client<as2_msgs::action::FollowPath>::SharedPtr follow_path_client_;
  std::shared_ptr<const as2_msgs::action::FollowPath::Feedback> follow_path_feedback_;
  bool navigation_aborted_ = false;
  bool follow_path_rejected_ = false;
  bool follow_path_succeeded_ = false;

  // FollowPath Pause/Resume 서비스 클라이언트
  as2::SynchronousServiceClient<std_srvs::srv::Trigger>::SharedPtr follow_path_pause_client_;
  as2::SynchronousServiceClient<std_srvs::srv::Trigger>::SharedPtr follow_path_resume_client_;

  // ── 드론 상태 ───────────────────────────────────────────────
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr drone_pose_sub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ── 시각화 ─────────────────────────────────────────────────
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr viz_pub_;
};
```

---

## 5. Behavior 실행 흐름

### 5.1 `on_activate()`

```
1. path_planner_plugin_->on_activate(drone_pose_, goal)
   └── 플러그인이 OccupancyGrid 수신 + 경로 계산
   └── 결과를 path_planner_plugin_->path_에 저장

2. (enable_visualization_) → viz_pub_ 발행

3. follow_path_client_->async_send_goal({path: plugin->path_})
   └── FollowPathBehavior에 계산된 경로 전달

4. 콜백 등록:
   - follow_path_response_cbk: goal_handle 저장
   - follow_path_feedback_cbk: 피드백 미러링
   - follow_path_result_cbk:   완료 여부 기록
```

### 5.2 `on_run()`

```
1. navigation_aborted_ → ABORTED
2. follow_path_rejected_ → FAILURE
3. follow_path_succeeded_ → SUCCESS
4. (아직 실행 중) → 피드백 미러링 → RUNNING
```

### 5.3 `on_pause()`

```
follow_path_pause_client_->sendRequest()
└── FollowPathBehavior/_behavior/pause 서비스 호출
```

### 5.4 `on_resume()`

```
follow_path_resume_client_->sendRequest()
└── FollowPathBehavior/_behavior/resume 서비스 호출
```

---

## 6. `PluginBase` 인터페이스

**파일**: `include/as2_behaviors_path_planning/path_planner_plugin_base.hpp`

```cpp
namespace as2_behaviors_path_planning {

class PluginBase {
public:
  // 필수 구현
  virtual void initialize(as2::Node* node_ptr, std::shared_ptr<tf2_ros::Buffer> tf_buffer) = 0;

  virtual bool on_activate(
    geometry_msgs::msg::PoseStamped drone_pose,
    as2_msgs::action::NavigateToPoint::Goal goal) = 0;

  virtual bool on_deactivate() = 0;
  virtual bool on_modify() = 0;
  virtual bool on_pause() = 0;
  virtual bool on_resume() = 0;
  virtual void on_execution_end() = 0;
  virtual as2_behavior::ExecutionStatus on_run() = 0;

public:
  // 플러그인이 채우는 결과 경로
  std::vector<geometry_msgs::msg::Point> path_;

protected:
  as2::Node* node_ptr_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
};

} // namespace as2_behaviors_path_planning
```

---

## 7. A* 플러그인

**파일**: `plugins/a_star/include/a_star.hpp`

### 클래스 구조

```cpp
namespace a_star {

class Plugin : public as2_behaviors_path_planning::PluginBase {
  AStarSearcher a_star_searcher_;
  nav_msgs::msg::OccupancyGrid last_occ_grid_;
  double safety_distance_;       // 장애물 팽창 거리 [m]
  bool enable_path_optimizer_;   // 경로 최적화 활성화
  bool enable_visualization_;    // 시각화 활성화

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occ_grid_sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr viz_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr viz_obstacle_grid_pub_;

private:
  void occ_grid_cbk(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
};

} // namespace a_star
```

### A* 동작 흐름

```
1. OccupancyGrid 구독 + 수신 대기
2. on_activate() 호출 시:
   a. safety_distance로 장애물 팽창 (inflation layer)
   b. 현재 드론 위치 → Grid 좌표 변환
   c. 목표 위치 → Grid 좌표 변환
   d. A* 탐색 실행
   e. Grid 경로 → World 좌표 역변환
   f. (enable_path_optimizer_) → 직선 단순화
   g. path_ 채우기
```

### `CellNode` 구조 (A* 노드)

**파일**: `common/include/cell_node.hpp`

```cpp
struct CellNode {
  int x, y;          // Grid 좌표
  double g;          // 시작점부터의 실제 비용
  double h;          // 목표까지의 추정 비용 (휴리스틱)
  double f;          // f = g + h
  CellNode* parent;  // 경로 역추적용

  // 비교 연산자 (우선순위 큐용)
  bool operator>(const CellNode& other) const { return f > other.f; }
};
```

### `GraphSearcher` 기반 클래스

**파일**: `common/include/graph_searcher.hpp`

```cpp
template<typename NodeT>
class GraphSearcher {
  // 범용 그래프 탐색 프레임워크
  // A*와 Voronoi 탐색 모두 이 클래스를 상속
};
```

---

## 8. Voronoi 플러그인

**파일**: `plugins/voronoi/include/voronoi.hpp`

### 구조

```cpp
namespace voronoi_planner {

class Plugin : public as2_behaviors_path_planning::PluginBase {
  VoronoiSearcher voronoi_searcher_;
  nav_msgs::msg::OccupancyGrid last_occ_grid_;
  // (A*와 동일한 인터페이스)
};

} // namespace voronoi_planner
```

### Voronoi 동작 원리

```
1. OccupancyGrid → DynamicVoronoi 다이어그램 생성
   └── Voronoi 경계(Voronoi edge): 장애물에서 최대로 멀어지는 경로
2. Voronoi 스켈레톤 추출
3. A*로 Voronoi 그래프 탐색
→ 결과: 장애물로부터 최대 거리를 유지하는 경로 (안전성 중시)
```

### 서드파티 라이브러리: `dynamicvoronoi`

```cpp
// 오픈소스 Dynamic Voronoi 라이브러리 통합
class DynamicVoronoi {
  void initializeEmpty(int sizeX, int sizeY);
  void occupyCell(int x, int y);
  void update();  // Voronoi 다이어그램 갱신
  bool isVoronoi(int x, int y);  // 셀이 Voronoi 경계인지 확인
};
```

---

## 9. ROS2 인터페이스

| 종류 | 이름 | 타입 |
|------|------|------|
| Action Server | `/<ns>/navigate_to_point` | `as2_msgs/NavigateToPoint` |
| Action Client | `/<ns>/follow_path` | `as2_msgs/FollowPath` |
| Subscription | `/<ns>/self_localization/pose` | `geometry_msgs/PoseStamped` |
| Subscription (plugin) | `/<ns>/map` | `nav_msgs/OccupancyGrid` |
| Service Client | `/<follow_path_node>/_behavior/pause` | `std_srvs/Trigger` |
| Service Client | `/<follow_path_node>/_behavior/resume` | `std_srvs/Trigger` |
| Publisher (viz) | `path_planner/path_marker` | `visualization_msgs/Marker` |

---

## 10. Action 타입

### `as2_msgs/action/NavigateToPoint`

```
# Goal
geometry_msgs/PoseStamped target_pose
as2_msgs/msg/YawMode yaw_mode
---
# Result
bool success
---
# Feedback
float32 actual_speed
float32 actual_distance_to_goal
uint32 remaining_waypoints
```

---

## 11. 설정 파라미터

**파일**: `config/behavior_default.yaml`

```yaml
enable_visualization: false        # RViz 경로 시각화
enable_path_optimizer: false       # 경로 단순화
safety_distance: 1.0              # 장애물 팽창 거리 [m] (드론 크기)
plugin_name: "a_star::Plugin"      # 사용할 플러그인
run_frequency: 10.0                # 실행 루프 주기 [Hz]
```

---

## 12. Launch 파일

```bash
# A* 플러그인 사용
ros2 launch as2_behaviors_path_planning a_star-path_planner_behavior.launch.py \
  namespace:=drone0

# Voronoi 플러그인 사용
ros2 launch as2_behaviors_path_planning voronoi-path_planner_behavior.launch.py \
  namespace:=drone0
```

---

## 13. 아키텍처 다이어그램

```
[미션 플래너 / BT]
        │  NavigateToPoint Goal
        ▼
PathPlannerBehavior
  ├── OccupancyGrid 수신
  ├── PathPlannerPlugin (A* or Voronoi)
  │     └── path_ = [Point, Point, ...]
  └── FollowPathBehavior (Action Client)
          │  FollowPath Goal (path_)
          ▼
    FollowPathBehavior
          │
          ▼
    [드론 이동 실행]
```

---

## 14. 알려진 이슈 및 주의사항

1. **OccupancyGrid 의존성**: 플러그인이 맵을 수신하기 전에 `on_activate()`가 호출되면 실패. 맵 서버가 먼저 구동되어야 함.

2. **2D 경로 계획**: A*와 Voronoi 모두 2D Grid 기반. 3D 장애물 환경에서는 고도 처리가 별도로 필요.

3. **FollowPath 서비스 이름 하드코드**: `follow_path_pause_client_`, `follow_path_resume_client_`의 서비스 이름이 코드 내 고정. FollowPathBehavior 노드 이름 변경 시 수동 수정 필요.

4. **경로 최적화 미완성**: `enable_path_optimizer_ = true` 옵션이 존재하지만 플러그인 구현에 따라 지원 여부가 다름.

5. **Modify 미지원**: `on_modify()` 플러그인 메서드는 기본 구현이 false 반환. 실행 중 목표 변경 불가.
