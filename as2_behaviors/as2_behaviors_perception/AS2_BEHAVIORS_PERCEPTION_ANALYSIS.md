# as2_behaviors_perception 패키지 상세 분석

**버전**: 1.1.3
**라이선스**: BSD-3-Clause

---

## 1. 패키지 개요

`as2_behaviors_perception`은 드론의 **카메라 기반 인식** behavior를 제공하는 패키지다.
현재 ArUco 마커 탐지 기능을 제공하며, 카메라 이미지에서 복수의 ArUco 마커를 실시간으로 탐지하고 6-DOF 포즈를 추정한다.

---

## 2. 파일 구조

```
as2_behaviors_perception/
├── detect_aruco_markers_behavior/
│   ├── include/detect_aruco_markers_behavior/
│   │   └── detect_aruco_markers_behavior.hpp  # 전체 클래스 선언
│   ├── src/
│   │   ├── detect_aruco_markers_behavior.cpp         # 구현
│   │   └── detect_aruco_markers_behavior_node.cpp    # main 진입점
│   ├── config/
│   │   ├── sim_params.yaml    # 시뮬레이션 파라미터
│   │   └── real_params.yaml   # 실제 하드웨어 파라미터
│   └── launch/
│       ├── detect_aruco_markers_behavior_sim_launch.py
│       └── detect_aruco_markers_behavior_real_launch.py
├── CMakeLists.txt
└── package.xml
```

---

## 3. 의존성

```xml
<depend>rclcpp</depend>
<depend>as2_behavior</depend>
<depend>as2_core</depend>
<depend>as2_msgs</depend>
<depend>sensor_msgs</depend>
<depend>cv_bridge</depend>         <!-- ROS ↔ OpenCV 이미지 변환 -->
<depend>image_transport</depend>   <!-- 효율적인 이미지 전송 -->
<depend>opencv</depend>            <!-- ArUco 탐지 (opencv_contrib) -->
<depend>eigen</depend>
<depend>tf2</depend>
```

---

## 4. `DetectArucoMarkersBehavior` 클래스

**파일**: `include/detect_aruco_markers_behavior/detect_aruco_markers_behavior.hpp`

```cpp
class DetectArucoMarkersBehavior
  : public as2_behavior::BehaviorServer<as2_msgs::action::DetectArucoMarkers>
{
  // ── 카메라 인터페이스 ────────────────────────────────────────
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cam_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  std::shared_ptr<as2::sensors::Camera> aruco_img_transport_;  // image_transport

  // ── ArUco 탐지 설정 ──────────────────────────────────────────
  std::vector<uint16_t> target_ids_;           // 탐지 대상 마커 ID 목록
  float aruco_size_;                            // 마커 크기 [m]
  std::string camera_model_;                    // "pinhole"
  std::string distorsion_model_;
  std::string img_encoding_;
  bool camera_qos_reliable_;

  // ── 카메라 캘리브레이션 ──────────────────────────────────────
  cv::Mat camera_matrix_;    // 3x3 내부 파라미터 행렬 K
  cv::Mat dist_coeffs_;      // 왜곡 계수 (k1,k2,p1,p2,k3 등)
  cv::Ptr<cv::aruco::Dictionary> aruco_dict_;  // ArUco 사전 (예: DICT_6X6_250)
  bool camera_params_available_;

  // ── 토픽 이름 ────────────────────────────────────────────────
  std::string camera_image_topic_ = "camera/image_raw";
  std::string camera_info_topic_ = "camera/camera_info";

  // ── 결과 퍼블리셔 ────────────────────────────────────────────
  rclcpp::Publisher<as2_msgs::msg::PoseStampedWithIDArray>::SharedPtr aruco_pose_pub_;
};
```

---

## 5. Action 타입

### `as2_msgs/action/DetectArucoMarkers`

```
# Goal
uint16[] marker_ids         # 탐지할 마커 ID 목록 (빈 배열 = 전체 탐지)
float32  marker_size        # 마커 크기 [m] (0이면 파라미터 사용)
---
# Result
bool success
as2_msgs/PoseStampedWithIDArray detected_markers  # 탐지된 마커 배열
---
# Feedback
uint16 num_detected        # 현재 탐지된 마커 수
```

---

## 6. 동작 흐름

### 6.1 `on_activate()`

```
1. loadParameters() → YAML 파라미터 로드
2. target_ids_ = goal->marker_ids
3. aruco_size_ = (goal->marker_size > 0) ? goal->marker_size : params["aruco_size"]
4. setup() → 구독자 및 퍼블리셔 생성
```

### 6.2 `setup()`

```cpp
void setup() {
  // 이미지 구독 설정
  cam_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    camera_image_topic_, qos,
    std::bind(&DetectArucoMarkersBehavior::imageCallback, this, _1));

  // 카메라 정보 구독
  cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    camera_info_topic_, qos,
    std::bind(&DetectArucoMarkersBehavior::camerainfoCallback, this, _1));

  // image_transport를 통한 디버그 이미지 퍼블리시
  aruco_img_transport_ = std::make_shared<as2::sensors::Camera>(
    "aruco_debug", this);
}
```

### 6.3 `imageCallback()` (핵심 처리)

```cpp
void imageCallback(const sensor_msgs::msg::Image::SharedPtr img) {
  if (!camera_params_available_) return;  // 카메라 정보 대기

  // 1. ROS Image → OpenCV Mat 변환
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, img_encoding_);
  cv::Mat gray;
  cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);

  // 2. ArUco 마커 탐지
  std::vector<int> marker_ids;
  std::vector<std::vector<cv::Point2f>> marker_corners;
  cv::aruco::detectMarkers(gray, aruco_dict_, marker_corners, marker_ids);

  // 3. ID 필터링
  // checkIdIsTarget()으로 target_ids_에 있는 마커만 처리

  // 4. 포즈 추정 (PnP 알고리즘)
  std::vector<cv::Vec3d> rvecs, tvecs;
  cv::aruco::estimatePoseSingleMarkers(
    marker_corners, aruco_size_,
    camera_matrix_, dist_coeffs_,
    rvecs, tvecs);

  // 5. 결과를 PoseStampedWithIDArray로 변환
  as2_msgs::msg::PoseStampedWithIDArray result_msg;
  for (size_t i = 0; i < marker_ids.size(); i++) {
    as2_msgs::msg::PoseStampedWithID pose_with_id;
    pose_with_id.id = std::to_string(marker_ids[i]);

    // 로드리게스 벡터 → 쿼터니온 변환
    pose_with_id.pose.pose.position.x = tvecs[i][0];
    pose_with_id.pose.pose.position.y = tvecs[i][1];
    pose_with_id.pose.pose.position.z = tvecs[i][2];
    // rvecs[i] → quaternion 변환
    result_msg.poses.push_back(pose_with_id);
  }

  // 6. 퍼블리시
  aruco_pose_pub_->publish(result_msg);
}
```

### 6.4 `camerainfoCallback()`

```cpp
void camerainfoCallback(const CameraInfo::SharedPtr info) {
  if (camera_params_available_) return;  // 이미 수신했으면 무시

  setCameraParameters(*info);
  // camera_matrix_ 설정: info->k (3x3)
  // dist_coeffs_ 설정: info->d
  camera_params_available_ = true;

  // 카메라 정보 수신 후 구독 해제 (효율성)
  cam_info_sub_.reset();
}
```

### 6.5 `on_run()`

```
1. camera_params_available_ 체크
2. (지속 실행 모드) → RUNNING 반환
   - 이미지 콜백에서 지속적으로 탐지 및 퍼블리시
3. goal에 시간 제한이 있으면 → 타임아웃 시 결과 반환
```

### 6.6 `on_deactivate()`

```
구독자 해제 → 이미지 처리 중단
```

---

## 7. ArUco 포즈 추정 원리

### 7.1 PnP (Perspective-n-Point) 문제

```
입력:
  - 마커 코너의 2D 이미지 좌표 (4점)
  - 마커의 3D 월드 좌표 (알려진 크기)
  - 카메라 내부 파라미터 (K, dist_coeffs)

출력:
  - rvec: 카메라 좌표계에서 마커의 회전 벡터 (Rodrigues)
  - tvec: 카메라 좌표계에서 마커의 이동 벡터
```

### 7.2 좌표 변환

```
마커 위치 (카메라 좌표계)
    → TF를 통해 world/earth 좌표계로 변환 가능
    (별도의 TF 변환 로직이 필요하면 상위 레이어에서 처리)
```

---

## 8. ROS2 인터페이스

| 종류 | 이름 | 타입 |
|------|------|------|
| Action Server | `/<ns>/detect_aruco_markers` | `as2_msgs/DetectArucoMarkers` |
| Subscription | `camera/image_raw` | `sensor_msgs/Image` |
| Subscription | `camera/camera_info` | `sensor_msgs/CameraInfo` |
| Publisher | `aruco_pose` | `as2_msgs/PoseStampedWithIDArray` |
| Publisher (debug) | `aruco_debug/image_raw` | `sensor_msgs/Image` (image_transport) |
| Publisher | `/<node>/_behavior/behavior_status` | `as2_msgs/BehaviorStatus` |

---

## 9. 설정 파라미터

### `config/sim_params.yaml` (시뮬레이션)

```yaml
/**:
  ros__parameters:
    aruco_size: 0.3           # [m] 마커 크기
    camera_model: "pinhole"
    camera_qos_reliable: true
```

### `config/real_params.yaml` (실제 하드웨어)

```yaml
/**:
  ros__parameters:
    aruco_size: 0.1           # [m] 실제 마커 크기
    camera_model: "pinhole"
    distorsion_model: "plumb_bob"
    camera_qos_reliable: false  # 실시간 스트리밍 = BEST_EFFORT QoS
```

---

## 10. 지원 ArUco 사전

OpenCV ArUco 모듈의 사전 타입 지원:
- `DICT_4X4_50`, `DICT_4X4_100`, `DICT_4X4_250`, `DICT_4X4_1000`
- `DICT_5X5_*`, `DICT_6X6_*`, `DICT_7X7_*`
- `DICT_ARUCO_ORIGINAL`

파라미터 `aruco_dictionary`로 선택 (기본: `DICT_6X6_250`).

---

## 11. `as2::sensors::Camera` 활용

`as2_core/sensor.hpp`의 Camera 센서 클래스를 사용하여 디버그 이미지를 퍼블리시:

```cpp
// 탐지된 마커가 그려진 이미지를 퍼블리시 (시각화용)
aruco_img_transport_->updateData(cv_ptr->image);
```

---

## 12. 알려진 이슈 및 주의사항

1. **`distorsion_model_` 오타**: 코드 전체에서 "distorsion" (s 빠짐, "distortion"이 맞음). 파라미터 이름도 동일하게 오타.

2. **카메라 정보 단회 수신**: `camerainfoCallback()`에서 구독 해제 후 재연결 불가. 런타임에 카메라 파라미터 변경 불가.

3. **target_ids_ 빈 배열**: 모든 마커를 탐지하지만, `checkIdIsTarget()`에서 빈 배열 처리가 필요. 구현에 따라 다름.

4. **OpenCV ArUco API 변경**: OpenCV 4.7+에서 `cv::aruco::detectMarkers()` API가 변경됨. 새 API (`ArucoDetector` 클래스)로의 마이그레이션 필요 가능.

5. **포즈 결과 좌표계**: `tvec`은 카메라 좌표계 기준. 드론 body frame이나 earth frame으로 변환하려면 상위 코드에서 TF 변환 필요.

6. **동기화 없음**: `imageCallback()`과 `on_run()`이 별도 실행. 결과 퍼블리시와 Action 피드백이 독립적으로 동작.
