# ROS2 기반 경찰차 차단 구역 감지 및 자율 우회 주행 시뮬레이션

> Intelligent Robotics Final Project (2026 Spring)

## 1. 프로젝트 개요

**팀명**: 1인 프로젝트
**팀원**: 박세은 (2248031, 컴퓨터공학과)
**역할 분담**: 1인 프로젝트로 시뮬레이션 환경 구축, 3D 모델 변환 및 경광등 플러그인 적용, 통제 구역 판단 ROS2 노드 구현, Nav2 통합, 시연, 보고서 작성을 전담함.

### 프로젝트 설명

2025년 12월 LA에서 발생한 Waymo 로보택시의 felony stop 진입 사건에 착안하여, 자율주행 시스템이 단순 객체 감지를 넘어 **경찰차의 존재와 경광등 작동 상태를 결합**하여 현장의 상황적 맥락을 판단해야 한다는 문제 의식에서 출발하였다. 본 프로젝트는 Gazebo 시뮬레이션 환경에서 경찰차와 경광등 점멸 상태를 기반으로 통제 구역을 판단하고, 진입 전 우회 경로를 자동 생성하는 ROS2 기반 자율주행 시스템을 구현한다.

### 핵심 기여

1. 직접 구현한 ROS2 노드 `RestrictedZoneDetector` — 카메라 영상에서 경찰차 bounding box, HSV 색상 필터링, 다중 프레임 점멸 판정을 거쳐 통제 구역 여부를 결정하고 가상 장애물을 publish.
2. **다중 프레임 윈도우 기반 점멸 판정 로직** — 단일 프레임 false positive를 회피하면서 점멸 신호를 안정적으로 인식.
3. **PointCloud2 가상 장애물 방식**으로 Nav2 obstacle layer 플러그인을 별도 작성하지 않고도 동적 장애물 반영.

## 2. 시스템 구성

```
┌────────────────┐    /camera/image_raw   ┌────────────────────────┐
│ Gazebo World   │ ─────────────────────▶ │ RestrictedZoneDetector │
│ + Turtlebot3   │                        │  (직접 구현 ROS2 노드)  │
│ + 경찰차/경광등 │                          │  ① YOLOv8 bbox 추론     │
└────────────────┘                        │  ② HSV 빨강/파랑 필터   │
       ▲                                  │  ③ 12-프레임 점멸 판정  │
       │ /cmd_vel                         │  ④ 가상 장애물 publish  │
       │                                  └───────────┬────────────┘
       │                                              │ /virtual_obstacle_points
       │                                              ▼
       │                                  ┌────────────────────────┐
       └──────── odom, scan, tf ◀──────── │     Nav2 Stack         │
                                          │ (AMCL + Costmap +      │
                                          │  Planner + Controller) │
                                          └────────────────────────┘
```

### ROS2 패키지 구조

```
felony_stop_sim/
├── worlds/intersection.world              # 도로 교차로 SDF 월드
├── models/police_car/                      # 경찰차 모델 + 경광등 FlashLightPlugin
│   ├── model.sdf
│   ├── model.config
│   └── meshes/police_car.obj
├── maps/intersection_map.{pgm,yaml}        # SLAM 생성 occupancy grid
├── launch/intersection_world.launch.py
├── launch/restricted_zone_detector.launch.py
└── nodes/restricted_zone_detector.py       # 통제 구역 판단 노드 (직접 구현)
```

## 3. 구현 환경

| 항목 | 버전 / 사양 |
|---|---|
| OS | Ubuntu 22.04 LTS |
| ROS2 | Humble Hawksbill |
| 시뮬레이터 | Gazebo Classic 11.10.2 (headless 모드) |
| 로봇 플랫폼 | Turtlebot3 Waffle Pi |
| 자율주행 스택 | Nav2 (slam_toolbox, AMCL, NavfnPlanner) |
| 객체 감지 | Ultralytics YOLOv8n (fine-tuning 진행 예정) |
| 색상 처리 | OpenCV (HSV 필터링) |
| 언어 | Python 3.10 (rclpy) |

## 4. 실행 방법

```bash
# 1. 빌드
cd ~/turtlebot3_ws
colcon build --packages-select felony_stop_sim
source install/setup.bash

# 2. 환경변수 (RTX 50 호환성 우회)
export TURTLEBOT3_MODEL=waffle_pi
export GAZEBO_MODEL_PATH=$HOME/turtlebot3_ws/install/felony_stop_sim/share/felony_stop_sim/models:$GAZEBO_MODEL_PATH:/usr/share/gazebo-11/models
export GAZEBO_MODEL_DATABASE_URI=""

# 3. 시뮬레이션 + Nav2 + 통제 구역 감지 노드 (각각 터미널 분리)
ros2 launch felony_stop_sim intersection_world.launch.py
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  use_sim_time:=true \
  map:=$HOME/turtlebot3_ws/src/felony_stop_sim/maps/intersection_map.yaml
ros2 launch felony_stop_sim restricted_zone_detector.launch.py

# 4. 초기 위치 강제 지정 (AMCL과 맵 origin 불일치 우회)
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  '{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: -7.0, z: 0.0}, orientation: {w: 1.0}}}}'
```

## 5. 발표 영상 (YouTube)

> 발표 영상 링크: [https://youtu.be/KvrbUIdFsjo](https://youtu.be/KvrbUIdFsjo)

YouTube에 Unlisted로 업로드 예정.

## 6. AI 사용 여부 및 사용 내용

본 프로젝트는 다음 영역에서 생성형 AI(Anthropic Claude)를 사용하였다.

| 사용 영역 | 사용 내용 | AI 결과물 처리 |
|---|---|---|
| 개발 환경 트러블슈팅 | RTX 50 + Gazebo OGRE 호환성 진단, AMCL TF 정렬 문제 디버깅 시 단계별 대화형 진단 | 제시된 진단 단계를 직접 실행하며 검증, 효과적이지 않은 제안은 폐기 |
| ROS2 노드 구조 설계 | `RestrictedZoneDetector` 클래스 골격, ROS2 pub/sub 패턴 작성 | 직접 검토 후 제안서 요구사항(HSV 임계값, 프레임 윈도우 크기)에 맞게 수정 |
| SDF/launch 파일 보일러플레이트 | Gazebo 월드, model.sdf, FlashLightPlugin 설정의 XML 구조 | 본인 의도에 맞게 좌표/크기/색상 직접 조정 |
| 보고서/발표자료 초안 | 중간보고서 및 최종 발표 슬라이드 구조 작성 | 본인이 직접 작성한 작업 내용/결과/트러블슈팅 경험을 기반으로 작성, 표현 검토 후 제출 |

AI는 페어 프로그래밍 및 디버깅 파트너로 사용하였으며, 모든 코드는 본인이 실행/검증하였고, 모든 시뮬레이션 환경 구축 결정과 트러블슈팅 우회 전략 채택은 본인이 직접 판단하였다.

## 7. 참고 자료

### 논문 및 기술 문서

1. Waymo 경찰 현장 진입 사건 (NBC News, 2025.12): https://www.nbcnews.com/news/us-news/driverless-waymo-vehicle-inadvertently-takes-riders-tense-police-stop-rcna246994
2. Khan (2021), *Deep Learning for Object Detection Using Parametric CAD Modelling and Gazebo Simulation*, Tampere University: https://trepo.tuni.fi/handle/10024/135807
3. Baimukashev et al. (2019), *Deep Learning Based Object Recognition Using Physically-Realistic Synthetic Depth Scenes*, MDPI: https://www.mdpi.com/2504-4990/1/3/51

### ROS2 / Gazebo 공식 문서

4. ROS2 Nav2: https://docs.nav2.org/
5. Turtlebot3 시뮬레이션 튜토리얼: https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/
6. Ultralytics YOLOv8 ROS 연동 가이드: https://docs.ultralytics.com/guides/ros-quickstart/
7. Gazebo FlashLightPlugin 튜토리얼: https://classic.gazebosim.org/tutorials?tut=flashlight_plugin
8. Nav2 Costmap2D 플러그인 작성 튜토리얼: https://docs.nav2.org/plugin_tutorials/docs/writing_new_costmap2d_plugin.html

### 데이터셋 및 모델

9. Roboflow police_car 데이터셋 (약 1,200장): https://universe.roboflow.com/enchongs-workspace/police_car
10. Sketchfab Police Car 3D 모델 (CC BY, Clyster): https://sketchfab.com/3d-models/police-car-cde61007802347849db5d7e8d1c87f3e

## 8. GitHub 링크

> [https://github.com/yellowsubmarine372/felony-stop-sim](https://github.com/yellowsubmarine372/felony-stop-sim)
