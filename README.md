# ros-bag-modifier

[![License](https://img.shields.io/badge/license-Apache--2.0-blue.svg)](LICENSE)
[![ROS](https://img.shields.io/badge/ROS-Noetic-22314E.svg)](http://wiki.ros.org/noetic)
[![tests](https://img.shields.io/badge/tests-34%20passing-brightgreen.svg)](#기하-모듈만-검증하기-의존성-불필요)

> 이 브랜치는 **ROS 1 Noetic** 판입니다. ROS 2 Humble·Jazzy 판은 `main` 브랜치에
> 있습니다 ([지원 ROS 버전](#지원-ros-버전) 참고).

기록된 ROS bag에서 **끊어진 객체 추적 궤적을 복원**하는 오프라인 도구입니다.
인지 모듈이 가림·센서 사각으로 트랙을 놓쳤다가 새 ID로 다시 잡은 구간을 재식별하고,
그 사이 빈 구간을 스플라인으로 보간해 하나의 연속된 궤적으로 되돌립니다.

> **English** — An offline tool that repairs broken object-tracking trajectories in
> recorded ROS bags. When a perception stack loses a track through occlusion and
> re-acquires it under a fresh id, downstream analysis sees two short trajectories
> instead of one. This tool re-identifies the pair and fills the gap with
> spline-interpolated poses, producing a bag that can be replayed as a continuous
> driving scenario.

---

## ⚠️ 먼저 읽어주세요 — 빌드 전제조건

**이 패키지는 공개 배포되지 않은 ROS 메시지·라이브러리 패키지에 의존합니다.**
`git clone` 후 바로 빌드되지 않습니다.

| 필요한 패키지 | 제공 내용 | 공개 여부 |
|---|---|---|
| `cyber_perception_msgs` | 인지 객체 스트림 메시지 | ❌ 비공개 |
| `jsk_recognition_msgs` | rviz 바운딩박스 (시각화 노드 전용) | ✅ 공개 |
| Eigen3 | 선형대수 | ✅ 공개 (MPL-2.0) |

메시지 필드 명세는 [인터페이스](#인터페이스) 절에 전부 기재했습니다.
동등한 `.msg`를 직접 정의하면 알고리즘 부분은 그대로 사용할 수 있습니다.

**의존성 없이도 동작하는 부분**: `bag_modifier_geometry` 라이브러리(기하 연산 +
운동 모델)와 그 단위 테스트는 ROS·비공개 의존성이 전혀 없어 단독으로 빌드·검증됩니다.

### 지원 ROS 버전

두 배포판을 **모두 지원**합니다. ROS 관례대로 배포판별 브랜치로 나뉘어 있으며,
알고리즘과 파라미터 기본값은 양쪽이 동일합니다.

| 브랜치 | ROS | 빌드 | 이 문서 |
|---|---|---|---|
| **`noetic-devel`** | **ROS 1 Noetic** | `catkin_make` | ← 지금 보고 계신 문서 |
| `main` | ROS 2 Humble, Jazzy | `colcon build` | [main 브랜치 README](https://github.com/keti-mobility/ros-bag-modifier/blob/main/README.md) |

Noetic은 2025년 5월에 EOL이 되었지만, 기존 ROS 1 자산을 쓰는 환경을 위해 계속
유지합니다. 새로 시작하는 환경이라면 `main`을 권합니다.

---

## 목차

- [이 도구가 푸는 문제](#이-도구가-푸는-문제)
- [파이프라인 내 위치](#파이프라인-내-위치)
- [아키텍처](#아키텍처)
- [동작 원리](#동작-원리)
- [설치 및 빌드](#설치-및-빌드)
- [사용법](#사용법)
- [파라미터](#파라미터)
- [인터페이스](#인터페이스)
- [제약사항 및 알려진 이슈](#제약사항-및-알려진-이슈)
- [인용](#인용)
- [라이선스](#라이선스)
- [사사](#사사)

---

## 이 도구가 푸는 문제

자율주행 차량의 인지 모듈은 객체가 다른 차량에 가려지거나 센서 시야를 벗어나면
트랙을 잃습니다. 다시 보이는 순간 그 객체는 **새로운 track ID**로 등록됩니다.

기록된 bag을 그대로 분석하면 이 현상이 다음 문제를 만듭니다.

- 하나의 차량이 여러 개의 짧은 궤적으로 쪼개져 **거동 분석·지표 산출이 왜곡**됨
- 시뮬레이터로 상황을 재현하면 객체가 **사라졌다가 다른 위치에서 튀어나옴**
- 레이블링 자동화의 입력으로 쓸 때 동일 객체가 **다른 개체로 집계**됨

이 도구는 원본 bag을 건드리지 않고, 끊어진 두 트랙을 하나로 잇고 그 사이를 채운
새 bag을 만들어 이 문제를 제거합니다.

```
입력 bag           트랙 39 ──────╴          ╶────── 트랙 71
                              (가림 구간, 8프레임)

출력 bag           트랙 39 ──────·····················──────
                              (보간된 자세)          ID 통일
```

## 파이프라인 내 위치

```
 [실차 주행]
     │  트리거 발생 시 센서/인지/제어 데이터 동기 로깅
     ▼
 recording.bag ────────▶ ★ ros-bag-modifier ────────▶ recording_repaired.bag
                          트랙 재식별 + 궤적 보간                │
                                                              │ rosbag play
                                                              ▼
                                                    bag-to-sim-interface
                                                    시뮬레이터 주행상황 재현
```

두 번째 단계는 별도 저장소인
[bag-to-sim-interface](https://github.com/keti-mobility/bag-to-sim-interface)에
있습니다. 두 저장소는 코드를 공유하지 않고 **토픽·메시지 계약만 공유**합니다.

## 아키텍처

의존 방향은 항상 안쪽(도메인)을 향합니다. `geometry`와 `track`은 ROS를 모릅니다.

```
┌──────────────────────────────────────────────────────────────┐
│  실행 파일                                                     │
│  bag_modifier   estimation_visualizer   repaired_bag_visualizer│
└───────┬──────────────────┬───────────────────────┬───────────┘
        │                  │                       │
        ▼                  ▼                       ▼
┌───────────────┐  ┌──────────────────┐   ┌────────────────┐
│ bag_rewriter  │  │ ros/             │   │ viz/           │
│ bag 입출력      │  │ obstacle_        │   │ marker_style   │
│               │  │ conversion       │   │ rviz 표현       │
│               │  │ msg ↔ 도메인 경계  │   │                │
└───────┬───────┘  └────────┬─────────┘   └───────┬────────┘
        │                   │                     │
        ▼                   ▼                     │
┌──────────────────────────────────────┐          │
│ track/                               │          │
│  track_repairer     재식별 + 갭 채우기  │          │
│  obstacle_pose_estimator  운동 모델    │◀─────────┘
│  obstacle_sample    프레임워크 무관 DTO │
└──────────────┬───────────────────────┘
               ▼
┌──────────────────────────────────────┐   ┌──────────────────┐
│ geometry/   ROS·외부 라이브러리 의존 0   │   │ spline/          │
│  vec2d  box2d  polygon_iou  angle_math│   │ QP 스플라인 래퍼    │
└──────────────────────────────────────┘   └──────────────────┘
```

| 빌드 타깃 | 내용 | 외부 의존 |
|---|---|---|
| `bag_modifier_geometry` | 기하 연산, CTRV/선형 운동 모델, 곡선 피팅 | Eigen3만 (ROS 없음) |
| `bag_modifier_core` | 메시지 변환, 트랙 보정, bag 입출력 | roscpp, rosbag, 인지 메시지 |
| `bag_modifier_viz` | rviz 마커 생성 | jsk_recognition_msgs |

## 동작 원리

### 1단계 — 트랙 상태 추적

bag을 프레임 순서대로 읽으며 세 집합을 유지합니다.

- **추적 중**: 현재 프레임에 있는 트랙
- **사라진 트랙**: 직전에 있었으나 이번 프레임에 없는 트랙. 재식별 후보로
  `reidentification_window`(기본 2초) 동안만 보관
- **신규 트랙**: 이번 프레임에 처음 나타난 트랙

### 2단계 — 사라진 트랙의 현재 자세 추정

사라진 트랙이 "지금이라면 어디 있을지"를 추정합니다. 세 모델을 순서대로 적용합니다.

| 조건 | 모델 | 근거 |
|---|---|---|
| 평균 속력 < 0.83 m/s (3 km/h) | **정지** — 마지막 자세 유지 | 저속에서 인지 heading이 신뢰할 수 없음 |
| 보행자 · 이력 1개 · yaw rate ≈ 0 | **선형** — 속도 적분 | 보행자 heading은 노이즈가 커 회전율로 못 씀 |
| 그 외 | **CTRV** — 등속·등회전율 | 차량의 실제 선회 거동에 부합 |

CTRV 변위:

```
Δx = (v/ω)·[sin(θ + ω·Δt) − sin θ]
Δy = (v/ω)·[−cos(θ + ω·Δt) + cos θ]
```

`v`는 이력 구간 평균 속력, `ω`는 이력 처음↔끝 heading 차이를 시간으로 나눈 값입니다.

### 3단계 — 재식별 매칭

신규 트랙 × 사라진 트랙의 모든 조합에 대해 **게이팅 후 점수화**합니다.

**게이팅** (하나라도 실패하면 후보 탈락)

- 객체 종류(`type.type`)가 동일할 것
- 추정 박스와 신규 박스가 실제로 겹칠 것 (분리축 정리)
- heading 차이 < `max_heading_error` (기본 π/2)
- 속력 차이 < `max_speed_error` (기본 13.9 m/s = 50 km/h)

**점수**

```
similarity = IoU
           + 0.5 · cos(Δheading)
           + 0.5 · sqrt(1 − (Δspeed / max_speed_error)²)
```

IoU 가중치가 암묵적으로 1이므로, heading·속력 일치가 겹침 부족을 절반까지만
보상합니다. 점수가 높은 쌍부터 **탐욕적으로** 확정하며, 신규·사라진 트랙 모두
최대 한 번만 매칭됩니다.

### 4단계 — 갭 보간

매칭된 쌍의 시작 자세와 끝 자세 사이를 채웁니다.

1. 두 끝점의 heading 차이로 **직선/원호**를 판정하고, 원호면 현 길이와 사잇각으로
   반지름과 각속도를 구합니다
2. 그 등회전율 모델로 중간 프레임마다 **제어점**을 생성합니다
3. 제어점들을 균일 3차 B-스플라인으로 **벌점 최소제곱** 피팅합니다. 중간 제어점은
   데이터로 들어가고 계수의 2차·3차 차분에 벌점을 주며, 양 끝점은 실제 관측값이므로
   위치와 접선을 KKT 등식제약으로 **정확히** 고정합니다
4. 피팅된 곡선 위의 점을 각 프레임의 보간 자세로 기록합니다

진행 방향이 heading과 150° 이상 어긋나면 후진으로 판정해 접선을 뒤집습니다.
후진하는 차량을 앞으로 돌려세우느라 곡선이 크게 휘는 것을 막습니다.

### 5단계 — ID 통일

`id_matching` 체인을 따라가 재식별된 트랙의 ID를 인지가 처음 부여한 ID로 되돌립니다.
한 트랙이 여러 번 끊겼다 이어질 수 있으므로 체인을 반복 추적하되, 순환이 생겨도
멈추도록 최대 64회로 제한합니다.

## 설치 및 빌드

### 요구사항

- Ubuntu 20.04
- ROS 1 Noetic (⚠️ 2025년 5월 EOL — [제약사항](#제약사항-및-알려진-이슈) 참고)
- C++14 컴파일러, CMake ≥ 3.0.2
- 비공개 의존 패키지 (위 [빌드 전제조건](#-먼저-읽어주세요--빌드-전제조건) 참고)

### 빌드

```bash
git clone https://github.com/keti-mobility/ros-bag-modifier.git
cd ros-bag-modifier

# 비공개 의존 패키지를 src/ 아래에 함께 두어야 합니다
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

ROS 2 Humble/Jazzy 판이 필요하면 `git checkout main` 후 `colcon build`를 쓰십시오.

### 기하 모듈만 검증하기 (의존성 불필요)

ROS나 비공개 패키지 없이도 핵심 알고리즘을 확인할 수 있습니다.

```bash
sudo apt install libgtest-dev g++
cd src/bag_modifier
g++ -std=c++14 -I include -I /usr/include/eigen3 \
    test/test_geometry.cc test/test_obstacle_pose_estimator.cc \
    test/test_spline_solver.cc \
    src/geometry/box2d.cc src/geometry/polygon_iou.cc \
    src/track/obstacle_pose_estimator.cc src/spline/spline_solver.cc \
    -lgtest -lgtest_main -pthread -o test_geometry
./test_geometry      # 34 tests
```

catkin 워크스페이스 안에서는 `catkin_make run_tests_bag_modifier`로도 실행됩니다.

## 사용법

### bag 복원

```bash
# launch 사용 (권장 — 파라미터 파일이 함께 적용됨)
roslaunch bag_modifier bag_modifier.launch bag:=/path/to/recording.bag

# 직접 실행
rosrun bag_modifier bag_modifier /path/to/recording.bag
```

출력 두 개가 생성됩니다. **원본 bag은 수정되지 않습니다.**

| 파일 | 내용 |
|---|---|
| `recording_repaired.bag` | 원본의 모든 토픽 + 복원된 `/obstacles` |
| `recording_repaired_interpolated_only.bag` | 합성된 자세만 (`/obstacles_modified`) |

접미사는 두 번째 인자로 바꿀 수 있습니다: `... recording.bag _v2`

실행이 끝나면 처리 통계가 출력됩니다.

```
[ INFO] repairing /data/recording.bag
[ INFO] wrote /data/recording_repaired.bag and /data/recording_repaired_interpolated_only.bag
[ INFO] messages: 148203, obstacle frames: 4512, duplicate frames skipped: 37
[ INFO] re-identified tracks: 63, interpolated poses: 412
```

### rviz로 확인

```bash
roslaunch bag_modifier visualize.launch x_offset:=332950.0 y_offset:=4140495.0
rosbag play recording_repaired.bag
```

`x_offset`/`y_offset`은 그려지는 모든 좌표에서 빼는 값입니다. 지도 좌표가 UTM
미터라 rviz의 float32 변환에서 정밀도를 잃기 때문에, 장면 근처의 한 점을 넣습니다.

| 노드 | 보여주는 것 |
|---|---|
| `estimation_visualizer` | 사라진 트랙의 **예측 자세** — 복원 전에 운동 모델을 검증 |
| `repaired_bag_visualizer` | 복원된 bag의 객체 — 박스, 클래스, ID, 진행/속도/가속도 화살표 |

## 파라미터

전부 [`config/bag_modifier.yaml`](src/bag_modifier/config/bag_modifier.yaml)에
있으며 launch가 private 네임스페이스로 올립니다.

### 재식별 게이팅

| 파라미터 | 기본값 | 의미 |
|---|---|---|
| `reidentification_window` | `2.0` s | 사라진 트랙을 후보로 유지하는 시간. 늘리면 매칭이 늘지만 예측이 흐트러진 뒤라 무관한 객체를 잇기 시작함 |
| `max_speed_error` | `13.9` m/s | 동일 객체로 인정하는 속력 차 한계 (50 km/h). 미관측 구간에 가속했을 수 있어 넉넉함 |
| `max_heading_error` | `1.5707963` rad | heading 차 한계 (π/2) |
| `heading_similarity_weight` | `0.5` | 점수에서 heading 항의 가중치 |
| `speed_similarity_weight` | `0.5` | 점수에서 속력 항의 가중치 |

### 운동 모델

| 파라미터 | 기본값 | 의미 |
|---|---|---|
| `stationary_speed_threshold` | `0.83` m/s | 이 아래는 정지로 보고 자세 유지 (3 km/h) |
| `history_size` | `10` | 트랙당 보관 샘플 수. 클수록 회전율 추정이 매끄럽지만 실제 선회에 늦게 반응 |

### 갭 보간

| 파라미터 | 기본값 | 의미 |
|---|---|---|
| `spline_segment_count` | `8` | 곡선을 이루는 구간 수. 많을수록 제어점을 가깝게 따라가고 덜 매끄러움 |
| `spline_fit_weight` | `1.0` | 적합 항 대 평활 항의 비중. 양 끝점은 등식제약이라 중간 제어점 추종에만 영향 |
| `spline_second_derivative_weight` | `0.2` | 2차 차분 벌점 |
| `spline_third_derivative_weight` | `1.0` | 3차 차분 벌점. 2차의 5배. 이 값에서 20 m 호를 3 mm로 따라가면서 제어점의 30 cm 노이즈는 무시함 |

### 입출력

| 파라미터 | 기본값 | 의미 |
|---|---|---|
| `obstacle_topic` | `/obstacles` | 복원 대상 토픽. 나머지 토픽은 그대로 복사됨 |
| `interpolated_topic` | `/obstacles_modified` | 보간 전용 bag에 쓰는 토픽 |
| `duplicate_frame_tolerance` | `1e-5` s | 이 간격 이내 연속 프레임은 기록기가 중복 기록한 것으로 보고 버림 |

## 인터페이스

### 토픽

| 노드 | 방향 | 토픽 | 타입 |
|---|---|---|---|
| `bag_modifier` | bag 읽기 | `/obstacles` | `cyber_perception_msgs/PerceptionObstacles` |
| `bag_modifier` | bag 쓰기 | `/obstacles`, `/obstacles_modified` | 〃 |
| `estimation_visualizer` | 구독 | `obstacles` | 〃 |
| `estimation_visualizer` | 발행 | `obstacles_estimation_vis` | `jsk_recognition_msgs/BoundingBoxArray` |
| `estimation_visualizer` | 발행 | `obstacles_estimation_vis_vel` | `visualization_msgs/MarkerArray` |
| `repaired_bag_visualizer` | 구독 | `obstacles_modified` | `cyber_perception_msgs/PerceptionObstacles` |
| `repaired_bag_visualizer` | 발행 | `obstacles_vis_modified` | `jsk_recognition_msgs/BoundingBoxArray` |
| `repaired_bag_visualizer` | 발행 | `obstacles_vis_vel_modified`, `converter_vis_vel`, `converter_vis_accel` | `visualization_msgs/MarkerArray` |

### 의존 메시지 명세

`cyber_perception_msgs`는 공개 배포되지 않습니다. 아래는 이 패키지가 **실제로
읽고 쓰는 필드**로, 동등한 메시지를 직접 정의할 때의 명세입니다.
사용처는 [`src/ros/obstacle_conversion.cc`](src/bag_modifier/src/ros/obstacle_conversion.cc)
한 곳에 모여 있으므로, 다른 메시지로 교체하려면 그 파일만 고치면 됩니다.

**`PerceptionObstacles`**

| 필드 | 타입 | 의미 |
|---|---|---|
| `cyber_header.timestamp_sec` | `float64` | 인지 프레임 시각 (초). **트랙 보정의 시간 기준** |
| `cyber_header.sequence_num` | `uint32` | 프레임 일련번호 (rviz 헤더에만 사용) |
| `perception_obstacle` | `PerceptionObstacle[]` | 이 프레임의 객체 목록 |

**`PerceptionObstacle`**

| 필드 | 타입 | 의미 |
|---|---|---|
| `id` | `int32` | 트랙 ID. 이 도구가 재식별 후 덮어씀 |
| `timestamp` | `float64` | 객체별 시각 (rviz 박스 헤더에만 사용) |
| `position` | `geometry_msgs/Point` | 지도 좌표계 위치 (m). x, y 사용 |
| `theta` | `float64` | heading (rad), +x축 기준 반시계 |
| `velocity` | `geometry_msgs/Point` | 속도 (m/s). x, y 사용 |
| `acceleration` | `geometry_msgs/Point` | 가속도 (m/s²). 시각화에만 사용 |
| `length` | `float64` | 진행 방향 길이 (m) |
| `width` | `float64` | 횡방향 폭 (m) |
| `height` | `float64` | 높이 (m) |
| `type` | `ObstacleType` | 객체 종류. **매칭 시 동일 종류만 후보** |

**`ObstacleType`**

| 필드 | 타입 |
|---|---|
| `type` | `uint8` |

| 값 | 상수 | 이 도구에서의 취급 |
|---|---|---|
| 0 | `UNKNOWN` | 일반 |
| 1 | `UNKNOWN_MOVABLE` | 일반 |
| 2 | `UNKNOWN_UNMOVABLE` | 일반 |
| 3 | `PEDESTRIAN` | **항상 선형 모델** |
| 4 | `BICYCLE` | 일반 |
| 5 | `VEHICLE` | 일반 |

### 이전 버전의 외부 솔버 인터페이스 (참고)

과거 판은 `utils`/`planner`가 제공하는 QP 스플라인 솔버의 다음 API를 사용했습니다.
**현재는 이 의존이 없습니다** — `main`과 동일한 자체 최소제곱 구현으로 대체했습니다.

```cpp
keti::planning::OsqpSpline2dSolver(const std::vector<double>& knots, int order);
void Reset(const std::vector<double>& knots, int order);
bool Solve();
Spline2dConstraint* mutable_constraint();
Spline2dKernel*     mutable_kernel();
const Spline2d&     spline() const;   // operator()(t) -> std::pair<double,double>

// Spline2dConstraint
bool Add2dBoundary(knots, angles, ref_points, longitudinal_bound, lateral_bound);
bool AddPointConstraint(double t, double x, double y);
bool AddPointAngleConstraint(double t, double angle);
bool AddSecondDerivativeSmoothConstraint();

// Spline2dKernel
void AddSecondOrderDerivativeMatrix(double weight);
void AddThirdOrderDerivativeMatrix(double weight);
void AddRegularization(double weight);
```

현재 구현은 [`src/spline/spline_solver.cc`](src/bag_modifier/src/spline/spline_solver.cc)
에 있으며, Eigen만 사용합니다.

## 제약사항 및 알려진 이슈

**빌드**

- **비공개 의존성이 `cyber_perception_msgs` 하나로 줄었습니다.** 위 명세대로
  동등한 메시지를 정의하면 빌드됩니다. 최소 msgs 패키지 동봉이 후속 작업입니다.
- **ROS 1 Noetic은 2025년 5월 EOL**입니다. 이 브랜치는 기존 ROS 1 자산을 쓰는
  환경을 위해 계속 유지하지만, 배포판 자체의 보안 갱신은 더 이상 없습니다.
  새 환경이라면 `main`(ROS 2)을 권합니다.

**알고리즘**

- **탐욕적 매칭**입니다. 전역 최적 할당(헝가리안 등)이 아니므로, 비슷한 점수의
  후보가 밀집한 혼잡 장면에서는 최적이 아닌 짝이 선택될 수 있습니다.
- **오프라인 전용**입니다. `bag_modifier`는 전체 프레임을 메모리에 올린 뒤 보간하므로,
  긴 녹화에서는 메모리 사용량이 프레임 수에 비례해 증가합니다. 실시간 사용 불가.
- **정량 평가가 없습니다.** 재식별 정확도(precision/recall)를 측정한 결과가 아직
  없어, 파라미터 기본값은 실제 녹화 데이터에 대한 육안 검증으로 정해졌습니다.
  다른 인지 스택·다른 주행 환경에서는 재조정이 필요할 수 있습니다.
- **z 좌표는 보간하지 않습니다.** 보간 자세의 높이는 시작 객체의 값을 그대로 씁니다.
  경사로·고가도로 구간에서는 부정확할 수 있습니다.
- 가속도는 보간 자세에 채워지지 않아 0으로 남습니다.

**테스트**

- 자동 테스트는 기하 연산과 운동 모델(23개)만 덮습니다. `TrackRepairer`의 매칭·
  보간 경로와 bag 입출력은 아직 테스트가 없으며, 실제 bag으로 수동 검증합니다.

## 인용

```bibtex
@software{keti_ros_bag_modifier,
  title  = {ros-bag-modifier: Trajectory repair for recorded autonomous driving perception data},
  author = {{Korea Electronics Technology Institute}},
  year   = {2026},
  url    = {https://github.com/keti-mobility/ros-bag-modifier},
  note   = {Developed under IITP grant RS-2023-00232046}
}
```

## 라이선스

[Apache License 2.0](LICENSE). 제3자 고지는 [NOTICE](NOTICE)를 참고하십시오.

이전 버전은 바운딩박스 겹침 계산에 CGAL을 사용했습니다. CGAL의 Boolean set
operations는 GPL로 배포되어 Apache-2.0과 호환되지 않으므로, 볼록 다각형 전용
자체 구현으로 대체했습니다.

## 사사

본 연구는 과학기술정보통신부 및 정보통신기획평가원의 자율주행기술개발혁신사업의
지원을 받아 수행된 연구임 (RS-2023-00232046, 비정상 주행 데이터 전송을 통한
클라우드 기반 원인 분석 기술 개발).

This work was supported by the Institute of Information & Communications
Technology Planning & Evaluation (IITP) grant funded by the Korea government
(MSIT) (No. RS-2023-00232046, Development of Cloud-based Cause Analysis
Technology via Abnormal Driving Data Transmission).
