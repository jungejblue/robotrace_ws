# robotrace_ws

ROS1 Noetic에서 **FAST-LIO2 로컬 odometry + FastVGICPCuda 사전 지도 정합 + 전후 RTK GNSS 초기화**를 실행하는 workspace입니다. 이 문서는 Ubuntu 22.04 호스트 PC에서 Docker로 개발·검증하는 과정을 설명합니다.

RTX 5060 Ti 개발 환경에서 CUDA 빌드와 `ochang2` 검증 bag 재생, metrics 생성이 확인된 구성입니다. 확인 범위는 아래 실행 기록 기준입니다. **GLIM 지도와 ENU 원점의 일치, 절대 위치 정확도, AGX Orin에서의 성능은 별도 검증이 필요합니다.**

## 1. 시작 전 준비

### 저장소 받기 — 호스트

아래 URL의 `OWNER`를 실제 GitHub 사용자 또는 조직 이름으로 바꿉니다. 저장소 이름이 다르면 `robotrace_ws.git`도 수정합니다. 대상 폴더가 이미 존재하면 새로 clone하지 않고 현재 폴더를 사용합니다.

```bash
robotrace_repo_url='https://github.com/OWNER/robotrace_ws.git'
git clone "$robotrace_repo_url" "$HOME/robotrace_ws"
cd "$HOME/robotrace_ws"
mkdir -p data results
```

이 저장소는 수정된 FAST-LIO2, fast_gicp와 필요한 Eigen/NVBIO 소스를 함께 포함합니다. **현재 submodule을 사용하지 않으며 `git submodule update`가 필요하지 않습니다.** `src/fast_gicp`를 공식 upstream으로 다시 clone하거나 `thirdparty/`를 삭제하면 현재 CUDA 빌드 수정 및 의존 소스가 사라질 수 있습니다.

### 데이터 별도 준비 — 호스트

PCD와 bag은 Git에 포함하지 않습니다. 데이터 제공자가 전달한 파일을 다음 위치에 복사합니다. 저장소에는 자동 다운로드 주소를 제공하지 않으므로 별도로 확보해야 합니다.

| 목적 | 필요한 파일 | 호스트 배치 위치 |
|---|---|---|
| 지도 정합 | prior PCD | `~/robotrace_ws/data/map_enu_v010.pcd` |
| 검증 재생 | 원본 bag 또는 준비된 정규화 bag | `~/robotrace_ws/data/ochang2.bag` 또는 `ochang2_localization.bag` |
| 지도 제작 기록 확인 | mapping bag, 검증 실행에는 선택 사항 | `~/robotrace_ws/data/ochang.bag` |

원본 파일이 Downloads에 있는 경우의 예시입니다. 실제 파일 위치에 맞게 입력 경로를 바꿉니다.

```bash
cd ~/robotrace_ws
cp "$HOME/Downloads/map_enu_v010.pcd" data/
cp "$HOME/Downloads/ochang2.bag" data/
ls -lh data/map_enu_v010.pcd data/ochang2.bag
```

이미 `ochang2_localization.bag`을 준비했다면 원본 bag 변환을 건너뛰고 재생에 사용할 수 있습니다. 실시간 센서로 실행할 때는 prior PCD만 필요하고 검증 bag은 필요하지 않습니다.

기본 GNSS 원점·외부 파라미터는 기존 Ouster/GNSS 장착과 지도에 맞춰져 있습니다. 다른 지도나 차량을 사용하면 `src/fast_lio/config/ouster.yaml`과 `src/robotrace_localization/config/localization.yaml`을 확인합니다. PCD 이름을 바꾸는 것만으로 GNSS와 지도 좌표계가 일치하지는 않습니다.

### 환경과 파일

| 항목 | 현재 개발 환경 |
|---|---|
| 호스트 | Ubuntu 22.04, NVIDIA 드라이버 및 NVIDIA Container Toolkit |
| GPU | RTX 5060 Ti, compute capability 12.0 |
| 컨테이너 | Ubuntu 20.04, ROS1 Noetic, CUDA 12.8.1, GCC/G++ 10 |
| workspace | 호스트 `~/robotrace_ws` → 컨테이너 `/ws` |
| 개발 이미지 / 컨테이너 | `robotrace:localization-cuda` / `robotrace-ws` |
| RViz 이미지 / 컨테이너 | `robotrace:rviz` / `robotrace-rviz` |
| prior map | `data/map_enu_v010.pcd` |
| mapping bag | `data/ochang.bag` — 지도 제작에 사용 |
| 검증 bag | `data/ochang2.bag` — localization 검증에 사용 |

호스트 터미널에서 파일과 GPU를 확인합니다.

```bash
cd ~/robotrace_ws
ls -lh data/map_enu_v010.pcd
nvidia-smi
nvidia-smi --query-gpu=name,compute_cap --format=csv
```

bag 검증은 실제 준비한 `ochang2.bag` 또는 `ochang2_localization.bag`을 사용합니다. 실시간 실행에서는 bag 파일이 없어도 됩니다.

### Docker GPU 접근 확인

다음 명령은 **호스트**에서 실행합니다. `nvidia-smi`의 CUDA Version은 드라이버가 지원하는 버전이며, 개발 컨테이너의 CUDA Toolkit 버전과 구분합니다.

```bash
sudo docker run --rm --runtime=nvidia --gpus all \
  nvidia/cuda:12.8.1-base-ubuntu20.04 nvidia-smi
```

GPU를 찾지 못하면 먼저 [NVIDIA Container Toolkit 설치 안내](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)에 따라 Toolkit을 설치합니다. 설치 후 Docker runtime 설정은 다음과 같습니다.

```bash
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

Docker 재시작은 실행 중인 컨테이너에 영향을 줄 수 있으므로 최초 환경 설정 시 수행합니다. 그다음 GPU 접근 확인 명령을 다시 실행합니다.

## 2. 개발 컨테이너와 빌드

### 처음 실행할 때 — 호스트

```bash
cd ~/robotrace_ws
./scripts/container.sh build
./scripts/container.sh start --name robotrace-ws --data-dir "$PWD/data"
./scripts/container.sh shell --name robotrace-ws
```

`--data-dir`를 지정해 새로 만든 컨테이너는 데이터를 `/ws/data`에 읽기 전용으로 연결합니다. 변환한 bag과 로그는 `/ws/results`에 저장합니다. `start`는 기존 컨테이너를 재사용하므로 이미 존재하는 컨테이너의 이미지·마운트를 바꾸지는 않습니다.

이 문서는 컨테이너 이름을 항상 `robotrace-ws`로 명시합니다. 스크립트의 이름 기본값은 `robotrace-localization`이므로 이름 옵션을 생략하면 다른 컨테이너를 선택할 수 있습니다.

### 빌드 — 컨테이너 내부

```bash
cd /ws
./scripts/build.sh --jobs 1 --cuda-arch 120
./scripts/test.sh
./scripts/doctor.sh
```

빌드는 Release 모드와 GCC/G++ 10을 사용합니다. 메모리가 충분하면 `--jobs 2`로 변경할 수 있습니다. 테스트는 Python 도구와 독립 C++ 좌표 계산을 확인하고, `doctor.sh`는 GPU·ROS 패키지·실행 파일·라이브러리 연결을 확인합니다. 이 검사들은 localization 정확도 측정을 대신하지 않습니다.

생성 파일을 확인하려면:

```bash
ls -lh \
  /ws/.build/devel/lib/libfast_vgicp_cuda.so \
  /ws/.build/devel/lib/fast_lio/fastlio_mapping \
  /ws/.build/devel/lib/robotrace_localization/gnss_initializer_node \
  /ws/.build/devel/lib/robotrace_localization/gpu_map_localizer_node
cuobjdump --list-elf /ws/.build/devel/lib/libfast_vgicp_cuda.so
```

RTX 5060 Ti용 빌드에서는 CUDA device binary에 `sm_120`이 표시되어야 합니다.

### 다음 접속부터 — 호스트

```bash
cd ~/robotrace_ws
./scripts/container.sh start --name robotrace-ws
./scripts/container.sh shell --name robotrace-ws
```

`shell`을 사용하면 ROS Noetic과 빌드된 `.build/devel` 환경을 자동으로 불러옵니다. 새 터미널도 동일한 명령으로 접속합니다.

### 폴더별 역할

| 위치 | 내용 |
|---|---|
| `src/fast_lio/` | Ouster 입력, IMU deskew, FAST-LIO2 로컬 odometry |
| `src/fast_gicp/` | FastVGICPCuda와 필요한 CUDA 의존 소스 |
| `src/robotrace_localization/` | GNSS 초기화, GPU 지도 정합, TF·상태·metrics |
| `docker/` | 이미지 정의, 사용자 환경과 ROS 자동 source |
| `scripts/` | 컨테이너·빌드·실행·bag 준비·진단 도구 |
| `tests/` | 좌표 계산과 bag/metrics 도구 테스트 |
| `.build/` | catkin 캐시, 실행 파일, 빌드 로그, 테스트 출력 |
| `data/` | 입력 PCD와 원본 bag |
| `results/` | 변환 bag, CSV/JSON, ROS 로그 |

일반적인 재빌드는 `./scripts/build.sh`만 실행합니다. 컴파일러·CUDA 환경·대상 architecture를 바꾸거나 캐시 오류가 생겼을 때만 다음 명령을 사용합니다.

```bash
./scripts/clean.sh --yes
./scripts/build.sh --jobs 1 --cuda-arch 120
```

`clean.sh --yes`는 `.build` 전체를 삭제합니다. `src`, `data`, `results`는 삭제하지 않습니다.

## 3. 검증 bag 준비

### 이미 정규화한 bag이 있는지 확인 — 컨테이너

```bash
ls -lh /ws/data/ochang2_localization.bag
ls -lh /ws/results/ochang2_localization.bag
```

둘 중 한 위치에 이미 준비한 파일이 있으면 그 경로를 사용합니다. 두 명령 중 없는 경로에 대한 오류는 정상이며, 두 위치 모두 없을 때 다음 변환을 수행합니다.

### 원본에서 새 파일 생성

```bash
python3 scripts/prepare_validation_bag.py \
  /ws/data/ochang2.bag \
  /ws/results/ochang2_localization.bag
```

출력은 다음 두 파일입니다.

- `results/ochang2_localization.bag`
- `results/ochang2_localization.time_alignment.json`

출력 bag 또는 보고서가 이미 존재하면 변환을 거부합니다. 다시 변환할 때는 다른 출력 파일명을 지정합니다. 원본 `ochang2.bag`은 변경하지 않습니다.

이 도구는 Ouster 장치 시각과 GNSS UTC가 다른 기록에서 LiDAR·IMU header에 공통 상수 offset을 적용합니다. GNSS 측정 시각을 검사하며, 점군 payload와 포인트별 상대 시간 `t`를 보존합니다. clock reset·과도한 drift·서로 다른 센서 시간 기준이 발견되면 중단합니다. **오프라인 시간 정규화이며 실제 센서의 하드웨어 동기화를 검증하는 기능은 아닙니다.**

정규화 bag에 포함되는 기본 토픽은 다음과 같습니다.

```text
/ouster/points
/ouster/imu
/ublox_gps_rear/navpvt
/ublox_gps_front/navpvt
/tf_static
```

### 입력 검사

다음 예시는 `results`에 생성한 bag을 사용합니다. 기존 파일이 `data`에 있다면 입력 경로를 바꿉니다.

```bash
rosbag info /ws/results/ochang2_localization.bag
python3 scripts/audit_bag.py \
  /ws/results/ochang2_localization.bag \
  --output /ws/results/ochang2_input_audit.json
```

기본적으로 첫 12초의 점군 schema, 필터 통과 점 수, 포인트 상대 시간, LiDAR–IMU 시간 겹침과 IMU 가속도를 검사합니다. 이 보고서는 입력 검사용이며, FAST-LIO 내부 deskew의 성공을 판정하는 결과는 아닙니다.

## 4. RViz 이미지 준비 — 최초 1회, 호스트

개발 이미지에는 RViz가 없으므로 별도 이미지를 만듭니다. 이미 `robotrace:rviz`를 만들었다면 설치 확인부터 진행합니다.

```bash
cd ~/robotrace_ws
robotrace_base_image="$(sudo docker inspect -f '{{.Config.Image}}' robotrace-ws)"
sudo docker build \
  --build-arg ROBOTRACE_BASE="$robotrace_base_image" \
  -t robotrace:rviz - <<'DOCKERFILE'
ARG ROBOTRACE_BASE=robotrace:localization-cuda
FROM ${ROBOTRACE_BASE}
RUN apt-get update \
    && apt-get install -y --no-install-recommends ros-noetic-rviz \
    && rm -rf /var/lib/apt/lists/*
DOCKERFILE
```

`ARG`의 기본값은 `InvalidDefaultArgInFrom` 경고를 방지합니다. 다음 명령으로 설치를 확인합니다.

```bash
sudo docker run --rm --entrypoint dpkg-query \
  robotrace:rviz -W ros-noetic-rviz
```

## 5. 터미널 4개로 localization 시각화

같은 호스트 PC에서 개발 컨테이너와 RViz 컨테이너가 host network를 공유하는 구성입니다. 아래 loopback 주소는 이 단일 PC 구성에만 사용합니다.

| 터미널 | 실행 위치 | 역할 |
|---|---|---|
| 1 | `robotrace-ws` | FAST-LIO + GNSS 초기화 + GPU 정합 |
| 2 | `robotrace-ws` | 전체 prior PCD 발행 |
| 3 | 호스트 → `robotrace-rviz` | RViz 화면 |
| 4 | `robotrace-ws` | 검증 bag 재생 |

화면이 필요 없는 성능 검증은 터미널 1·4만 사용합니다. 결과를 비교할 때는 RViz 실행 여부도 기록합니다.

터미널 1·2·4는 각각 호스트에서 다음 명령으로 접속합니다.

```bash
cd ~/robotrace_ws
./scripts/container.sh shell --name robotrace-ws
```

### 터미널 1 — backend 시작

컨테이너 내부에서:

```bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_IP=127.0.0.1
./scripts/run.sh bag \
  --map /ws/data/map_enu_v010.pcd \
  --metrics /ws/results/ochang2_visual_metrics.csv
```

ROS master는 `roslaunch`가 시작하므로 별도 `roscore`는 필요하지 않습니다. GPU 이름과 `prior map: ... raw / ... target points` 로그가 표시되면 지도 로딩이 끝난 것입니다.

CSV는 실행 시 생성되므로 미리 만들 필요가 없습니다. **동일한 metrics 경로로 다시 실행하면 기존 CSV를 덮어씁니다.** 비교 실험은 파일명을 바꿉니다.

### 터미널 2 — 전체 prior map 발행

컨테이너 내부에서:

```bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_IP=127.0.0.1
rosrun pcl_ros pcd_to_pointcloud \
  /ws/data/map_enu_v010.pcd \
  _frame_id:=map _latch:=true _interval:=0.0 \
  cloud_pcd:=/robotrace/prior_map __name:=prior_map_view
```

이 터미널을 계속 열어 둡니다. 전체 지도는 확인용으로 한 번 발행하고 latch로 유지합니다. bag의 `/clock`이 시작되기 전에는 발행이 대기할 수 있으므로 화면에 지도 표시를 기다리느라 bag 시작을 미루지 않습니다.

### 터미널 3 — RViz 시작

**호스트의 그래픽 데스크톱 터미널**에서 실행합니다. `DISPLAY`가 설정되어 있고 X11 또는 XWayland를 사용할 수 있어야 합니다.

```bash
xhost +si:localuser:root
sudo docker run --rm -it \
  --name robotrace-rviz --network host \
  --runtime=nvidia --gpus all \
  -e DISPLAY="$DISPLAY" \
  -e ROS_MASTER_URI=http://localhost:11311 \
  -e ROS_IP=127.0.0.1 \
  -e QT_X11_NO_MITSHM=1 \
  -e NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics,display \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --entrypoint /bin/bash \
  robotrace:rviz -lc \
  'source /opt/ros/noetic/setup.bash && exec rviz'
```

RViz 컨테이너는 ROS 토픽과 TF를 구독하므로 workspace를 마운트할 필요가 없습니다. `graphics,display`는 OpenGL/X11 접근을 위해 지정합니다. RViz를 닫은 뒤 호스트에서 권한을 회수합니다.

```bash
xhost -si:localuser:root
```

### 터미널 4 — bag 재생

컨테이너 내부에서:

```bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_IP=127.0.0.1
./scripts/replay.sh
```

기본 명령은 `results/ochang2_localization.bag`을 먼저 찾고, 없으면 `data/ochang2_localization.bag`을 찾습니다. 다른 경로를 명시하려면:

```bash
./scripts/replay.sh /ws/data/ochang2_localization.bag
```

`replay.sh`는 `--clock --pause`를 자동 적용합니다. **터미널 1에서 지도 로딩 완료를 확인한 뒤 터미널 4에서 Space를 눌러 재생**합니다. 최초 약 3초는 정지 상태에서 IMU 약 300개를 수집하는 초기화 구간입니다.

관찰을 위해 느리게 다시 재생하려면 터미널 1·4를 모두 Ctrl-C로 종료합니다. 터미널 1의 backend를 다시 시작하고 지도 로딩이 끝난 뒤, 터미널 4에서 다음 명령을 사용합니다.

```bash
./scripts/replay.sh /ws/results/ochang2_localization.bag --rate 0.5
```

정상 속도 성능 측정은 `--rate 0.5`를 사용하지 않습니다. 처음부터 다시 재생할 때는 **터미널 1의 launch와 터미널 4의 재생을 모두 Ctrl-C로 종료하고 다시 실행**합니다. 실행 중 `--loop`로 시각을 되감지 않습니다.

### RViz 설정 — 점군 3개를 색으로 구분

1. `Global Options → Fixed Frame`을 **`map`**으로 설정합니다.
2. `Add → By display type → PointCloud2`로 display를 세 개 추가합니다.
3. 각 display의 `Topic`과 색을 아래 표처럼 설정합니다.

| 이름 | Topic | Color Transformer | Color RGB | Size (Pixels) |
|---|---|---|---|---|
| Prior map | `/robotrace/prior_map` | `FlatColor` | 회색 `(160, 160, 160)` | 1 |
| Submap | `/robotrace/submap` | `FlatColor` | 파랑 `(0, 128, 255)` | 2 |
| Current LiDAR | `/lio/cloud_body` | `FlatColor` | 초록 `(0, 255, 0)` | 3 |

공통으로 `Position Transformer=XYZ`, `Style=Points`, `Alpha=1`, `Decay Time=0`을 설정합니다. `Color Transformer`를 `FlatColor`로 바꾸면 `Color`에서 색을 선택할 수 있습니다. `Intensity`를 사용하면 지정한 단색 구분이 적용되지 않습니다.

추가로 `Path` display에 `/robotrace/path`, `Pose` display에 `/robotrace/pose`를 설정합니다. Pose 화살표는 **IMU 원점의 자세**입니다.

`/lio/cloud_body`는 `lio_imu` 좌표계 점군이지만, RViz가 `map → odom → lio_imu` TF를 적용해 지도 위에 표시합니다. header의 frame 이름만 `map`으로 바꾸거나 임의의 identity `map → odom` TF를 추가하면 안 됩니다.

- **Prior map**: 전체 사전 지도. 터미널 2가 발행합니다.
- **Submap**: 현재 예측 위치 주변에서 잘라낸 GPU 정합용 지도. backend가 GNSS 초기화 후 만들고 발행합니다.
- **Current LiDAR**: FAST-LIO가 움직임을 보정한 현재 스캔. 전체 지도와 겹치는지 확인합니다.

전체 지도와 submap이 겹쳐 색이 가려지면 Prior map display를 잠시 끄고 파란 지도와 초록 스캔을 비교합니다. `Panels → Views`에서 `TopDownOrtho`를 선택하면 평면 궤적을 보기 쉽습니다. 3D 확인은 `Orbit`으로 바꾸고 `Distance`를 약 100 m부터 조정합니다.

## 6. 상태와 측정 결과 확인

### 상태 모니터링 — 별도 개발 컨테이너 터미널

```bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_IP=127.0.0.1
rostopic echo /robotrace/localization_status
```

다음 명령들은 각각 계속 실행됩니다. 한 명령을 Ctrl-C로 종료한 뒤 다음 명령을 실행합니다.

```bash
rostopic echo /robotrace/localization_valid
```

```bash
rostopic hz /lio/odom
```

```bash
rostopic hz /robotrace/pose
```

| 상태 | 의미 |
|---|---|
| `WAIT_GNSS` | 유효한 GNSS 초기값 또는 필요한 LIO 입력 대기 |
| `GNSS_SEEDED` | GNSS 초기값 확보, 아직 지도 정합 승인 전 |
| `TRACKING` | 지도 정합 품질 기준 통과 |
| `DEGRADED` | 추적 이후 정합 거부 발생 |
| `LOST` | 연속 정합 거부 10회. 정합 시도는 계속 수행 |
| `STALE_INPUT` | ROS 시각 기준으로 입력·결과가 오래됨 |
| `ERROR` | CUDA 또는 처리 예외 |

정합 거부 중에도 마지막 전역 보정과 LIO로 Pose가 계속 발행될 수 있습니다. **화살표가 움직이는 것만으로 정상 측위를 판정하지 말고 상태와 `localization_valid`를 함께 확인**합니다. bag을 pause하면 ROS 시각도 멈추므로 이 상태 감시는 실제 벽시계 정지 시간을 측정하지 않습니다.

### metrics 요약 — bag 재생 종료 후

```bash
python3 scripts/summarize_metrics.py \
  /ws/results/ochang2_visual_metrics.csv \
  --output /ws/results/ochang2_visual_summary.json
cat /ws/results/ochang2_visual_summary.json
```

CSV는 backend가 생성하고, 요약 도구는 기존 CSV에서 JSON을 만듭니다. `preprocess_ms`, `registration_ms`, `validation_ms`, `total_ms`의 단위는 ms이며 `scan_age_s`, `result_age_s`는 초입니다.

정합 거부 원인과 시도 빈도를 확인하려면:

```bash
python3 - <<'PY'
import csv
from collections import Counter

path = '/ws/results/ochang2_visual_metrics.csv'
with open(path, newline='') as f:
    rows = list(csv.DictReader(f))
print('기록된 시도 수:', len(rows))
print('정합 거부 원인:', dict(Counter(
    r['reason'] for r in rows if r['state'] != 'TRACKING'
)))
if len(rows) > 1:
    span = float(rows[-1]['stamp']) - float(rows[0]['stamp'])
    if span > 0:
        print('실제 정합 시도 빈도 [Hz]:', round((len(rows)-1)/span, 3))
PY
```

5 Hz 정합의 목표 주기는 200 ms입니다. 평균 정합 시간뿐 아니라 `total_ms` p95/p99, `result_age_s`, 연속 거부 구간을 함께 봅니다. `tracking_fraction`의 분모는 **CSV에 기록된 처리 시도**이며 전체 입력 스캔이나 운행 시간은 아닙니다. `dropped_scans`에는 20 Hz 입력을 5 Hz 정합에 맞추면서 건너뛴 스캔도 포함됩니다.

### 참고 실행 기록

RTX 5060 Ti 개발 환경에서 기록한 `ochang2_gpu_metrics.csv`와 요약 로그 기준입니다. 원본 결과 파일은 저장소에 포함하지 않으며, 아래 수치는 참고 기록입니다. 당시 코드 commit을 기록한 재현용 benchmark는 아니므로 새로운 실행이나 릴리스의 성능을 보장하지 않습니다.

| 항목 | 확인값 |
|---|---|
| bag 길이 | 약 192초 |
| 기록된 처리 시도 | 836회 |
| 상태 | TRACKING 768, DEGRADED 67, GNSS_SEEDED 1 |
| 정합 승인 비율 | 기록된 시도 기준 약 91.87% |
| 거부 사유 | `not_converged` 68회 |
| 실제 시도 빈도 | 약 4.423 Hz |
| 정합 시간 p50 / p95 | 약 2.63 / 6.59 ms |
| 전체 처리 p50 / p95 / p99 | 약 4.69 / 8.15 / 146.22 ms |
| 전체 처리 최댓값 | 약 184.82 ms |
| 결과 age p95 / 최댓값 | 약 0.0529 / 0.2232초 |

시간 백분위는 첫 기록 행을 제외한 steady 요약을 사용했고, 최댓값은 전체 기록 기준입니다. 느린 행은 주로 전처리 시간이 길었습니다. 주변 지도 갱신 비용이 원인일 가능성은 있지만 현재 CSV만으로 확정하지 않습니다.

재현 실험에서는 코드 commit, 설정 파일, 입력 파일 checksum, 재생 속도, RViz 실행 여부를 결과와 함께 기록합니다. 예를 들어 호스트에서 다음 명령으로 입력과 설정을 식별할 수 있습니다.

```bash
cd ~/robotrace_ws
git rev-parse HEAD
sha256sum \
  src/fast_lio/config/ouster.yaml \
  src/robotrace_localization/config/localization.yaml \
  data/map_enu_v010.pcd
```

`not_converged`는 수렴 검사에서 거부됐다는 뜻입니다. 해당 행은 뒤의 품질 검사를 모두 수행하지 않았을 수 있어, 이 사유만으로 점군 겹침이나 위치 정확도를 판정할 수 없습니다. CSV에는 프로그램 시작부터의 지도 로딩·CUDA context 생성·GNSS 대기 시간, CPU 사용률, 절대 위치 오차가 포함되지 않습니다.

## 7. 구현 구조와 수정할 설정

### 처리 흐름

1. FAST-LIO2가 LiDAR·IMU로 deskew와 로컬 odometry를 계산합니다. FAST-LIO는 prior PCD를 직접 읽지 않고 로컬 점군 지도를 사용합니다.
2. rear/front RTK FIX의 측정 시각을 맞춰 rear ENU 위치와 전후 baseline yaw를 계산합니다. LIO 중력과 lever arm을 이용해 IMU 위치·자세로 변환합니다.
3. 이 초기 자세로 `map → odom`을 만들고, 예측 위치 주변의 prior map을 잘라 CUDA VGICP 정합을 수행합니다.
4. 정합 결과가 품질 기준을 통과하면 `map → odom`을 갱신합니다. LIO Pose를 이 전역 변환으로 지도 좌표계에 표현합니다.

**GNSS는 초기화와 요청에 따른 재초기화에 사용합니다. 주행 중 GNSS를 EKF나 factor graph에 계속 융합하는 기능은 현재 포함하지 않습니다.** GPU 정합 보정도 FAST-LIO EKF 내부로 되먹임하지 않습니다. GLIM loop closure는 사전 지도 제작 단계의 기능입니다.

### 설정 파일과 적용 방법

| 파일 | 수정할 내용 |
|---|---|
| `src/fast_lio/config/ouster.yaml` | 입력 토픽, 채널 수, 시간 단위, blind, LiDAR–IMU extrinsic |
| `src/robotrace_localization/config/localization.yaml` | ENU 원점, GNSS 조건, lever arm, 정합 범위·주기·품질 조건 |
| `src/fast_lio/launch/odometry.launch` | LIO 노드와 실행 파라미터 |
| `src/robotrace_localization/launch/localization.launch` | 전체 노드 연결, TF, launch 인자 |

YAML 변경은 launch를 재시작하면 적용됩니다. C++ 또는 CMake를 변경하면 `./scripts/build.sh`로 다시 빌드합니다. `run.sh`의 첫 인자는 반드시 `bag` 또는 `live`입니다.

```bash
./scripts/run.sh bag \
  --map /ws/data/map_enu_v010.pcd \
  --metrics /ws/results/experiment_01.csv \
  --config /ws/src/robotrace_localization/config/localization.yaml
```

`map_path`, `metrics_path`, `nearest_neighbor_method`는 launch에서 YAML 이후에 설정됩니다. 지도·CSV 경로는 `--map`, `--metrics`로 지정합니다. 이웃 검색 방식을 바꾸려면 launch 인자를 직접 지정합니다.

```bash
roslaunch robotrace_localization localization.launch \
  use_sim_time:=true \
  map_path:=/ws/data/map_enu_v010.pcd \
  metrics_path:=/ws/results/ochang2_bruteforce_metrics.csv \
  nearest_neighbor_method:=GPU_BRUTEFORCE
```

기본값 `CPU_PARALLEL_KDTREE`는 공분산 계산용 이웃 검색을 CPU에서 수행한다는 의미이며 CUDA VGICP 정합은 활성화되어 있습니다. `GPU_BRUTEFORCE`나 `GPU_RBF_KERNEL`이 더 빠른지는 동일 bag에서 측정해 결정합니다.

### GNSS 초기화 조건

| 항목 | 현재 값 또는 조건 |
|---|---|
| ENU 원점 | 위도 `36.729539`, 경도 `127.4408323`, 타원체 높이 `103.230 m` |
| 원점 근거 | `ochang.bag` 첫 rear RTK FIX를 원점으로 썼다는 가정 |
| RTK 검사 | 양쪽 NavPVT의 `(flags & 0xC3) == 0x83`, `fixType=3`, 유효한 UTC |
| 시간 매칭 | UTC/iTOW 측정 epoch 차이 ≤ 0.01초 |
| 수평 / 수직 accuracy | 각각 ≤ 0.2 / 0.5 m |
| 수평 안테나 baseline | 0.8–2.2 m — 실측 장착 길이와 비교 필요 |
| yaw | `atan2(front_N-rear_N, front_E-rear_E)` + offset, 기본 offset 0 |
| IMU→rear 벡터 | IMU 좌표계 `[-0.4213, 0.0118, -0.4876] m` |
| IMU 위치 | `p_map_imu = p_map_rear - R_map_imu * t_imu_rear` |
| 초기 roll/pitch | LIO 중력 방향으로 계산 |

`/robotrace/gnss_seed`의 위치는 rear 안테나 위치입니다. backend에서 lever arm을 적용한 `/robotrace/pose`는 IMU 원점입니다. GNSS 시각을 둘러싸는 LIO 자세가 필요하며, 자세 보간 간격 한도는 0.12초입니다.

ENU 원점은 GLIM 출력과 아직 독립 확인되지 않았습니다. GNSS 초기값이 있어도 좌표계 오프셋·yaw 오프셋·지도 오차가 존재할 수 있으므로 첫 정합을 통해 확인합니다.

backend 재초기화를 요청하려면:

```bash
rosservice call /gpu_map_localizer/reset "{}"
```

이 명령은 FAST-LIO 자체나 bag 시각을 초기화하지 않습니다. `LOST`에서 GNSS 재초기화를 자동 수행하지는 않습니다.

### LiDAR·IMU 입력과 TF

| 항목 | 현재 구성 |
|---|---|
| LiDAR / IMU 토픽 | `/ouster/points` / `/ouster/imu` |
| 입력 frame | `os_sensor` / `os_imu` |
| Ouster 채널 / 상대 시간 | 64 / `t` UINT32, ns |
| 필수 점군 필드 | x/y/z/intensity FLOAT32, t UINT32, ring UINT16 |
| 입력 속도 | LiDAR 약 20 Hz, IMU 약 100 Hz |
| LiDAR→IMU 회전 / 이동 | I / `[-0.006253, 0.011775, -0.007645] m` |
| TF 연결 | `map → odom → lio_imu → os_sensor` |

`lio_imu`는 `os_imu`와 같은 물리 좌표계에 붙인 별도 이름입니다. bag의 `os_sensor → os_imu`, `os_sensor → os_lidar` TF와 parent 충돌을 피합니다. 점군이 이미 `os_sensor`에 표현되어 있으므로 `os_sensor → os_lidar`의 180° 회전을 다시 적용하지 않습니다.

### 현재 정합 기준

| 분류 | 기준 |
|---|---|
| 정합 주기 | 5 Hz |
| source / target voxel | 0.3 / 0.3 m, 원본 PCD 변경 없음 |
| source 거리 | 4–60 m |
| target 영역 | 수평 반경 80 m, 수직 ±20 m |
| target 갱신 | 수평 이동 10 m 또는 수직 이동 5 m 이상 |
| VGICP | resolution 1.0 m, 최대 30회 반복, CPU 이웃 검색 2 threads |
| 최소 점 수 | source 200, target 1,000 |
| 수렴·수치 검사 | 수렴 완료, 유한한 변환과 Hessian |
| 예측 대비 보정량 | 이동 ≤ 2 m, 3D 회전 ≤ 15° |
| Hessian 조건 | 정규화된 고유값 비율 > `1e-8` |
| 검증 대응점 | 최대 1,000개 샘플, 거리 ≤ 1 m를 inlier로 분류 |
| 품질 통과 | inlier 비율 ≥ 0.5, inlier RMSE ≤ 0.5 m |
| 시간 조건 | scan age와 result age 한도 각각 1초 |

이 값들은 현재 승인·거부 기준입니다. 0.5 m RMSE 제한을 차량 위치 오차 0.5 m 보장으로 해석하면 안 됩니다. 기준 완화는 거부를 줄일 수 있지만 잘못된 정합 승인도 늘릴 수 있으므로, 영상·상태·독립 기준 위치와 함께 검증합니다.

## 8. 자주 발생하는 문제

### ROS master 연결 거부

`Failed to contact master at [localhost:11311]`이면 터미널 1의 `run.sh bag`을 먼저 실행합니다. 재생은 launch가 master와 노드를 시작한 다음 수행합니다. 터미널과 RViz 컨테이너의 `ROS_MASTER_URI`도 확인합니다.

### RViz 화면이 비어 있음

호스트에서 RViz 컨테이너에 접속합니다.

```bash
sudo docker exec -it robotrace-rviz bash
```

접속한 컨테이너에서 먼저 확인합니다.

```bash
source /opt/ros/noetic/setup.bash
rostopic list
timeout 5s rostopic echo -n 1 /clock
timeout 5s rostopic echo -n 1 /robotrace/prior_map/header
timeout 5s rostopic echo -n 1 /robotrace/prior_map/width
```

현재 점군의 지도 TF는 별도로 확인합니다. 다음 명령은 Ctrl-C로 종료합니다.

```bash
rosrun tf tf_echo map lio_imu
```

| 증상 | 확인할 내용 |
|---|---|
| `rostopic list`도 실패 | master 실행 여부, host network, ROS 주소 |
| `/clock`이 안 옴 | bag 시작 여부, Space 해제, 재생 종료 여부 |
| `/clock`은 오지만 prior map 없음 | 터미널 2, PCD 경로, publisher 실행 여부 |
| 지도만 보이고 현재 점군 없음 | GNSS 초기화, LIO 출력, `map → lio_imu` TF |
| display 상태 OK인데 점이 안 보임 | Fixed Frame `map`, 카메라 위치·거리, Alpha·Size |
| 색이 구분되지 않음 | 각 display의 `FlatColor`, 전체 지도 겹침 여부 |

전체 prior map 표시는 GNSS 초기화가 없어도 가능합니다. 현재 점군을 map 좌표계에 표시하려면 GNSS 초기화와 TF가 필요합니다.

### `I have no name!`, group ID 1000 경고

일반 `sudo docker exec -it robotrace-ws bash`는 호스트 UID/GID에 대응하는 사용자 이름과 전용 bash 환경을 불러오지 않을 수 있습니다. 권장 접속은 다음과 같습니다.

```bash
cd ~/robotrace_ws
./scripts/container.sh shell --name robotrace-ws
```

이미 컨테이너에 접속했다면:

```bash
exec bash --rcfile /ws/docker/bashrc
```

### 컴파일러 캐시 / `catkin_workspace` 오류

`Cached C compiler differs`이면 `clean.sh --yes` 후 재빌드합니다. `Unknown CMake command "catkin_workspace"`이면 컨테이너에서 공식 catkin toplevel 파일로 복구하고 재빌드합니다.

```bash
cp /opt/ros/noetic/share/catkin/cmake/toplevel.cmake /ws/src/CMakeLists.txt
./scripts/clean.sh --yes
./scripts/build.sh --jobs 1 --cuda-arch 120
```

`src/CMakeLists.txt`는 일반 파일로 유지합니다. 기존 workspace의 `build`, `devel_cuda` 환경 대신 현재 `.build/devel`을 사용합니다.

### 점군이 처리되지 않음

| 로그 | 확인할 내용 |
|---|---|
| `LIO_BAD_CLOUD_FIELD` / `LIO_BAD_CLOUD_LAYOUT` | 필드 datatype·offset·payload 크기 |
| `LIO_EMPTY_INPUT` | NaN/Inf, ring 범위, t 범위, blind 필터 |
| `LIO_WAIT_IMU` | scan end까지의 IMU 수신과 시간 기준 |
| `LIO_INITIALIZING` | 초기 IMU 약 300개, 차량 정지 |
| `LIO_EMPTY_DESKEW` | 초기화 이후 deskew 입력·출력 |
| `LIO_TOO_FEW_VOXEL` | voxel 처리 후 유효 점 수 |
| `LIO_TIME_REWIND` | bag 처음부터 재생 시 launch도 재시작했는지 |

Ouster `ring`은 UINT16을 사용합니다. `t`가 0.1초를 넘거나 ring이 64채널 범위를 벗어난 점 등은 제외합니다. `/robotrace/submap`이 없으면 입력 점군뿐 아니라 GNSS seed와 LIO 초기화부터 확인합니다.

### 실행 인자 / 파일 권한

`First argument must be bag or live`이면 명령을 다음 형태로 다시 입력합니다.

```bash
./scripts/run.sh bag --metrics /ws/results/ochang2_visual_metrics.csv
```

스크립트 실행 권한이 없다면 호스트에서:

```bash
cd ~/robotrace_ws
chmod u+x scripts/*.sh
```

VS Code에서 소스 수정이 막히고 root 소유로 확인되면 **호스트**에서 소유권을 복구합니다.

```bash
ls -ld ~/robotrace_ws/src
sudo chown -R "$(id -u):$(id -g)" "$HOME/robotrace_ws/src"
```

## 9. 실시간 운용과 AGX Orin 이식

실시간 실행은 센서 드라이버를 별도로 준비한 다음 컨테이너에서 시작합니다.

```bash
./scripts/run.sh live \
  --map /ws/data/map_enu_v010.pcd \
  --metrics /ws/results/live_gpu_metrics.csv
```

LiDAR·IMU·GNSS 측정 시각은 같은 기준이어야 하며, 시작할 때 차량을 정지시킵니다. 다른 PC의 센서 드라이버와 통신할 경우 `ROS_IP=127.0.0.1` 대신 실제 접근 가능한 주소를 설정합니다. 이 workspace에는 센서 드라이버나 주행 제어기가 포함되어 있지 않습니다.

현재 Dockerfile은 **x86_64 개발 PC용**입니다. AGX Orin에서는 JetPack/L4T에 맞는 ARM64 CUDA·ROS Noetic 환경을 따로 구성하고 소스를 장치에서 다시 빌드해야 합니다. 환경 준비 후 대상 architecture는 다음과 같습니다.

```bash
./scripts/build.sh --jobs 1 --cuda-arch 87
```

단, 현재 빌드 스크립트는 GCC 10을 요구하므로 Orin의 CUDA host compiler 호환성을 먼저 확인하고 필요하면 환경과 스크립트를 조정합니다. RTX용 이미지나 `.build` 복사만으로 이식이 끝나지는 않습니다. Orin에서는 처리 시간 p95/p99, CPU/GPU 사용률, 메모리, 온도와 측위 품질을 다시 측정합니다.

## 10. Git 저장소 관리 — 호스트

### 포함·제외 정책

| Git에 포함 | Git에서 제외 |
|---|---|
| `src/`, 필요한 `thirdparty/`, `docker/`, `scripts/`, `tests/` | `.build/`와 이전 `build_*`, `devel_*` 등의 빌드 출력 |
| YAML·launch·CMake·manifest·README·라이선스 고지 | `data/`의 PCD와 bag |
| `data/.gitkeep`, `results/.gitkeep`, 선택적으로 작성한 각 폴더의 `README.md` | `results/`의 CSV·JSON·정규화 bag·ROS 로그 |

전체 입력과 실행 결과는 로컬에 보관하거나 별도 데이터 공유 경로로 전달합니다. 현재 Git LFS와 submodule은 사용하지 않습니다. `.gitignore`는 로컬 파일을 삭제하지 않고 새 Git 추적 대상에서 제외합니다.

### 최초 commit 전에 확인

아직 Git 저장소가 아닌 workspace에서만 `git init -b main`을 실행합니다. 상위 폴더의 저장소에 속해 있는지 먼저 확인하려면 `git rev-parse --show-toplevel`을 사용합니다. 출력이 workspace 경로와 다르면 상위 저장소에 잘못 추가하지 않도록 Git 경계를 먼저 확인합니다.

```bash
cd ~/robotrace_ws
find src -name .git -print
git init -b main
```

현재 소스 동봉 방식이라면 `find` 출력이 없어야 합니다. 중첩 `.git`이 발견되면 내부 변경·이력을 확인한 뒤 진행합니다. `.gitmodules`는 현재 구성에 필요하지 않습니다.

제외 규칙을 검사하고 commit 후보를 확인합니다.

```bash
git check-ignore -v \
  .build/catkin/CMakeCache.txt \
  data/ochang2.bag \
  results/ros/example.log

git add .
git diff --cached --stat
git diff --cached --check -- \
  README.md .gitignore scripts docker tests src/robotrace_localization
git ls-files --stage | awk '$1 == "160000" {print}'
git ls-files --stage scripts/build.sh docker/entrypoint.sh
```

`check-ignore`는 위 세 경로에 적용되는 제외 규칙을 표시해야 합니다. 공백 검사는 문서·도구·자체 localization 패키지에 적용합니다. mode `160000` 검사에는 출력이 없어야 하고, 직접 실행하는 shell 파일의 mode는 `100755`여야 합니다. mode가 `100644`이면 실행 권한을 복구하고 다시 stage합니다.

```bash
chmod u+x scripts/*.sh docker/entrypoint.sh
git add scripts docker/entrypoint.sh
```

큰 파일이 실수로 stage됐는지 확인하려면 다음 읽기 전용 검사를 사용합니다. 출력이 있으면 해당 파일의 포함 필요성을 검토합니다.

```bash
python3 - <<'PY'
import pathlib
import subprocess

paths = subprocess.check_output(['git', 'ls-files', '-z']).split(b'\0')
for raw in paths:
    if not raw:
        continue
    path = pathlib.Path(raw.decode('utf-8', errors='surrogateescape'))
    if path.is_file() and not path.is_symlink():
        size = path.stat().st_size
        if size > 50 * 1024 * 1024:
            print(f'{size / 1024**2:.1f} MiB: {path}')
PY
```

### 이미 추적 중인 데이터가 있을 때

`.gitignore`는 이미 추적하는 파일에는 적용되지 않습니다. 먼저 추적 목록을 확인합니다.

```bash
git ls-files -- .build data results
```

목록에 제외할 데이터·출력이 있다면 Git index에서만 제거하고, 제외 규칙에 따라 다시 stage합니다. 아래 명령은 로컬 입력과 결과 파일을 보존합니다.

```bash
git rm -r --cached --ignore-unmatch .build data results
git add .
git diff --cached --stat
```

이미 commit한 큰 파일은 이 과정만으로 과거 commit에서 사라지지 않습니다. 최초 게시 전에 현재 index와 기존 이력을 구분해 확인합니다.

### commit과 원격 연결

위 검사를 마친 뒤 commit합니다. GitHub에는 README·LICENSE 자동 생성 없이 빈 저장소를 먼저 만들고, URL의 `OWNER`와 저장소 이름을 실제 값으로 바꿉니다. 아래 예시는 SSH 인증이 준비되어 있을 때 사용합니다.

```bash
git commit -m "Initial FAST-LIO2 CUDA localization workspace"
robotrace_remote_url='git@github.com:OWNER/robotrace_ws.git'
git remote add origin "$robotrace_remote_url"
git push -u origin main
```

commit에서 `Author identity unknown`이 나오면 아래 값을 본인의 이름과 이메일로 바꿔 이 저장소의 작성자 정보를 설정한 뒤 commit을 다시 실행합니다.

```bash
git config user.name "YOUR_NAME"
git config user.email "YOUR_EMAIL"
```

이미 `origin`이 설정되어 있으면 먼저 `git remote -v`로 확인합니다. URL 변경이 필요한 경우에만 `git remote set-url origin "$robotrace_remote_url"`을 사용합니다. 기존 원격 commit이 있는 저장소에는 위 빈 저장소 절차를 그대로 적용하지 않습니다.

게시 후에는 별도 경로에 새로 clone하고 기존 개발 컨테이너와 다른 이름으로 접속해 `build.sh`, `test.sh`, `doctor.sh`를 실행합니다. 빌드 캐시 없이 소스와 명시한 개발 환경으로 빌드되는지 확인하는 절차입니다.

## 11. 소스 출처와 라이선스

- FAST-LIO는 제공받은 FAST_LIO_LOCALIZATION snapshot에서 파생했습니다. 정확한 입력 snapshot의 원본 repository·commit은 현재 확인되지 않았으며, 공식 FAST-LIO 최신 버전과 동일하다고 간주하지 않습니다. 원본 [LICENSE](src/fast_lio/LICENSE)와 파일별 저작권 고지를 보존합니다.
- Fast GICP upstream: [koide3/fast_gicp](https://github.com/koide3/fast_gicp), 고정 commit `0e7ec1441c99f7be453db2ea216d5de029387417`. CUDA architecture, Thrust 헤더, `setResolution()` 및 catkin 연결 수정은 [UPSTREAM.md](src/fast_gicp/UPSTREAM.md)에 기록되어 있습니다. 원본 revision과 원본 파일 hash는 [UPSTREAM_MANIFEST.json](src/fast_gicp/UPSTREAM_MANIFEST.json)을 확인합니다.
- Eigen/NVBIO는 필요한 소스를 함께 포함하며 upstream revision은 위 manifest에 기록되어 있습니다. 해당 디렉토리를 `.gitignore`에 추가하지 않습니다.

| 소스 범위 | 고지 파일 |
|---|---|
| FAST-LIO 및 포함 의존 소스 | [fast_lio/LICENSE](src/fast_lio/LICENSE), 각 파일의 GPL/BSD 고지 |
| Fast GICP | [fast_gicp/LICENSE](src/fast_gicp/LICENSE) |
| CUDA Eigen 헤더 | [Eigen/COPYING.README](src/fast_gicp/thirdparty/Eigen/COPYING.README)와 같은 폴더의 `COPYING.*` |
| NVBIO 헤더 | [NVBIO/LICENSE](src/fast_gicp/thirdparty/nvbio/LICENSE) |

`robotrace_localization/package.xml`의 BSD-3-Clause 선언은 전체 동봉 소스에 대한 단일 라이선스 선언이 아닙니다. 현재 자체 패키지의 별도 LICENSE 전문과 maintainer 정보 보완은 배포 메타데이터 정리 항목이며, 이 README가 그 파일들을 대신하지 않습니다. 원본 저작권·라이선스 고지는 유지합니다.

실행 절차와 위 검증 범위는 이 루트 README를 기준으로 확인합니다.

공식 참고 문서:

- [Git ignore 규칙과 이미 추적 중인 파일](https://git-scm.com/docs/gitignore)
- [Git submodule과 commit 고정 방식](https://git-scm.com/docs/gitsubmodules)
- [GitHub 일반 Git 파일 크기 제한](https://docs.github.com/en/repositories/working-with-files/managing-large-files/about-large-files-on-github)
- [GitHub Git LFS와 파일 크기 제한](https://docs.github.com/en/repositories/working-with-files/managing-large-files/about-git-large-file-storage)
- [NVIDIA Container Toolkit 설치](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
- [NVIDIA Docker GPU·그래픽 capability](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/docker-specialized.html)
- [Docker ARG 기본값 검사](https://docs.docker.com/reference/build-checks/invalid-default-arg-in-from/)
