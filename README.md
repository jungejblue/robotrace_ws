# robotrace_ws

ROS1 Noetic에서 **FAST-LIO2 로컬 odometry + FastVGICPCuda 사전 지도 정합 + 전후 RTK GNSS 초기화**를 실행하는 workspace입니다. Ubuntu 22.04 호스트에서 Ubuntu 20.04/Noetic/CUDA 개발 컨테이너를 사용합니다.

FAST-LIO가 LiDAR·IMU로 로컬 자세를 추정하고, GNSS가 지도 좌표계 초기값을 제공합니다. GPU 정합이 `map → odom`을 보정합니다. **GNSS는 초기화·요청에 따른 재초기화에 사용하며 주행 중 EKF/factor graph에 계속 융합하지 않습니다.** GPU 보정도 FAST-LIO EKF에 되먹임하지 않습니다.

RTX 5060 Ti 환경의 기존 구성에서 CUDA 빌드·`ochang2` bag 재생·metrics 생성이 확인되었습니다. 이 수정본의 전체 CUDA/ROS 재빌드와 실제 bag 재검증은 사용자 환경에서 수행해야 합니다. GLIM 지도와 ENU 원점의 일치, 절대 위치 정확도, AGX Orin 성능은 별도 검증 대상입니다.

## 준비

다음 명령은 **호스트**에서 실행합니다.

```bash
git clone https://github.com/jungejblue/robotrace_ws.git "$HOME/robotrace_ws"
cd "$HOME/robotrace_ws"
mkdir -p data results
nvidia-smi
nvidia-smi --query-gpu=name,compute_cap --format=csv
```

이미 clone했다면 해당 폴더를 사용합니다. 수정된 FAST-LIO·fast_gicp와 필요한 Eigen/NVBIO를 함께 포함하므로 **submodule 초기화가 필요하지 않습니다.** 동봉한 `src/fast_gicp`를 upstream으로 덮어쓰지 않습니다.

| 항목 | 기본 개발 구성 |
|---|---|
| 호스트 | Ubuntu 22.04, NVIDIA 드라이버·Container Toolkit |
| GPU | RTX 5060 Ti, compute capability 12.0 |
| 컨테이너 | Ubuntu 20.04, ROS1 Noetic, CUDA 12.8.1, GCC/G++ 10 |
| workspace | 호스트 `~/robotrace_ws` → 컨테이너 `/ws` |
| 개발 이미지 / 컨테이너 | `robotrace:localization-cuda` / `robotrace-ws` |
| RViz 이미지 / 컨테이너 | `robotrace:rviz` / `robotrace-rviz` |

PCD와 bag은 저장소에 포함하지 않습니다. 별도로 확보한 파일을 배치합니다. 입력 형식과 준비 방법은 [data/README.md](data/README.md)를 확인합니다.

| 용도 | 파일 |
|---|---|
| 사전 지도 | `data/map_enu_v010.pcd` |
| 검증 입력 | `data/ochang2.bag` 또는 준비된 `data/ochang2_localization.bag` |
| 지도 제작 기록 확인 — 선택 사항 | `data/ochang.bag` |

```bash
cp "$HOME/Downloads/map_enu_v010.pcd" data/
cp "$HOME/Downloads/ochang2.bag" data/
ls -lh data/map_enu_v010.pcd data/ochang2.bag
sudo docker run --rm --runtime=nvidia --gpus all \
  nvidia/cuda:12.8.1-base-ubuntu20.04 nvidia-smi
```

입력 경로는 실제 파일 위치에 맞춥니다. GPU 접근 확인이 실패하면 [NVIDIA Container Toolkit 공식 설치 안내](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)에 따라 설치한 뒤 호스트에서 runtime을 설정합니다.

```bash
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

Docker 재시작은 실행 중인 컨테이너에 영향을 줍니다. `nvidia-smi`의 CUDA Version은 드라이버 지원 버전이며 컨테이너 Toolkit 버전과 다릅니다.

## 컨테이너 생성·빌드

**호스트**에서:

```bash
cd ~/robotrace_ws
./scripts/container.sh build
./scripts/container.sh start --data-dir "$PWD/data"
./scripts/container.sh shell
```

기본 컨테이너 이름은 `robotrace-ws`입니다. `--data-dir`로 생성한 컨테이너는 `/ws/data`를 읽기 전용으로 연결합니다. 변환 bag·결과는 `/ws/results`에 기록합니다. 기존 컨테이너의 이미지·마운트는 `start`로 바뀌지 않으므로 다른 구성이 필요하면 새 `--name`을 사용합니다.

**컨테이너 내부**에서:

```bash
cd /ws
./scripts/build.sh --jobs 1 --cuda-arch 120
./scripts/test.sh
./scripts/doctor.sh
```

빌드는 Release·GCC/G++ 10을 사용합니다. 메모리가 충분하면 `--jobs 2`로 바꿀 수 있습니다. `test.sh`는 오프라인 도구와 좌표 계산, `doctor.sh`는 GPU·ROS·실행 파일·라이브러리 연결을 확인합니다. 측위 정확도는 별도로 평가해야 합니다.

```bash
ls -lh \
  .build/devel/lib/libfast_vgicp_cuda.so \
  .build/devel/lib/fast_lio/fastlio_mapping \
  .build/devel/lib/robotrace_localization/gnss_initializer_node \
  .build/devel/lib/robotrace_localization/gpu_map_localizer_node
cuobjdump --list-elf .build/devel/lib/libfast_vgicp_cuda.so
```

RTX 5060 Ti용 device binary에는 `sm_120`이 표시되어야 합니다. 새 호스트 터미널에서는 다음 명령으로 접속하며 ROS와 `.build/devel` 환경은 자동으로 불러옵니다.

```bash
cd ~/robotrace_ws
./scripts/container.sh start
./scripts/container.sh shell
```

일반 재빌드는 `build.sh`를 사용합니다. 컴파일러·CUDA 대상이 바뀌거나 캐시 충돌이 발생하면 `.build`만 삭제하고 다시 빌드합니다.

```bash
./scripts/clean.sh --yes
./scripts/build.sh --jobs 1 --cuda-arch 120
```

## 검증 bag 준비

이미 준비된 `ochang2_localization.bag`이 있으면 이 변환을 건너뜁니다. 원본 bag만 있으면 **컨테이너에서** 실행합니다.

```bash
python3 scripts/prepare_validation_bag.py \
  /ws/data/ochang2.bag \
  /ws/results/ochang2_localization.bag
rosbag info /ws/results/ochang2_localization.bag
python3 scripts/audit_bag.py \
  /ws/results/ochang2_localization.bag \
  --output /ws/results/ochang2_input_audit.json
```

원본은 보존하며 정규화 bag과 `ochang2_localization.time_alignment.json`을 생성합니다. 기존 출력이 있으면 다른 파일명을 사용합니다. LiDAR·IMU header에 공통 상수 offset을 적용하고 점군 payload의 상대 시간 `t`를 보존합니다. clock reset·과도한 drift·시간 기준 불일치를 발견하면 중단합니다. 이 처리는 실제 센서의 하드웨어 동기화 검증을 대신하지 않습니다.

입력 검사는 기본 첫 12초의 schema·유효 점·상대 시간·IMU 겹침을 확인합니다. FAST-LIO deskew 성공이나 측위 정확도 판정은 실제 실행으로 확인합니다.

## 화면 없이 bag 검증

호스트 터미널 두 개에서 각각 `./scripts/container.sh shell`로 **같은 `robotrace-ws` 컨테이너**에 접속합니다. 단일 PC에서 각 개발 컨테이너 터미널에 다음 환경을 설정합니다. 새 터미널에는 다른 터미널의 환경 변수가 전달되지 않습니다.

```bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_IP=127.0.0.1
```

**터미널 1 — localization:**

```bash
cd /ws
./scripts/run.sh bag \
  --map /ws/data/map_enu_v010.pcd \
  --metrics /ws/results/ochang2_gpu_metrics.csv
```

`roslaunch`가 ROS master를 시작하므로 별도 `roscore`는 필요하지 않습니다. GPU 이름과 `prior map: ... raw / ... target points` 로그를 확인합니다. CSV는 실행 중 생성되며 **같은 경로로 다시 실행하면 덮어씁니다.**

**터미널 2 — bag:**

```bash
cd /ws
./scripts/replay.sh
```

기본값은 `results/ochang2_localization.bag`을 먼저 찾고, 없으면 `data/ochang2_localization.bag`을 사용합니다. 다른 파일은 명시합니다.

```bash
./scripts/replay.sh /ws/data/ochang2_localization.bag
```

`--clock --pause`로 시작합니다. **터미널 1의 지도 로딩 완료 후 Space를 눌러 재생합니다.** 초기 IMU 약 300개를 모으는 약 3초 동안 차량은 정지 상태여야 합니다.

처음부터 재생할 때는 localization과 재생을 모두 Ctrl-C로 종료하고 다시 시작합니다. 실행 중 `--loop`로 시각을 되감지 않습니다. 관찰용 느린 재생은 `replay.sh [bag경로] --rate 0.5`이며 성능 측정에는 정상 속도를 사용합니다.

## RViz로 지도·현재 스캔 확인

**호스트 그래픽 데스크톱 터미널**에서 최초 한 번 RViz 이미지를 준비합니다. `DISPLAY`와 X11/XWayland가 필요합니다.

```bash
cd ~/robotrace_ws
./scripts/rviz.sh --build
```

다음 순서로 터미널 네 개를 사용합니다. 개발 컨테이너 터미널 1·2·4는 각각 `./scripts/container.sh shell`로 접속한 뒤 **모든 터미널에서** 아래 환경을 설정합니다. host network를 공유하는 단일 PC 구성에서 ROS 노드가 컨테이너 hostname 대신 loopback 주소를 알리도록 합니다.

```bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_IP=127.0.0.1
```

호스트의 RViz 스크립트도 기본적으로 같은 ROS 주소를 사용합니다.

| 터미널 | 위치 | 실행 |
|---|---|---|
| 1 | `robotrace-ws` | 위 `run.sh bag` 명령 |
| 2 | `robotrace-ws` | 아래 전체 prior map 발행 |
| 3 | 호스트 | `./scripts/rviz.sh` |
| 4 | `robotrace-ws` | 위 `replay.sh` 명령, 지도 로딩 후 Space |

**터미널 2:**

```bash
rosrun pcl_ros pcd_to_pointcloud \
  /ws/data/map_enu_v010.pcd \
  _frame_id:=map _latch:=true _interval:=0.0 \
  cloud_pcd:=/robotrace/prior_map __name:=prior_map_view
```

발행 터미널을 열어 둡니다. bag `/clock`이 시작되기 전에는 발행이 대기할 수 있으므로 RViz 지도가 나타나기를 기다리며 재생 시작을 미루지 않습니다.

**터미널 3 — 호스트:**

```bash
cd ~/robotrace_ws
./scripts/rviz.sh
```

스크립트가 GPU/OpenGL·X11 연결과 필요한 임시 X 접근 권한을 처리하고 저장된 [RViz 설정](src/robotrace_localization/rviz/localization.rviz)을 불러옵니다. 사용자가 이미 설정한 X 접근 권한은 종료 시 그대로 유지합니다.

| Display | Topic | 색 |
|---|---|---|
| Prior map | `/robotrace/prior_map` | 회색 |
| Submap | `/robotrace/submap` | 파랑 |
| Current LiDAR | `/lio/cloud_body` | 초록 |
| Path / Pose | `/robotrace/path`, `/robotrace/pose` | 궤적 / IMU 자세 |

Fixed Frame은 **`map`**입니다. 현재 스캔은 `lio_imu` frame으로 발행하지만 RViz가 `map → odom → lio_imu` TF로 지도에 표시합니다. 세 점군은 `FlatColor`로 구분합니다. 지도와 submap 색이 겹치면 Prior map display를 잠시 끕니다. GNSS 초기화 전에는 현재 스캔의 map TF와 submap이 없을 수 있습니다.

RViz는 확인용 부하를 추가하므로 성능 실험에는 실행 여부를 기록합니다. 화면이 비어 있으면 개발 컨테이너에서 확인합니다.

```bash
rostopic list
timeout 5s rostopic echo -n 1 /clock
timeout 5s rostopic echo -n 1 /robotrace/prior_map/header
rosrun tf tf_echo map lio_imu
```

## 상태·metrics 확인

별도 개발 컨테이너 터미널에서:

```bash
rostopic echo /robotrace/localization_status
```

위 명령을 Ctrl-C로 종료한 뒤 유효 여부나 발행 속도를 확인할 수 있습니다.

```bash
rostopic echo /robotrace/localization_valid
```

| 상태 | 의미 |
|---|---|
| `WAIT_GNSS` / `GNSS_SEEDED` | 초기값·필요한 LIO 입력 대기 / 초기값 확보 |
| `TRACKING` | 지도 정합 품질 기준 통과 |
| `DEGRADED` / `LOST` | 정합 거부 / 연속 거부 10회, 시도는 계속 수행 |
| `STALE_INPUT` / `ERROR` | ROS 시각 기준 입력·결과 지연 / 처리 예외 |

거부 중에도 마지막 전역 보정과 LIO로 Pose가 계속 발행될 수 있습니다. **Pose 이동만으로 정상 측위를 판단하지 않고 상태와 유효 플래그를 함께 봅니다.** bag pause 시 ROS 시각도 멈춥니다.

재생 종료 후:

```bash
python3 scripts/summarize_metrics.py \
  /ws/results/ochang2_gpu_metrics.csv \
  --output /ws/results/ochang2_summary.json
cat /ws/results/ochang2_summary.json
```

`tracking_fraction`은 CSV에 기록된 처리 시도 기준입니다. `dropped_scans`에는 20 Hz 입력을 5 Hz 정합에 맞추며 건너뛴 스캔도 포함됩니다. `total_ms` p95/p99와 `result_age_s`, 연속 거부를 함께 확인합니다. 결과 파일·재현 기록은 [results/README.md](results/README.md)를 참고합니다.

기존 RTX 5060 Ti 실행 기록은 192초 bag에서 836회 시도, TRACKING 768회(91.87%), `not_converged` 68회, 약 4.423 Hz였습니다. 첫 행 제외 요약의 정합 시간 p50/p95는 2.63/6.59 ms, 전체 처리 p50/p95/p99는 4.69/8.15/146.22 ms였습니다. 전체 기록 최댓값은 184.82 ms입니다. 해당 기록은 당시 commit이 고정된 benchmark가 아니며 이 수정본이나 Orin의 성능을 보장하지 않습니다. CSV는 절대 위치 정확도·전체 시작 지연·CPU 사용률을 측정하지 않습니다.

## 다른 지도·센서 또는 실시간 실행

먼저 [calibration 문서](docs/calibration.md)에 따라 두 YAML을 확인합니다.

| 파일 | 설정 |
|---|---|
| `src/fast_lio/config/ouster.yaml` | LiDAR/IMU 토픽·frame·시간·LiDAR→IMU extrinsic |
| `src/robotrace_localization/config/localization.yaml` | 공용 GNSS seed 토픽·ENU 원점·lever arm·정합 기준 |

GNSS seed 발행·구독은 공용 `robotrace/gnss_seed_topic`을 사용합니다. static TF는 FAST-LIO와 동일한 extrinsic에서 생성하므로 별도 TF 숫자를 수정하지 않습니다. YAML 변경은 launch 재시작, C++ 변경은 재빌드 후 적용합니다.

실시간 센서 드라이버를 별도로 준비한 뒤 **컨테이너**에서:

```bash
./scripts/run.sh live \
  --map /ws/data/map_enu_v010.pcd \
  --metrics /ws/results/live_gpu_metrics.csv
```

LiDAR·IMU와 GNSS 측정 시각을 같은 기준으로 맞추고 정지 상태에서 시작합니다. 여러 PC로 통신할 때는 접근 가능한 `ROS_IP`/`ROS_MASTER_URI`를 사용합니다. 센서 드라이버·주행 제어기는 포함하지 않습니다. AGX Orin의 ARM64·JetPack 환경은 별도로 준비해야 합니다.

## 문제 해결·문서 안내

| 문제 | 확인·조치 |
|---|---|
| ROS master 연결 거부 | `run.sh bag`을 먼저 실행하고 ROS 주소 확인 |
| `I have no name!` | `container.sh shell`로 접속 |
| 컴파일러 캐시 충돌 | `clean.sh --yes` 후 재빌드 |
| `LIO_INITIALIZING` | 초기 정지·IMU 약 300개 확보 |
| `LIO_BAD_CLOUD_FIELD` / `LIO_EMPTY_INPUT` | [입력 schema](data/README.md)·NaN·ring·t·blind 확인 |
| `LIO_WAIT_IMU` | scan end까지 IMU 입력·시간 기준 확인 |
| `LIO_TIME_REWIND` | launch와 bag을 모두 재시작 |
| 정합 계속 거부 | 원점·extrinsic·lever arm·입력 시각을 먼저 확인 |
| 실행 인자 오류 | `run.sh` 첫 인자를 `bag` 또는 `live`로 입력 |

| 위치 | 역할 |
|---|---|
| `src/fast_lio`, `src/fast_gicp`, `src/robotrace_localization` | LIO frontend·CUDA 정합·GNSS/backend |
| `docker`, `scripts`, `tests` | 환경·실행 도구·오프라인 테스트 |
| `.build`, `data`, `results` | 빌드 출력·입력·실험 결과, Git 제외 |
| [docs/calibration.md](docs/calibration.md) | 좌표계·GNSS 초기화·정합 기준·Orin 이식 |
| [docs/repository-maintenance.md](docs/repository-maintenance.md) | Git 관리·동봉 소스 갱신 |
| [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) | 소스 출처·라이선스 범위 |

[오프라인 CI](.github/workflows/offline-checks.yml)는 Python·독립 C++ 테스트와 shell/XML 문법을 검사합니다. ROS/CUDA 빌드와 GPU·bag 실행은 대상 개발 환경에서 별도로 검증합니다.

자체 localization 코드의 라이선스는 [BSD-3-Clause](src/robotrace_localization/LICENSE)입니다. FAST-LIO·Fast GICP·Eigen·NVBIO에는 각 원본 라이선스가 적용됩니다. [FAST-LIO 출처 기록](src/fast_lio/UPSTREAM.md)에서 입력 snapshot의 정확한 원본 repository·commit은 미확인 상태로 공개하며, 확인되지 않은 revision을 추정해 기입하지 않습니다.

공식 참고 자료: [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html), [NVIDIA GPU·그래픽 capability](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/docker-specialized.html), [FAST-LIO](https://github.com/hku-mars/FAST_LIO), [Fast GICP](https://github.com/koide3/fast_gicp).
