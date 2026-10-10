# 좌표계·calibration·정합 설정

기본값은 기존 Ouster 64채널·전후 GNSS 장착과 `map_enu_v010.pcd` 검증에 사용한 구성입니다. 다른 센서·장착·지도로 변경하기 전에 아래 설정을 확인합니다. 실행 명령은 [루트 README](../README.md)를 참고합니다.

## 처리 흐름

1. FAST-LIO2가 LiDAR·IMU로 deskew와 로컬 odometry를 계산합니다. FAST-LIO는 prior PCD를 직접 읽지 않고 로컬 점군 지도를 사용합니다.
2. rear/front RTK FIX의 측정 시각을 맞춰 rear ENU 위치와 전후 baseline yaw를 계산합니다. LIO 중력과 lever arm을 이용해 IMU 위치·자세로 변환합니다.
3. 이 초기 자세로 `map → odom`을 만들고, 예측 위치 주변의 prior map을 잘라 CUDA VGICP 정합을 수행합니다.
4. 정합 결과가 품질 기준을 통과하면 `map → odom`을 갱신합니다. LIO Pose를 이 전역 변환으로 지도 좌표계에 표현합니다.

**GNSS는 초기화와 요청에 따른 재초기화에 사용합니다. 주행 중 GNSS를 EKF나 factor graph에 계속 융합하는 기능은 현재 포함하지 않습니다.** GPU 정합 보정도 FAST-LIO EKF 내부로 되먹임하지 않습니다. GLIM loop closure는 사전 지도 제작 단계의 기능입니다.

## 설정 파일과 적용 방법

| 파일 | 수정할 내용 |
|---|---|
| [src/fast_lio/config/ouster.yaml](../src/fast_lio/config/ouster.yaml) | 입력 토픽·frame, 채널 수, 시간 단위, blind, LiDAR–IMU extrinsic |
| [src/robotrace_localization/config/localization.yaml](../src/robotrace_localization/config/localization.yaml) | 공용 GNSS seed 토픽, ENU 원점, GNSS 조건, lever arm, 정합 범위·주기·품질 조건 |
| [src/fast_lio/launch/odometry.launch](../src/fast_lio/launch/odometry.launch) | LIO 노드와 실행 파라미터 |
| [src/robotrace_localization/launch/localization.launch](../src/robotrace_localization/launch/localization.launch) | 전체 노드 연결, TF, launch 인자 |

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

## 공용 GNSS seed 토픽

`localization.yaml`의 공용 설정을 GNSS 발행 노드와 GPU 구독 노드가 함께 사용합니다.

```yaml
robotrace:
  gnss_seed_topic: /robotrace/gnss_seed
```

토픽 이름을 바꿀 때는 이 값만 변경하고 전체 launch를 재시작합니다. 발행·구독 이름이 일치하는지 확인합니다.

```bash
rosparam get /robotrace/gnss_seed_topic
rostopic info /robotrace/gnss_seed
```

기존 사용자 YAML의 `gnss_initializer/seed_topic`은 공용 `robotrace/gnss_seed_topic`으로 옮깁니다. 이전 값이 공용 값과 다르면 노드가 설정 충돌로 중단하며, 같은 값이면 마이그레이션 경고를 남깁니다. 사용자 정의 이름이라면 `rostopic info`의 인자도 해당 이름으로 바꿉니다. 이 메시지의 위치는 rear 안테나 위치이며 IMU 위치로 변환하기 전의 값입니다.

## GNSS 초기화 조건

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

## LiDAR·IMU 입력과 TF

| 항목 | 현재 구성 |
|---|---|
| LiDAR / IMU 토픽 | `/ouster/points` / `/ouster/imu` |
| 입력 frame | `os_sensor` / `os_imu` |
| Ouster 채널 / 상대 시간 | 64 / `t` UINT32, ns |
| 필수 점군 필드 | x/y/z/intensity FLOAT32, t UINT32, ring UINT16 |
| 입력 속도 | LiDAR 약 20 Hz, IMU 약 100 Hz |
| LiDAR→IMU 회전 / 이동 | I / `[-0.006253, 0.011775, -0.007645] m` |
| TF 연결 | `map → odom → lio_imu → os_sensor` |

`mapping/extrinsic_R`과 `mapping/extrinsic_T`는 LiDAR 입력 좌표를 IMU 좌표로 변환합니다.

```text
p_imu = R_imu_lidar * p_lidar + t_imu_lidar
```

새 static TF 노드는 FAST-LIO가 읽는 동일한 `/mapping/extrinsic_R,T`를 사용해 IMU parent에서 LiDAR child로의 TF를 발행합니다. 부모 이름은 `/publish/body_frame`, 자식 이름은 `/common/lidar_frame`에서 읽습니다. **장착 변경 시 `ouster.yaml`의 extrinsic을 수정하며, launch의 별도 숫자를 수정할 필요가 없습니다.** 입력 점군 header도 설정한 LiDAR frame과 일치해야 합니다. 이 TF는 고정 calibration용이므로 `mapping/extrinsic_est_en=false`를 유지합니다. 유효하지 않은 회전행렬·이동·frame 또는 online extrinsic 설정은 시작 시 거부합니다.

```bash
rosparam get /mapping/extrinsic_T
rosparam get /mapping/extrinsic_R
rosparam get /common/lidar_frame
rosrun tf tf_echo lio_imu os_sensor
```

`lio_imu`는 `os_imu`와 같은 물리 좌표계에 붙인 별도 이름입니다. bag의 `os_sensor → os_imu`, `os_sensor → os_lidar` TF와 parent 충돌을 피합니다. 점군이 이미 `os_sensor`에 표현되어 있으므로 `os_sensor → os_lidar`의 180° 회전을 다시 적용하지 않습니다.

## 현재 정합 기준

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

## 다른 지도·차량으로 변경할 때

1. PCD의 좌표계와 GNSS ENU 원점·고도 기준을 먼저 확인합니다. 현재 값은 mapping bag의 첫 rear FIX를 원점으로 사용했다는 가정이며, GLIM export와의 일치를 독립 검증한 값이 아닙니다.
2. LiDAR/IMU 토픽·frame·시간 단위와 LiDAR→IMU extrinsic을 `ouster.yaml`에 입력합니다. 실시간 센서 시각은 GNSS 측정 UTC와 같은 기준이어야 합니다.
3. GNSS 안테나의 실측 baseline·IMU 좌표계 lever arm·yaw offset을 `localization.yaml`에 입력합니다. 단위는 m/rad이며 rear→IMU 벡터와 IMU→rear 벡터를 혼동하지 않습니다.
4. 정지 상태에서 초기화하고 첫 정합 이후 지도·현재 스캔·상태를 확인합니다. 지도 일치가 불명확하면 정합 gate를 완화하기 전에 좌표계와 calibration을 확인합니다.

정합 RMSE나 승인 비율은 절대 위치 정확도를 보장하지 않습니다. 독립 기준 위치·yaw와 오차를 측정해야 정확도를 평가할 수 있습니다.

## AGX Orin 이식

현재 Dockerfile은 x86_64 개발 PC용입니다. Orin에서는 JetPack/L4T에 맞는 ARM64 CUDA·ROS Noetic 환경을 별도로 준비하고 장치에서 다시 빌드합니다.

```bash
./scripts/build.sh --jobs 1 --cuda-arch 87
```

빌드 스크립트의 GCC 10 요구와 JetPack CUDA host compiler 호환성을 먼저 확인하고 필요하면 환경을 조정합니다. x86 이미지나 `.build` 복사만으로 이식이 끝나지 않습니다. 처리 시간 p95/p99, CPU/GPU 사용률, 메모리, 온도와 위치·yaw 오차를 장치에서 다시 측정합니다.
