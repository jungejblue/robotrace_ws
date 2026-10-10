# 입력 데이터 준비

입력 PCD·bag은 Git에 포함하지 않습니다. 데이터 제공자에게 별도로 받아 이 폴더에 배치합니다. 현재 공개 다운로드 주소와 검증용 소형 데이터셋은 제공하지 않습니다.

| 파일 | 역할 |
|---|---|
| `map_enu_v010.pcd` | GLIM loop closure·0.1 m 다운샘플링으로 만든 prior map |
| `ochang.bag` | 위 지도 제작에 사용한 기록, localization 실행에는 선택 사항 |
| `ochang2.bag` | 별도 검증 입력, 약 192초 |
| `ochang2_localization.bag` | 검증 입력의 필요한 토픽과 시간 기준을 정규화한 사본 |

개발 컨테이너를 `container.sh start --data-dir "$PWD/data"`로 만들면 `/ws/data`는 읽기 전용입니다. 변환 bag은 `/ws/results`에 저장합니다.

## 필요한 토픽과 형식

| 토픽 | ROS1 타입 | 요구 사항 |
|---|---|---|
| `/ouster/points` | `sensor_msgs/PointCloud2` | `os_sensor` frame, 아래 필드 |
| `/ouster/imu` | `sensor_msgs/Imu` | `os_imu` frame, 유효한 가속도·각속도 |
| `/ublox_gps_rear/navpvt` | `ublox_msgs/NavPVT` | UTC/iTOW, 위치·정확도·RTK flags |
| `/ublox_gps_front/navpvt` | `ublox_msgs/NavPVT` | rear와 같은 측정 epoch의 RTK FIX |
| `/tf_static` | `tf2_msgs/TFMessage` | 원본 Ouster static TF, 아래 관계 |

PointCloud2의 필수 필드는 각 count=1입니다. 추가 필드는 허용합니다.

| 필드 | datatype | 의미 |
|---|---|---|
| x / y / z / intensity | FLOAT32 (`7`) | `os_sensor`에 표현한 XYZ와 intensity |
| t | UINT32 (`6`) | scan 시작 이후 상대 시간, 기본 ns |
| ring | UINT16 (`4`) | 채널 번호, 기본 0–63 |

점군 `point_step`, offset·row padding은 실제 payload와 일치해야 합니다. 시간 정규화는 header만 수정하며 per-point `t`를 바꾸지 않습니다. 실제 점군 frame이 `os_lidar`이면 기존 `os_sensor` extrinsic을 그대로 사용하지 않습니다.

기존 bag은 `os_sensor → os_lidar`와 `os_sensor → os_imu` 관계를 포함합니다. localization은 `map → odom → lio_imu → os_sensor`를 생성하며 `lio_imu`는 물리적 `os_imu`와 일치하는 별도 TF 이름입니다. 기존 static TF와 parent 충돌이 없도록 유지합니다.

## 파일 확인·정규화 — 컨테이너

```bash
cd /ws
ls -lh data/map_enu_v010.pcd data/ochang2.bag
rosbag info data/ochang2.bag
python3 scripts/prepare_validation_bag.py \
  data/ochang2.bag results/ochang2_localization.bag
python3 scripts/audit_bag.py \
  results/ochang2_localization.bag \
  --output results/ochang2_input_audit.json
```

원본은 보존합니다. 기존 출력 bag·time_alignment 보고서가 있으면 변환을 거부하므로 새 출력명을 지정합니다. unindexed bag은 원본을 보존할 공간과 처리 시간을 확보한 뒤 ROS1 `rosbag reindex`로 복구해야 합니다.

## 다른 데이터로 교체

- 지도와 GNSS ENU 원점·축 방향·고도 기준이 일치하는지 확인합니다. 기본 원점은 `ochang.bag` 첫 rear FIX를 원점으로 사용했다는 가정이며 GLIM export와 독립 확인한 값은 아닙니다.
- 센서 토픽·frame·시간 단위·extrinsic은 `src/fast_lio/config/ouster.yaml`에서, GNSS 원점·lever arm·baseline은 `src/robotrace_localization/config/localization.yaml`에서 확인합니다.
- 원본/정규화 입력과 설정의 checksum을 실험 결과와 함께 기록합니다. 입력 파일이 바뀌면 동일 파일명만으로 실험을 식별하지 않습니다.

[실행 절차](../README.md) · [calibration](../docs/calibration.md) · [결과 기록](../results/README.md)
