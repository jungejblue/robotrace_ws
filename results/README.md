# 실행 결과

변환 bag·CSV·JSON·ROS 로그를 저장하는 폴더입니다. 이 README와 `.gitkeep`을 제외한 결과는 Git에서 제외합니다. 원본 입력은 `data/`에 보존합니다.

| 출력 | 생성 도구 |
|---|---|
| `ochang2_localization.bag` | `scripts/prepare_validation_bag.py` |
| `ochang2_localization.time_alignment.json` | 위 변환의 시각 검사·offset 기록 |
| `ochang2_input_audit.json` | `scripts/audit_bag.py` |
| `ochang2_gpu_metrics.csv` | GPU backend, `run.sh --metrics` 경로 |
| `ochang2_summary.json` | `scripts/summarize_metrics.py` |
| `ros/` | ROS 노드 로그 |

CSV는 실행 시 생성하며 같은 경로로 다시 실행하면 기존 내용을 덮어씁니다. 비교할 실행마다 다른 이름을 사용합니다.

## 생성·요약 — 컨테이너

```bash
./scripts/run.sh bag \
  --map /ws/data/map_enu_v010.pcd \
  --metrics /ws/results/ochang2_gpu_metrics.csv
```

별도 터미널에서 bag을 재생하고 localization을 종료한 뒤 요약합니다.

```bash
python3 scripts/summarize_metrics.py \
  /ws/results/ochang2_gpu_metrics.csv \
  --output /ws/results/ochang2_summary.json
```

거부 사유와 실제 시도 빈도를 확인하려면:

```bash
python3 - <<'PY'
import csv
from collections import Counter

with open('/ws/results/ochang2_gpu_metrics.csv', newline='') as f:
    rows = list(csv.DictReader(f))
print('기록된 시도:', len(rows))
print('거부 원인:', dict(Counter(
    r['reason'] for r in rows if r['state'] != 'TRACKING'
)))
if len(rows) > 1:
    span = float(rows[-1]['stamp']) - float(rows[0]['stamp'])
    if span > 0:
        print('정합 시도 빈도 [Hz]:', (len(rows)-1)/span)
PY
```

## 결과 해석

- `preprocess_ms`, `registration_ms`, `validation_ms`, `total_ms`는 ms이고 `scan_age_s`, `result_age_s`는 초입니다.
- `tracking_fraction`은 CSV에 기록된 처리 시도 중 TRACKING 비율입니다. 전체 입력 스캔 비율이나 운행 시간 비율이 아닙니다.
- `dropped_scans`에는 20 Hz 입력에서 5 Hz 정합을 위해 건너뛴 스캔도 포함됩니다.
- 첫 기록 행은 전체 초기화 지연이 아닙니다. 지도 로딩·CUDA context·GNSS 대기 시간은 CSV 이전에 발생할 수 있습니다.
- `not_converged`는 수렴 검사에서 거부됐다는 뜻이며, 해당 행에서 뒤의 품질 검사가 모두 실행됐다고 간주하지 않습니다.
- 처리 시간 요약에는 거부·오류 행도 포함합니다. 평균뿐 아니라 p95/p99·결과 age·연속 거부 구간을 확인합니다.
- CSV만으로 절대 위치·yaw 정확도, CPU/GPU 사용률·메모리·온도를 판단할 수 없습니다.

## 재현 기록 — 호스트

```bash
cd ~/robotrace_ws
git rev-parse HEAD > results/experiment_commit.txt
sha256sum \
  src/fast_lio/config/ouster.yaml \
  src/robotrace_localization/config/localization.yaml \
  data/map_enu_v010.pcd > results/experiment_inputs.sha256
```

사용한 bag checksum, GPU·드라이버·CUDA, 재생 속도와 RViz 실행 여부도 함께 기록합니다. 수정 중인 working tree는 commit만으로 재현되지 않으므로 diff 또는 적용한 업데이트 패키지를 보존합니다. 큰 bag checksum 계산은 파일 크기에 따라 시간이 걸립니다.

[실행 절차](../README.md) · [정합 기준](../docs/calibration.md)
