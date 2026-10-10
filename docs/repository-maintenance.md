# 저장소 유지관리

일반 사용자는 [루트 README](../README.md)의 clone·빌드·실행 절차를 사용합니다. 이 문서는 배포 파일 점검, 소스 갱신과 새 원격 저장소 게시를 다룹니다.

## 기존 저장소 변경을 commit하기

```bash
cd ~/robotrace_ws
git status --short
git diff --check
git diff --stat
```

수정 파일을 검토한 다음 필요한 파일만 stage합니다. 원격 branch 보호 정책이 있으면 새 branch에서 작업하고 pull request로 반영합니다.

```bash
git switch -c maintenance/update-localization
git add README.md docs scripts src tests .gitignore
git diff --cached --stat
git diff --cached --check
git commit -m "Update localization configuration and user documentation"
git push -u origin maintenance/update-localization
```

기록된 테스트와 실행 환경이 실제 검증 범위와 일치하는지 확인합니다. GPU·bag 재생 없이 통과한 정적 검사나 오프라인 테스트를 전체 localization 검증으로 표기하지 않습니다.


## 포함·제외 정책

| Git에 포함 | Git에서 제외 |
|---|---|
| `src/`, 필요한 `thirdparty/`, `docker/`, `scripts/`, `tests/`, `docs/`, `.github/` | `.build/`와 이전 `build_*`, `devel_*` 등의 빌드 출력 |
| YAML·launch·CMake·manifest·README·라이선스 고지 | `data/`의 PCD와 bag |
| `data/.gitkeep`, `results/.gitkeep`, 각 폴더의 `README.md` | `results/`의 CSV·JSON·정규화 bag·ROS 로그 |

전체 입력과 실행 결과는 로컬에 보관하거나 별도 데이터 공유 경로로 전달합니다. 현재 Git LFS와 submodule은 사용하지 않습니다. `.gitignore`는 로컬 파일을 삭제하지 않고 새 Git 추적 대상에서 제외합니다.

## 최초 commit 전에 확인

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

## 이미 추적 중인 데이터가 있을 때

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

## commit과 원격 연결

위 검사를 마친 뒤 commit합니다. 새 원격 저장소를 만들 때는 README·LICENSE 자동 생성 없이 빈 저장소를 만들고 아래 URL을 대상 저장소 주소로 바꿉니다. 현재 공개 저장소를 clone한 사용자는 원격 연결 절차를 다시 수행하지 않습니다. 아래 예시는 SSH 인증이 준비되어 있을 때 사용합니다.

```bash
git commit -m "Initial FAST-LIO2 CUDA localization workspace"
robotrace_remote_url='git@github.com:jungejblue/robotrace_ws.git'
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

## 동봉 소스 갱신

현재 저장소는 submodule 대신 수정된 소스를 직접 포함합니다. `src/fast_gicp`를 upstream으로 덮어쓰면 CUDA architecture·Thrust·catkin 연결 수정이 사라질 수 있습니다.

1. 원본 revision과 원본 파일 hash를 `src/fast_gicp/UPSTREAM_MANIFEST.json`에서 확인합니다.
2. upstream 변경과 현재 수정 내역을 비교하고 필요한 수정만 적용합니다.
3. `UPSTREAM.md`와 manifest를 함께 갱신하고 원본 저작권·라이선스를 유지합니다.
4. 오프라인 테스트 후 실제 대상 CUDA/ROS 환경에서 새로 빌드하고 동일 bag으로 검증합니다.

FAST-LIO의 입력 snapshot 원본 repository·commit은 현재 미확인입니다. 확인 전에는 특정 upstream commit으로 추정해 기록하지 않습니다. [소스 출처 및 라이선스](../THIRD_PARTY_NOTICES.md)를 함께 확인합니다.

## 공식 참고 자료

- [Git ignore 규칙](https://git-scm.com/docs/gitignore)
- [Git submodule](https://git-scm.com/docs/gitsubmodules)
- [GitHub 일반 Git 파일 크기 제한](https://docs.github.com/en/repositories/working-with-files/managing-large-files/about-large-files-on-github)
- [GitHub Git LFS](https://docs.github.com/en/repositories/working-with-files/managing-large-files/about-git-large-file-storage)
