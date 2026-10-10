# Source provenance and license notices

This workspace contains several separately licensed packages and retained
third-party sources. There is no single license declaration for the whole tree.
Existing copyright headers and license texts remain authoritative for their
respective files.

| Component | Retained source or revision | License information |
|---|---|---|
| Robotrace localization package | `src/robotrace_localization` | Its existing `package.xml` declaration is BSD-3-Clause; the full text is now included in [`LICENSE`](src/robotrace_localization/LICENSE). This does not relicense other packages. |
| FAST-LIO frontend | Supplied FAST_LIO_LOCALIZATION snapshot; original fork/commit unconfirmed | Supplied [GPL version 2 text](src/fast_lio/LICENSE), plus existing BSD notices in LOAM-derived and IKFoM/MTK headers, reproduced in [`NOTICE`](src/fast_lio/NOTICE). See [`UPSTREAM.md`](src/fast_lio/UPSTREAM.md). |
| fast_gicp | [`koide3/fast_gicp` at `0e7ec1441c99f7be453db2ea216d5de029387417`](https://github.com/koide3/fast_gicp/tree/0e7ec1441c99f7be453db2ea216d5de029387417) | [BSD-3-Clause](src/fast_gicp/LICENSE), copyright SMRT-AIST. Local integration changes are listed in [`UPSTREAM.md`](src/fast_gicp/UPSTREAM.md). |
| Eigen headers used by CUDA | [`libeigen/eigen` at `1fd5ce1002a6f30e1169b529b291216a18be2f7e`](https://gitlab.com/libeigen/eigen/-/tree/1fd5ce1002a6f30e1169b529b291216a18be2f7e) | Primarily MPL-2.0, with additional per-file terms described in [`COPYING.README`](src/fast_gicp/thirdparty/Eigen/COPYING.README). All supplied `COPYING.*` texts are retained. |
| NVBIO CUDA kNN headers | [`NVlabs/nvbio` at `9bb7e6363c65f65e46f21df09bef98e404250f10`](https://github.com/NVlabs/nvbio/tree/9bb7e6363c65f65e46f21df09bef98e404250f10); nine retained headers | [BSD-3-Clause](src/fast_gicp/thirdparty/nvbio/LICENSE), copyright NVIDIA Corporation. |

The fast_gicp upstream revisions and original file hashes are recorded in
[`UPSTREAM_MANIFEST.json`](src/fast_gicp/UPSTREAM_MANIFEST.json). The distribution
Eigen headers used by host-side PCL, ROS Noetic, PCL and the CUDA toolkit are
external dependencies installed in the development image, not bundled source
submodules. They retain their own license terms.

`catkin_make install` installs the package license and provenance documents,
including the retained Eigen/NVBIO license texts. Keep these notices and the
corresponding source available when redistributing this workspace or its
compiled packages. No new repository-wide MIT/BSD license is assigned here.
