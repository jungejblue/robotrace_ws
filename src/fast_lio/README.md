# Ouster FAST-LIO2 local odometry

This ROS1 Noetic package retains the FAST-LIO2/IKFoM estimator and ikd-Tree
from the supplied FAST_LIO_LOCALIZATION source. It is the local odometry
frontend of the clean `robotrace_ws` workspace. The GPU
`robotrace_localization` package owns the read-only prior map, rear/front RTK
GNSS seed and `map -> odom` correction; GNSS is not inserted into the local EKF.
Upstream notices and LICENSE are retained in source files and this directory.

Only Ouster PointCloud2 + sensor_msgs/Imu are supported. No Livox driver,
Livox SDK, Python embedding, matplotlib or startup ICP is required.

- Config: `config/ouster.yaml`; launch: `launch/odometry.launch`.
- Required point fields: x/y/z/intensity FLOAT32; t UINT32 (nanoseconds since
  scan start); ring UINT16; count=1 for each. Extra fields are ignored.
- Input XYZ/time/ring and IMU acceleration/gyro are validated. Invalid XYZ,
  offsets over `preprocess/max_scan_duration_s` (default 0.1 s), invalid rings,
  and points within `preprocess/blind` are removed before deskew.
- Ouster organized point arrays need not be sorted by time. Scan end is
  computed from the maximum retained t, then points are sorted once for deskew.
- About 300 valid IMU samples are needed for initialization (about 3 s at
  100 Hz). Start from a stationary vehicle; `LIO_INITIALIZING` is expected.
- `LIO_EMPTY_INPUT`, `LIO_WAIT_IMU`, `LIO_EMPTY_DESKEW`, and
  `LIO_TOO_FEW_VOXEL` identify different failed stages. The previous repeated
  `No point` issue remains unverified until the user's actual bag is replayed.
- Duplicate/slightly out-of-order stamps are dropped. A backward jump of
  at least 0.1 s clears all input queues and stops the node; restart the whole
  launch before rewinding/replaying a bag. The local EKF is not reset in place.
- `runtime_pos_log_enable=true` prints console timing only. No file is opened
  in source `Log/` or `PCD/`; experiment metrics belong under workspace results/.
- State/odometry is the IMU origin, with odom-frame gravity published separately
  for GNSS global initialization. `lio_imu` is a virtual frame coincident with
  Ouster os_imu, avoiding duplicate parents from the bag's static TF.

Example standalone frontend (a ROS master and synchronized LiDAR/IMU input
are required):

```bash
roslaunch fast_lio odometry.launch
```

Use the top-level workspace instructions for building and running the full
FAST-LIO2 + FastVGICPCuda + GNSS initialization pipeline. This cleaned package
has been statically checked; full ROS/CUDA compilation and bag replay must
be performed in the user's Docker environment.
