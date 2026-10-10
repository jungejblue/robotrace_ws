# FAST-LIO frontend provenance

This package is a modified FAST_LIO_LOCALIZATION source snapshot supplied during
the Robotrace integration. Its exact source repository, branch and original
commit have not been recovered. It must not be described as an unmodified or
commit-pinned checkout of `hku-mars/FAST_LIO` or another fork.

The initial Robotrace snapshot is recorded by
[`robotrace_ws` commit `1cabb1601654e7e7ae9b0c7b346ff4b898353246`](https://github.com/jungejblue/robotrace_ws/tree/1cabb1601654e7e7ae9b0c7b346ff4b898353246/src/fast_lio).
That commit is a reproducible Robotrace baseline, not the missing upstream
revision. When the original fork is identified, add its URL and commit here.

The retained frontend uses FAST-LIO2's IMU propagation, scan deskewing, iterated
state update and ikd-Tree local map. The integration:

- limits the input adapter to Ouster `sensor_msgs/PointCloud2` with a `uint16`
  `ring` field and nanosecond `t` field;
- runs local odometry independently of the prior PCD, leaving global prior-map
  registration to `robotrace_localization`;
- publishes the `odom` to `lio_imu` transform and deskewed body-frame cloud;
- provides a ROS1 catkin build without a mandatory Livox driver;
- limits the default FAST-LIO nearest-neighbor workers to three;
- rejects timestamp rollback that would invalidate frontend history.

`LICENSE` retains the supplied GPL version 2 text. LOAM-derived and IKFoM/MTK
files also retain their existing BSD notices in their source headers; verbatim
copies are collected in `NOTICE` for binary-install documentation. These files
are not relicensed by the BSD license of the separate Robotrace localization
package.
