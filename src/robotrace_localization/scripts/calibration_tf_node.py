#!/usr/bin/env python3
"""Publish the fixed IMU <- input-cloud calibration used by FAST-LIO.

The validation helper deliberately has no ROS or NumPy dependency so its frame
and rigid-transform contract can be checked by the offline test suite.
"""

import math


def calibration_matrix(parent_frame, child_frame, translation, rotation,
                       extrinsic_estimation=False):
    """Return T_parent<-child using FAST-LIO's row-major extrinsic_R.

    A TF edge parent -> child stores the child's pose in parent coordinates,
    so this matrix is used directly, without inverting the calibration.
    """
    for label, frame in (("IMU", parent_frame), ("LiDAR", child_frame)):
        if (not isinstance(frame, str) or not frame or frame.startswith("/")
                or any(c.isspace() for c in frame)):
            raise ValueError("{} frame must be a nonempty TF frame without "
                             "a leading slash or whitespace".format(label))
    if parent_frame == child_frame:
        raise ValueError("IMU and input-cloud frames must differ")
    if not isinstance(extrinsic_estimation, bool):
        raise ValueError("/mapping/extrinsic_est_en must be a boolean")
    if extrinsic_estimation:
        raise ValueError("The calibration TF is static; set "
                         "/mapping/extrinsic_est_en=false")

    def finite_vector(value, size, parameter):
        if not isinstance(value, (list, tuple)) or len(value) != size:
            raise ValueError("{} must contain {} numbers".format(parameter, size))
        if any(isinstance(v, bool) or not isinstance(v, (int, float))
               for v in value):
            raise ValueError("{} must contain numeric values".format(parameter))
        numbers = [float(v) for v in value]
        if not all(math.isfinite(v) for v in numbers):
            raise ValueError("{} must be finite".format(parameter))
        return numbers

    t = finite_vector(translation, 3, "/mapping/extrinsic_T")
    values = finite_vector(rotation, 9, "/mapping/extrinsic_R")
    r = [values[i:i + 3] for i in (0, 3, 6)]
    tolerance = 1e-5
    for i in range(3):
        for j in range(3):
            dot = sum(r[k][i] * r[k][j] for k in range(3))
            if abs(dot - (1.0 if i == j else 0.0)) > tolerance:
                raise ValueError("/mapping/extrinsic_R must be orthonormal")
    determinant = (
        r[0][0] * (r[1][1] * r[2][2] - r[1][2] * r[2][1])
        - r[0][1] * (r[1][0] * r[2][2] - r[1][2] * r[2][0])
        + r[0][2] * (r[1][0] * r[2][1] - r[1][1] * r[2][0]))
    if abs(determinant - 1.0) > tolerance:
        raise ValueError("/mapping/extrinsic_R must have determinant +1")
    return [r[i] + [t[i]] for i in range(3)] + [[0.0, 0.0, 0.0, 1.0]]


def main():
    import rospy
    import tf2_ros
    from geometry_msgs.msg import TransformStamped
    from tf.transformations import quaternion_from_matrix

    rospy.init_node("lio_imu_to_lidar")
    try:
        parent = rospy.get_param("/publish/body_frame")
        child = rospy.get_param("/common/lidar_frame")
        matrix = calibration_matrix(
            parent, child,
            rospy.get_param("/mapping/extrinsic_T"),
            rospy.get_param("/mapping/extrinsic_R"),
            rospy.get_param("/mapping/extrinsic_est_en", False))
        q = quaternion_from_matrix(matrix)
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = parent
        transform.child_frame_id = child
        transform.transform.translation.x = matrix[0][3]
        transform.transform.translation.y = matrix[1][3]
        transform.transform.translation.z = matrix[2][3]
        transform.transform.rotation.x = float(q[0])
        transform.transform.rotation.y = float(q[1])
        transform.transform.rotation.z = float(q[2])
        transform.transform.rotation.w = float(q[3])
        broadcaster = tf2_ros.StaticTransformBroadcaster()
        broadcaster.sendTransform(transform)
        rospy.loginfo("Calibration TF %s -> %s uses /mapping/extrinsic_T,R: "
                      "t=%s, q(x,y,z,w)=%s", parent, child,
                      [matrix[i][3] for i in range(3)], q.tolist())
        rospy.spin()
    except (KeyError, TypeError, ValueError) as error:
        rospy.logfatal("Calibration TF configuration failed: %s", error)
        raise SystemExit(1)


if __name__ == "__main__":
    main()
