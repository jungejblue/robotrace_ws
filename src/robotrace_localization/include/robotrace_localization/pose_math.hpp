#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <stdexcept>

namespace robotrace_localization {
inline double yawOf(const Eigen::Matrix3d& r) { return std::atan2(r(1, 0), r(0, 0)); }

// FAST-LIO's odom axes initially follow the IMU; its gravity need not be -Z.
// All rotations below are active transforms, T_target_source maps source to target.
inline Eigen::Isometry3d seedMapOdom(const Eigen::Isometry3d& odom_imu,
                                    const Eigen::Vector3d& gravity_odom,
                                    const Eigen::Vector3d& rear_enu,
                                    double imu_yaw_enu,
                                    const Eigen::Vector3d& imu_to_rear) {
  if (!odom_imu.matrix().allFinite() || !gravity_odom.allFinite() ||
      !rear_enu.allFinite() || !imu_to_rear.allFinite() ||
      !std::isfinite(imu_yaw_enu) || gravity_odom.norm() < 1.0)
    throw std::invalid_argument("invalid GNSS/LIO/gravity seed");
  const Eigen::Matrix3d level = Eigen::Quaterniond::FromTwoVectors(
      gravity_odom.normalized(), Eigen::Vector3d(0, 0, -1)).toRotationMatrix();
  const Eigen::Matrix3d level_imu = level * odom_imu.linear();
  if (level_imu.col(0).head<2>().norm() < 0.1)
    throw std::invalid_argument("IMU forward axis is almost vertical");
  const Eigen::Matrix3d r = Eigen::AngleAxisd(
      imu_yaw_enu - yawOf(level_imu), Eigen::Vector3d::UnitZ()).toRotationMatrix() * level;
  Eigen::Isometry3d map_odom = Eigen::Isometry3d::Identity();
  map_odom.linear() = r;
  map_odom.translation() = rear_enu - r * odom_imu.linear() * imu_to_rear - r * odom_imu.translation();
  return map_odom;
}

inline Eigen::Isometry3d interpolatePose(const Eigen::Isometry3d& a,
                                        const Eigen::Isometry3d& b, double fraction) {
  if (!a.matrix().allFinite() || !b.matrix().allFinite() ||
      !std::isfinite(fraction) || fraction < 0.0 || fraction > 1.0)
    throw std::invalid_argument("pose interpolation outside brackets");
  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  out.translation() = (1.0 - fraction) * a.translation() + fraction * b.translation();
  out.linear() = Eigen::Quaterniond(a.linear()).slerp(fraction, Eigen::Quaterniond(b.linear())).normalized().toRotationMatrix();
  return out;
}
}  // namespace robotrace_localization
