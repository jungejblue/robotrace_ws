#include <robotrace_localization/pose_math.hpp>
#include <cstdlib>
#include <iostream>
#include <limits>

namespace rl = robotrace_localization;
constexpr double kPi = 3.14159265358979323846;

void check(bool condition, const char* text) {
  if (!condition) { std::cerr << "FAIL: " << text << '\n'; std::exit(1); }
}
template <typename Callable>
void rejects(Callable f, const char* text) {
  bool rejected = false;
  try { f(); } catch (const std::invalid_argument&) { rejected = true; }
  check(rejected, text);
}
Eigen::Matrix3d rotation(double roll, double pitch, double yaw) {
  return (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
}

int main() {
  const Eigen::Vector3d lever(-0.421253, 0.011775, -0.487645);
  const Eigen::Vector3d rear(2.171, -13.905, -0.156);
  const Eigen::Vector3d down(0.0, 0.0, -9.81);
  const double yaw = 126.05 * kPi / 180.0;
  const auto identity = Eigen::Isometry3d::Identity();
  const auto map_odom = rl::seedMapOdom(identity, down, rear, yaw, lever);
  const auto map_imu = map_odom * identity;
  check(std::abs(rl::yawOf(map_imu.linear()) - yaw) < 1e-12,
        "GNSS yaw is the IMU forward heading in ENU");
  check((map_imu * lever - rear).norm() < 1e-12,
        "full IMU-to-rear lever arm reproduces rear GNSS position");
  check(std::abs(map_imu.translation().x() - 1.932) < 0.002 &&
        std::abs(map_imu.translation().y() + 13.557) < 0.002 &&
        std::abs(map_imu.translation().z() - 0.331645) < 1e-12,
        "project flat-ground initial IMU position and lever-arm sign");

  Eigen::Isometry3d odom_imu = Eigen::Isometry3d::Identity();
  odom_imu.linear() = rotation(-0.12, 0.08, -0.70);
  odom_imu.translation() = Eigen::Vector3d(7.0, -2.0, 0.8);
  const Eigen::Matrix3d expected_map_imu = rotation(0.25, -0.12, 1.1);
  const Eigen::Matrix3d expected_map_odom = expected_map_imu * odom_imu.linear().transpose();
  const Eigen::Vector3d gravity_odom = expected_map_odom.transpose() * down;
  const auto tilted = rl::seedMapOdom(odom_imu, gravity_odom, rear, 1.1, lever);
  const auto tilted_imu = tilted * odom_imu;
  check((tilted.linear() * gravity_odom - down).norm() < 1e-10,
        "non-level odom gravity maps to negative ENU Z");
  check((tilted_imu.linear() - expected_map_imu).norm() < 1e-10,
        "GNSS yaw preserves IMU roll/pitch inferred from gravity");
  check((tilted_imu * lever - rear).norm() < 1e-10,
        "lever arm uses full roll/pitch/yaw rotation");
  check(std::abs(tilted_imu.linear().determinant() - 1.0) < 1e-12,
        "seed rotation remains a proper orthonormal transform");

  Eigen::Isometry3d a = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d b = Eigen::Isometry3d::Identity();
  a.linear() = rotation(0.0, 0.0, 179.0 * kPi / 180.0);
  b.linear() = rotation(0.0, 0.0, -179.0 * kPi / 180.0);
  a.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);
  b.translation() = Eigen::Vector3d(5.0, 4.0, 1.0);
  const auto half = rl::interpolatePose(a, b, 0.5);
  check((half.translation() - Eigen::Vector3d(3.0, 3.0, 2.0)).norm() < 1e-12,
        "bracketed pose translation interpolation");
  check(std::abs(std::abs(rl::yawOf(half.linear())) - kPi) < 1e-12,
        "quaternion interpolation uses shortest rotation across yaw wrap");
  check((rl::interpolatePose(a, b, 0.0).matrix() - a.matrix()).norm() < 1e-12 &&
        (rl::interpolatePose(a, b, 1.0).matrix() - b.matrix()).norm() < 1e-12,
        "pose interpolation preserves bracket endpoints");
  rejects([&] { rl::interpolatePose(a, b, -0.01); }, "past extrapolation rejected");
  rejects([&] { rl::interpolatePose(a, b, 1.01); }, "future extrapolation rejected");
  rejects([&] { rl::interpolatePose(a, b,
      std::numeric_limits<double>::quiet_NaN()); }, "NaN interpolation fraction rejected");
  auto invalid_pose = a;
  invalid_pose.translation().x() = std::numeric_limits<double>::quiet_NaN();
  rejects([&] { rl::interpolatePose(invalid_pose, b, 0.5); },
          "NaN pose endpoint rejected");
  rejects([&] { rl::seedMapOdom(identity, Eigen::Vector3d::Zero(), rear, yaw, lever); },
          "invalid gravity seed rejected");
  Eigen::Isometry3d vertical = Eigen::Isometry3d::Identity();
  vertical.linear() = rotation(0.0, kPi / 2.0, 0.0);
  rejects([&] { rl::seedMapOdom(vertical, down, rear, yaw, lever); },
          "vertical forward axis cannot define planar yaw");
  std::cout << "PASS: GNSS yaw, tilted gravity, 3-D lever arm and bracketed pose interpolation\n";
  return 0;
}
