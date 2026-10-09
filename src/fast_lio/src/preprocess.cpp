#include "preprocess.h"
#include <algorithm>
#include <cmath>
#include <limits>

bool Preprocess::validate_layout(const sensor_msgs::PointCloud2& msg) const {
  struct Required { const char* name; std::uint8_t datatype; std::uint32_t bytes; };
  const Required required[] = {
    {"x", sensor_msgs::PointField::FLOAT32, 4},
    {"y", sensor_msgs::PointField::FLOAT32, 4},
    {"z", sensor_msgs::PointField::FLOAT32, 4},
    {"intensity", sensor_msgs::PointField::FLOAT32, 4},
    {"t", sensor_msgs::PointField::UINT32, 4},
    {"ring", sensor_msgs::PointField::UINT16, 2}
  };
  if (msg.width == 0 || msg.height == 0 || msg.point_step == 0 ||
      static_cast<std::uint64_t>(msg.row_step) <
        static_cast<std::uint64_t>(msg.width) * msg.point_step ||
      static_cast<std::uint64_t>(msg.data.size()) <
        static_cast<std::uint64_t>(msg.height) * msg.row_step) {
    ROS_ERROR_THROTTLE(1.0, "LIO_BAD_CLOUD_LAYOUT: empty or truncated PointCloud2 data.");
    return false;
  }
  // This project's ROS1/Ouster recordings and target machines are little endian.
  if (msg.is_bigendian) {
    ROS_ERROR_THROTTLE(1.0, "LIO_BAD_CLOUD_LAYOUT: big-endian clouds are unsupported.");
    return false;
  }
  for (const auto& expected : required) {
    const auto field = std::find_if(msg.fields.begin(), msg.fields.end(),
      [&](const sensor_msgs::PointField& f) { return f.name == expected.name; });
    if (field == msg.fields.end() || field->datatype != expected.datatype ||
        field->count != 1 || field->offset > msg.point_step ||
        expected.bytes > msg.point_step - field->offset) {
      ROS_ERROR_THROTTLE(1.0,
        "LIO_BAD_CLOUD_FIELD: %s requires datatype=%u, count=1 and a valid offset.",
        expected.name, static_cast<unsigned>(expected.datatype));
      return false;
    }
  }
  return true;
}

bool Preprocess::process(const sensor_msgs::PointCloud2::ConstPtr& msg,
                         PointCloudXYZI::Ptr& pcl_out) {
  pcl_out->clear();
  stats = PreprocessStats();
  stats.input = static_cast<std::size_t>(msg->width) * msg->height;
  if (!validate_layout(*msg)) return false;

  pcl::PointCloud<ouster_ros::Point> input;
  pcl::fromROSMsg(*msg, input);
  // FAST-LIO stores each point's offset from scan start in milliseconds.
  const double to_ms[] = {1.0e3, 1.0, 1.0e-3, 1.0e-6};
  const double scale = to_ms[time_unit];
  pcl_out->reserve(input.size() / point_filter_num + 1);
  for (std::size_t i = 0; i < input.size(); ++i) {
    const auto& p = input.points[i];
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      ++stats.nonfinite;
      continue;
    }
    const double relative_s = static_cast<double>(p.t) * scale * 1.0e-3;
    if (!std::isfinite(relative_s) || relative_s < 0.0 ||
        relative_s > max_scan_duration_s) {
      ++stats.invalid_time;
      continue;
    }
    if (p.ring >= N_SCANS) { ++stats.invalid_ring; continue; }
    const double range_squared = static_cast<double>(p.x) * p.x +
      static_cast<double>(p.y) * p.y + static_cast<double>(p.z) * p.z;
    if (range_squared <= blind * blind) { ++stats.blind; continue; }
    if (i % point_filter_num != 0) continue;
    PointType point{};
    point.x = p.x; point.y = p.y; point.z = p.z;
    point.intensity = std::isfinite(p.intensity) ? p.intensity : 0.0f;
    point.normal_x = point.normal_y = point.normal_z = 0.0f;
    point.curvature = static_cast<float>(relative_s * 1.0e3);
    pcl_out->push_back(point);
  }
  // Keep organized input ordering here. sync_packages finds the maximum t;
  // UndistortPcl performs the one necessary sort before deskew.
  pcl_out->is_dense = true;
  stats.output = pcl_out->size();
  if (stats.nonfinite || stats.invalid_time || stats.invalid_ring) {
    ROS_WARN_THROTTLE(1.0,
      "LIO_POINT_FILTER: input=%zu nonfinite=%zu invalid_t=%zu invalid_ring=%zu blind=%zu kept=%zu",
      stats.input, stats.nonfinite, stats.invalid_time, stats.invalid_ring,
      stats.blind, stats.output);
  }
  if (pcl_out->empty()) {
    ROS_WARN_THROTTLE(1.0,
      "LIO_EMPTY_INPUT: input=%zu nonfinite=%zu invalid_t=%zu invalid_ring=%zu blind=%zu; check XYZ/t/ring/range.",
      stats.input, stats.nonfinite, stats.invalid_time, stats.invalid_ring, stats.blind);
    return false;
  }
  return true;
}
