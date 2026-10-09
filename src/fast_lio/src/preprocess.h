#ifndef ROBOTRACE_OUSTER_PREPROCESS_H
#define ROBOTRACE_OUSTER_PREPROCESS_H

#include <cstdint>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/register_point_struct.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

using PointType = pcl::PointXYZINormal;
using PointCloudXYZI = pcl::PointCloud<PointType>;
enum LID_TYPE { OUST64 = 3 };
enum TIME_UNIT { SEC = 0, MS = 1, US = 2, NS = 3 };

// Register only fields consumed by FAST-LIO. Ouster reflectivity, ambient,
// range and other optional fields can remain in the input without matching.
namespace ouster_ros {
struct EIGEN_ALIGN16 Point {
  PCL_ADD_POINT4D;
  float intensity;
  std::uint32_t t;
  std::uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
  (float, x, x)(float, y, y)(float, z, z)
  (float, intensity, intensity)
  (std::uint32_t, t, t)(std::uint16_t, ring, ring))

struct PreprocessStats {
  std::size_t input = 0;
  std::size_t nonfinite = 0;
  std::size_t invalid_time = 0;
  std::size_t invalid_ring = 0;
  std::size_t blind = 0;
  std::size_t output = 0;
};

class Preprocess {
public:
  bool process(const sensor_msgs::PointCloud2::ConstPtr& msg,
               PointCloudXYZI::Ptr& pcl_out);
  int lidar_type = OUST64;
  int point_filter_num = 4;
  int N_SCANS = 64;
  int SCAN_RATE = 20;
  int time_unit = NS;
  double blind = 4.0;
  double max_scan_duration_s = 0.1;
  bool feature_enabled = false;
  PreprocessStats stats;

private:
  bool validate_layout(const sensor_msgs::PointCloud2& msg) const;
};
#endif
