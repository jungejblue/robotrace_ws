#include <robotrace_localization/gnss_seed.hpp>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <ublox_msgs/NavPVT.h>

#include <deque>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>

namespace robotrace_localization {
class GnssInitializer {
 public:
  GnssInitializer() : pnh_("~") {
    double lat0 = 36.729539, lon0 = 127.4408323, h0 = 103.230;
    pnh_.param("enu_origin/latitude", lat0, lat0);
    pnh_.param("enu_origin/longitude", lon0, lon0);
    pnh_.param("enu_origin/ellipsoid_height", h0, h0);
    origin_.reset(new EnuOrigin(lat0, lon0, h0));
    pnh_.param("pair_tolerance_s", cfg_.pair_tolerance_s,
               cfg_.pair_tolerance_s);
    pnh_.param("max_h_accuracy_m", cfg_.max_h_accuracy_m,
               cfg_.max_h_accuracy_m);
    pnh_.param("max_v_accuracy_m", cfg_.max_v_accuracy_m,
               cfg_.max_v_accuracy_m);
    pnh_.param("min_h_accuracy_m", cfg_.min_h_accuracy_m,
               cfg_.min_h_accuracy_m);
    pnh_.param("min_v_accuracy_m", cfg_.min_v_accuracy_m,
               cfg_.min_v_accuracy_m);
    pnh_.param("baseline_min_m", cfg_.baseline_min_m, cfg_.baseline_min_m);
    pnh_.param("baseline_max_m", cfg_.baseline_max_m, cfg_.baseline_max_m);
    pnh_.param("yaw_offset_rad", cfg_.yaw_offset_rad, cfg_.yaw_offset_rad);
    pnh_.param("min_yaw_sigma_rad", cfg_.min_yaw_sigma_rad,
               cfg_.min_yaw_sigma_rad);
    pnh_.param("max_yaw_sigma_rad", cfg_.max_yaw_sigma_rad,
               cfg_.max_yaw_sigma_rad);
    pnh_.param("max_sample_age_s", max_age_s_, 1.0);
    pnh_.param("future_tolerance_s", future_tolerance_s_, 0.10);
    pnh_.param("buffer_size", buffer_size_, 100);
    pnh_.param<std::string>("map_frame", map_frame_, "map");

    if (!configurationValid())
      throw std::invalid_argument("Invalid GNSS initializer parameters");
    std::string rear_topic, front_topic, seed_topic;
    pnh_.param<std::string>("rear_topic", rear_topic,
                           "/ublox_gps_rear/navpvt");
    pnh_.param<std::string>("front_topic", front_topic,
                           "/ublox_gps_front/navpvt");
    pnh_.param<std::string>("seed_topic", seed_topic,
                           "/robotrace/gnss_seed");
    seed_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
        seed_topic, 1, true);
    valid_pub_ = nh_.advertise<std_msgs::Bool>("/robotrace/gnss_seed_valid", 1,
                                            true);
    diagnostics_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>(
        "/robotrace/gnss_diagnostics", 10);
    rear_sub_ = nh_.subscribe<ublox_msgs::NavPVT>(
        rear_topic, 100, &GnssInitializer::rearCallback, this);
    front_sub_ = nh_.subscribe<ublox_msgs::NavPVT>(
        front_topic, 100, &GnssInitializer::frontCallback, this);
    timer_ = nh_.createTimer(ros::Duration(0.5), &GnssInitializer::timer, this);
    publishStatus("Waiting for UTC-valid front/rear RTK FIX measurements");
    ROS_INFO_STREAM("GNSS seed uses rear-antenna ENU position and IMU yaw only; "
                    "configured ENU origin LLH=" << std::setprecision(12)
                    << lat0 << ", " << lon0 << ", " << h0);
    ROS_WARN("Confirm baseline_min_m/baseline_max_m against the measured "
             "horizontal antenna baseline before deployment. NavPVT has no "
             "Header; source UTC fields supply the measurement timestamp.");
  }

 private:
  bool configurationValid() const {
    const double values[] = {cfg_.pair_tolerance_s, cfg_.max_h_accuracy_m,
      cfg_.max_v_accuracy_m, cfg_.min_h_accuracy_m, cfg_.min_v_accuracy_m,
      cfg_.baseline_min_m, cfg_.baseline_max_m, cfg_.yaw_offset_rad,
      cfg_.min_yaw_sigma_rad, cfg_.max_yaw_sigma_rad, max_age_s_,
      future_tolerance_s_};
    for (double v : values) if (!std::isfinite(v)) return false;
    return cfg_.pair_tolerance_s >= 0.0 && cfg_.pair_tolerance_s <= 0.2 &&
      cfg_.min_h_accuracy_m > 0.0 && cfg_.min_v_accuracy_m > 0.0 &&
      cfg_.max_h_accuracy_m >= cfg_.min_h_accuracy_m &&
      cfg_.max_v_accuracy_m >= cfg_.min_v_accuracy_m &&
      cfg_.baseline_min_m > 0.0 &&
      cfg_.baseline_max_m > cfg_.baseline_min_m &&
      cfg_.min_yaw_sigma_rad > 0.0 &&
      cfg_.max_yaw_sigma_rad >= cfg_.min_yaw_sigma_rad && max_age_s_ > 0.0 &&
      future_tolerance_s_ >= 0.0 && buffer_size_ >= 2 && !map_frame_.empty();
  }

  static GnssSample decode(const ublox_msgs::NavPVT& m) {
    GnssSample s;
    s.stamp = utcUnixSeconds(m.year, m.month, m.day, m.hour, m.min, m.sec,
                            m.nano, m.valid);
    s.tow_ms = m.iTOW;
    s.flags = m.flags;
    s.fix_type = m.fixType;
    s.lat_deg = m.lat * 1e-7;
    s.lon_deg = m.lon * 1e-7;
    s.height_m = m.height * 1e-3;  // Ellipsoid height, never hMSL.
    s.h_accuracy_m = m.hAcc * 1e-3;
    s.v_accuracy_m = m.vAcc * 1e-3;
    return s;
  }

  void rearCallback(const ublox_msgs::NavPVT::ConstPtr& msg) {
    receive(*msg, true);
  }
  void frontCallback(const ublox_msgs::NavPVT::ConstPtr& msg) {
    receive(*msg, false);
  }

  void checkClockReset(double now) {
    if (last_clock_ > 0.0 && now < last_clock_ - 0.10) {
      rear_.clear(); front_.clear();
      have_seed_ = false;
      last_published_stamp_ = 0.0;
      ROS_WARN("ROS clock moved backwards; GNSS pair buffers reset");
    }
    last_clock_ = now;
  }

  bool fresh(double stamp, double now) const {
    return now > 0.0 && now - stamp <= max_age_s_ &&
           stamp - now <= future_tolerance_s_;
  }

  void trim(std::deque<GnssSample>* queue, double now) {
    for (auto it = queue->begin(); it != queue->end();) {
      if (!fresh(it->stamp, now)) it = queue->erase(it);
      else ++it;
    }
    while (queue->size() > static_cast<std::size_t>(buffer_size_))
      queue->pop_front();
  }

  void receive(const ublox_msgs::NavPVT& msg, bool is_rear) {
    const double now = ros::Time::now().toSec();
    checkClockReset(now);
    GnssSample s;
    std::string reason;
    try { s = decode(msg); }
    catch (const std::exception& e) {
      last_reason_ = std::string(is_rear ? "rear: " : "front: ") + e.what();
      ROS_WARN_STREAM_THROTTLE(2.0, last_reason_);
      return;
    }
    if (!sampleValid(s, cfg_, &reason)) {
      last_reason_ = std::string(is_rear ? "rear: " : "front: ") + reason;
      return;
    }
    if (!fresh(s.stamp, now)) {
      last_reason_ = "GNSS UTC epoch differs from ROS time: use a common "
                     "UTC clock (bag replay requires --clock)";
      ROS_WARN_STREAM_THROTTLE(2.0, last_reason_);
      return;
    }
    trim(&rear_, now); trim(&front_, now);
    auto& own = is_rear ? rear_ : front_;
    auto& other = is_rear ? front_ : rear_;
    // Match measurement epochs, not latest-message arrival order. Search the
    // bounded opposite queue for the closest UTC AND iTOW-compatible sample.
    auto best = other.end();
    double best_dt = std::numeric_limits<double>::infinity();
    for (auto it = other.begin(); it != other.end(); ++it) {
      const double dt = std::abs(s.stamp - it->stamp);
      if (dt <= cfg_.pair_tolerance_s + 1e-6 &&
          gpsTowDifferenceSeconds(s.tow_ms, it->tow_ms) <=
              cfg_.pair_tolerance_s + 1e-6 && dt < best_dt) {
        best = it; best_dt = dt;
      }
    }
    if (best == other.end()) {
      own.push_back(s); trim(&own, now);
      last_reason_ = "Waiting for a matching UTC/iTOW RTK FIX epoch";
      return;
    }
    const GnssSample rear = is_rear ? s : *best;
    const GnssSample front = is_rear ? *best : s;
    other.erase(best);  // Each antenna measurement is used at most once.
    Seed seed;
    if (!makeSeed(rear, front, *origin_, cfg_, &seed, &reason)) {
      last_reason_ = reason;
      ROS_WARN_STREAM_THROTTLE(2.0, "GNSS seed rejected: " << reason);
      return;
    }
    if (seed.stamp <= last_published_stamp_) return;
    geometry_msgs::PoseWithCovarianceStamped out;
    out.header.stamp.fromSec(seed.stamp);
    out.header.frame_id = map_frame_;
    out.pose.pose.position.x = seed.rear_enu.x;
    out.pose.pose.position.y = seed.rear_enu.y;
    out.pose.pose.position.z = seed.rear_enu.z;
    out.pose.pose.orientation.z = std::sin(seed.yaw_rad * 0.5);
    out.pose.pose.orientation.w = std::cos(seed.yaw_rad * 0.5);
    out.pose.covariance.fill(0.0);
    out.pose.covariance[0] = seed.h_variance_m2;
    out.pose.covariance[7] = seed.h_variance_m2;
    out.pose.covariance[14] = seed.v_variance_m2;
    // Roll/pitch are explicitly unobserved by this message. The quaternion's
    // zero roll/pitch is only a yaw carrier, not an attitude measurement.
    out.pose.covariance[21] = 1e6;
    out.pose.covariance[28] = 1e6;
    out.pose.covariance[35] = seed.yaw_variance_rad2;
    seed_pub_.publish(out);
    latest_seed_ = seed;
    latest_rear_hacc_ = rear.h_accuracy_m;
    latest_front_hacc_ = front.h_accuracy_m;
    have_seed_ = true;
    last_published_stamp_ = seed.stamp;
    last_reason_ = "Valid rear-antenna ENU + IMU yaw seed";
    publishStatus(last_reason_);
    ROS_INFO_STREAM_THROTTLE(2.0, "GNSS seed E/N/U=" << seed.rear_enu.x << "/"
        << seed.rear_enu.y << "/" << seed.rear_enu.z << " m, yaw="
        << seed.yaw_rad * 180.0 / kPi << " deg, baseline="
        << seed.baseline_m << " m, epoch difference=" << seed.pair_dt_s
        << " s");
  }

  static std::string number(double value) {
    std::ostringstream s;
    s << std::fixed << std::setprecision(6) << value;
    return s.str();
  }

  void publishStatus(const std::string& reason) {
    const double now = ros::Time::now().toSec();
    const bool valid = have_seed_ && fresh(latest_seed_.stamp, now);
    std_msgs::Bool b; b.data = valid; valid_pub_.publish(b);
    diagnostic_msgs::DiagnosticArray array;
    array.header.stamp = ros::Time::now();
    diagnostic_msgs::DiagnosticStatus status;
    status.name = "robotrace/gnss_initializer";
    status.hardware_id = "dual_gnss";
    status.level = valid ? diagnostic_msgs::DiagnosticStatus::OK :
                          diagnostic_msgs::DiagnosticStatus::WARN;
    status.message = valid ? "Fresh rear-antenna ENU + yaw seed" : reason;
    const auto add = [&status](const std::string& k, const std::string& v) {
      diagnostic_msgs::KeyValue p; p.key = k; p.value = v;
      status.values.push_back(p);
    };
    add("seed_position_frame", "rear antenna in map ENU; no IMU lever arm applied");
    add("seed_attitude", "IMU yaw only; roll/pitch unobserved");
    add("source_time", "NavPVT UTC calendar+nano; paired by UTC and iTOW");
    add("rear_buffer_size", std::to_string(rear_.size()));
    add("front_buffer_size", std::to_string(front_.size()));
    add("last_rejection_or_state", last_reason_);
    if (have_seed_) {
      add("seed_age_s", number(now - latest_seed_.stamp));
      add("pair_dt_s", number(latest_seed_.pair_dt_s));
      add("horizontal_baseline_m", number(latest_seed_.baseline_m));
      add("rear_h_accuracy_m", number(latest_rear_hacc_));
      add("front_h_accuracy_m", number(latest_front_hacc_));
      add("yaw_sigma_rad", number(std::sqrt(latest_seed_.yaw_variance_rad2)));
    }
    array.status.push_back(status);
    diagnostics_pub_.publish(array);
  }

  void timer(const ros::TimerEvent&) {
    const double now = ros::Time::now().toSec();
    checkClockReset(now);
    trim(&rear_, now); trim(&front_, now);
    publishStatus(have_seed_ ? "Previous GNSS seed is stale; waiting for a "
                             "fresh RTK FIX pair" : last_reason_);
  }

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber rear_sub_, front_sub_;
  ros::Publisher seed_pub_, valid_pub_, diagnostics_pub_;
  ros::Timer timer_;
  std::unique_ptr<EnuOrigin> origin_;
  SeedConfig cfg_;
  std::deque<GnssSample> rear_, front_;
  Seed latest_seed_;
  std::string map_frame_, last_reason_ = "Waiting for RTK FIX pair";
  double max_age_s_ = 1.0, future_tolerance_s_ = 0.10;
  double last_clock_ = 0.0, last_published_stamp_ = 0.0;
  double latest_rear_hacc_ = 0.0, latest_front_hacc_ = 0.0;
  int buffer_size_ = 100;
  bool have_seed_ = false;
};
}  // namespace robotrace_localization

int main(int argc, char** argv) {
  ros::init(argc, argv, "gnss_initializer");
  try {
    robotrace_localization::GnssInitializer node;
    ros::spin();
  } catch (const std::exception& e) {
    ROS_FATAL_STREAM("GNSS initializer startup failed: " << e.what());
    return 1;
  }
  return 0;
}
