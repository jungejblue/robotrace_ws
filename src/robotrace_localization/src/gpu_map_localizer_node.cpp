#include <robotrace_localization/pose_math.hpp>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <tf2_ros/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <fast_gicp/gicp/fast_vgicp_cuda.hpp>
#include <cuda_runtime_api.h>
#include <omp.h>
#include <Eigen/Eigenvalues>
#include <Eigen/StdVector>
#include <boost/bind/bind.hpp>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <thread>

namespace rl = robotrace_localization;
using Cloud = pcl::PointCloud<pcl::PointXYZ>;
using Policy = message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2, nav_msgs::Odometry>;
using Clock = std::chrono::steady_clock;
static double msSince(Clock::time_point t) {
  return std::chrono::duration<double, std::milli>(Clock::now() - t).count();
}
static Eigen::Isometry3d poseOf(const geometry_msgs::Pose& p) {
  Eigen::Quaterniond q(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
  if (!q.coeffs().allFinite() || std::abs(q.norm() - 1.0) > 0.05)
    throw std::invalid_argument("invalid odometry quaternion");
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.linear() = q.normalized().toRotationMatrix();
  t.translation() = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
  if (!t.matrix().allFinite()) throw std::invalid_argument("non-finite odometry pose");
  return t;
}
static geometry_msgs::Pose poseMsg(const Eigen::Isometry3d& t) {
  geometry_msgs::Pose p;
  p.position.x = t.translation().x(); p.position.y = t.translation().y(); p.position.z = t.translation().z();
  Eigen::Quaterniond q(t.linear()); q.normalize();
  p.orientation.w = q.w(); p.orientation.x = q.x(); p.orientation.y = q.y(); p.orientation.z = q.z();
  return p;
}

class GpuMapLocalizer {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  GpuMapLocalizer() : nh_(), pnh_("~"), cloud_sub_(nh_, "/lio/cloud_body", 5),
      odom_sub_(nh_, "/lio/odom", 50), sync_(Policy(50), cloud_sub_, odom_sub_) {
    pnh_.param<std::string>("map_path", map_path_, "/ws/data/map_enu_v010.pcd");
    pnh_.param<std::string>("map_frame", map_frame_, "map");
    pnh_.param<std::string>("odom_frame", odom_frame_, "odom");
    pnh_.param<std::string>("body_frame", body_frame_, "lio_imu");
    pnh_.param<std::string>("nearest_neighbor_method", nn_method_, "CPU_PARALLEL_KDTREE");
    pnh_.param<std::string>("metrics_path", metrics_path_, "");
    pnh_.param("registration_hz", hz_, 5.0);
    pnh_.param("source_leaf_m", source_leaf_, 0.3);
    pnh_.param("target_leaf_m", target_leaf_, 0.3);
    pnh_.param("vgicp_resolution_m", resolution_, 1.0);
    pnh_.param("crop_radius_m", radius_, 80.0);
    pnh_.param("crop_half_height_m", height_, 20.0);
    pnh_.param("recenter_distance_m", recenter_, 10.0);
    pnh_.param("source_max_range_m", max_range_, 60.0);
    pnh_.param("source_min_range_m", min_range_, 4.0);
    pnh_.param("max_scan_age_s", max_age_, 1.0);
    pnh_.param("max_seed_age_s", max_seed_age_, 1.0);
    pnh_.param("pose_buffer_s", pose_buffer_s_, 5.0);
    pnh_.param("max_interpolation_gap_s", interpolation_gap_, 0.12);
    pnh_.param("max_correction_translation_m", max_translation_, 2.0);
    pnh_.param("max_correction_rotation_deg", max_rotation_deg_, 15.0);
    pnh_.param("validation_distance_m", validation_distance_, 1.0);
    pnh_.param("min_inlier_fraction", min_inlier_fraction_, 0.5);
    pnh_.param("max_inlier_rmse_m", max_rmse_, 0.5);
    pnh_.param("min_hessian_eigen_ratio", min_eigen_ratio_, 1e-8);
    pnh_.param("max_iterations", max_iterations_, 30);
    pnh_.param("min_source_points", min_source_, 200);
    pnh_.param("min_target_points", min_target_, 1000);
    pnh_.param("validation_samples", validation_samples_, 1000);
    pnh_.param("cpu_threads", cpu_threads_, 2);
    pnh_.param("rejections_before_lost", max_rejections_, 10);
    pnh_.param("publish_submap", publish_submap_, true);
    pnh_.param("publish_path", publish_path_, true);
    std::vector<double> lever;
    if (!pnh_.getParam("imu_to_rear_antenna_m", lever) || lever.size() != 3)
      throw std::invalid_argument("imu_to_rear_antenna_m must contain three values");
    lever_ = Eigen::Vector3d(lever[0], lever[1], lever[2]);
    const double scalars[] = {hz_,source_leaf_,target_leaf_,resolution_,radius_,height_,recenter_,
      max_range_,min_range_,max_age_,max_seed_age_,pose_buffer_s_,interpolation_gap_,
      max_translation_,max_rotation_deg_,validation_distance_,min_inlier_fraction_,max_rmse_,min_eigen_ratio_};
    for(double v:scalars) if(!std::isfinite(v)) throw std::invalid_argument("non-finite localization setting");
    if (!lever_.allFinite() || hz_ <= 0 || source_leaf_ <= 0 || target_leaf_ <= 0 || resolution_ <= 0 ||
        radius_ <= max_range_ + recenter_ || height_ <= 0 || recenter_ <= 0 || min_range_ < 0 ||
        max_range_ <= min_range_ || max_age_ <= 0 || max_seed_age_ <= 0 || interpolation_gap_ <= 0 ||
        pose_buffer_s_ <= max_seed_age_ || max_translation_ <= 0 || max_rotation_deg_ <= 0 ||
        validation_distance_ <= 0 || min_inlier_fraction_ <= 0 || min_inlier_fraction_ > 1 || max_rmse_ <= 0 ||
        min_eigen_ratio_ < 0 || min_source_ < 10 || min_target_ < 10 || max_iterations_ < 1 ||
        validation_samples_ < 10 || cpu_threads_ < 1 || max_rejections_ < 1 || map_frame_ == odom_frame_ ||
        body_frame_ == odom_frame_ || body_frame_ == map_frame_)
      throw std::invalid_argument("invalid localization settings");
    int count = 0;
    if (cudaGetDeviceCount(&count) != cudaSuccess || count < 1)
      throw std::runtime_error("CUDA GPU unavailable; check container runtime and CUDA libraries");
    cudaDeviceProp prop{};
    if (cudaGetDeviceProperties(&prop, 0) != cudaSuccess) throw std::runtime_error("CUDA device query failed");
    if (cudaSetDevice(0) != cudaSuccess) throw std::runtime_error("CUDA device selection failed");
    ROS_INFO("FastVGICPCuda GPU=%s sm_%d%d", prop.name, prop.major, prop.minor);
    full_map_.reset(new Cloud);
    Cloud raw;
    if (pcl::io::loadPCDFile(map_path_, raw) != 0) throw std::runtime_error("cannot load prior map: " + map_path_);
    Cloud::Ptr finite(new Cloud);
    finite->reserve(raw.size());
    for (const auto& p : raw)
      if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) finite->push_back(p);
    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setLeafSize(target_leaf_, target_leaf_, target_leaf_);
    filter.setInputCloud(finite); filter.filter(*full_map_);
    if (full_map_->size() < static_cast<std::size_t>(min_target_)) throw std::runtime_error("prior map too small");
    ROS_INFO("prior map: %zu raw / %zu target points (original PCD unchanged)", raw.size(), full_map_->size());
    reg_.reset(new fast_gicp::FastVGICPCuda<pcl::PointXYZ, pcl::PointXYZ>);
    reg_->setMaximumIterations(max_iterations_);
    // The vendored setResolution updates both the public member and CUDA core.
    reg_->setResolution(resolution_);
    reg_->setTransformationEpsilon(1e-3);
    reg_->setRotationEpsilon(1e-3);
    reg_->setNeighborSearchMethod(fast_gicp::NeighborSearchMethod::DIRECT7);
    if (nn_method_ == "CPU_PARALLEL_KDTREE")
      reg_->setNearestNeighborSearchMethod(fast_gicp::NearestNeighborMethod::CPU_PARALLEL_KDTREE);
    else if (nn_method_ == "GPU_BRUTEFORCE")
      reg_->setNearestNeighborSearchMethod(fast_gicp::NearestNeighborMethod::GPU_BRUTEFORCE);
    else if (nn_method_ == "GPU_RBF_KERNEL") {
      reg_->setNearestNeighborSearchMethod(fast_gicp::NearestNeighborMethod::GPU_RBF_KERNEL);
      double width; pnh_.param("rbf_kernel_width_m", width, 0.5);
      if (!std::isfinite(width) || width <= 0) throw std::invalid_argument("rbf_kernel_width_m must be finite and positive");
      reg_->setKernelWidth(width);
    } else throw std::invalid_argument("unsupported nearest_neighbor_method");
    ROS_INFO_STREAM("Registration backend=FastVGICPCuda, covariance neighbors=" << nn_method_
                    << ", voxel resolution=" << resolution_ << " m, rate=" << hz_ << " Hz");
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/robotrace/pose", 5);
    path_pub_ = nh_.advertise<nav_msgs::Path>("/robotrace/path", 1);
    submap_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/robotrace/submap", 1, true);
    status_pub_ = nh_.advertise<std_msgs::String>("/robotrace/localization_status", 1, true);
    valid_pub_ = nh_.advertise<std_msgs::Bool>("/robotrace/localization_valid", 1, true);
    diagnostics_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>("/robotrace/localization_diagnostics", 2);
    seed_sub_ = nh_.subscribe("/robotrace/gnss_seed", 20, &GpuMapLocalizer::seedCallback, this);
    gravity_sub_ = nh_.subscribe("/lio/gravity", 10, &GpuMapLocalizer::gravityCallback, this);
    reset_srv_ = pnh_.advertiseService("reset", &GpuMapLocalizer::resetCallback, this);
    sync_.registerCallback(boost::bind(&GpuMapLocalizer::scanCallback, this, boost::placeholders::_1, boost::placeholders::_2));
    if (!metrics_path_.empty()) {
      metrics_.open(metrics_path_, std::ios::out | std::ios::trunc);
      if (!metrics_) throw std::runtime_error("cannot open metrics CSV; create its parent directory: " + metrics_path_);
      metrics_ << "stamp,state,source_points,target_points,preprocess_ms,registration_ms,validation_ms,total_ms,scan_age_s,result_age_s,inlier_fraction,rmse_m,hessian_ratio,correction_m,correction_deg,dropped_scans,reason\n";
    }
    report("WAIT_GNSS", "waiting for LIO, gravity and synchronized RTK FIX pair", false, ros::Time::now());
    watchdog_ = nh_.createWallTimer(ros::WallDuration(0.2), &GpuMapLocalizer::watchdog, this);
    worker_ = std::thread(&GpuMapLocalizer::work, this);
  }
  ~GpuMapLocalizer() { stop_ = true; cv_.notify_all(); if (worker_.joinable()) worker_.join(); }

 private:
  struct Sample {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double time;
    Eigen::Isometry3d pose;
  };
  struct Job { sensor_msgs::PointCloud2ConstPtr cloud; nav_msgs::OdometryConstPtr odom; };
  void seedCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg) {
    if (msg->header.frame_id != map_frame_) { ROS_ERROR_THROTTLE(5, "GNSS seed map frame mismatch"); return; }
    if (msg->header.stamp.isZero()) { ROS_WARN_THROTTLE(5, "GNSS seed measurement timestamp is zero"); return; }
    try { poseOf(msg->pose.pose); } catch (const std::exception& e) { ROS_WARN("%s", e.what()); return; }
    std::lock_guard<std::mutex> lock(mutex_);
    // Retain measurement history: the newest GNSS sample may still be later
    // than the last completed LIO scan, and must not hide a bracketable seed.
    const auto at = std::upper_bound(seeds_.begin(), seeds_.end(), msg->header.stamp,
        [](const ros::Time& stamp, const geometry_msgs::PoseWithCovarianceStampedConstPtr& seed) {
          return stamp < seed->header.stamp;
        });
    seeds_.insert(at, msg);
    while (seeds_.size() > 200 || (seeds_.size() > 1 &&
        (seeds_.back()->header.stamp - seeds_.front()->header.stamp).toSec() > pose_buffer_s_))
      seeds_.pop_front();
  }
  void gravityCallback(const geometry_msgs::Vector3StampedConstPtr& msg) {
    Eigen::Vector3d g(msg->vector.x, msg->vector.y, msg->vector.z);
    if (msg->header.frame_id != odom_frame_ || msg->header.stamp.isZero() ||
        !g.allFinite() || g.norm() < 5 || g.norm() > 15) {
      ROS_WARN_THROTTLE(5.0, "Ignoring invalid LIO gravity; expected finite gravity in odom with nonzero stamp");
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_); gravity_ = g; gravity_time_ = msg->header.stamp.toSec();
  }
  bool resetCallback(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response) {
    reset_requested_ = true; cv_.notify_all(); response.success = true;
    response.message = "reset requested; waits for a fresh RTK pair, LIO and first accepted map match"; return true;
  }
  void scanCallback(const sensor_msgs::PointCloud2ConstPtr& cloud, const nav_msgs::OdometryConstPtr& odom) {
    if (cloud->header.frame_id != body_frame_ || odom->header.frame_id != odom_frame_ || odom->child_frame_id != body_frame_) {
      ROS_ERROR_THROTTLE(5, "cloud/odometry frame mismatch; expected odom -> lio_imu"); return;
    }
    Eigen::Isometry3d pose;
    try { pose = poseOf(odom->pose.pose); } catch (const std::exception& e) { ROS_WARN("%s", e.what()); return; }
    std::lock_guard<std::mutex> lock(mutex_);
    const double time = odom->header.stamp.toSec();
    if (!poses_.empty() && time <= poses_.back().time) {
      ROS_ERROR_THROTTLE(5, "non-monotonic LIO time; restart nodes after bag seek/replay"); return;
    }
    poses_.push_back({time, pose});
    while (poses_.size() > 1000 || (!poses_.empty() && time - poses_.front().time > pose_buffer_s_)) poses_.pop_front();
    if (pending_.cloud) ++dropped_;
    pending_ = {cloud, odom}; cv_.notify_one();
    // Use the latest committed map correction at every LIO scan, independent
    // of the slower CUDA worker. TF timestamps therefore follow LIO at 20 Hz.
    publishPose(*odom);
  }
  bool initialize(double scan_time) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (seeds_.empty() || poses_.size() < 2 || gravity_time_ <= 0 || std::abs(scan_time - gravity_time_) > 0.5) return false;
    geometry_msgs::PoseWithCovarianceStampedConstPtr selected;
    Eigen::Isometry3d at_seed = Eigen::Isometry3d::Identity();
    for (auto seed = seeds_.rbegin(); seed != seeds_.rend(); ++seed) {
      const double stamp = (*seed)->header.stamp.toSec();
      if (stamp > scan_time) continue;  // Never initialize a past scan from future GNSS.
      if (scan_time - stamp > max_seed_age_) break;
      for (std::size_t i = 1; i < poses_.size(); ++i) {
        const Sample& a = poses_[i-1]; const Sample& b = poses_[i];
        if (stamp < a.time || stamp > b.time || b.time - a.time > interpolation_gap_) continue;
        at_seed = rl::interpolatePose(a.pose, b.pose, (stamp - a.time) / (b.time - a.time));
        selected = *seed;
        break;
      }
      if (selected) break;
    }
    if (!selected) return false;  // no unbounded extrapolation of a GNSS seed
    const double stamp = selected->header.stamp.toSec();
    const auto& p = selected->pose.pose;
    Eigen::Quaterniond q(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
    map_odom_ = rl::seedMapOdom(at_seed, gravity_, Eigen::Vector3d(p.position.x,p.position.y,p.position.z),
                              rl::yawOf(q.toRotationMatrix()), lever_);
    initialized_ = true; rejections_ = 0; tracked_ = false;
    ROS_INFO("GNSS initialized map->odom at measurement time %.6f; waiting for map match", stamp);
    return true;
  }
  bool makeTarget(const Eigen::Vector3d& center, const ros::Time& stamp) {
    if (target_ && (center - target_center_).head<2>().norm() < recenter_ &&
        std::abs(center.z() - target_center_.z()) < height_ * 0.25) return true;
    Cloud::Ptr candidate(new Cloud);
    for (const auto& p : *full_map_) {
      const double dx = p.x - center.x(), dy = p.y - center.y();
      if (dx*dx + dy*dy <= radius_*radius_ && std::abs(p.z - center.z()) <= height_) candidate->push_back(p);
    }
    if (candidate->size() < static_cast<std::size_t>(min_target_)) return false;
    candidate->is_dense = true;
    reg_->setInputTarget(candidate);  // covariance and GPU voxel map rebuilt only on submap changes
    if (cudaDeviceSynchronize() != cudaSuccess) throw std::runtime_error("CUDA target preprocessing failed");
    target_ = candidate; target_center_ = center; validation_tree_.setInputCloud(target_);
    if (publish_submap_) {
      sensor_msgs::PointCloud2 msg; pcl::toROSMsg(*target_, msg); msg.header.frame_id = map_frame_; msg.header.stamp = stamp;
      submap_pub_.publish(msg);
    }
    return true;
  }
  bool validate(const Cloud& source, const Eigen::Isometry3d& estimate, const Eigen::Isometry3d& guess,
                double& fraction, double& rmse, double& ratio, double& distance, double& angle, std::string& reason) {
    if (!reg_->hasConverged() || !estimate.matrix().allFinite()) { reason = "not_converged"; return false; }
    distance = (estimate.translation() - guess.translation()).norm();
    angle = Eigen::AngleAxisd(guess.linear().transpose() * estimate.linear()).angle() * 180.0 / std::acos(-1.0);
    if (distance > max_translation_ || angle > max_rotation_deg_) { reason = "correction_gate"; return false; }
    // VGICP Hessian is rotation-first. Normalize diagonal before checking numerical rank.
    Eigen::Matrix<double,6,6> h = reg_->getFinalHessian();
    if (!h.allFinite()) { reason = "invalid_hessian"; return false; }
    h = (h + h.transpose()).eval() * 0.5;
    Eigen::Matrix<double,6,6> scale = Eigen::Matrix<double,6,6>::Zero();
    for (int i=0;i<6;++i) {
      if (h(i,i) <= 1e-12) { reason = "degenerate_hessian"; return false; }
      scale(i,i) = 1.0 / std::sqrt(h(i,i));
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,6,6>> solver(scale*h*scale);
    if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite() || solver.eigenvalues().maxCoeff() <= 0) {
      reason = "invalid_hessian"; return false;
    }
    ratio = solver.eigenvalues().minCoeff() / solver.eigenvalues().maxCoeff();
    if (ratio <= min_eigen_ratio_) { reason = "degenerate_hessian"; return false; }
    int hits=0, samples=0; double squares=0;
    std::vector<int> indices(1); std::vector<float> distances(1);
    const std::size_t stride = std::max<std::size_t>(1, source.size()/static_cast<std::size_t>(validation_samples_));
    for (std::size_t i=0;i<source.size() && samples<validation_samples_;i+=stride) {
      const Eigen::Vector3d p = estimate * source[i].getVector3fMap().cast<double>();
      pcl::PointXYZ query; query.x=p.x(); query.y=p.y(); query.z=p.z(); ++samples;
      if (validation_tree_.nearestKSearch(query,1,indices,distances)>0 && distances[0]<=validation_distance_*validation_distance_) {
        ++hits; squares += distances[0];
      }
    }
    fraction = samples ? double(hits)/samples : 0;
    rmse = hits ? std::sqrt(squares/hits) : std::numeric_limits<double>::infinity();
    if (fraction < min_inlier_fraction_ || rmse > max_rmse_) { reason = "overlap_or_residual_gate"; return false; }
    reason = "accepted"; return true;
  }
  void publishPose(const nav_msgs::Odometry& odom) {
    std::lock_guard<std::mutex> lock(output_mutex_);
    if (!output_initialized_ || odom.header.stamp.toSec() <= last_output_stamp_) return;
    last_output_stamp_ = odom.header.stamp.toSec();
    const Eigen::Isometry3d map_imu = output_map_odom_ * poseOf(odom.pose.pose);
    geometry_msgs::PoseStamped pose; pose.header.stamp=odom.header.stamp; pose.header.frame_id=map_frame_; pose.pose=poseMsg(map_imu);
    pose_pub_.publish(pose);
    geometry_msgs::TransformStamped tf; tf.header=pose.header; tf.child_frame_id=odom_frame_;
    const auto correction=poseMsg(output_map_odom_); tf.transform.translation.x=correction.position.x;
    tf.transform.translation.y=correction.position.y; tf.transform.translation.z=correction.position.z;
    tf.transform.rotation=correction.orientation; broadcaster_.sendTransform(tf);
    if (publish_path_ && (path_.poses.empty() || (pose.header.stamp-path_.poses.back().header.stamp).toSec() >= 1.0/hz_)) {
      path_.header=pose.header; path_.poses.push_back(pose);
      if(path_.poses.size()>5000) path_.poses.erase(path_.poses.begin()); path_pub_.publish(path_);
    }
  }
  void commitCorrection() {
    std::lock_guard<std::mutex> lock(output_mutex_);
    output_map_odom_=map_odom_; output_initialized_=initialized_;
  }
  void watchdog(const ros::WallTimerEvent&) {
    const double stamp=last_result_stamp_.load(), now=ros::Time::now().toSec();
    if(stamp>0 && now-stamp>max_age_ && !stale_reported_.exchange(true))
      report("STALE_INPUT","no recent completed registration; pose is LIO propagation only",false,ros::Time::now(),true);
  }
  void report(const std::string& state, const std::string& reason, bool valid, const ros::Time& stamp, bool heartbeat=false) {
    std::lock_guard<std::mutex> lock(report_mutex_);
    if (state != last_reported_state_) {
      ROS_INFO_STREAM("Localization " << state << ": " << reason);
      last_reported_state_ = state;
    }
    if(!heartbeat) { last_result_stamp_=stamp.toSec(); stale_reported_=false; }
    std_msgs::String s; s.data=state+": "+reason; status_pub_.publish(s);
    std_msgs::Bool b; b.data=valid; valid_pub_.publish(b);
    diagnostic_msgs::DiagnosticArray array; array.header.stamp=stamp;
    diagnostic_msgs::DiagnosticStatus d; d.name="robotrace/gpu_localization"; d.hardware_id="FastVGICPCuda";
    d.level=valid?diagnostic_msgs::DiagnosticStatus::OK:diagnostic_msgs::DiagnosticStatus::WARN; d.message=s.data;
    array.status.push_back(d); diagnostics_pub_.publish(array);
  }
  void work() {
    omp_set_num_threads(cpu_threads_);
    // CUDA's selected device is thread-local. Use the same primary device as
    // the startup capability check when the registration worker first runs.
    if (cudaSetDevice(0) != cudaSuccess) {
      report("ERROR", "CUDA worker device selection failed", false, ros::Time::now());
      ROS_ERROR("CUDA worker cannot select device 0");
      return;
    }
    double last_time=-1;
    while (ros::ok() && !stop_) {
      Job job;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait_for(lock,std::chrono::milliseconds(200),[&]{return stop_ || reset_requested_ || pending_.cloud;});
        if(stop_) break;
        if(reset_requested_.exchange(false)) {
          initialized_=false; tracked_=false; rejections_=0; target_.reset(); last_time=-1;
          { std::lock_guard<std::mutex> output_lock(output_mutex_); output_initialized_=false; path_.poses.clear(); }
          report("WAIT_GNSS","explicit reset",false,ros::Time::now());
        }
        if(!pending_.cloud) continue;
        job=pending_; pending_=Job{};
      }
      const double stamp=job.cloud->header.stamp.toSec();
      if(last_time>0 && stamp-last_time<1.0/hz_) { ++dropped_; continue; }
      last_time=stamp;
      const auto begin=Clock::now(); double prep_ms=0, reg_ms=0, validation_ms=0;
      double fraction=0, rmse=std::numeric_limits<double>::infinity(), ratio=0, distance=0, angle=0;
      int source_size=0; std::string reason, state;
      bool accepted=false;
      const double age=(ros::Time::now()-job.cloud->header.stamp).toSec();
      try {
        if(age>max_age_ || age < -0.2) { report("STALE_INPUT","check /clock and sensor timestamp normalization",false,job.cloud->header.stamp); continue; }
        if(!initialized_ && !initialize(stamp)) { report("WAIT_GNSS","need fresh GNSS seed bracketed by LIO poses and gravity",false,job.cloud->header.stamp); continue; }
        const Eigen::Isometry3d odom_imu=poseOf(job.odom->pose.pose);
        const Eigen::Isometry3d guess=map_odom_*odom_imu;
        Cloud raw; pcl::fromROSMsg(*job.cloud,raw);
        Cloud::Ptr finite(new Cloud); finite->reserve(raw.size());
        for(const auto& p:raw) {
          const double r2=p.x*p.x+p.y*p.y+p.z*p.z;
          if(std::isfinite(r2) && r2>=min_range_*min_range_ && r2<=max_range_*max_range_) finite->push_back(p);
        }
        Cloud::Ptr source(new Cloud); pcl::VoxelGrid<pcl::PointXYZ> down;
        down.setLeafSize(source_leaf_,source_leaf_,source_leaf_); down.setInputCloud(finite); down.filter(*source);
        source_size=source->size();
        if(source_size<min_source_) reason="too_few_source_points";
        else if(!makeTarget(guess.translation(),job.cloud->header.stamp)) reason="too_few_target_points_check_ENU_origin";
        else {
          reg_->setInputSource(source);
          if(cudaDeviceSynchronize()!=cudaSuccess) throw std::runtime_error("CUDA source preprocessing failed");
          prep_ms=msSince(begin); const auto reg_begin=Clock::now(); Cloud aligned;
          reg_->align(aligned,guess.matrix().cast<float>());
          if(cudaDeviceSynchronize()!=cudaSuccess) throw std::runtime_error("CUDA registration failed");
          reg_ms=msSince(reg_begin); const auto validate_begin=Clock::now();
          const Eigen::Matrix4d matrix=reg_->getFinalTransformation().cast<double>();
          Eigen::Isometry3d estimate=Eigen::Isometry3d::Identity(); estimate.matrix()=matrix;
          accepted=validate(*source,estimate,guess,fraction,rmse,ratio,distance,angle,reason);
          validation_ms=msSince(validate_begin);
          if((ros::Time::now()-job.cloud->header.stamp).toSec()>max_age_) { accepted=false; reason="result_too_old"; }
          if(accepted) { map_odom_=estimate*odom_imu.inverse(); tracked_=true; rejections_=0; }
        }
        if(!accepted) ++rejections_;
        state=accepted?"TRACKING":(rejections_>=max_rejections_?"LOST":(tracked_?"DEGRADED":"GNSS_SEEDED"));
        // Failed matches never move map->odom. Pose continues with LIO and is marked invalid.
        commitCorrection(); report(state,reason,accepted,job.cloud->header.stamp);
      } catch(const std::exception& e) { state="ERROR"; reason=e.what(); report(state,reason,false,job.cloud->header.stamp); ROS_ERROR_THROTTLE(2,"GPU localization: %s",e.what()); }
      if(metrics_) {
        std::replace(reason.begin(),reason.end(),',',';');
        metrics_<<std::setprecision(12)<<stamp<<','<<state<<','<<source_size<<','<<(target_?target_->size():0)<<','<<prep_ms<<','<<reg_ms<<','<<validation_ms<<','<<msSince(begin)<<','<<age<<','<<(ros::Time::now()-job.cloud->header.stamp).toSec()<<','<<fraction<<','<<rmse<<','<<ratio<<','<<distance<<','<<angle<<','<<dropped_.load()<<','<<reason<<'\n'; metrics_.flush();
      }
    }
  }
  ros::NodeHandle nh_,pnh_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_;
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
  message_filters::Synchronizer<Policy> sync_;
  ros::Subscriber seed_sub_,gravity_sub_; ros::ServiceServer reset_srv_; ros::WallTimer watchdog_;
  ros::Publisher pose_pub_,path_pub_,submap_pub_,status_pub_,valid_pub_,diagnostics_pub_;
  tf2_ros::TransformBroadcaster broadcaster_;
  std::string map_path_,map_frame_,odom_frame_,body_frame_,nn_method_,metrics_path_,last_reported_state_;
  double hz_,source_leaf_,target_leaf_,resolution_,radius_,height_,recenter_,max_range_,min_range_,max_age_,max_seed_age_,pose_buffer_s_,interpolation_gap_,max_translation_,max_rotation_deg_,validation_distance_,min_inlier_fraction_,max_rmse_,min_eigen_ratio_;
  int max_iterations_,min_source_,min_target_,validation_samples_,cpu_threads_,max_rejections_;
  bool publish_submap_,publish_path_,initialized_=false,tracked_=false;
  int rejections_=0;
  Cloud::Ptr full_map_,target_; Eigen::Vector3d target_center_=Eigen::Vector3d::Zero(),lever_,gravity_=Eigen::Vector3d::Zero();
  double gravity_time_=0;
  Eigen::Isometry3d map_odom_=Eigen::Isometry3d::Identity();
  Eigen::Isometry3d output_map_odom_=Eigen::Isometry3d::Identity();
  bool output_initialized_=false; double last_output_stamp_=-1;
  std::unique_ptr<fast_gicp::FastVGICPCuda<pcl::PointXYZ,pcl::PointXYZ>> reg_;
  pcl::KdTreeFLANN<pcl::PointXYZ> validation_tree_;
  std::mutex mutex_; std::condition_variable cv_; Job pending_;
  std::mutex output_mutex_,report_mutex_;
  std::deque<Sample, Eigen::aligned_allocator<Sample>> poses_;
  std::deque<geometry_msgs::PoseWithCovarianceStampedConstPtr> seeds_;
  std::atomic<bool> stop_{false},reset_requested_{false}; std::atomic<unsigned long> dropped_{0};
  std::atomic<double> last_result_stamp_{0}; std::atomic<bool> stale_reported_{false};
  std::thread worker_; nav_msgs::Path path_; std::ofstream metrics_;
};
int main(int argc,char** argv) {
  ros::init(argc,argv,"gpu_map_localizer");
  try { GpuMapLocalizer node; ros::AsyncSpinner spinner(2); spinner.start(); ros::waitForShutdown(); }
  catch(const std::exception& e) { ROS_FATAL("gpu_map_localizer: %s",e.what()); return 1; }
  return 0;
}
