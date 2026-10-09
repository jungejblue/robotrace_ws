// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Livox               dev@livoxtech.com

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#include <omp.h>
#include <mutex>
#include <cmath>
#include <algorithm>
#include <math.h>
#include <thread>
#include <csignal>
#include <unistd.h>
#include <so3_math.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include "IMU_Processing.hpp"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include "preprocess.h"
#include <ikd-Tree/ikd_Tree.h>

#define INIT_TIME           (0.1)
#define LASER_POINT_COV     (0.001)
#define PUBFRAME_PERIOD     (20)


/*** Time Log Variables ***/
double kdtree_incremental_time = 0.0, kdtree_search_time = 0.0, kdtree_delete_time = 0.0;
double match_time = 0, solve_time = 0, solve_const_H_time = 0;
int    kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0, kdtree_delete_counter = 0;
bool   runtime_pos_log = false, extrinsic_est_en = false, path_en = false;
/**************************/

float res_last[100000] = {0.0};
float DET_RANGE = 300.0f;
const float MOV_THRESHOLD = 1.5f;
double time_diff_lidar_to_imu = 0.0;

mutex mtx_buffer;
condition_variable sig_buffer;

string lid_topic, imu_topic;
string odom_frame = "odom", body_frame = "lio_imu";

double res_mean_last = 0.05, total_residual = 0.0;
double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
double filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0;
double cube_len = 0, HALF_FOV_COS = 0, FOV_DEG = 0, total_distance = 0, lidar_end_time = 0, first_lidar_time = 0.0;  
int    effct_feat_num = 0, time_log_counter = 0, scan_count = 0, publish_count = 0;
int    iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0, laserCloudValidNum = 0;
bool   point_selected_surf[100000] = {0};
bool   lidar_pushed, flg_first_scan = true, flg_exit = false, flg_EKF_inited;
bool   scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;
bool   first_odom = true;
bool   system_initialized = false;
int scans_acc_icp_counter = 0;
size_t scan_counter = 0;

vector<BoxPointType> cub_needrm;
vector<PointVector>  Nearest_Points; 
vector<double>       extrinT(3, 0.0);
vector<double>       extrinR(9, 0.0);
deque<double>                     time_buffer;
deque<PointCloudXYZI::Ptr>        lidar_buffer;
deque<sensor_msgs::Imu::ConstPtr> imu_buffer;

PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1));

pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterMap;

KD_TREE<PointType> ikdtree;

V3F XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0);
V3F XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0);
V3D euler_cur;
V3D position_last(Zero3d);
V3D Lidar_T_wrt_IMU(Zero3d);
M3D Lidar_R_wrt_IMU(Eye3d);

/*** EKF inputs and output ***/
MeasureGroup Measures;
esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
state_ikfom state_point;
vect3 pos_lid;

nav_msgs::Path path;
nav_msgs::Odometry odomAftMapped, prev_odomAftMapped;
geometry_msgs::Quaternion geoQuat;
geometry_msgs::PoseStamped msg_body_pose;
ros::Publisher pubLioGravity;

shared_ptr<Preprocess> p_pre(new Preprocess());
shared_ptr<ImuProcess> p_imu(new ImuProcess());

void SigHandle(int sig)
{
    flg_exit = true;
    ROS_WARN("catch sig %d", sig);
    sig_buffer.notify_all();
}

void pointBodyToWorld_ikfom(PointType const * const pi, PointType * const po, state_ikfom &s)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}


void pointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

template<typename T>
void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
    V3D p_body(pi[0], pi[1], pi[2]);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}

void RGBpointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void RGBpointBodyLidarToIMU(PointType const * const pi, PointType * const po)
{
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu(state_point.offset_R_L_I*p_body_lidar + state_point.offset_T_L_I);

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

void points_cache_collect()
{
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);
}

BoxPointType LocalMap_Points;
bool Localmap_Initialized = false;
void lasermap_fov_segment()
{
    cub_needrm.clear();
    kdtree_delete_counter = 0;
    kdtree_delete_time = 0.0;    
    pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);
    V3D pos_LiD = pos_lid;
    if (!Localmap_Initialized){
        for (int i = 0; i < 3; i++){
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++){
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) need_move = true;
    }
    if (!need_move) return;
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD -1)));
    for (int i = 0; i < 3; i++){
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    points_cache_collect();
    double delete_begin = omp_get_wtime();
    if(cub_needrm.size() > 0) kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
    kdtree_delete_time = omp_get_wtime() - delete_begin;
}

void stop_on_time_rewind(const char* source, double timestamp, double previous)
{
    lidar_buffer.clear();
    time_buffer.clear();
    imu_buffer.clear();
    lidar_pushed = false;
    ROS_FATAL("LIO_TIME_REWIND: %s stamp %.9f < %.9f. Buffers cleared; restart the complete launch before replaying from an earlier time. EKF reset is intentionally not attempted.",
      source, timestamp, previous);
    flg_exit = true;
    ros::shutdown();
}

void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(mtx_buffer);
    const double stamp = msg->header.stamp.toSec();
    if (!std::isfinite(stamp) || stamp <= 0.0)
    {
        ROS_ERROR_THROTTLE(1.0, "LIO_BAD_LIDAR_STAMP: nonpositive/nonfinite scan timestamp.");
        return;
    }
    if (stamp <= last_timestamp_lidar)
    {
        if (last_timestamp_lidar - stamp >= 0.1)
            stop_on_time_rewind("LiDAR", stamp, last_timestamp_lidar);
        else
            ROS_WARN_THROTTLE(1.0, "LIO_OUT_OF_ORDER: dropped duplicate/backward LiDAR stamp %.9f < %.9f", stamp, last_timestamp_lidar);
        return;
    }
    last_timestamp_lidar = stamp;
    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    if (!p_pre->process(msg, ptr)) return;
    ++scan_count;
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(stamp);
    sig_buffer.notify_all();
}

void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in)
{
    const auto& a = msg_in->linear_acceleration;
    const auto& w = msg_in->angular_velocity;
    const double acceleration_norm_squared = a.x*a.x + a.y*a.y + a.z*a.z;
    if (!std::isfinite(a.x) || !std::isfinite(a.y) || !std::isfinite(a.z) ||
        !std::isfinite(w.x) || !std::isfinite(w.y) || !std::isfinite(w.z) ||
        !std::isfinite(acceleration_norm_squared) || acceleration_norm_squared < 1.0e-12)
    {
        // No orientation or covariance validity gate: FAST-LIO uses raw a/w,
        // and both g-scaled and SI acceleration are supported upstream.
        ROS_ERROR_THROTTLE(1.0, "LIO_INVALID_IMU: rejected nonfinite a/w or zero acceleration.");
        return;
    }
    const double timestamp = msg_in->header.stamp.toSec() - time_diff_lidar_to_imu;
    if (!std::isfinite(timestamp) || timestamp <= 0.0)
    {
        ROS_ERROR_THROTTLE(1.0, "LIO_BAD_IMU_STAMP: invalid stamp after time offset.");
        return;
    }
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));
    msg->header.stamp = ros::Time().fromSec(timestamp);
    std::lock_guard<std::mutex> lock(mtx_buffer);
    if (timestamp <= last_timestamp_imu)
    {
        if (last_timestamp_imu - timestamp >= 0.1)
            stop_on_time_rewind("IMU", timestamp, last_timestamp_imu);
        else
            ROS_WARN_THROTTLE(1.0, "LIO_OUT_OF_ORDER: dropped duplicate/backward IMU stamp %.9f <= %.9f", timestamp, last_timestamp_imu);
        return;
    }
    last_timestamp_imu = timestamp;
    imu_buffer.push_back(msg);
    sig_buffer.notify_all();
}

double lidar_mean_scantime = 0.05;
int    scan_num = 0;
bool sync_packages(MeasureGroup &meas)
{
    std::lock_guard<std::mutex> lock(mtx_buffer);
    if (lidar_buffer.empty() || time_buffer.empty() || imu_buffer.empty()) {
        return false;
    }

    /*** push a lidar scan ***/
    if(!lidar_pushed)
    {
        meas.lidar = lidar_buffer.front();
        meas.lidar_beg_time = time_buffer.front();
        const auto latest_point = std::max_element(meas.lidar->points.begin(), meas.lidar->points.end(),
          [](const PointType& a, const PointType& b) { return a.curvature < b.curvature; });
        const double actual_scan_duration = latest_point == meas.lidar->points.end() ? 0.0 : latest_point->curvature * 1.0e-3;
        if (meas.lidar->points.size() <= 1) // time too little
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
            ROS_WARN_THROTTLE(1.0, "LIO_TOO_FEW_INPUT: kept=%zu, using configured scan duration %.6f", meas.lidar->size(), lidar_mean_scantime);
        }
        else if (actual_scan_duration < 0.5 * lidar_mean_scantime)
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
        }
        else
        {
            scan_num ++;
            lidar_end_time = meas.lidar_beg_time + actual_scan_duration;
            lidar_mean_scantime += (actual_scan_duration - lidar_mean_scantime) / scan_num;
        }

        meas.lidar_end_time = lidar_end_time;

        lidar_pushed = true;
    }

    if (last_timestamp_imu < lidar_end_time)
    {
        ROS_WARN_THROTTLE(1.0,
          "LIO_WAIT_IMU: scan_start=%.6f scan_end=%.6f last_imu=%.6f lidar_queue=%zu imu_queue=%zu",
          meas.lidar_beg_time, lidar_end_time, last_timestamp_imu, lidar_buffer.size(), imu_buffer.size());
        return false;
    }

    /*** push imu data, and pop from imu buffer ***/
    meas.imu.clear();
    while (!imu_buffer.empty() && imu_buffer.front()->header.stamp.toSec() <= lidar_end_time)
    {
        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
    }
    lidar_buffer.pop_front();
    time_buffer.pop_front();
    lidar_pushed = false;
    return true;
}

int process_increments = 0;
void map_incremental()
{
    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);
    for (int i = 0; i < feats_down_size; i++)
    {
        /* transform to world frame */
        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
        /* decide if need add to map */
        if (!Nearest_Points[i].empty() && flg_EKF_inited)
        {
            const PointVector &points_near = Nearest_Points[i];
            bool need_add = true;
            BoxPointType Box_of_Point;
            PointType downsample_result, mid_point; 
            mid_point.x = floor(feats_down_world->points[i].x/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.y = floor(feats_down_world->points[i].y/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.z = floor(feats_down_world->points[i].z/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            float dist  = calc_dist(feats_down_world->points[i],mid_point);
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min && fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min && fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min){
                PointNoNeedDownsample.push_back(feats_down_world->points[i]);
                continue;
            }
            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i ++)
            {
                if (points_near.size() < NUM_MATCH_POINTS) break;
                if (calc_dist(points_near[readd_i], mid_point) < dist)
                {
                    need_add = false;
                    break;
                }
            }
            if (need_add) PointToAdd.push_back(feats_down_world->points[i]);
        }
        else
        {
            PointToAdd.push_back(feats_down_world->points[i]);
        }
    }

    double st_time = omp_get_wtime();
    ROS_INFO_ONCE("Started Updating Map tree.");
    add_point_size = ikdtree.Add_Points(PointToAdd, true);
    ikdtree.Add_Points(PointNoNeedDownsample, false); 
    add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
    kdtree_incremental_time = omp_get_wtime() - st_time;
}

void publish_frame_world(const ros::Publisher & pubLaserCloudFull)
{

    if(scan_pub_en)
    {
        PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
        int size = laserCloudFullRes->points.size();
        PointCloudXYZI::Ptr laserCloudWorld( \
                        new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&laserCloudFullRes->points[i], \
                                &laserCloudWorld->points[i]);
        }

        sensor_msgs::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
        laserCloudmsg.header.frame_id = odom_frame;
        pubLaserCloudFull.publish(laserCloudmsg);
        publish_count -= PUBFRAME_PERIOD;
    }

}

void publish_frame_body(const ros::Publisher & pubLaserCloudFull_body)
{
    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
        RGBpointBodyLidarToIMU(&feats_undistort->points[i], \
                            &laserCloudIMUBody->points[i]);
    }

    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.frame_id = body_frame;
    pubLaserCloudFull_body.publish(laserCloudmsg);
    publish_count -= PUBFRAME_PERIOD;
}

template<typename T>
void set_posestamp(T & out)
{
    out.pose.position.x = state_point.pos(0);
    out.pose.position.y = state_point.pos(1);
    out.pose.position.z = state_point.pos(2);
    out.pose.orientation.x = geoQuat.x;
    out.pose.orientation.y = geoQuat.y;
    out.pose.orientation.z = geoQuat.z;
    out.pose.orientation.w = geoQuat.w;
    
}

void publish_odometry(const ros::Publisher & pubOdomAftMapped)
{
    // State position is the IMU origin, not the vehicle base or LiDAR origin.
    odomAftMapped.header.frame_id = odom_frame;
    odomAftMapped.child_frame_id = body_frame;
    odomAftMapped.header.stamp = ros::Time().fromSec(lidar_end_time);// ros::Time().fromSec(lidar_end_time);
    set_posestamp(odomAftMapped.pose);
    if (first_odom) {
        odomAftMapped.twist.twist.linear.x = 0.0;
        odomAftMapped.twist.twist.linear.y = 0.0;
        odomAftMapped.twist.twist.linear.z = 0.0;
        odomAftMapped.twist.twist.angular.x = 0.0;
        odomAftMapped.twist.twist.angular.y = 0.0;
        odomAftMapped.twist.twist.angular.z = 0.0;
        first_odom = false;
    }
    else {
        // nav_msgs/Odometry specifies twist in child_frame_id.
        const V3D velocity_body = state_point.rot.conjugate() * state_point.vel;
        odomAftMapped.twist.twist.linear.x = velocity_body.x();
        odomAftMapped.twist.twist.linear.y = velocity_body.y();
        odomAftMapped.twist.twist.linear.z = velocity_body.z();
        if (!Measures.imu.empty())
        {
            const auto &w = Measures.imu.back()->angular_velocity;
            odomAftMapped.twist.twist.angular.x = w.x - state_point.bg.x();
            odomAftMapped.twist.twist.angular.y = w.y - state_point.bg.y();
            odomAftMapped.twist.twist.angular.z = w.z - state_point.bg.z();
        }
    }
    auto P = kf.get_P();
    // state_ikfom orders position, rotation. Rotation increments are in the
    // body tangent frame; convert them into fixed-frame rotation covariance.
    Eigen::Matrix<double, 6, 6> pose_jacobian = Eigen::Matrix<double, 6, 6>::Identity();
    pose_jacobian.block<3, 3>(3, 3) = state_point.rot.toRotationMatrix();
    const Eigen::Matrix<double, 6, 6> pose_covariance =
        pose_jacobian * P.block<6, 6>(0, 0) * pose_jacobian.transpose();
    for (int i = 0; i < 6; i ++)
    {
        for (int j = 0; j < 6; ++j)
            odomAftMapped.pose.covariance[i * 6 + j] = pose_covariance(i, j);
    }
    geometry_msgs::Vector3Stamped gravity_message;
    gravity_message.header = odomAftMapped.header;
    gravity_message.vector.x = state_point.grav[0];
    gravity_message.vector.y = state_point.grav[1];
    gravity_message.vector.z = state_point.grav[2];
    pubLioGravity.publish(gravity_message);
    pubOdomAftMapped.publish(odomAftMapped);

    static tf::TransformBroadcaster br;
    tf::Transform                   transform;
    tf::Quaternion                  q;
    transform.setOrigin(tf::Vector3(odomAftMapped.pose.pose.position.x, \
                                    odomAftMapped.pose.pose.position.y, \
                                    odomAftMapped.pose.pose.position.z));
    q.setW(odomAftMapped.pose.pose.orientation.w);
    q.setX(odomAftMapped.pose.pose.orientation.x);
    q.setY(odomAftMapped.pose.pose.orientation.y);
    q.setZ(odomAftMapped.pose.pose.orientation.z);
    transform.setRotation( q );
    br.sendTransform( tf::StampedTransform( transform, odomAftMapped.header.stamp, odom_frame, body_frame ) );
    prev_odomAftMapped = odomAftMapped;
}

void publish_path(const ros::Publisher pubPath)
{
    msg_body_pose.pose.position = odomAftMapped.pose.pose.position;
    msg_body_pose.pose.orientation = odomAftMapped.pose.pose.orientation;
    msg_body_pose.header.stamp = ros::Time().fromSec(lidar_end_time);
    msg_body_pose.header.frame_id = odom_frame;

    /*** if path is too large, the rvis will crash ***/
    static int jjj = 0;
    jjj++;
    if (jjj % 10 == 0) 
    {
        path.poses.push_back(msg_body_pose);
        path.header.stamp = msg_body_pose.header.stamp;
        pubPath.publish(path);
    }
}

void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    double match_start = omp_get_wtime();
    laserCloudOri->clear(); 
    corr_normvect->clear(); 
    total_residual = 0.0; 

    /** closest surface search and residual computation **/
    #ifdef MP_EN
        omp_set_num_threads(MP_PROC_NUM);
        #pragma omp parallel for
    #endif
    for (int i = 0; i < feats_down_size; i++)
    {
        PointType &point_body  = feats_down_body->points[i]; 
        PointType &point_world = feats_down_world->points[i]; 

        /* transform to world frame */
        V3D p_body(point_body.x, point_body.y, point_body.z);
        V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = point_body.intensity;

        vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

        auto &points_near = Nearest_Points[i];

        if (ekfom_data.converge)
        {
            /** Find the closest surfaces in the map **/
            ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
            point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false : true;
        }

        if (!point_selected_surf[i]) continue;

        VF(4) pabcd;
        point_selected_surf[i] = false;
        if (esti_plane(pabcd, points_near, 0.1f))
        {
            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
            float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

            if (s > 0.9)
            {
                point_selected_surf[i] = true;
                normvec->points[i].x = pabcd(0);
                normvec->points[i].y = pabcd(1);
                normvec->points[i].z = pabcd(2);
                normvec->points[i].intensity = pd2;
                res_last[i] = abs(pd2);
            }
        }
    }
    
    effct_feat_num = 0;

    for (int i = 0; i < feats_down_size; i++)
    {
        if (point_selected_surf[i])
        {
            laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
            corr_normvect->points[effct_feat_num] = normvec->points[i];
            total_residual += res_last[i];
            effct_feat_num ++;
        }
    }

    if (effct_feat_num < 1)
    {
        ekfom_data.valid = false;
        ROS_WARN("No Effective Points! \n");
        return;
    }

    res_mean_last = total_residual / effct_feat_num;
    match_time  += omp_get_wtime() - match_start;
    double solve_start_  = omp_get_wtime();
    
    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
    ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12); //23
    ekfom_data.h.resize(effct_feat_num);

    for (int i = 0; i < effct_feat_num; i++)
    {
        const PointType &laser_p  = laserCloudOri->points[i];
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat<<SKEW_SYM_MATRX(point_this);

        /*** get the normal vector of closest surface/corner ***/
        const PointType &norm_p = corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

        /*** calculate the Measuremnt Jacobian matrix H ***/
        V3D C(s.rot.conjugate() *norm_vec);
        V3D A(point_crossmat * C);
        if (extrinsic_est_en)
        {
            V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C); //s.rot.conjugate()*norm_vec);
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
        }
        else
        {
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }

        /*** Measuremnt: distance to the closest surface/corner ***/
        ekfom_data.h(i) = -norm_p.intensity;
    }
    solve_time += omp_get_wtime() - solve_start_;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "laserMapping");
    ros::NodeHandle nh;

    nh.param<bool>("publish/path_en",path_en, false);
    nh.param<bool>("publish/scan_publish_en",scan_pub_en, false);
    nh.param<bool>("publish/dense_publish_en",dense_pub_en, false);
    nh.param<bool>("publish/scan_bodyframe_pub_en",scan_body_pub_en, true);
    nh.param<string>("publish/odom_frame", odom_frame, "odom");
    nh.param<string>("publish/body_frame", body_frame, "lio_imu");
    nh.param<int>("max_iteration",NUM_MAX_ITERATIONS,4);
    nh.param<string>("common/lid_topic",lid_topic,"/ouster/points");
    nh.param<string>("common/imu_topic", imu_topic,"/ouster/imu");
    nh.param<double>("common/time_offset_lidar_to_imu", time_diff_lidar_to_imu, 0.0);
    nh.param<double>("filter_size_surf",filter_size_surf_min,0.5);
    nh.param<double>("filter_size_map",filter_size_map_min,0.5);
    nh.param<double>("cube_side_length",cube_len,200);
    nh.param<float>("mapping/det_range",DET_RANGE,300.f);
    nh.param<double>("mapping/fov_degree",fov_deg,180);
    nh.param<double>("mapping/gyr_cov",gyr_cov,0.1);
    nh.param<double>("mapping/acc_cov",acc_cov,0.1);
    nh.param<double>("mapping/b_gyr_cov",b_gyr_cov,0.0001);
    nh.param<double>("mapping/b_acc_cov",b_acc_cov,0.0001);
    nh.param<double>("preprocess/blind", p_pre->blind, 0.01);
    nh.param<int>("preprocess/lidar_type", p_pre->lidar_type, OUST64);
    nh.param<int>("preprocess/scan_line", p_pre->N_SCANS, 64);
    nh.param<int>("preprocess/timestamp_unit", p_pre->time_unit, NS);
    nh.param<int>("preprocess/scan_rate", p_pre->SCAN_RATE, 20);
    nh.param<double>("preprocess/max_scan_duration_s", p_pre->max_scan_duration_s, 0.1);
    nh.param<int>("point_filter_num", p_pre->point_filter_num, 2);
    nh.param<bool>("feature_extract_enable", p_pre->feature_enabled, false);
    nh.param<bool>("runtime_pos_log_enable", runtime_pos_log, 0);
    nh.param<bool>("mapping/extrinsic_est_en", extrinsic_est_en, false);
    nh.param<vector<double>>("mapping/extrinsic_T", extrinT, vector<double>());
    nh.param<vector<double>>("mapping/extrinsic_R", extrinR, vector<double>());

    if (extrinT.size() != 3 || extrinR.size() != 9)
    {
        ROS_FATAL("extrinsic_T/R must have 3/9 entries.");
        return 1;
    }
    if (p_pre->lidar_type != OUST64 || p_pre->time_unit != NS || p_pre->feature_enabled ||
        p_pre->point_filter_num < 1 || p_pre->N_SCANS < 1 || p_pre->N_SCANS > 128 ||
        p_pre->SCAN_RATE < 1 || !std::isfinite(p_pre->max_scan_duration_s) ||
        p_pre->max_scan_duration_s <= 0.0 || !std::isfinite(p_pre->blind) || p_pre->blind < 0.0 ||
        !std::isfinite(filter_size_surf_min) || filter_size_surf_min <= 0.0 ||
        !std::isfinite(filter_size_map_min) || filter_size_map_min <= 0.0 ||
        !std::isfinite(cube_len) || cube_len <= 0.0 ||
        !std::isfinite(DET_RANGE) || DET_RANGE <= 0.0 || NUM_MAX_ITERATIONS < 1 ||
        !std::isfinite(time_diff_lidar_to_imu) ||
        !std::isfinite(acc_cov) || acc_cov <= 0.0 || !std::isfinite(gyr_cov) || gyr_cov <= 0.0 ||
        !std::isfinite(b_acc_cov) || b_acc_cov <= 0.0 || !std::isfinite(b_gyr_cov) || b_gyr_cov <= 0.0)
    {
        ROS_FATAL("Ouster frontend requires lidar_type=3, timestamp_unit=3, feature_extract_enable=false, and positive preprocessing/voxel settings.");
        return 1;
    }
    for (double value : extrinT) if (!std::isfinite(value)) { ROS_FATAL("Invalid extrinsic translation."); return 1; }
    for (double value : extrinR) if (!std::isfinite(value)) { ROS_FATAL("Invalid extrinsic rotation."); return 1; }
    if (odom_frame.empty() || body_frame.empty() || odom_frame == body_frame)
    {
        ROS_FATAL("Odometry parent and IMU body frames must be distinct, nonempty names.");
        return 1;
    }

    lidar_mean_scantime = 1.0 / p_pre->SCAN_RATE;
    
    path.header.stamp    = ros::Time::now();
    path.header.frame_id = odom_frame;

    /*** variables definition ***/
    
    FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
    HALF_FOV_COS = cos((FOV_DEG) * 0.5 * PI_M / 180.0);


    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(res_last, -1000.0f, sizeof(res_last));
    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
    downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(res_last, -1000.0f, sizeof(res_last));

    Lidar_T_wrt_IMU<<VEC_FROM_ARRAY(extrinT);
    Lidar_R_wrt_IMU<<MAT_FROM_ARRAY(extrinR);
    p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
    p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
    p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
    p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));

    double epsi[23] = {0.001};
    fill(epsi, epsi+23, 0.001);
    kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS, epsi);

    // No files are opened under src/. Runtime logging is an optional console
    // timing summary; experiment results are owned by the localization node.
    ros::Subscriber sub_pcl = nh.subscribe(lid_topic, 10, standard_pcl_cbk);
    ros::Subscriber sub_imu = nh.subscribe(imu_topic, 200, imu_cbk);
    ros::Publisher pubLaserCloudFull = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_registered", 2);
    ros::Publisher pubLaserCloudFull_body = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_registered_body", 4);
    ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry> 
            ("/Odometry", 50);
    ros::Publisher pubPath          = nh.advertise<nav_msgs::Path> 
            ("/path", 4);
    pubLioGravity = nh.advertise<geometry_msgs::Vector3Stamped>("/lio_gravity", 1, true);
//------------------------------------------------------------------------------------------------------
    signal(SIGINT, SigHandle);
    ros::Rate rate(5000);
    bool status = ros::ok();

    while (status)
    {
        if (flg_exit) break;
        ros::spinOnce();

        if (!system_initialized)
        {
            ikdtree.set_downsample_param(filter_size_map_min);
            system_initialized = true;
            ROS_INFO("Ouster local odometry enabled. Prior map and GNSS initialization are owned by the GPU localizer.");
        }

        if(sync_packages(Measures) && system_initialized) 
        {
            if (flg_first_scan)
            {
                first_lidar_time = Measures.lidar_beg_time;
                p_imu->first_lidar_time = first_lidar_time;
                flg_first_scan = false;
                continue;
            }

            double t0, t1, t3, t5;

            match_time = 0;
            kdtree_search_time = 0.0;
            solve_time = 0;
            solve_const_H_time = 0;
            t0 = omp_get_wtime();

            p_imu->Process(Measures, kf, feats_undistort);
            state_point = kf.get_x();
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;

            if (!feats_undistort || feats_undistort->empty())
            {
                const double imu_first = Measures.imu.empty() ? -1.0 : Measures.imu.front()->header.stamp.toSec();
                const double imu_last = Measures.imu.empty() ? -1.0 : Measures.imu.back()->header.stamp.toSec();
                if (p_imu->last_scan_initializing())
                    ROS_INFO_THROTTLE(1.0, "LIO_INITIALIZING: IMU=%d/300 input=%zu scan_imu=%zu; no odometry yet is expected.",
                      p_imu->initialization_samples(), Measures.lidar->size(), Measures.imu.size());
                else
                    ROS_WARN_THROTTLE(1.0, "LIO_EMPTY_DESKEW: input=%zu scan_imu=%zu imu_span=[%.6f %.6f] scan_span=[%.6f %.6f] mean_acc=%.6f",
                      Measures.lidar->size(), Measures.imu.size(), imu_first, imu_last,
                      Measures.lidar_beg_time, Measures.lidar_end_time, p_imu->mean_acc_norm());
                continue;
            }
            auto& deskewed = feats_undistort->points;
            const std::size_t before_finite_filter = deskewed.size();
            deskewed.erase(std::remove_if(deskewed.begin(), deskewed.end(),
              [](const PointType& p) { return !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) || !std::isfinite(p.curvature); }), deskewed.end());
            feats_undistort->width = deskewed.size();
            feats_undistort->height = 1;
            feats_undistort->is_dense = true;
            if (deskewed.size() != before_finite_filter)
                ROS_WARN_THROTTLE(1.0, "LIO_NONFINITE_DESKEW: removed=%zu before=%zu; inspect IMU/deskew.", before_finite_filter - deskewed.size(), before_finite_filter);


            flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? \
                            false : true;
            /*** Segment the map in lidar FOV ***/
            lasermap_fov_segment();

            /*** downsample the feature points in a scan ***/
            downSizeFilterSurf.setInputCloud(feats_undistort);
            downSizeFilterSurf.filter(*feats_down_body);
            t1 = omp_get_wtime();
            feats_down_size = feats_down_body->points.size();

            kdtree_size_st = ikdtree.size();
            

            /*** ICP and iterated Kalman filter update ***/
            if (feats_down_size < 5)
            {
                ROS_WARN_THROTTLE(1.0, "LIO_TOO_FEW_VOXEL: input=%zu deskew_finite=%zu voxel=%d leaf=%.3f scan_imu=%zu", Measures.lidar->size(), feats_undistort->size(), feats_down_size, filter_size_surf_min, Measures.imu.size());
                continue;
            }
            
            if (feats_down_size > 100000)
            {
                ROS_ERROR_THROTTLE(1.0, "LIO_TOO_MANY_VOXEL: %d exceeds frontend scratch capacity; increase point_filter_num or filter_size_surf.", feats_down_size);
                continue;
            }
            normvec->resize(feats_down_size);
            feats_down_world->resize(feats_down_size);

            if (ikdtree.Root_Node == nullptr)
            {
                for (int i = 0; i < feats_down_size; ++i)
                    pointBodyToWorld(&feats_down_body->points[i], &feats_down_world->points[i]);
                ikdtree.Build(feats_down_world->points);
                euler_cur = SO3ToEuler(state_point.rot);
                geoQuat.x = state_point.rot.coeffs()[0];
                geoQuat.y = state_point.rot.coeffs()[1];
                geoQuat.z = state_point.rot.coeffs()[2];
                geoQuat.w = state_point.rot.coeffs()[3];
                publish_odometry(pubOdomAftMapped);
                ++scan_counter;
                if (scan_pub_en) publish_frame_world(pubLaserCloudFull);
                if (path_en) publish_path(pubPath);
                if (scan_body_pub_en) publish_frame_body(pubLaserCloudFull_body);
                ROS_INFO("Local ikd-tree initialized with %d points; GNSS map initialization is handled by the GPU localizer.", feats_down_size);
                continue;
            }

            Nearest_Points.resize(feats_down_size);


            /*** iterated state estimation ***/
            double t_update_start = omp_get_wtime();
            double solve_H_time = 0;
            kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
            state_point = kf.get_x();
            euler_cur = SO3ToEuler(state_point.rot);
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
            geoQuat.x = state_point.rot.coeffs()[0];
            geoQuat.y = state_point.rot.coeffs()[1];
            geoQuat.z = state_point.rot.coeffs()[2];
            geoQuat.w = state_point.rot.coeffs()[3];

            double t_update_end = omp_get_wtime();

            /******* Publish odometry *******/
            publish_odometry(pubOdomAftMapped);
            scan_counter ++;

            /*** add the feature points to map kdtree ***/
            t3 = omp_get_wtime();
            map_incremental();
            t5 = omp_get_wtime();
            
            /******* Publish points *******/
            if (scan_pub_en)      publish_frame_world(pubLaserCloudFull);
            if (path_en)                         publish_path(pubPath);
            if (scan_body_pub_en) publish_frame_body(pubLaserCloudFull_body);

            if (runtime_pos_log)
                ROS_INFO_THROTTLE(1.0, "LIO_TIMING: scan_total_ms=%.3f deskew_voxel_ms=%.3f update_ms=%.3f map_ms=%.3f points=%d",
                  1000.0*(t5-t0), 1000.0*(t1-t0), 1000.0*(t_update_end-t_update_start),
                  1000.0*(t5-t3), feats_down_size);
        }

        status = ros::ok();
        rate.sleep();
    }

    return 0;
}
