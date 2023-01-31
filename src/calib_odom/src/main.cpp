#include <string>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/JointState.h>
#include <nav_core/recovery_behavior.h>
#include <tf/transform_broadcaster.h>
#include <tf/message_filter.h>

#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include "sensor_msgs/LaserScan.h"
#include "stdio.h"

#include "boost/asio.hpp"
#include "boost/bind.hpp"
#include "math.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

//pclz
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/registration.h>
#include <pcl/point_clound.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>

//Eigen
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include "odom_calib.hpp"
#include <csm/csm_all.h>

using namespace std;
using namespace boost::asio; 


OdomCalib odom_calib;

std::vector<geometry_msgs::PointStamped> mcu_path;

Eigen::Vector3d cal_delete_distence(Eigen::Vector3d odom_pose);

/*
获取激光数据类
*/
class Scan2 {
  public:
    Scan2();

    //进行PL-ICP需要的变量
    LDP m_prevLDP;
    sm_params m_PIICPParams;
    sm_result m_OutputResult;

    //odom && scan进行位姿积分
    Eigen::Vector3d scan_pos_cal;
    Eigen::Vector3d odom_pos_cal;

    std::vector<Eigen::Vector3d> odom_increments;

    std::string odom_frame_;
    std::string base_frame_;

    ros::NodeHandle node_;
    tf::TransformListener tf_;

    ros::Subscriber calib_flag_sub_;

    ros::Publisher odom_path_pub_, scan_path_pub_, calib_path_pub_;

    nav_msgs::Path path_odom, path_scan;

    ros::Time current_time;

    //　进行时间同步
    message_filters::Subscrible<sensor_msgs::LaserScan>* scan_filter_sub_;
    tf::MessageFilter<sensor_msgs::LaserScan>* scan_filter_;

    void CalibFlagCallBack(const std_msgs::Empty &msg);

    //回调函数
    void scanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan2);

    //tf树查询里程计位姿
    bool getOdomPose(Eigen::Vector3d& pose, const ros::Time& t);

    //发布odom & laser path
    void pub_msg(Eigen::Vector3d& pose, nav_msgs::Path& path, ros::Publisher& mcu_path_pub_);

    //为了发布correct path
    void publishPathEigen(std::vector<Eigen::Vector3d>& path_eigen, ros::Publisher& path_pub_);

    //进行pi-icp对相关函数
    void SetPIICPParams();
    void LaserScanToLDP(sensor_msgs::LaserScan* pScan, LDP& ldp);
    Eigen::Vector3d PIICPBetweenTwoFrames(LDP& currentLDPScan, Eigen::Vector3d tmpPose);
};

Eigen::Vector3d now_pose, last_pose;

void Scan2::pub_msg(Eigen::Vector3d& pose, nav_msgs::Path& path, ros::Publisher& mcu_path_pub_) {
  current_time = ros::Time::now();
  geometry_msgs::PoseStamped this_pose_stamped;
  this_pose_stamped.pose.position.x = pose(0);
  this_pose_stamped.pose.position.y = pose(1);

  geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(pose(2));
  this_pose_stamped.pose.orientation.x = goal_quat.x;
  this_pose_stamped.pose.orientation.y = goal_quat.y;
  this_pose_stamped.pose.orientation.z = goal_quat.z;
  this_pose_stamped.pose.orientation.w = goal_quat.w;

  this_pose_stamped.header.stamp = current_time;
  this_pose_stamped.header.frame_id = "odom";
  path.poses.push_back(this_pose_stamped);
  mcu_path_pub_.pushlish(path);
}

void Scan2::publishPathEigen(std::vector<Eigen::Vector3d>& path_eigen, ros::Publisher& path_pub_) {
  nav_msgs::Path visual_path;

  current_time = ros::Time::now();

  visual_path.header.stamp = ros::Time::now();
  visual_path.header.frame_id = "odom";

  geometry_msgs::PoseStamped tmpPose;
  tmpPose.header.stamp = current_time;
  temPose.header.frame_id = "odom";

  for (int i = 0; i < path_eigen.size(); i ++ ) {
    Eigen::Vector3d poseEigen = path_eigen[i];

    tmpPose.pose.position.x = poseEigen(0);
    tmpPose.pose.position.y = poseEigen(1);

    geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(poseEigen(2));
    tmpPose.pose.orientation.x = goal_quat.x;
    tmpPose.pose.orientation.y = goal_quat.y;
    tmpPose.pose.orientation.z = goal_quat.z;
    tmpPose.pose.orientation.w = goal_quat.w;

    visual_path.pose.push_back(tmpPose);
  }
  path_pub_.publish(visual_path);
}

/*
得到时刻ｔ　时候　机器人在里程计坐标下的坐标
*/
bool Scan2::getOdomPose(Eigen::Vector3d& pose, const ros::Time& t) {

  //Get the robot's pose
  tf::Stamped<tf::Pose> ident(tf::Transform(tf::createQuaternionFromRPY(0, 0, 0),
    tf::Vector3(0, 0, 0)), t, base_frame_);

  tf::Stamped<tf::Transform> odom_pose;
  try {
    tf_.transformPose(odom_frame_, ident, odom_pose);
  } catch (tf::TransformException e) {
    ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
    return false;
  }

  double yaw = tf::getYaw(odom_pose.getRotation());

  pose << odom_pose.getOrigin().x(),
          odom_pose.getOrigin().y(),
          yaw;

  //pub_msg(pose, path_odom, odom_path_pub_);

  return true;
}

Scan2::Scan2() {
  ros::NodeHandle private_nh_("~");

  m_prevLDP = NULL;
  SetPIICPParams();

  scan_pose_cal.setZero();
  odom_pose_cal.setZero();
  odom_increments.clear();

  if (!private_nh_.getParam("odom_frame", odom_frame_))
    odom_frame = "odom";
  if (!private_nh_.getParam("base_frame", base_frame_))
    base_frame = "base_link";

  
}