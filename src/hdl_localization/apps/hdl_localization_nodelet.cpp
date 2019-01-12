#include <mutex>
#include <memory>
#include <iostream>
#include <fstream>
#include <boost/circular_buffer.hpp>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <pcl/filters/voxel_grid.h>

#include <pclomp/ndt_omp.h>

#include <hdl_localization/pose_estimator.hpp>
#include <pcl/registration/icp.h>
#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include "UTM.h"
namespace hdl_localization {

class HdlLocalizationNodelet : public nodelet::Nodelet {
public:
  using PointT = pcl::PointXYZ;

  HdlLocalizationNodelet() {
  }
  virtual ~HdlLocalizationNodelet() {
  }


  void onInit() override {
    nh = getNodeHandle();
    mt_nh = getMTNodeHandle();
    private_nh = getPrivateNodeHandle();

    processing_time.resize(16);

    points_sub = mt_nh.subscribe("/velodyne_points", 5, &HdlLocalizationNodelet::points_callback, this);
    globalmap_sub = nh.subscribe("/globalmap", 1, &HdlLocalizationNodelet::globalmap_callback, this);
    fix_sub = nh.subscribe("/fix", 1, &HdlLocalizationNodelet::fix_callback, this);
    is_match_vaild_sub = nh.subscribe("/initialpose", 1, &HdlLocalizationNodelet::is_match_vaild_callback, this);

    localization_pub = nh.advertise<nav_msgs::Odometry>("/lidar_localization", 5, false);
    pure_lidar_localization_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/pure_lidar_localization", 5, false);
    aligned_pub = nh.advertise<sensor_msgs::PointCloud2>("/aligned_points", 5, false);
    fix_map_localization_pub = nh.advertise<sensor_msgs::NavSatFix>("/fix_map_localization", 5, false);

    initialize_params();  
  }

private:
  void initialize_params() {

    // intialize scan matching method
    std::string ndt_neighbor_search_method = private_nh.param<std::string>("ndt_neighbor_search_method", "DIRECT7");
    double ndt_resolution = private_nh.param<double>("ndt_resolution", 1.0);
    pclomp::NormalDistributionsTransform<PointT, PointT>::Ptr ndt(new pclomp::NormalDistributionsTransform<PointT, PointT>());
    ndt->setTransformationEpsilon(0.01);
    ndt->setResolution(ndt_resolution);
    if(ndt_neighbor_search_method == "DIRECT1") {
      NODELET_INFO("search_method DIRECT1 is selected");
      ndt->setNeighborhoodSearchMethod(pclomp::DIRECT1);
    } else if(ndt_neighbor_search_method == "DIRECT7") {
      NODELET_INFO("search_method DIRECT7 is selected");
      ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
    } else {
      if(ndt_neighbor_search_method == "KDTREE") {
        NODELET_INFO("search_method KDTREE is selected");
      } else {
        NODELET_WARN("invalid search method was given");
        NODELET_WARN("default method is selected (KDTREE)");
      }
      ndt->setNeighborhoodSearchMethod(pclomp::KDTREE);
    }
    registration = ndt;

    // 
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    // trans_lidar_output and trans_lidar_gps from static tf
    geometry_msgs::TransformStamped transformStamped;
    Eigen::Affine3d transform_eigen;
    try{
      transformStamped = tfBuffer.lookupTransform("rslidar", "output_link", ros::Time(0), ros::Duration(1.0));
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    transform_eigen = tf2::transformToEigen(transformStamped);
    trans_lidar_output = transform_eigen.matrix();

    try{
      transformStamped = tfBuffer.lookupTransform("gps_link", "rslidar", ros::Time(0), ros::Duration(1.0));
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    transform_eigen = tf2::transformToEigen(transformStamped);
    trans_gps_lidar = transform_eigen.matrix();

    // trans_utm_map from parameters
    // trans_base_output
    Eigen::AngleAxisd rotation_base_output(0.0, Eigen::Vector3d::UnitZ());
    Eigen::Translation3d translation_base_output(-0.825123, 0.000, -0.942);
    Eigen::Matrix4d trans_base_output = (translation_base_output * rotation_base_output).matrix();

    double map_x = private_nh.param<double>("map_x", 0.0);
    double map_y = private_nh.param<double>("map_y", 0.0);
    double map_z = private_nh.param<double>("map_z", 0.0);
    double map_theta = private_nh.param<double>("map_theta", 0.0);
    Eigen::AngleAxisd rotation_utm_map(DegToRad(map_theta), Eigen::Vector3d::UnitZ());
    Eigen::Translation3d translation_utm_map(map_x, map_y, map_z);
    trans_utm_map = (translation_utm_map * rotation_utm_map).matrix(); 
    NODELET_INFO("trans_utm_map init OK!!!");
    init_trans_utm_map = true;

    ros::Rate rate(20.0);
    std::ofstream result_file;
    std::string resultfile_path = private_nh.param<std::string>("resultfile_path", "/home/neousys/Data/jdd/result.txt");
    result_file.open(resultfile_path);
    if (!result_file)
    {
      std::cout << "can not open result txt" << std::endl;
    }
    while (nh.ok()){
      if (!init_trans_map_lidar)  continue;  
      try{
        // use rslidar->map but not rslidar->odom for more precise result
        transformStamped = tfBuffer.lookupTransform("odom", "rslidar", ros::Time(0), ros::Duration(1.0));
      }
      catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
      }
      transform_eigen = tf2::transformToEigen(transformStamped);
      trans_odom_lidar = transform_eigen.matrix();
      trans_map_lidar = trans_map_odom * trans_odom_lidar; 
      trans_utm_output = trans_utm_map * trans_base_output.inverse() * trans_map_lidar * trans_lidar_output;
      
      // publish output -> map 
      publish_localization_pose(transformStamped.header.stamp, trans_map_lidar, "map", "rslidar");
      // publish lidar->odom
      // publish_localization_pose(transformStamped.header.stamp, trans_odom_lidar, "odom", "rslidar");

      // plot to rviz

      // write trans_utm_output to result file, use tum file format
    
      Eigen::Quaterniond quat_utm_output(trans_utm_output.block<3, 3>(0, 0));
      result_file << std::setprecision(16)
            << transformStamped.header.stamp.sec+transformStamped.header.stamp.nsec*1e-9
            << " "
            << trans_utm_output.block<3, 1>(0, 3)[0]
            << " "
            << trans_utm_output.block<3, 1>(0, 3)[1]
            << " "
            << trans_utm_output.block<3, 1>(0, 3)[2]
            << " "
            << quat_utm_output.x()
            << " "
            << quat_utm_output.y()
            << " "
            << quat_utm_output.z()
            << " "
            << quat_utm_output.w()
            << std::endl;

      ros::spinOnce();
      rate.sleep();
    }
    result_file.close();
  }

private:
  /**
   * @brief callback for point cloud data
   * @param points_msg
   */
  void points_callback(const sensor_msgs::PointCloud2ConstPtr& points_msg) {
    if(!globalmap) {
      NODELET_ERROR("globalmap has not been received!!");
      return;
    }

    if(!init_trans_map_lidar) {
      NODELET_ERROR("need GPS data to init trans_map_lidar!!");
      return;
    }   
    if(lidar_count++%5!=0)
      return;

    const auto& stamp = points_msg->header.stamp;
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*points_msg, *cloud);

    if(cloud->empty()) {
      NODELET_ERROR("cloud is empty!!");
      return;
    }

    std::vector<int> target_indices;                              //保存去除的点的索引
    pcl::removeNaNFromPointCloud(*cloud, *cloud, target_indices); //去除点云中的NaN点

    // relocalization
    auto t1 = ros::WallTime::now();
    pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>());
    registration->setInputSource(cloud);
    registration->align(*aligned, trans_map_lidar.cast<float>());
    std::cout << "map_lidar Normal Distributions Transform has converged:" << registration->hasConverged()
            << " map_lidar score: " << registration->getFitnessScore() << std::endl;
    auto t2 = ros::WallTime::now();
    processing_time.push_back((t2 - t1).toSec());
    double avg_processing_time = std::accumulate(processing_time.begin(), processing_time.end(), 0.0) / processing_time.size();
    NODELET_INFO_STREAM("processing_time: " << avg_processing_time * 1000.0 << "[msec]");
    if (is_match_vaild)
    {
      trans_map_lidar = registration->getFinalTransformation().cast<double>();
      // publish trans_map_lidar
      geometry_msgs::PoseWithCovarianceStamped lidar_pure_pose;
      lidar_pure_pose.header.stamp = stamp;
      lidar_pure_pose.header.frame_id = "map";
      lidar_pure_pose.pose = matrix2PoseWithCovariance(trans_map_lidar);
      pure_lidar_localization_pub.publish(lidar_pure_pose);

      trans_map_odom = trans_map_lidar * trans_odom_lidar.inverse();
      std::cout << "trans_map_lidar translation = \n" << trans_map_lidar.block<3, 1>(0, 3) << std::endl;
      std::cout << " trans_map_lidar euler = \n" << trans_map_lidar.block<3, 3>(0, 0).eulerAngles(0, 1, 2) << std::endl;
    }
    publish_fix_map_localization(stamp, trans_map_lidar);
    if(aligned_pub.getNumSubscribers()) {
      aligned->header.frame_id = "map";
      aligned->header.stamp = cloud->header.stamp;
      aligned_pub.publish(aligned);
    }
  }

  /**
   * @brief callback for globalmap input
   * @param points_msg
   */
  void globalmap_callback(const sensor_msgs::PointCloud2ConstPtr& points_msg) {
    NODELET_INFO("globalmap received!");
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*points_msg, *cloud);
    globalmap = cloud;

    registration->setInputTarget(globalmap);
  }

  /**
   * @brief callback for fix input, using initial gps to init trans_map_lidar
   * @param fix_msg
   */
  void fix_callback(const sensor_msgs::NavSatFix& fix_msg) {
    if (init_trans_map_lidar || !init_trans_utm_map)
      return;
    if (fix_msg.status.status != 4)
    {
      NODELET_ERROR("GPS signal is not good, can not init trans_map_lidar!!!");
      return;
    }
    double gps_x;
    double gps_y;
    double gps_z;
    double gps_theta;
    LatLonToUTMXY(fix_msg.latitude, fix_msg.longitude, 50, gps_x, gps_y);
    std::cout << "gps_x = " << gps_x << " gps_y = " << gps_y << std::endl;
    gps_theta = DegToRad(fix_msg.position_covariance[0]);
    gps_z = fix_msg.altitude;
    Eigen::AngleAxisd rotation_utm_gps(gps_theta, Eigen::Vector3d::UnitZ());
    Eigen::Translation3d translation_utm_gps(gps_x, gps_y, gps_z);
    Eigen::Matrix4d trans_utm_gps = (translation_utm_gps * rotation_utm_gps).matrix(); 
    trans_map_lidar = trans_utm_map.inverse() * trans_utm_gps * trans_gps_lidar;
    trans_map_odom = trans_map_lidar;
    init_trans_map_lidar = true;
    trans_utm_output = trans_utm_map * trans_map_lidar * trans_lidar_output;
    std::cout << std::setprecision(12) << "initial trans_utm_output translation = \n" << trans_utm_output.block<3, 1>(0, 3) << std::endl 
    << "initial trans_utm_output euler = \n" << trans_utm_output.block<3, 3>(0, 0).eulerAngles(0, 1, 2) << std::endl;

    NODELET_INFO("GPS init trans_map_lidar OK!!!");
  }

  /**
   * @brief publish localization_pose
   * @param stamp  timestamp
   * @param pose   lidar localization pose to be published
   */
  void publish_localization_pose(const ros::Time& stamp, const Eigen::Matrix4d& pose, const std::string& frame_id, const std::string& child_frame_id) {
    // broadcast the transform over tf
    geometry_msgs::TransformStamped odom_trans = matrix2transform(stamp, pose, frame_id, child_frame_id);

    // publish the transform
    nav_msgs::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = frame_id;

    odom.pose.pose.position.x = pose(0, 3);
    odom.pose.pose.position.y = pose(1, 3);
    odom.pose.pose.position.z = pose(2, 3);
    odom.pose.pose.orientation = odom_trans.transform.rotation;

    odom.child_frame_id = child_frame_id;
    odom.twist.twist.linear.x = 0.0;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = 0.0;

    odom.pose.covariance[0] = 0.02;
    odom.pose.covariance[7] = 0.02;
    odom.pose.covariance[14] = 0.02;
    odom.pose.covariance[21] = 2/180*M_PI;
    odom.pose.covariance[28] = 2/180*M_PI;
    odom.pose.covariance[35] = 2/180*M_PI;

    localization_pub.publish(odom);
  }

  /**
   * @brief publish fix_map_localization
   * @param stamp  timestamp
   * @param pose   lidar localization pose to be published
   */
  void publish_fix_map_localization(const ros::Time& stamp, const Eigen::Matrix4d& pose) {

    // publish the transform
    sensor_msgs::NavSatFix fix_map_localization;
    fix_map_localization.header.stamp = stamp;
    fix_map_localization.header.frame_id = "rslidar";
    if (is_match_vaild)
    {
      fix_map_localization.status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
    }else {
      fix_map_localization.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
    }
    fix_map_localization.latitude = pose.block<3, 1>(0, 3)[0];
    fix_map_localization.longitude = pose.block<3, 1>(0, 3)[1];
    fix_map_localization.altitude = pose.block<3, 1>(0, 3)[2];
    fix_map_localization.position_covariance[0] = pose.block<3, 3>(0, 0).eulerAngles(0, 1, 2)[0];
    fix_map_localization.position_covariance[1] = pose.block<3, 3>(0, 0).eulerAngles(0, 1, 2)[1];
    fix_map_localization.position_covariance[2] = pose.block<3, 3>(0, 0).eulerAngles(0, 1, 2)[2];
    fix_map_localization_pub.publish(fix_map_localization);
  }
  /**
   * @brief subscribe rivz topic "/initialpose" to change match_vaild state
   */
  void is_match_vaild_callback(const geometry_msgs::PoseWithCovarianceStamped& msg){
      // use rviz  to set is_match_vaild
      if (is_match_vaild == true)
      {
        is_match_vaild = false;
        std::cout << " set is_match_vaild to false" << std::endl;
      }else if(is_match_vaild == false)
      {
        is_match_vaild = true;
        std::cout << " set is_match_vaild to true" << std::endl;    

        // use gps to reinitialize   
        // init_trans_map_lidar = false;   
      }
  }
  /**
   * @brief convert a Eigen::Matrix to TransformedStamped
   * @param stamp           timestamp
   * @param pose            pose matrix
   * @param frame_id        frame_id
   * @param child_frame_id  child_frame_id
   * @return transform
   */
  geometry_msgs::TransformStamped matrix2transform(const ros::Time& stamp, const Eigen::Matrix4d& pose, const std::string& frame_id, const std::string& child_frame_id) {
    Eigen::Quaterniond quat(pose.block<3, 3>(0, 0));
    quat.normalize();
    geometry_msgs::Quaternion odom_quat;
    odom_quat.w = quat.w();
    odom_quat.x = quat.x();
    odom_quat.y = quat.y();
    odom_quat.z = quat.z();

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = stamp;
    odom_trans.header.frame_id = frame_id;
    odom_trans.child_frame_id = child_frame_id;

    odom_trans.transform.translation.x = pose(0, 3);
    odom_trans.transform.translation.y = pose(1, 3);
    odom_trans.transform.translation.z = pose(2, 3);
    odom_trans.transform.rotation = odom_quat;

    return odom_trans;
  }

  geometry_msgs::PoseWithCovariance matrix2PoseWithCovariance(const Eigen::Matrix4d pose) {
    Eigen::Quaterniond quat(pose.block<3, 3>(0, 0));
    quat.normalize();
    geometry_msgs::PoseWithCovariance pose_pub;
    pose_pub.pose.orientation.w = quat.w();
    pose_pub.pose.orientation.x = quat.x();
    pose_pub.pose.orientation.y = quat.y();
    pose_pub.pose.orientation.z = quat.z();

    pose_pub.pose.position.x = pose.block<3, 1>(0, 3)[0];
    pose_pub.pose.position.y = pose.block<3, 1>(0, 3)[1];
    pose_pub.pose.position.z = pose.block<3, 1>(0, 3)[2];

    pose_pub.covariance[0] = 0.0004;
    pose_pub.covariance[7] = 0.0004;
    pose_pub.covariance[14] = 0.0004;
    pose_pub.covariance[21] = 1/180*M_PI/180*M_PI;
    pose_pub.covariance[28] = 1/180*M_PI/180*M_PI;
    pose_pub.covariance[35] = 1/180*M_PI/180*M_PI;
    return pose_pub;
  }

private:
  // ROS
  ros::NodeHandle nh;
  ros::NodeHandle mt_nh;
  ros::NodeHandle private_nh;

  ros::Subscriber points_sub;
  ros::Subscriber globalmap_sub;
  ros::Subscriber fix_sub;
  ros::Subscriber is_match_vaild_sub;

  ros::Publisher localization_pub;
  ros::Publisher aligned_pub;
  ros::Publisher fix_map_localization_pub;
  ros::Publisher pure_lidar_localization_pub;
  // globalmap and registration method
  pcl::PointCloud<PointT>::Ptr globalmap;
  pclomp::NormalDistributionsTransform<PointT, PointT>::Ptr registration;

  // processing time buffer
  boost::circular_buffer<double> processing_time;

  Eigen::Matrix4d trans_map_lidar = Eigen::Matrix4d::Identity(4,4);
  Eigen::Matrix4d trans_odom_lidar = Eigen::Matrix4d::Identity(4,4);
  Eigen::Matrix4d trans_map_odom = Eigen::Matrix4d::Identity(4,4);
  Eigen::Matrix4d trans_lidar_output = Eigen::Matrix4d::Identity(4,4);
  Eigen::Matrix4d trans_utm_output = Eigen::Matrix4d::Identity(4,4);
  Eigen::Matrix4d trans_utm_map = Eigen::Matrix4d::Identity(4,4);
  Eigen::Matrix4d trans_gps_lidar = Eigen::Matrix4d::Identity(4,4);
  Eigen::Matrix4d trans_output_base = Eigen::Matrix4d::Identity(4,4);
  int lidar_count = 0;
  bool init_trans_map_lidar = false;
  bool init_trans_utm_map = false;
  bool is_match_vaild = true;
};

}

PLUGINLIB_EXPORT_CLASS(hdl_localization::HdlLocalizationNodelet, nodelet::Nodelet)