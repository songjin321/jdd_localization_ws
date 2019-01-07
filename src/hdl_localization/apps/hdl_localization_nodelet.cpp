#include <mutex>
#include <memory>
#include <iostream>
#include <boost/circular_buffer.hpp>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
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


    use_imu = private_nh.param<bool>("use_imu", true);
    invert_imu = private_nh.param<bool>("invert_imu", false);
    if(use_imu) {
      NODELET_INFO("enable imu-based prediction");
      imu_sub = mt_nh.subscribe("/gpsimu_driver/imu_data", 256, &HdlLocalizationNodelet::imu_callback, this);
    }
    points_sub = mt_nh.subscribe("/velodyne_points", 5, &HdlLocalizationNodelet::points_callback, this);
    globalmap_sub = nh.subscribe("/globalmap", 1, &HdlLocalizationNodelet::globalmap_callback, this);
    //jdd_globalmap_sub = nh.subscribe("/jdd_globalmap", 1, &HdlLocalizationNodelet::globalmap_jdd_callback, this);
    initialpose_sub = nh.subscribe("/initialpose", 8, &HdlLocalizationNodelet::initialpose_callback, this);

    pose_pub = nh.advertise<nav_msgs::Odometry>("/odom", 5, false);
    aligned_pub = nh.advertise<sensor_msgs::PointCloud2>("/aligned_points", 5, false);
    aligned_jdd_pub = nh.advertise<sensor_msgs::PointCloud2>("/aligned_jdd_points", 5, false);

    // globalmap_pub = nh.advertise<sensor_msgs::PointCloud2>("/globalmap", 5, true);
    // globalmap_pub_jdd = nh.advertise<sensor_msgs::PointCloud2>("/jdd_globalmap", 5, true);
    initialize_params();  
  }

private:
  void initialize_params() {
    // intialize scan matching method
    double downsample_resolution = private_nh.param<double>("downsample_resolution", 0.1);
    std::string ndt_neighbor_search_method = private_nh.param<std::string>("ndt_neighbor_search_method", "DIRECT7");

    double ndt_resolution = private_nh.param<double>("ndt_resolution", 1.0);
    boost::shared_ptr<pcl::VoxelGrid<PointT>> voxelgrid(new pcl::VoxelGrid<PointT>());
    voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
    downsample_filter = voxelgrid;

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

    // ndt_jdd 
    pclomp::NormalDistributionsTransform<PointT, PointT>::Ptr ndt_jdd(new pclomp::NormalDistributionsTransform<PointT, PointT>());
    ndt_jdd->setTransformationEpsilon(0.01);
    ndt_jdd->setResolution(ndt_resolution);
    registration_jdd = ndt_jdd;
    // use icp
    /*
    pcl::IterativeClosestPoint<PointT, PointT>::Ptr icp(new pcl::IterativeClosestPoint<PointT, PointT>());
    icp->setMaximumIterations(100);
    icp->setEuclideanFitnessEpsilon(0.01);
    icp->setTransformationEpsilon(1e-10);
    icp->setMaxCorrespondenceDistance(4);
    icp->setRANSACOutlierRejectionThreshold(0.02);
    registration = icp;
    */

    // initialize pose estimator
    Eigen::AngleAxisf init_rotation(0.0 / 180.0 * M_PI, Eigen::Vector3f::UnitZ());
    Eigen::Translation3f init_translation(0.0,  0.0,  0.0);
    init_guess = (init_translation * init_rotation).matrix();

    // initialize jdd pose estimator
    Eigen::AngleAxisf init_rotation_jdd(-145.9 / 180.0 * M_PI, Eigen::Vector3f::UnitZ());
    Eigen::Translation3f init_translation_jdd(-173.692, 4.57266, 22.1286);
    init_guess_jdd = (init_translation_jdd * init_rotation_jdd).matrix(); 

    // read point cloud map
    /*
    std::cout << "begin read point cloud map !" << std::endl;
    std::string globalmap_pcd = private_nh.param<std::string>("globalmap_pcd", "");
    std::cout << globalmap_pcd << std::endl;
    if (pcl::io::loadPCDFile<PointT>(globalmap_pcd, *globalmap) == -1)
    {
      PCL_ERROR("Couldn't read file\n");
    }
    std::cout << "load ok" << std::endl;
    registration->setInputTarget(globalmap);
    std::cout << "set ok" << std::endl;
    sensor_msgs::PointCloud2 globalmap_ros;
    pcl::toROSMsg(*globalmap, globalmap_ros);
    std::cout << "convert ok" << std::endl;
    globalmap_ros.header.frame_id = "map"; 
    globalmap_pub.publish(globalmap_ros);
    std::cout << "pub ok" << std::endl;

    std::string jdd_globalmap_pcd = private_nh.param<std::string>("jdd_globalmap_pcd", "");
    pcl::io::loadPCDFile(jdd_globalmap_pcd, *globalmap_jdd);
    registration_jdd->setInputTarget(globalmap_jdd);
    sensor_msgs::PointCloud2 globalmap_jdd_ros;
    pcl::toROSMsg(*globalmap_jdd, globalmap_jdd_ros);
    globalmap_jdd_ros.header.frame_id = "map"; 
    globalmap_pub_jdd.publish(globalmap_jdd_ros);
    std::cout << "cannot read point cloud map !" << std::endl;
    */
  }

private:
  /**
   * @brief callback for imu data
   * @param imu_msg
   */
  void imu_callback(const sensor_msgs::ImuConstPtr& imu_msg) {
    std::lock_guard<std::mutex> lock(imu_data_mutex);
    imu_data.push_back(imu_msg);
  }

  /**
   * @brief callback for point cloud data
   * @param points_msg
   */
  void points_callback(const sensor_msgs::PointCloud2ConstPtr& points_msg) {
    lidar_count++;
    /*
    std::lock_guard<std::mutex> estimator_lock(pose_estimator_mutex);
    if(!pose_estimator) {
      NODELET_ERROR("waiting for initial pose input!!");
      return;
    }
    */
   
    if(!globalmap) {
      NODELET_ERROR("globalmap has not been received!!");
      return;
    }

    const auto& stamp = points_msg->header.stamp;
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*points_msg, *cloud);

    if(cloud->empty()) {
      NODELET_ERROR("cloud is empty!!");
      return;
    }

    // auto filtered = downsample(cloud);
    std::vector<int> target_indices;                                            //保存去除的点的索引
    pcl::removeNaNFromPointCloud(*cloud, *cloud, target_indices); //去除点云中的NaN点
    // predict
    /*
    if(!use_imu) {
      pose_estimator->predict(stamp, Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero());
    } else {
      std::lock_guard<std::mutex> lock(imu_data_mutex);
      auto imu_iter = imu_data.begin();
      for(imu_iter; imu_iter != imu_data.end(); imu_iter++) {
        if(stamp < (*imu_iter)->header.stamp) {
          break;
        }
        const auto& acc = (*imu_iter)->linear_acceleration;
        const auto& gyro = (*imu_iter)->angular_velocity;
        double gyro_sign = invert_imu ? -1.0 : 1.0;
        pose_estimator->predict((*imu_iter)->header.stamp, Eigen::Vector3f(acc.x, acc.y, acc.z), gyro_sign * Eigen::Vector3f(gyro.x, gyro.y, gyro.z));
      }
      imu_data.erase(imu_data.begin(), imu_iter);
    }
    */
    // correct
    auto t1 = ros::WallTime::now();

    pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>());
    registration->setInputSource(cloud);
    registration->align(*aligned, init_guess);
    std::cout << "map_lidar Normal Distributions Transform has converged:" << registration->hasConverged()
            << " map_lidar score: " << registration->getFitnessScore() << std::endl;
    Eigen::Matrix4f trans_map_lidar = registration->getFinalTransformation();
    trans_jdd_map = init_guess_jdd;
    Eigen::Matrix4f trans_jdd_lidar = trans_jdd_map * trans_map_lidar;
    init_guess = trans_map_lidar;
    // log
    // std::cout << "Normal Distributions Transform has converged:" << registration->ndthasConverged()
    //           << " score: " << registration->getFitnessScore() << std::endl;
    Eigen::Vector3f p = trans_map_lidar.block<3, 1>(0, 3);
    Eigen::Quaternionf q(trans_map_lidar.block<3, 3>(0, 0));
    std::cout << "map_lidar translation = \n" << p << std::endl;
    std::cout << " map_lidar euler = \n" << trans_map_lidar.block<3, 3>(0, 0).eulerAngles(0, 1, 2) << std::endl;

    auto t2 = ros::WallTime::now();

    processing_time.push_back((t2 - t1).toSec());
    double avg_processing_time = std::accumulate(processing_time.begin(), processing_time.end(), 0.0) / processing_time.size();
    NODELET_INFO_STREAM("processing_time: " << avg_processing_time * 1000.0 << "[msec]");

    if(aligned_pub.getNumSubscribers()) {
      aligned->header.frame_id = "map";
      aligned->header.stamp = cloud->header.stamp;
      aligned_pub.publish(aligned);
    }

    // correct jdd
    /*
    if (lidar_count%10 == 0)
    {
      auto t1 = ros::WallTime::now();

      pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>());
      registration_jdd->setInputSource(cloud);
      registration_jdd->align(*aligned, init_guess_jdd);
      std::cout << "jdd_lidar Normal Distributions Transform has converged:" << registration_jdd->hasConverged()
              << " jdd_lidar score: " << registration_jdd->getFitnessScore() << std::endl;
      Eigen::Matrix4f trans_jdd_lidar = registration_jdd->getFinalTransformation();
      trans_jdd_map = trans_jdd_lidar * trans_map_lidar.inverse();
      std::cout << "jdd_map translation = \n" << trans_jdd_map.block<3, 1>(0, 3) << std::endl;
      std::cout << " jdd_map euler = \n" << trans_jdd_map.block<3, 3>(0, 0).eulerAngles(0, 1, 2) << std::endl;
      init_guess_jdd = trans_jdd_lidar;
      if(aligned_pub.getNumSubscribers()) {
        aligned->header.frame_id = "map";
        aligned->header.stamp = cloud->header.stamp;
        aligned_jdd_pub.publish(aligned);
      }
    }
    */
    // publish_odometry(points_msg->header.stamp, pose_estimator->matrix());
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
   * @brief callback for jdd_globalmap input
   * @param points_msg
   */
  void globalmap_jdd_callback(const sensor_msgs::PointCloud2ConstPtr& points_msg) {
    NODELET_INFO("jdd globalmap received!");
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*points_msg, *cloud);
    globalmap_jdd = cloud;

    registration_jdd->setInputTarget(globalmap_jdd);
  }

  /**
   * @brief callback for initial pose input ("2D Pose Estimate" on rviz)
   * @param pose_msg
   */
  void initialpose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose_msg) {
    NODELET_INFO("initial pose received!!");
    std::lock_guard<std::mutex> lock(pose_estimator_mutex);
    const auto& p = pose_msg->pose.pose.position;
    const auto& q = pose_msg->pose.pose.orientation;

  }

  /**
   * @brief downsampling
   * @param cloud   input cloud
   * @return downsampled cloud
   */
  pcl::PointCloud<PointT>::ConstPtr downsample(const pcl::PointCloud<PointT>::ConstPtr& cloud) const {
    if(!downsample_filter) {
      return cloud;
    }

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    downsample_filter->setInputCloud(cloud);
    downsample_filter->filter(*filtered);
    filtered->header = cloud->header;

    return filtered;
  }

  /**
   * @brief publish odometry
   * @param stamp  timestamp
   * @param pose   odometry pose to be published
   */
  void publish_odometry(const ros::Time& stamp, const Eigen::Matrix4f& pose) {
    // broadcast the transform over tf
    geometry_msgs::TransformStamped odom_trans = matrix2transform(stamp, pose, "map", "velodyne");
    pose_broadcaster.sendTransform(odom_trans);

    // publish the transform
    nav_msgs::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "map";

    odom.pose.pose.position.x = pose(0, 3);
    odom.pose.pose.position.y = pose(1, 3);
    odom.pose.pose.position.z = pose(2, 3);
    odom.pose.pose.orientation = odom_trans.transform.rotation;

    odom.child_frame_id = "velodyne";
    odom.twist.twist.linear.x = 0.0;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = 0.0;

    pose_pub.publish(odom);
  }

  /**
   * @brief convert a Eigen::Matrix to TransformedStamped
   * @param stamp           timestamp
   * @param pose            pose matrix
   * @param frame_id        frame_id
   * @param child_frame_id  child_frame_id
   * @return transform
   */
  geometry_msgs::TransformStamped matrix2transform(const ros::Time& stamp, const Eigen::Matrix4f& pose, const std::string& frame_id, const std::string& child_frame_id) {
    Eigen::Quaternionf quat(pose.block<3, 3>(0, 0));
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

private:
  // ROS
  ros::NodeHandle nh;
  ros::NodeHandle mt_nh;
  ros::NodeHandle private_nh;

  bool use_imu;
  bool invert_imu;
  ros::Subscriber imu_sub;
  ros::Subscriber points_sub;
  ros::Subscriber globalmap_sub;
  ros::Subscriber jdd_globalmap_sub;
  ros::Subscriber initialpose_sub;

  ros::Publisher pose_pub;
  ros::Publisher aligned_pub;
  ros::Publisher aligned_jdd_pub;
  tf::TransformBroadcaster pose_broadcaster;

  // imu input buffer
  std::mutex imu_data_mutex;
  std::vector<sensor_msgs::ImuConstPtr> imu_data;

  // globalmap and registration method
  pcl::PointCloud<PointT>::Ptr globalmap;
  pcl::Filter<PointT>::Ptr downsample_filter;
  pclomp::NormalDistributionsTransform<PointT, PointT>::Ptr registration;

  // jdd
  pclomp::NormalDistributionsTransform<PointT, PointT>::Ptr registration_jdd;
  pcl::PointCloud<PointT>::Ptr globalmap_jdd;

  // pose estimator
  std::mutex pose_estimator_mutex;
  std::unique_ptr<hdl_localization::PoseEstimator> pose_estimator;
  // processing time buffer
  boost::circular_buffer<double> processing_time;
  Eigen::Matrix4f init_guess;
  Eigen::Matrix4f init_guess_jdd;
  // lidar count
  int lidar_count = 0;

  Eigen::Matrix4f trans_jdd_map;

  // publish cloud map
  ros::Publisher globalmap_pub;
  ros::Publisher globalmap_pub_jdd;
};

}


PLUGINLIB_EXPORT_CLASS(hdl_localization::HdlLocalizationNodelet, nodelet::Nodelet)