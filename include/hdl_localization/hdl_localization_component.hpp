#pragma once

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <pcl_conversions/pcl_conversions.h>

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <std_srvs/srv/empty.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pclomp/ndt_omp.h>

#include "pose_estimator.hpp"
#include "delta_estimater.hpp"

#include <hdl_localization_msgs/msg/scan_matching_status.hpp>
#include <hdl_global_localization_msgs/srv/set_global_map.hpp>
#include <hdl_global_localization_msgs/srv/query_global_localization.hpp>

namespace hdl_localization {
class HdlLocalization : public rclcpp::Node {
  using PointT = pcl::PointXYZI;

private:
  std::string robot_odom_frame_id;
  std::string odom_child_frame_id;

  bool use_imu;
  bool invert_acc;
  bool invert_gyro;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr globalmap_sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_sub;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pose_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_pub;
  rclcpp::Publisher<hdl_localization_msgs::msg::ScanMatchingStatus>::SharedPtr status_pub;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  std::vector<sensor_msgs::msg::Imu::ConstPtr> imu_data;

  std::unique_ptr<hdl_localization::PoseEstimator> pose_estimator;

  // globalmap and registration method
  pcl::PointCloud<PointT>::Ptr globalmap;
  pcl::Filter<PointT>::Ptr downsample_filter;
  pcl::Registration<PointT, PointT>::Ptr registration;

  // global localization
  bool use_global_localization;
  bool relocalizing;
  std::unique_ptr<DeltaEstimater> delta_estimater;

  pcl::PointCloud<PointT>::ConstPtr last_scan;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr relocalize_service_;
  rclcpp::Client<hdl_global_localization_msgs::srv::SetGlobalMap>::SharedPtr set_global_map_client_;
  rclcpp::Client<hdl_global_localization_msgs::srv::QueryGlobalLocalization>::SharedPtr query_global_localization_client_;

public:
  HdlLocalization(const rclcpp::NodeOptions& options) : HdlLocalization("", options) {}
  HdlLocalization(const std::string& name_space = "", const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
  : Node("hdl_localization_node", name_space, options),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_),
    tf_broadcaster_(this) {
    using namespace std::chrono_literals;
    initialize_params();

    robot_odom_frame_id = this->declare_parameter<std::string>("robot_odom_frame_id", "robot_odom");
    odom_child_frame_id = this->declare_parameter<std::string>("odom_child_frame_id", "base_link");

    use_imu = this->declare_parameter<bool>("use_imu", true);
    invert_acc = this->declare_parameter<bool>("invert_acc", false);
    invert_gyro = this->declare_parameter<bool>("invert_gyro", false);

    if (use_imu) {
      RCLCPP_INFO(this->get_logger(), "enable imu-based prediction");
      imu_sub =
        this->create_subscription<sensor_msgs::msg::Imu>("/gpsimu_driver/imu_data", rclcpp::QoS(10), std::bind(&HdlLocalization::imu_callback, this, std::placeholders::_1));
    }

    points_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/velodyne_points",
      rclcpp::QoS(1).keep_last(1),
      std::bind(&HdlLocalization::points_callback, this, std::placeholders::_1));
    globalmap_sub =
      this->create_subscription<sensor_msgs::msg::PointCloud2>("/globalmap", rclcpp::QoS(1), std::bind(&HdlLocalization::globalmap_callback, this, std::placeholders::_1));
    initialpose_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose",
      rclcpp::QoS(10),
      std::bind(&HdlLocalization::initialpose_callback, this, std::placeholders::_1));

    pose_pub = this->create_publisher<nav_msgs::msg::Odometry>("/odom", rclcpp::QoS(1));
    aligned_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/aligned_points", rclcpp::QoS(10));
    status_pub = this->create_publisher<hdl_localization_msgs::msg::ScanMatchingStatus>("/status", rclcpp::QoS(1));

    use_global_localization = this->declare_parameter<bool>("use_global_localization", true);
    if (use_global_localization) {
      RCLCPP_INFO(this->get_logger(), "wait for global localization services");
      relocalize_service_ = this->create_service<std_srvs::srv::Empty>("/relocalize", std::bind(&HdlLocalization::relocalize, this, std::placeholders::_1));
      set_global_map_client_ = this->create_client<hdl_global_localization_msgs::srv::SetGlobalMap>("/hdl_global_localization/set_global_map");
      query_global_localization_client_ = this->create_client<hdl_global_localization_msgs::srv::QueryGlobalLocalization>("/hdl_global_localization/query");
      while (!set_global_map_client_->wait_for_service(1s))
        ;
      while (!query_global_localization_client_->wait_for_service(1s))
        ;
    }
  }

  void imu_callback(const sensor_msgs::msg::Imu::ConstPtr& imu_msg) { imu_data.push_back(imu_msg); }

  void points_callback(const sensor_msgs::msg::PointCloud2::ConstPtr& points_msg) {
    if (!globalmap) {
      RCLCPP_ERROR(this->get_logger(), "globalmap has not been received!!");
      return;
    }

    const auto& stamp = points_msg->header.stamp;
    pcl::PointCloud<PointT>::Ptr pcl_cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*points_msg, *pcl_cloud);

    if (pcl_cloud->empty()) {
      RCLCPP_ERROR(this->get_logger(), "cloud is empty!!");
      return;
    }

    // transform pointcloud into odom_child_frame_id
    pcl::PointCloud<PointT>::Ptr cloud = std::make_shared<pcl::PointCloud<PointT>>();
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform(odom_child_frame_id, pcl_cloud->header.frame_id, stamp, rclcpp::Duration::from_seconds(0.1));
    } catch (tf2::LookupException& e) {
      RCLCPP_WARN(this->get_logger(), "%s", e.what());
      return;
    } catch (tf2::ExtrapolationException& e) {
      RCLCPP_WARN(this->get_logger(), "%s", e.what());
      return;
    }

    Eigen::Affine3d affine_conversion(tf2::transformToEigen(transform).affine().cast<float>());
    pcl::transformPointCloud(*pcl_cloud, *cloud, affine_conversion);

    auto filtered = downsample(cloud);
    last_scan = filtered;

    if (relocalizing) {
      delta_estimater->add_frame(filtered);
    }

    if (!pose_estimator) {
      RCLCPP_ERROR(this->get_logger(), "waiting for initial pose input!!");
      return;
    }
    Eigen::Matrix4f before = pose_estimator->matrix();

    // predict
    if (!use_imu) {
      pose_estimator->predict(stamp);
    } else {
      auto imu_iter = imu_data.begin();
      for (imu_iter; imu_iter != imu_data.end(); imu_iter++) {
        if (stamp.nanosec < (*imu_iter)->header.stamp.nanosec) {
          break;
        }

        const auto& acc = (*imu_iter)->linear_acceleration;
        const auto& gyro = (*imu_iter)->angular_velocity;
        double acc_sign = invert_acc ? -1.0 : 1.0;
        double gyro_sign = invert_gyro ? -1.0 : 1.0;
        pose_estimator->predict((*imu_iter)->header.stamp, acc_sign * Eigen::Vector3f(acc.x, acc.y, acc.z), gyro_sign * Eigen::Vector3f(gyro.x, gyro.y, gyro.z));
      }
      imu_data.erase(imu_data.begin(), imu_iter);
    }

    // odometry-based prediction
    rclcpp::Time last_correction_time = pose_estimator->last_correction_time();
    if (this->declare_parameter<bool>("enable_robot_odometry_prediction", false) && !(last_correction_time.seconds() == 0.0)) {
      geometry_msgs::msg::TransformStamped odom_delta;

      try {
        odom_delta =
          tf_buffer_.lookupTransform(odom_child_frame_id, last_correction_time, odom_child_frame_id, rclcpp::Time(stamp), robot_odom_frame_id, rclcpp::Duration::from_seconds(0.1));
      } catch (tf2::LookupException& e) {
        try {
          odom_delta =
            tf_buffer_.lookupTransform(odom_child_frame_id, last_correction_time, odom_child_frame_id, rclcpp::Time(0), robot_odom_frame_id, rclcpp::Duration::from_seconds(0.0));
        } catch (tf2::LookupException& e) {
          RCLCPP_WARN(this->get_logger(), "%s", e.what());
        }
      }

      if (odom_delta.header.stamp.nanosec == 0) {
        RCLCPP_WARN_STREAM(this->get_logger(), "failed to look up transform between " << cloud->header.frame_id << " and " << robot_odom_frame_id);
      } else {
        Eigen::Isometry3d delta = tf2::transformToEigen(odom_delta);
        pose_estimator->predict_odom(delta.cast<float>().matrix());
      }
    }

    // correct
    auto aligned = pose_estimator->correct(stamp, filtered);

    if (aligned_pub->get_subscription_count() > 0) {
      sensor_msgs::msg::PointCloud2 aligned_msg;
      pcl::toROSMsg(*aligned, aligned_msg);
      aligned_msg.header.frame_id = "map";
      aligned_msg.header.stamp = stamp;
      aligned_pub->publish(aligned_msg);
    }

    publish_odometry(points_msg->header.stamp, pose_estimator->matrix());
  }

  void globalmap_callback(const sensor_msgs::msg::PointCloud2::ConstPtr& point_msg) {
    RCLCPP_INFO(this->get_logger(), "globalmap received");
    pcl::PointCloud<PointT>::Ptr cloud = std::make_shared<pcl::PointCloud<PointT>>();
    pcl::fromROSMsg(*point_msg, *cloud);
    globalmap = cloud;

    registration->setInputTarget(globalmap);

    if (use_global_localization) {
      RCLCPP_INFO(this->get_logger(), "set globalmap for global localization!");
      auto req = std::make_shared<hdl_global_localization_msgs::srv::SetGlobalMap::Request>();
      pcl::toROSMsg(*globalmap, req->global_map);

      set_global_map_client_->async_send_request(req, [this](rclcpp::Client<hdl_global_localization_msgs::srv::SetGlobalMap>::SharedFuture future) {
        RCLCPP_INFO(this->get_logger(), "global map set");
      });
    }
  }

  void initialpose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstPtr& pose_msg) {
    RCLCPP_INFO(this->get_logger(), "initial pose reveived");
    const auto& p = pose_msg->pose.pose.position;
    const auto& q = pose_msg->pose.pose.orientation;
    pose_estimator.reset(new hdl_localization::PoseEstimator(
      registration,
      Eigen::Vector3f(p.x, p.y, p.z),
      Eigen::Quaternionf(q.w, q.x, q.y, q.z),
      this->declare_parameter<double>("cool_time_duration", 0.5)));
  }

  bool relocalize(std_srvs::srv::Empty::Request& req, std_srvs::srv::Empty::Response& res) {
    if (last_scan == nullptr) {
      RCLCPP_INFO_STREAM(this->get_logger(), "no scan has been received");
      return false;
    }

    relocalizing = true;
    delta_estimater->reset();
    pcl::PointCloud<PointT>::ConstPtr scan = last_scan;

    // hdl_global_localization_msgs::srv::QueryGlobalLocalization srv;
    auto query_req = std::make_shared<hdl_global_localization_msgs::srv::QueryGlobalLocalization::Request>();
    pcl::toROSMsg(*scan, query_req->cloud);
    query_req->max_num_candidates = 1;

    query_global_localization_client_->async_send_request(query_req, [this](rclcpp::Client<hdl_global_localization_msgs::srv::QueryGlobalLocalization>::SharedFuture future) {
      const auto& result = future.get()->poses[0];
      RCLCPP_INFO_STREAM(this->get_logger(), "--- Global localization result ---");
      RCLCPP_INFO_STREAM(this->get_logger(), "Trans :" << result.position.x << " " << result.position.y << " " << result.position.z);
      RCLCPP_INFO_STREAM(this->get_logger(), "Quat  :" << result.orientation.x << " " << result.orientation.y << " " << result.orientation.z << " " << result.orientation.w);
      RCLCPP_INFO_STREAM(this->get_logger(), "Error :" << future.get()->errors[0]);
      RCLCPP_INFO_STREAM(this->get_logger(), "Inlier:" << future.get()->inlier_fractions[0]);

      Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
      pose.linear() = Eigen::Quaternionf(result.orientation.w, result.orientation.x, result.orientation.y, result.orientation.z).toRotationMatrix();
      pose.translation() = Eigen::Vector3f(result.position.x, result.position.y, result.position.z);
      pose = pose * delta_estimater->estimated_delta();

      this->pose_estimator.reset(
        new hdl_localization::PoseEstimator(registration, pose.translation(), Eigen::Quaternionf(pose.linear()), this->declare_parameter<double>("cool_time_duration", 0.5)));

      relocalizing = false;
    });

    return true;
  }

  void publish_odometry(const rclcpp::Time& stamp, const Eigen::Matrix4f& pose) {
    // broadcast the transform over tf
    geometry_msgs::msg::TransformStamped frame_wrt_odom;
    bool tf_error_ = false;

    try {
      frame_wrt_odom = tf_buffer_.lookupTransform(robot_odom_frame_id, odom_child_frame_id, rclcpp::Time(0), rclcpp::Duration::from_seconds(0.1));
    } catch (tf2::LookupException& e) {
      RCLCPP_ERROR(this->get_logger(), "%s", e.what());
      geometry_msgs::msg::TransformStamped odom_trans = tf2::eigenToTransform(Eigen::Isometry3d(pose.cast<double>()));
      odom_trans.header.stamp = stamp;
      odom_trans.header.frame_id = "map";
      odom_trans.child_frame_id = odom_child_frame_id;
      tf_broadcaster_.sendTransform(odom_trans);
      tf_error_ = true;
    }

    if (!tf_error_) {
      geometry_msgs::msg::TransformStamped map_wrt_frame = tf2::eigenToTransform(Eigen::Isometry3d(pose.inverse().cast<double>()));
      map_wrt_frame.header.stamp = stamp;
      map_wrt_frame.header.frame_id = odom_child_frame_id;
      map_wrt_frame.child_frame_id = "map";

      Eigen::Matrix4f frame2odom = tf2::transformToEigen(frame_wrt_odom).cast<float>().matrix();

      geometry_msgs::msg::TransformStamped map_wrt_odom;
      tf2::doTransform(map_wrt_frame, map_wrt_odom, frame_wrt_odom);

      tf2::Transform odom_wrt_map;
      tf2::fromMsg(map_wrt_odom.transform, odom_wrt_map);
      odom_wrt_map = odom_wrt_map.inverse();

      geometry_msgs::msg::TransformStamped odom_trans;
      odom_trans.transform = tf2::toMsg(odom_wrt_map);
      odom_trans.header.stamp = stamp;
      odom_trans.header.frame_id = "map";
      odom_trans.child_frame_id = robot_odom_frame_id;

      tf_broadcaster_.sendTransform(odom_trans);
    }

    // publish the transform
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "map";

    odom.pose.pose = tf2::toMsg(Eigen::Isometry3d(pose.cast<double>()));
    odom.child_frame_id = odom_child_frame_id;
    odom.twist.twist.linear.x = 0.0;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = 0.0;

    pose_pub->publish(odom);
  }

  pcl::PointCloud<PointT>::ConstPtr downsample(const pcl::PointCloud<PointT>::ConstPtr& cloud) const {
    if (!downsample_filter) {
      return cloud;
    }

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    downsample_filter->setInputCloud(cloud);
    downsample_filter->filter(*filtered);
    filtered->header = cloud->header;

    return filtered;
  }

  pcl::Registration<PointT, PointT>::Ptr create_registration() {
    std::string reg_method = this->declare_parameter<std::string>("reg_method", "NDT_OMP");
    std::string ndt_neighbor_search_method = this->declare_parameter<std::string>("ndt_neighbor_search_method", "DIRECT7");
    double ndt_neighbor_search_radius = this->declare_parameter<double>("ndt_neighbor_search_radius", 2.0);
    double ndt_resolution = this->declare_parameter<double>("ndt_resolution", 1.0);

    if (reg_method == "NDT_OMP") {
      RCLCPP_INFO(this->get_logger(), "NDT_OMP is selected");
      pclomp::NormalDistributionsTransform<PointT, PointT>::Ptr ndt(new pclomp::NormalDistributionsTransform<PointT, PointT>());
      ndt->setTransformationEpsilon(0.01);
      ndt->setResolution(ndt_resolution);
      if (ndt_neighbor_search_method == "DIRECT1") {
        RCLCPP_INFO(this->get_logger(), "search_method DIRECT1 is selected");
        ndt->setNeighborhoodSearchMethod(pclomp::DIRECT1);
      } else if (ndt_neighbor_search_method == "DIRECT7") {
        RCLCPP_INFO(this->get_logger(), "search_method DIRECT7 is selected");
        ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
      } else {
        if (ndt_neighbor_search_method == "KDTREE") {
          RCLCPP_INFO(this->get_logger(), "search_method KDTREE is selected");
        } else {
          RCLCPP_WARN(this->get_logger(), "invalid search method was given");
          RCLCPP_WARN(this->get_logger(), "default method is selected (KDTREE)");
        }
        ndt->setNeighborhoodSearchMethod(pclomp::KDTREE);
      }
      return ndt;
    }

    RCLCPP_ERROR_STREAM(this->get_logger(), "unknown registration method:" << reg_method);
    return nullptr;
  }

  void initialize_params() {
    // intialize scan matching method
    double downsample_resolution = this->declare_parameter<double>("downsample_resolution", 0.1);
    std::shared_ptr<pcl::VoxelGrid<PointT>> voxelgrid(new pcl::VoxelGrid<PointT>());
    voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
    downsample_filter = voxelgrid;

    RCLCPP_INFO(this->get_logger(), "create registration method for localization");
    registration = create_registration();

    // global localization
    RCLCPP_INFO(this->get_logger(), "create registration method for fallback during relocalization");
    relocalizing = false;
    delta_estimater.reset(new DeltaEstimater(create_registration()));

    // initialize pose estimator
    if (this->declare_parameter<bool>("specify_init_pose", true)) {
      RCLCPP_INFO(this->get_logger(), "initialize pose estimator with specified parameters!!");
      pose_estimator.reset(new hdl_localization::PoseEstimator(
        registration,
        Eigen::Vector3f(this->declare_parameter<double>("init_pos_x", 0.0), this->declare_parameter<double>("init_pos_y", 0.0), this->declare_parameter<double>("init_pos_z", 0.0)),
        Eigen::Quaternionf(
          this->declare_parameter<double>("init_ori_w", 1.0),
          this->declare_parameter<double>("init_ori_x", 0.0),
          this->declare_parameter<double>("init_ori_y", 0.0),
          this->declare_parameter<double>("init_ori_z", 0.0)),
        this->declare_parameter<double>("cool_time_duration", 0.5)));
    }
  }
};
}  // namespace hdl_localization