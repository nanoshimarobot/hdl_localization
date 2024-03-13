#pragma once

#include <memory>
#include <iostream>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

namespace hdl_localization {
class GlobalmapServer : public rclcpp::Node {
  using PointT = pcl::PointXYZI;

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr globalmap_pub;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr map_update_sub;

  rclcpp::TimerBase::SharedPtr globalmap_pub_timer;
  pcl::PointCloud<PointT>::Ptr globalmap;
  sensor_msgs::msg::PointCloud2 globalmap_msg;

public:
  GlobalmapServer(const rclcpp::NodeOptions& options) : GlobalmapServer("", options) {}
  GlobalmapServer(const std::string& name_space = "", const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("globalmap_server_node", name_space, options) {
    using namespace std::chrono_literals;
    initialize_params();

    globalmap_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/globalmap", rclcpp::QoS(1));
    map_update_sub =
      this->create_subscription<std_msgs::msg::String>("/map_request/pcd", rclcpp::QoS(10), std::bind(&GlobalmapServer::map_update_callback, this, std::placeholders::_1));

    globalmap_pub_timer = this->create_wall_timer(1s, [this]() { this->globalmap_pub->publish(this->globalmap_msg); });
  }

  void initialize_params() {
    // read globalmap from a pcd file
    std::string globalmap_pcd = this->declare_parameter<std::string>("globalmap_pcd", "");
    globalmap.reset(new pcl::PointCloud<PointT>());
    pcl::io::loadPCDFile(globalmap_pcd, *globalmap);
    globalmap->header.frame_id = "map";

    std::ifstream utm_file(globalmap_pcd + ".utm");
    if (utm_file.is_open() && this->declare_parameter<bool>("convert_utm_to_local", true)) {
      double utm_easting;
      double utm_northing;
      double altitude;
      utm_file >> utm_easting >> utm_northing >> altitude;
      for (auto& pt : globalmap->points) {
        pt.getVector3fMap() -= Eigen::Vector3f(utm_easting, utm_northing, altitude);
      }
      RCLCPP_INFO_STREAM(
        this->get_logger(),
        "Global map offset by UTM reference coordinates (x = " << utm_easting << ", y = " << utm_northing << ") and altitude (z = " << altitude << ")");
    }

    // downsample globalmap
    double downsample_resolution = this->declare_parameter<double>("downsample_resolution", 0.1);
    std::shared_ptr<pcl::VoxelGrid<PointT>> voxelgrid(new pcl::VoxelGrid<PointT>());
    voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
    voxelgrid->setInputCloud(globalmap);

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    voxelgrid->filter(*filtered);

    globalmap = filtered;
    pcl::toROSMsg(*globalmap, globalmap_msg);
  }

  void map_update_callback(const std_msgs::msg::String& msg) {
    RCLCPP_INFO_STREAM(this->get_logger(), "Received map request, map path : " << msg.data);
    std::string globalmap_pcd = msg.data;
    globalmap.reset(new pcl::PointCloud<PointT>());
    pcl::io::loadPCDFile(globalmap_pcd, *globalmap);
    globalmap->header.frame_id = "map";

    // downsample globalmap
    double downsample_resolution = this->declare_parameter<double>("downsample_resolution", 0.1);
    boost::shared_ptr<pcl::VoxelGrid<PointT>> voxelgrid(new pcl::VoxelGrid<PointT>());
    voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
    voxelgrid->setInputCloud(globalmap);

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    voxelgrid->filter(*filtered);

    globalmap = filtered;

    pcl::toROSMsg(*globalmap, globalmap_msg);
    globalmap_pub->publish(globalmap_msg);
  }
};
}  // namespace hdl_localization