#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <hdl_localization/globalmap_server_component.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hdl_localization::GlobalmapServer>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}