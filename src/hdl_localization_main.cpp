#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <hdl_localization/hdl_localization_component.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hdl_localization::HdlLocalization>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}