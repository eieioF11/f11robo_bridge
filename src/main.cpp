#include <rclcpp/rclcpp.hpp>
#include "f11robo_bridge/f11robo_bridge_component.hpp"

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<F11RoboBridge>());
  rclcpp::shutdown();
  return 0;
}