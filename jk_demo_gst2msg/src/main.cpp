
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "jk_GstNode.hpp"

int main(int argc, char* argv[]) {

  // Force flush of the stdout buffer
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  RCLCPP_INFO(rclcpp::get_logger("jk_gstr2img_MAIN"), "Starting the publisher node process...");
  
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<JK::GstImgPublisher>());

  rclcpp::shutdown();
  
  return 0;
  
}
