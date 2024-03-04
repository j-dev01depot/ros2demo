
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "jk_CompNodeSubscriber.hpp"
#include "jk_CompNodeService.hpp"



int main(int argc, char* argv[]) {

  // Force flush of the stdout buffer
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  RCLCPP_INFO(rclcpp::get_logger("jk_demo_msg2cv_MAIN"), "Starting the composable nodes process...");

  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  rclcpp::executors::SingleThreadedExecutor exec;

  auto subscr = std::make_shared<JK::Img2CVsubscriber>(options);
  auto srv = std::make_shared<JK::Img2CVservice>(options);
  
  // use the StreamSelector interface to decouple the nodes 
  subscr->setStreamSelector(srv->getStreamSelector());
    
  exec.add_node(subscr);
  exec.add_node(srv);

  exec.spin();
  
  rclcpp::shutdown();
  
  return 0;
  
}
