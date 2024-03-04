
#ifndef JK_COMPNODE_SERVICE_HPP
#define JK_COMPNODE_SERVICE_HPP

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "jk_StreamSelectorImpl.hpp"

namespace JK {


class Img2CVservice : public rclcpp::Node {

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr p_service {nullptr};

  std::shared_ptr<StreamSelectorImpl> p_streamSelector {nullptr};
  
  void switchToNextVideoStream(const std_srvs::srv::Trigger::Request::SharedPtr request, 
                               const std_srvs::srv::Trigger::Response::SharedPtr response);

public:
  
  // JK: for composable nodes, make sure the constructor takes a NodeOptions argument
  Img2CVservice(const rclcpp::NodeOptions & options);

  std::shared_ptr<JK::StreamSelector> getStreamSelector();
  
};

}

#endif

