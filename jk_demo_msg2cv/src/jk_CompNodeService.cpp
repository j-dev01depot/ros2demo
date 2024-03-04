
#include "jk_CompNodeService.hpp"

using std::placeholders::_1;
using std::placeholders::_2;


JK::Img2CVservice::Img2CVservice(const rclcpp::NodeOptions & options): Node("jk_img2cv_srv", options) {
  
  p_streamSelector = std::make_shared<StreamSelectorImpl>();
  
  p_service = create_service<std_srvs::srv::Trigger>("switch_videostream", std::bind(&JK::Img2CVservice::switchToNextVideoStream, this, _1, _2));
  
  RCLCPP_DEBUG(get_logger(), "Service created.");
  
}


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

std::shared_ptr<JK::StreamSelector> JK::Img2CVservice::getStreamSelector(){
  return p_streamSelector;
}
  
  
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

void JK::Img2CVservice::switchToNextVideoStream(const std_srvs::srv::Trigger::Request::SharedPtr request, 
                                                const  std_srvs::srv::Trigger::Response::SharedPtr response){
    RCLCPP_INFO(get_logger(), "Stream switch requested.");
                              
    if(p_streamSelector){
      p_streamSelector->switchToNextStream();
    } else {
      RCLCPP_ERROR(get_logger(), "StreamSelectorImpl is null.");
      response->success = 0;
      response->message = "Video stream switch failed.";
    }
    
}


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

/*
 * ###   JK: for composable nodes, add the following rows   ###
 */ 
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(JK::Img2CVservice)
/* ### */

