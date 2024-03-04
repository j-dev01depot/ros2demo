
#ifndef JK_GSTNODE_HPP
#define JK_GSTNODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <gst/gst.h>
#include <gst/app/gstappsink.h>

namespace JK {


class GstImgPublisher : public rclcpp::Node {
  
  GstElement* p_pipeline {nullptr};
  GstElement* p_sink {nullptr};
    
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr p_publisher {nullptr};
  rclcpp::TimerBase::SharedPtr p_timer {nullptr};
  
  bool initVideoStream(const std::string& videoFileName, const bool showViewerWindow);
  
public:

  GstImgPublisher();
  GstImgPublisher(const GstImgPublisher&) = delete;
  GstImgPublisher& operator=(const GstImgPublisher&) = delete;
  GstImgPublisher(GstImgPublisher&&) = delete;
  GstImgPublisher& operator=(GstImgPublisher&&) = delete;
  ~GstImgPublisher();
  
  void getMessageCallback();

};

} 

#endif
