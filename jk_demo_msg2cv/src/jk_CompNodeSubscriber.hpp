
#ifndef JK_COMPNODE_SUBSCRIBER_HPP
#define JK_COMPNODE_SUBSCRIBER_HPP

#include <vector>
#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <opencv2/opencv.hpp>

#include "jk_StreamSelector.hpp"

namespace JK {


class Img2CVsubscriber : public rclcpp::Node {

  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> m_subscrVect;
  
  // StreamSelector interface
  std::shared_ptr<StreamSelector> p_streamSelector = nullptr;
  
  cv::CascadeClassifier m_classifier;
  std::vector<cv::Rect> m_detectedFaces;

  void img2cvCallback(const sensor_msgs::msg::Image& msg, int streamIndex);
  
public:
  
  // JK: for composable nodes, make sure the constructor takes a NodeOptions argument
  Img2CVsubscriber(const rclcpp::NodeOptions & options);
  
  void setStreamSelector(std::shared_ptr<StreamSelector> selector);  
  
};

}

#endif

