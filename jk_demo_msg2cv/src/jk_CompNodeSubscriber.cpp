
#include <filesystem>

#include "jk_CompNodeSubscriber.hpp"

using std::placeholders::_1;



JK::Img2CVsubscriber::Img2CVsubscriber(const rclcpp::NodeOptions & options) : Node("jk_img2cv_subscr", options) {

  // node parameter: visibility of the OpenCV default video viewer 
  declare_parameter("show_viewer", false);
  bool showOpenCVviewer {false};
  get_parameter("show_viewer", showOpenCVviewer);
  if (showOpenCVviewer)
    RCLCPP_INFO(get_logger(),"Stream viewer set to visible.");
  
  // node parameter: topics list to subscribe
  declare_parameter("topic_list", std::vector<std::string>{});
  std::vector<std::string> topicNamesVect;
  get_parameter("topic_list", topicNamesVect);
  if(topicNamesVect.empty()){
    RCLCPP_FATAL(get_logger(), "The topics list parameter is empty.");
    return;
  } else
      for (const std::string& topicName: topicNamesVect)
        RCLCPP_INFO_STREAM(get_logger(), "Topic name to subscribe: " << topicName.c_str());

  // node parameter: the pre-trained face detector file name
  std::string defaultFileName {"NO_FILENAME"};
  declare_parameter("haarcascades_abs_path_filename", defaultFileName);
  std::string haarFileName {""};
  get_parameter("haarcascades_abs_path_filename", haarFileName);
  if(haarFileName.compare(defaultFileName) == 0) {
    RCLCPP_FATAL(get_logger(),"No pre-trained face detector file has been set up; check the startup node parameters.");
    return;
  }
  if(!std::filesystem::is_regular_file(haarFileName)) {
    RCLCPP_FATAL(get_logger(),"The provided file name is not of a regular file.");
    return;
  }
  
  
  for(size_t i {0}; i < topicNamesVect.size(); ++i) {
    // to use a callback with more than two parameters, initialize a std::function 
    // with the std::bind, then pass the std::function to the create_subscription
    std::function<void(const sensor_msgs::msg::Image&)> fn = std::bind(&JK::Img2CVsubscriber::img2cvCallback, this, _1, i);
    m_subscrVect.push_back(create_subscription<sensor_msgs::msg::Image>(topicNamesVect[i].c_str(), 10, fn));
  }
  
  m_classifier.load(haarFileName.c_str());
  m_detectedFaces.reserve(20);

  RCLCPP_DEBUG(get_logger(), "Subscriber created.");
  
};


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

void JK::Img2CVsubscriber::img2cvCallback(const sensor_msgs::msg::Image& msg, int streamIndex) {
  
  if (streamIndex != p_streamSelector->getSelectedStream())
      return;
  
  RCLCPP_INFO_STREAM_ONCE(get_logger(), "Generating image from stream: " << streamIndex);
  RCLCPP_DEBUG_STREAM(get_logger(), "Generating image from stream: " << streamIndex);
  
  // TODO: check error on cv::Mat creation
  cv::Mat frame(msg.height, msg.width, CV_8UC3, (void *) msg.data.data());
  
  m_detectedFaces.erase(m_detectedFaces.begin(), m_detectedFaces.end());
  m_classifier.detectMultiScale(frame, m_detectedFaces, 1.3, 5);
  for (const cv::Rect& rct: m_detectedFaces)
    cv::rectangle(frame, rct.tl(), rct.br(), cv::Scalar(50, 50, 255), 2);
  
  cv::imshow("frame", frame);
  cv::pollKey();

}  


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

void JK::Img2CVsubscriber::setStreamSelector(std::shared_ptr<StreamSelector> selector){
  
  if(selector){
    p_streamSelector = selector;
    p_streamSelector->setStreamsTotalNumber(m_subscrVect.size());
    RCLCPP_INFO(get_logger(),"Stream selector initialized.");
  }
  
}


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

/*
 * ###   JK: for composable nodes, add the following rows   ###
 */ 
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(JK::Img2CVsubscriber)
/* ### */

