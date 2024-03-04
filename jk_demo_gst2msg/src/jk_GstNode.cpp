
#include <filesystem>
#include <string>
#include <chrono>

#include <gst/gst.h>
#include <gst/app/gstappsink.h>

#include "jk_GstNode.hpp"


//  The "gst2img_node" node name is for: GStreamer to Img message node.
JK::GstImgPublisher::GstImgPublisher(): Node("gst2img_node") {

  // node parameter: time interval in milliseconds between triggers of the callback
  declare_parameter("milliseconds", -1);
  int millisInterval;
  get_parameter("milliseconds", millisInterval);
  if(millisInterval == -1) {
    millisInterval = 1000;
    RCLCPP_WARN(get_logger(),"Node launch parameter not set, using the interval default value: '%d' milliseconds. ", millisInterval);
  } else
      RCLCPP_INFO(get_logger(),"Using the node launch paramenter to set a '%d' milliseconds publish interval.", millisInterval);
  
  // node parameter: visibility of the Gstreamer default video viewer for testing purposes
  declare_parameter("show_viewer", false);
  bool showViewerWindow {false};
  get_parameter("show_viewer", showViewerWindow);
  if (showViewerWindow)
    RCLCPP_INFO(get_logger(),"Stream viewer set to visible.");
  
  // node parameter: video file name
  std::string defaultFileName {"NO_VIDEO_FILENAME"};
  declare_parameter("video_abs_path_filename", defaultFileName);
  std::string fullFileName {""};
  get_parameter("video_abs_path_filename", fullFileName);
  RCLCPP_INFO_STREAM(get_logger(),"Video file name: '" << fullFileName.c_str() << "'");
  if(fullFileName.compare(defaultFileName) == 0) {
    RCLCPP_FATAL(get_logger(),"No video file has been set up; check the node parameters.");
    return;
  }
  if(!std::filesystem::is_regular_file(fullFileName)) {
    RCLCPP_FATAL(get_logger(),"The provided file name is not of a regular file.");
    return;
  }
  
  
  if (initVideoStream(fullFileName, showViewerWindow)) {
    p_publisher = create_publisher<sensor_msgs::msg::Image>("gst2img_topic", 10);
    auto duration = std::chrono::duration<int, std::milli>(millisInterval);
    p_timer = create_wall_timer(duration, std::bind(&GstImgPublisher::getMessageCallback, this));
  } else {
      RCLCPP_FATAL_STREAM(get_logger(),"Node initialization failed! " << __FILE__ << " #" << __LINE__);
    }
  
}


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

JK::GstImgPublisher::~GstImgPublisher(){
  if(p_pipeline){
    gst_element_set_state(p_pipeline, GST_STATE_NULL);
    gst_object_unref(p_pipeline);
  }
}


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

// returns false if the video initialization fails
bool JK::GstImgPublisher::initVideoStream(const std::string& fileName, const bool showViewerWindow) {

  gst_init(nullptr, nullptr);

  std::string pipelineDefinition;
  if(showViewerWindow)
    pipelineDefinition = "uridecodebin uri=file://" + fileName + " ! videoconvert "
                         "! videoscale ! tee name=t t. ! queue ! appsink "
                         "name=gst2msg_sink max-buffers=1 sync=1 drop=true "
                         "caps=video/x-raw,format=BGR t. ! queue ! autovideosink";
  else
    pipelineDefinition = "uridecodebin uri=file://" + fileName + " ! videoconvert ! "
                         "videoscale ! appsink name=gst2msg_sink max-buffers=1 "
                         "sync=1 drop=true caps=video/x-raw,format=BGR";
                        
  // always initialize to NULL or nullptr
  GError* gstError = nullptr;
  p_pipeline = gst_parse_launch(pipelineDefinition.c_str(), &gstError);

  if(gstError){ 
    RCLCPP_DEBUG_STREAM(get_logger(), gstError->message << "; " << __FILE__ << " #" << __LINE__);
    return false;
  }
  
  p_sink = gst_bin_get_by_name(GST_BIN (p_pipeline), "gst2msg_sink");
  if(!p_sink){ 
    RCLCPP_DEBUG_STREAM(get_logger(), "Gstreamer sink creation failed;" << __FILE__ << " #" << __LINE__);
    return false;
  }

  if ( gst_element_set_state(p_pipeline, GST_STATE_PLAYING) != GST_STATE_CHANGE_SUCCESS){
    RCLCPP_DEBUG_STREAM(get_logger(), "Gstreamer pipeline play failed;" << __FILE__ << " #" << __LINE__);
    // return false;
  }
 
  return true;
   
}


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

void JK::GstImgPublisher::getMessageCallback() {
  
  if (gst_app_sink_is_eos(GST_APP_SINK(p_sink))) {
    // RCLCPP_WARN_ONCE(get_logger(), "Stream ended.");
    RCLCPP_WARN(get_logger(), "Stream ended.");
    return;
  }
  GstSample* sample = gst_app_sink_pull_sample(GST_APP_SINK(p_sink));
  if(!sample){ 
    RCLCPP_DEBUG_STREAM(get_logger(), "Gstreamer sample pull failed;" << __FILE__ << " #" << __LINE__);
    return;
  }
  GstCaps* caps = gst_sample_get_caps(sample);
  if(!caps){ 
    RCLCPP_DEBUG_STREAM(get_logger(), "Gstreamer sample caps extraction failed;" << __FILE__ << " #" << __LINE__);
    return;
  }
  GstStructure* s = gst_caps_get_structure(caps, 0);
  int sampleWidth, sampleHeight;
  gst_structure_get_int(s, "width", &sampleWidth);
  gst_structure_get_int(s, "height", &sampleHeight);
  GstBuffer *buffer = gst_sample_get_buffer(sample);
  GstMapInfo map;
  gst_buffer_map(buffer, &map, GST_MAP_READ);
       
  auto imgMsg { sensor_msgs::msg::Image() };
  imgMsg.width = sampleWidth;
  imgMsg.height = sampleHeight;
  imgMsg.data.resize(map.size);
  imgMsg.step = sampleWidth * 3;
  
  std::copy(map.data, map.data + map.size, imgMsg.data.begin());
  RCLCPP_INFO_STREAM_ONCE(get_logger(), "Publishing " << sampleWidth << "x" << sampleHeight << "frames...");
  p_publisher->publish(imgMsg);
  
  gst_buffer_unmap(buffer, &map);
  gst_sample_unref(sample);
        
}

