cmake_minimum_required(VERSION 3.8)
project(jk_demo_gst2msg)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


# --- JK: added to use GStreamer and OpenCV
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-audio-1.0 gstreamer-video-1.0  gstreamer-app-1.0)
message("GST_INCLUDE_DIRS = ${GST_INCLUDE_DIRS}")
message("GST_LIBRARY_DIRS = ${GST_LIBRARY_DIRS}")
message("GST_LIBRARIES = ${GST_LIBRARIES}")

link_directories( ${GST_LIBRARY_DIRS}  )
include_directories( ${GST_INCLUDE_DIRS}  )

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
# ---


# "JK Test Gstreamer to Msg Executable"
add_executable(jk_demo_gst2m_exec 
                src/main.cpp
                src/jk_GstNode.cpp)
target_link_libraries(jk_demo_gst2m_exec ${GST_LIBRARIES})   # JK: added to use GStreamer
target_include_directories(jk_demo_gst2m_exec PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(jk_demo_gst2m_exec PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  jk_demo_gst2m_exec
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
)


install(TARGETS jk_demo_gst2m_exec
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()

