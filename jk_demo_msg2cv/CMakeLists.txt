cmake_minimum_required(VERSION 3.8)
project(jk_demo_msg2cv)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)   # added for the nodes dealing with the image messages
find_package(std_srvs REQUIRED)   # added for the service node
find_package(rclcpp_components REQUIRED)   # added for the composable nodes


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


# --- JK: composable nodes
# jk subscriber node
add_library(jk_compnode_subscriber src/jk_CompNodeSubscriber.cpp)   
target_link_libraries(jk_compnode_subscriber ${OpenCV_LIBS})
target_include_directories(jk_compnode_subscriber PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(jk_compnode_subscriber PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  jk_compnode_subscriber
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "std_srvs"
  "rclcpp_components"   # JK: added for composable nodes
)
rclcpp_components_register_node(
    jk_compnode_subscriber
    PLUGIN "JK::Img2CVsubscriber"
    EXECUTABLE jk_compnode_subscriber_EXEC
)
# ---


# --- JK: composable nodes
# jk service node
add_library(jk_compnode_service src/jk_CompNodeService.cpp)
target_link_libraries(jk_compnode_service ${OpenCV_LIBS})
target_include_directories(jk_compnode_service PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(jk_compnode_service PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  jk_compnode_service
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "std_srvs"
  "rclcpp_components"   # JK: added for composable nodes
)
rclcpp_components_register_node(
    jk_compnode_service
    PLUGIN "JK::Img2CVservice"
    EXECUTABLE jk_compnode_service_EXEC
)
# ---
 
 
add_executable(jk_demo_m2cv_faces_exec
                src/jk_StreamSelectorImpl.cpp
                src/jk_CompNodeSubscriber.cpp
                src/jk_CompNodeService.cpp
                src/main.cpp)
target_link_libraries(jk_demo_m2cv_faces_exec ${OpenCV_LIBS})
target_include_directories(jk_demo_m2cv_faces_exec PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(jk_demo_m2cv_faces_exec PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  jk_demo_m2cv_faces_exec
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "std_srvs"
  "rclcpp_components"
)
 

# --- JK: composable nodes 
ament_export_targets(export_jk_compnode_subscriber)
install(TARGETS jk_compnode_subscriber
        EXPORT export_jk_compnode_subscriber
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
# ---
# --- JK: composable nodes 
ament_export_targets(export_jk_compnode_service)
install(TARGETS jk_compnode_service
        EXPORT export_jk_compnode_service
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
# ---


install(TARGETS jk_demo_m2cv_faces_exec
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


# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()

