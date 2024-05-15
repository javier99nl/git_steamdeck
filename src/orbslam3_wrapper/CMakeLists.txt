cmake_minimum_required(VERSION 3.5)
project(ros2_orbslam3)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules/")

set(CMAKE_CXX_STANDARD 14)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
find_package(PCL 1.10 REQUIRED COMPONENTS common io)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(Pangolin REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(std_srvs REQUIRED)

set(ORB_SLAM3_DIR $ENV{ORB_SLAM3_DIR})

include_directories(
    ${ORB_SLAM3_DIR}
    ${ORB_SLAM3_DIR}/include
    ${ORB_SLAM3_DIR}/include/CameraModels
    ${ORB_SLAM3_DIR}/Thirdparty/Sophus)

set(LIBS 
    ${ORB_SLAM3_DIR}/lib/libORB_SLAM3.so
    ${ORB_SLAM3_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
    ${ORB_SLAM3_DIR}/Thirdparty/g2o/lib/libg2o.so
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
    ${Pangolin_LIBRARIES}
    ${image_transport_LIBRARIES}
    ${tf2_ros_LIBRARIES}
    ${tf2_LIBRARIES}
    -lboost_serialization
    -lcrypto)

add_executable(mono src/orbslam3_wrapper/src/mono.cpp src/orbslam3_wrapper/src/common.cpp)
target_link_libraries(mono ${LIBS})
ament_target_dependencies(mono rclcpp sensor_msgs cv_bridge message_filters tf2_ros tf2 geometry_msgs tf2_geometry_msgs image_transport std_srvs pcl_conversions Pangolin std_srvs)
install(TARGETS mono
  DESTINATION lib/${PROJECT_NAME})

add_executable(rgbd src/orbslam3_wrapper/src/rgbd.cpp src/orbslam3_wrapper/src/common.cpp)
target_link_libraries(rgbd ${LIBS})
ament_target_dependencies(rgbd rclcpp sensor_msgs cv_bridge message_filters tf2_ros tf2 geometry_msgs tf2_geometry_msgs image_transport std_srvs Pangolin)
install(TARGETS rgbd
  DESTINATION lib/${PROJECT_NAME})

add_executable(stereo src/orbslam3_wrapper/src/stereo.cpp src/orbslam3_wrapper/src/common.cpp)
target_link_libraries(stereo ${LIBS})
ament_target_dependencies(stereo rclcpp sensor_msgs cv_bridge message_filters tf2_ros tf2 geometry_msgs tf2_geometry_msgs image_transport std_srvs Pangolin)
install(TARGETS stereo
  DESTINATION lib/${PROJECT_NAME})

ament_package()
