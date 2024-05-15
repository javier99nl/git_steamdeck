#include "common.hpp"

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <iostream>
#include <memory>
#include <mutex>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

// ORB-SLAM3-specific libraries. Directory is defined in CMakeLists.txt:
// ${ORB_SLAM3_DIR}
#include "Converter.h"
#include "System.h"
#include "Tracking.h"

#include "std_srvs/srv/empty.hpp"

using std::placeholders::_1;

class MonoSlamNode : public rclcpp::Node {
public:
  MonoSlamNode(const std::string &vocabFile, const std::string &settingsFile,
               const bool visualize);

  ~MonoSlamNode();

private:
  using ImageMsg = sensor_msgs::msg::Image;

  void GrabFrame(const sensor_msgs::msg::Image::SharedPtr msgRGB);

  std::shared_ptr<ORB_SLAM3::System> slam;

  rclcpp::Subscription<ImageMsg>::SharedPtr img_sub;

  //Checkmap variables
  bool checkMap(std::shared_ptr<ORB_SLAM3::System> slam,rclcpp::Time msgTime, cv::Mat Tcw);
  int map_id;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr client;
  rclcpp::Time endTime;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  bool visualize = !strcmp(argv[3], "true");
  auto mono = std::make_shared<MonoSlamNode>(argv[1], argv[2], visualize);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(mono);
  
  while(rclcpp::ok())
  {
    executor.spin_once(std::chrono::milliseconds(100));
  }
  rclcpp::shutdown();

  return 0;
}

MonoSlamNode::MonoSlamNode(const std::string &vocabFile,
                           const std::string &settingsFile,
                           const bool visualize)
    : Node("orbslam3") {
  slam = std::make_shared<ORB_SLAM3::System>(
      vocabFile, settingsFile, ORB_SLAM3::System::MONOCULAR, visualize);
  img_sub = this->create_subscription<ImageMsg>(
      "/model/bluerov2/stereo_camera/left/image_raw", 10,
      std::bind(&MonoSlamNode::GrabFrame, this, std::placeholders::_1));//orbslam3/camera "/raybot/camera/image_raw"
  client=this->create_client<std_srvs::srv::Empty>("/octomap_server/reset");
  endTime=this->now() + rclcpp::Duration(5, 0);
  setup_ros_publishers(*this);
}

MonoSlamNode::~MonoSlamNode() {
  if (!slam->isShutDown()) {
    // Stop all threads
    slam->Shutdown();
    slam->SaveKeyFrameImages("filename");

    // Save camera trajectory
    // slam->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
  }
}

void MonoSlamNode::GrabFrame(const ImageMsg::SharedPtr msgImg) {
  const cv_bridge::CvImageConstPtr cv_ptrImg = cv_bridge::toCvShare(msgImg);
  cv::Mat img = cv_ptrImg->image.clone();  // clone the image if you want to keep the original one unchanged
  cv::cvtColor(img, img, cv::COLOR_RGB2BGR);  // Convert the image color from RGB to BGR
  cv::Mat Tcw = ORB_SLAM3::Converter::toCvMat(slam->TrackMonocular(img, cv_ptrImg->header.stamp.sec).matrix());
  //cv::Mat Tcw = ORB_SLAM3::Converter::toCvMat(slam->TrackMonocular(cv_ptrImg->image, cv_ptrImg->header.stamp.sec).matrix());
  //cv::Matx<float, 4, 4> Tcw_matx(Tcw.ptr<float>());
  //cv::Matx<float, 4, 4> Tcw_inverse;
 // cv::invert(Tcw_matx, Tcw_inverse);
  //cv::Mat Twc= cv::Mat(4, 4, CV_32F, Tcw_inverse.val);// Matrix trnasf. from the world to the camera
  cv::Mat Twc=Tcw;
  rclcpp::Time current_frame_time = cv_ptrImg->header.stamp;

  publish_ros_pose_tf(*this, Twc, current_frame_time,ORB_SLAM3::System::MONOCULAR);

  publish_ros_tracking_mappoints(slam->GetTrackedMapPoints(), current_frame_time);
  
  ORB_SLAM3::Atlas* atlas_pointer = slam->GetAtlasPointer();
  ORB_SLAM3::Map* map_pointer = atlas_pointer->GetCurrentMap();
  std::vector<ORB_SLAM3::MapPoint*> all_map_points = map_pointer->GetAllMapPoints();
  publish_ros_quality_mappoints(all_map_points,current_frame_time);

  //bool reset_map =checkMap(slam,current_frame_time,Twc);
  bool reset_map=false;
  if(reset_map==false){
    //publish_ros_quality_mappoints(slam->GetTrackedMapPoints(),current_frame_time);
  }else{
    while (!client->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_INFO(get_logger(), "Waiting for service to become available...");
    }
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    // Call the service
    auto future = client->async_send_request(request);
    
    std::vector<ORB_SLAM3::MapPoint*> map_points = map_pointer->GetAllMapPoints();
    std::thread t(&publish_ros_all_quality_mappoints, map_points, current_frame_time, std::move(future));
    t.detach();
  }
}
  
  bool MonoSlamNode::checkMap(std::shared_ptr<ORB_SLAM3::System> slam,rclcpp::Time msg_time,cv::Mat Twc)
  {
    bool reset=false;
    ORB_SLAM3::Atlas* atlas_pointer = slam->GetAtlasPointer();
    ORB_SLAM3::Map* map_pointer = atlas_pointer->GetCurrentMap();
    if(this->now() > endTime)
    {
      endTime=this->now() + rclcpp::Duration(10, 0);
      reset=true;
    }
    int new_map_id=map_pointer->GetId();
    if(new_map_id!= map_id){
      map_id=new_map_id;
      std::cout<<"Map Changed! The new Id is: "<< map_id<<endl;
      reset=true;
    }
    return reset;
  }

