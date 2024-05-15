#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <octomap/octomap.h>
#include <chrono>
#include "octomap_msgs/msg/octomap.hpp"
#include "octomap_msgs/conversions.h"
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/transform_listener.h>
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"

#include <Eigen/Geometry>
#include <Eigen/Core>


using namespace std::chrono_literals;

class GlobalPlanner : public rclcpp::Node
{
public:
  GlobalPlanner() : Node("global_planner")
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("z_axis", 10);
    suscriber_pose = this->create_subscription<geometry_msgs::msg::PoseStamped>("/orbslam3/pose", 1, std::bind(&GlobalPlanner::poseCallback, this, std::placeholders::_1));
  }

private:
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr poseStamped)
  {
    geometry_msgs::msg::Pose::SharedPtr pose = std::make_shared<geometry_msgs::msg::Pose>(poseStamped->pose);
    //Extract Rotation and translation matrix from Camera Pose
    const auto& position = pose->position;
    const auto& quaternion = pose->orientation;
    Eigen::Quaterniond q(quaternion.w, quaternion.x, quaternion.y, quaternion.z);
    Eigen::Matrix3d rotation_matrix = q.toRotationMatrix();
    Eigen::Vector3d z_axis = rotation_matrix.col(2).normalized();
    // Create a new PoseStamped message and set its position and orientation
    //MESSAGE FOR GETTING THE DIRECTION OF THE CAMERA
    
    geometry_msgs::msg::PoseStamped z_axis_pose;
    z_axis_pose.header.stamp = this->now();
    z_axis_pose.header.frame_id = poseStamped->header.frame_id;
    double point_of_focus=1;

    //z_axis_pose.pose.position=position;
    z_axis_pose.pose.position.x = position.x + z_axis.x()*(point_of_focus);
    z_axis_pose.pose.position.y = position.y + z_axis.y()*(point_of_focus);
    z_axis_pose.pose.position.z = position.z + z_axis.z()*(point_of_focus);
    std::cout<<"Position: "<< z_axis_pose.pose.position.x<<" , "<< z_axis_pose.pose.position.y<<" , "<< z_axis_pose.pose.position.z<<" , "<<std::endl;
    Eigen::Quaterniond z_axis_quaternion = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(), z_axis);
    z_axis_pose.pose.orientation.x = z_axis_quaternion.x();
    z_axis_pose.pose.orientation.y = z_axis_quaternion.y();
    z_axis_pose.pose.orientation.z = z_axis_quaternion.z();
    z_axis_pose.pose.orientation.w = z_axis_quaternion.w();

    

    publisher_->publish(z_axis_pose);
  }


  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr suscriber_pose;
  //Eigen::Matrix3d k_matrix={ 407.06, 0.0 , 384.5, 0.0,    407.06,   246.5,0.0,    0.0,       1.0, }; 
  Eigen::Matrix3d K_matrix;
};
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto global_planner = std::make_shared<GlobalPlanner>();

  // Run the node
  rclcpp::spin(global_planner);

  rclcpp::shutdown();
  return 0;
  
}