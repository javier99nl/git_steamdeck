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
    publisher_z = this->create_publisher<geometry_msgs::msg::PoseStamped>("z_axis", 10);
    publisher_x = this->create_publisher<geometry_msgs::msg::PoseStamped>("x_axis", 10);
    publisher_y = this->create_publisher<geometry_msgs::msg::PoseStamped>("y_axis", 10);
    suscriber_pose = this->create_subscription<geometry_msgs::msg::PoseStamped>("/orbslam3/pose", 1, std::bind(&GlobalPlanner::poseCallback, this, std::placeholders::_1));
  }

private:
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr poseStamped)
  {
    geometry_msgs::msg::Pose::SharedPtr pose = std::make_shared<geometry_msgs::msg::Pose>(poseStamped->pose);
    //Extract Rotation and translation matrix from Camera Posepublisher_z
    const auto& position = pose->position;
    const auto& quaternion = pose->orientation;
    Eigen::Quaterniond q(quaternion.w, quaternion.x, quaternion.y, quaternion.z);
    Eigen::Matrix3d rotation_matrix = q.toRotationMatrix();
    Eigen::Vector3d z_axis = -1.0* rotation_matrix.col(0).normalized();
    // Create a new PoseStamped message and set its position and orientation
    //MESSAGE FOR GETTING THE DIRECTION OF THE CAMERA
    
    geometry_msgs::msg::PoseStamped z_axis_pose;
    z_axis_pose.header.stamp = this->now();
    z_axis_pose.header.frame_id = poseStamped->header.frame_id;
    double point_of_focus=1;



    //X AXIS que va a ser el eje Z 
    geometry_msgs::msg::PoseStamped x_axis_pose;
    x_axis_pose.header.stamp = this->now();
    x_axis_pose.header.frame_id = poseStamped->header.frame_id;
    
    Eigen::Vector3d x_axis = -1.0* rotation_matrix.col(2).normalized();
    x_axis_pose.pose.position.x = position.x + x_axis.x()*(point_of_focus);
    x_axis_pose.pose.position.y = position.y + x_axis.y()*(point_of_focus);
    x_axis_pose.pose.position.z = position.z + x_axis.z()*(point_of_focus);
    std::cout<<"Position X: "<< x_axis_pose.pose.position.x<<" , "<< x_axis_pose.pose.position.y<<" , "<< x_axis_pose.pose.position.z<<" , "<<std::endl;
    Eigen::Quaterniond x_axis_quaternion = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(), x_axis);
    x_axis_pose.pose.orientation.x = x_axis_quaternion.x();
    x_axis_pose.pose.orientation.z = x_axis_quaternion.z();
    x_axis_pose.pose.orientation.w = x_axis_quaternion.w();

    publisher_x->publish(x_axis_pose);

    //Y AXIS
    geometry_msgs::msg::PoseStamped y_axis_pose;
    y_axis_pose.header.stamp = this->now();
    y_axis_pose.header.frame_id = poseStamped->header.frame_id;
    Eigen::Vector3d y_axis = -1.0* rotation_matrix.col(2).normalized();
    y_axis_pose.pose.position.x = position.x + y_axis.x()*(point_of_focus);
    y_axis_pose.pose.position.y = position.y + y_axis.y()*(point_of_focus);
    y_axis_pose.pose.position.z = position.z + y_axis.z()*(point_of_focus);
    std::cout<<"Position Y: "<< y_axis_pose.pose.position.x<<" , "<< y_axis_pose.pose.position.y<<" , "<< y_axis_pose.pose.position.z<<" , "<<std::endl;
    Eigen::Quaterniond y_axis_quaternion = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitY(), y_axis);
    y_axis_pose.pose.orientation.x = y_axis_quaternion.x();
    y_axis_pose.pose.orientation.y = y_axis_quaternion.y();
    y_axis_pose.pose.orientation.z = y_axis_quaternion.z();
    y_axis_pose.pose.orientation.w = x_axis_quaternion.w();

    publisher_y->publish(y_axis_pose);

        //Z AXIS 
    // Y AXIS ES Z
    Eigen::Vector3d  yy_axis = -1.0* rotation_matrix.col(2).normalized();
    yy_axis=-1*yy_axis;
    z_axis_pose.pose.position.x = position.x + yy_axis.x()*(point_of_focus);
    z_axis_pose.pose.position.y = position.y + yy_axis.y()*(point_of_focus);
    z_axis_pose.pose.position.z = position.z + yy_axis.z()*(point_of_focus);
    std::cout<<"Position Z: "<< z_axis_pose.pose.position.x<<" , "<< z_axis_pose.pose.position.y<<" , "<< z_axis_pose.pose.position.z<<" , "<<std::endl;
    Eigen::Quaterniond z_axis_quaternion = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), z_axis);
    z_axis_pose.pose.orientation.x = z_axis_quaternion.x();
    z_axis_pose.pose.orientation.y = z_axis_quaternion.y();
    z_axis_pose.pose.orientation.z = z_axis_quaternion.z();
    z_axis_pose.pose.orientation.w = z_axis_quaternion.w();

    publisher_z->publish(z_axis_pose);
  }


  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_z;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_x;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_y;
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
/*

k:
- 407.0646129842357
- 0.0
- 384.5
- 0.0
- 407.0646129842357
- 246.5
- 0.0
- 0.0
- 1.0
R AND T 
transforms:
- header:
    stamp:
      sec: 1678710713
      nanosec: 48470084
    frame_id: world
  child_frame_id: pose
  transform:
    translation:
      x: 0.04668046906590462
      y: -0.08963384479284286
      z: 0.5345287322998047
    rotation:
      x: 0.010188883931569541
      y: -0.1318796836319271
      z: -0.006718750596138669
      w: 0.9911906047671152
---



*/