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
    pub_cam_pose = this->create_publisher<geometry_msgs::msg::PoseStamped>("camera_pose", 10);
    pub_raybot_next_position = this->create_publisher<geometry_msgs::msg::PoseStamped>("raybot/cmd_pose", 10);
    sub_pose = this->create_subscription<geometry_msgs::msg::PoseStamped>("/orbslam3/pose", 1, std::bind(&GlobalPlanner::poseCallback, this, std::placeholders::_1));
    sub_octomap = this->create_subscription<octomap_msgs::msg::Octomap>("/octomap_full", 1, octomapCallback);
  }
private:

void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg){
  octomap::AbstractOcTree* abstractOcTree = octomap_msgs::fullMsgToMap(*msg);
  if(!abstractOcTree){
    std::cout << "Abstract Octree did not convert correctly" << std::endl;
    return;
  }
  octree=std::shared_ptr<octomap::ColorOcTree>(dynamic_cast<octomap::ColorOcTree*>(abstractOcTree));
  if (!octree) {
    std::cout << "Not transformed correctly" << std::endl;
    return;
  }
  std::cout<<"The number of points in the octomap is: "<<octree->getNumLeafNodes();
  
}

void getNextPosition(const geometry_msgs::msg::Pose::SharedPtr pose)
{
  const auto& position = pose->position;
  const auto& quaternion = pose->orientation;
  Eigen::Quaterniond q(quaternion.w, quaternion.x, quaternion.y, quaternion.z);
  Eigen::Matrix3d rotation_matrix = q.toRotationMatrix();
  Eigen::Vector3d camera_vector= rotation_matrix.col(2).normalized();
  double focal_distance=1;
  //Point where the camera sees sharped (from the camera position, in the direction of the camera (near the quay wall))
  //The focal point must be the center of the cube 
  octomap::point3d focal_point(position.x+camera_vector.x()*(focal_distance),position.y+camera_vector.y()*(focal_distance),position.z+camera_vector.z()*(focal_distance));
  float robot_length=10.0;
  octomap::point3d p1(focal_point.x()-robot_length/2,focal_point.y()-robot_length/2,focal_point.z()-robot_length/2);
  octomap::point3d p2(focal_point.x()+robot_length/2,focal_point.y()+robot_length/2,focal_point.z()+robot_length/2);

  int max_number_points=0;
  for (int i=-1; i<2;i++)
  {
    for (int j= -1; j<2;j++)
    {
      for (int k= -1; k<2;k++)
      {
        if(i!=0 or j!=0 or k!=0)
        {
            octomap::point3d q1(p1.x() + i* robot_length,p1.y()+j*robot_length, p1.z()+k*robot_length);
            octomap::point3d q2(p2.x() + i* robot_length,p2.y()+j*robot_length, p2.z()+k*robot_length);
            //Now we have defined the sides of the cubes
            octree->setBBXMin(q1);
            octree->setBBXMax(q2);
            int number_points=0;
            for (auto it = octree->begin_leafs_bbx(q1,q2); it != octree->end_leafs_bbx(); ++it) 
            {
              // Do something with the node
              if (octree->isNodeOccupied(*it))
               {
                  octomap::ColorOcTreeNode node=(*it);
                  octomap::ColorOcTreeNode::Color color = node.getColor();
                  //std::cout << std::endl << "The Color of the node is formed by Red: " << static_cast<int>(color.r) << " Blue: " << static_cast<int>(color.b) << " Green: " << static_cast<int>(color.g);
                  const OcTreeKey key=it.getKey();
                  point3d node_pos=octree->keyToCoord(key);
                  //std::cout<<std::endl<<"The positions of the node are: X= "<<node_pos.x()<< " Y: "<<node_pos.y()<< " Z: "<< node_pos.z()<<std::endl;
                  number_points++;
                }
             }
             if(number_points>max_number_points){
              max_number_points=number_points;
              std::cout<<std::endl<<"The box with most points is = i: "<< i<< " j: "<< j<< " K: "<<k <<" And the number of points are: "<< max_number_points<<std::endl;
             }else{
               //std::cout<<std::endl<< "And the number of points are: "<< max_number_points<<std::endl;
             }
        }
      }
    }
    
  }
  lastMapCheck = std::chrono::high_resolution_clock::now();
}

  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr poseStamped)
  {
    geometry_msgs::msg::Pose::SharedPtr pose=std::make_shared<geometry_msgs::msg::Pose>(poseStamped->pose);
   if (!octree) {
    //std::cout << "Octomap not received yet" << std::endl;
    return;
  }

  //Octree is not empty
  actualTime = std::chrono::high_resolution_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::seconds>(actualTime - lastMapCheck); // calculate elapsed time

    if (elapsedTime.count() >= 1) { // if 2 seconds have passed
      if (!octree) {
        return;
      }
      getNextPosition(pose);
    }
}

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_cam_pose;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose;
  rclcpp::Subscription<octomap_msgs::msg::msg>::SharedPtr sub_octomap;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_raybot_next_position;

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