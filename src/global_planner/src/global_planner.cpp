#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <octomap/octomap.h>
#include <chrono>
#include "octomap_msgs/msg/octomap.hpp"
#include "octomap_msgs/conversions.h"
#include <geometry_msgs/msg/pose.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/transform_listener.h>
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include <Eigen/Geometry>
#include <Eigen/Core>
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <thread>
#include <ctime>
#include <cmath>
#include <random>
#include <limits>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.hpp"
#include "octomap_msgs/conversions.h"

#include "octomap_ros/conversions.hpp"

using namespace std::chrono_literals;
using namespace octomap;
using visualization_msgs::msg::MarkerArray;
using std_msgs::msg::ColorRGBA;

// Global variables for the timer
std::chrono::time_point<std::chrono::high_resolution_clock> lastMapCheck; // global start time variable
std::chrono::time_point<std::chrono::high_resolution_clock> actualTime; // global end time variable
struct qualityPoint{
  std::chrono::time_point<std::chrono::high_resolution_clock> pointTime;
  octomap::OcTreeKey key;
}; 
struct Point {
    double x;
    double y;
    double z;
};

class GlobalPlanner : public rclcpp::Node
{
public:
  GlobalPlanner() : Node("global_planner")
  { 
    point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("virtual_point_cloud_centers", 1);
    marker_pub = this->create_publisher<MarkerArray>("virtual_octomap", 1);
    pub_full_cube=this->create_publisher<visualization_msgs::msg::Marker>("cube_vis",10);
    pub_next_cube=this->create_publisher<visualization_msgs::msg::Marker>("next_cube_vis",10);
    pub_camera_cube=this->create_publisher<visualization_msgs::msg::Marker>("camera_cube_vis",10);
    pub_cam_pose = this->create_publisher<geometry_msgs::msg::PoseStamped>("camera_pose", 10);
    pub_raybot_next_position = this->create_publisher<geometry_msgs::msg::Twist>("raybot/cmd_vel", 10);
    sub_pose = this->create_subscription<geometry_msgs::msg::PoseStamped>("/orbslam3/pose", 1, std::bind(&GlobalPlanner::poseCallback, this, std::placeholders::_1));
    sub_octomap = this->create_subscription<octomap_msgs::msg::Octomap>("/octomap_full", 1, std::bind(&GlobalPlanner::octomapCallback, this, std::placeholders::_1));
    timer=this->create_wall_timer(1000ms,std::bind(&GlobalPlanner::timer_add_points,this));
  }
private:
void timer_add_points()
{
  if (!octree) {
    return;
  }
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> disX(focal_point.x()-robot_length/2, focal_point.x()+robot_length/2);
  std::uniform_real_distribution<> disY(focal_point.y()-robot_length/2, focal_point.y()+robot_length/2);
  std::uniform_real_distribution<> disZ(focal_point.z()-robot_length/2, focal_point.z()+robot_length/2);
  //dis(gen)
  for(int i=0;i<100;i++)
  {
    octomap::point3d point(disX(gen),disY(gen),disZ(gen));
    octomap::OcTreeKey key;
    octomap::ColorOcTreeNode* virtual_node;
    if (octree->coordToKeyChecked(point, key)) 
      {
        virtual_node=octree->updateNode(key,true);
         if(virtual_node) virtual_node->setColor(0,255,0);
      }
      qualityPoint virtualPoint;
      virtualPoint.key=key;
      virtualPoint.pointTime=std::chrono::high_resolution_clock::now();
      qualityPoints.push_back(virtualPoint);
  }
  //add_points_around_that point
  //add_them to the vector
  //set the time
}

void publishOctomap(){
  // finish MarkerArray:
  MarkerArray occupied_nodes_vis;
  pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
  occupied_nodes_vis.markers.resize(octree->getTreeDepth() + 1);
  for(octomap::ColorOcTree::iterator it=octree->begin(octree->getTreeDepth() + 1);it!=octree->end();++it)
  {

    if (octree->isNodeOccupied(*it)) {
    // create marker:
            double z = it.getZ();
            double x = it.getX();
            double y = it.getY();
            int r = it->getColor().r;
            int g = it->getColor().g;
            if(g==255 and r==0){
              
              int b = it->getColor().b;
              pcl::PointXYZRGB _point = pcl::PointXYZRGB();
              _point.x = x;
              _point.y = y;
              _point.z = z;
              _point.r = r;
              _point.g = g;
              _point.b = b;
              pcl_cloud.push_back(_point);
              unsigned idx = it.getDepth();
              assert(idx < occupied_nodes_vis.markers.size());

              geometry_msgs::msg::Point cube_center;
              cube_center.x = x;
              cube_center.y = y;
              cube_center.z = z;

              occupied_nodes_vis.markers[idx].points.push_back(cube_center);
              ColorRGBA _color;
              _color.r = (r / 255.);
              _color.g = (g / 255.);
              _color.b = (b / 255.);
              // TODO(someone): EVALUATE: potentially use occupancy as measure for alpha channel?
              _color.a = 1.0;
              occupied_nodes_vis.markers[idx].colors.push_back(_color);
            
              for (size_t i = 0; i < occupied_nodes_vis.markers.size(); ++i) {
                double size = octree->getNodeSize(i);

                occupied_nodes_vis.markers[i].header.frame_id = "world";
                occupied_nodes_vis.markers[i].header.stamp = this->now();
                occupied_nodes_vis.markers[i].ns = "world";
                occupied_nodes_vis.markers[i].id = i;
                occupied_nodes_vis.markers[i].type = visualization_msgs::msg::Marker::CUBE_LIST;
                occupied_nodes_vis.markers[i].scale.x = size;
                occupied_nodes_vis.markers[i].scale.y = size;
                occupied_nodes_vis.markers[i].scale.z = size;
                //if (!use_colored_map_) {
                  //occupied_nodes_vis.markers[i].color = color_;
                //}


                if (occupied_nodes_vis.markers[i].points.size() > 0) {
                  occupied_nodes_vis.markers[i].action = visualization_msgs::msg::Marker::ADD;
                } else {
                  occupied_nodes_vis.markers[i].action = visualization_msgs::msg::Marker::DELETE;
                }
              }
            }
    }
    sensor_msgs::msg::PointCloud2 cloud;
    pcl::toROSMsg(pcl_cloud, cloud);
    cloud.header.frame_id = "world";
    cloud.header.stamp = this->now();
    point_cloud_pub_->publish(cloud);
    //marker_pub->publish(occupied_nodes_vis);
  }
}
void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg)
{
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
  if(qualityPoints.size()!=0){
    octomap::ColorOcTreeNode* virtual_node;
    for(int i=0;i<qualityPoints.size();i++){
      auto elapsedTime = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now()- qualityPoints[i].pointTime); 
        if(elapsedTime.count()>= 20){
         //Remove POINT
         qualityPoints.erase(qualityPoints.begin() + i);
         //std::cout<<std::endl<<"20 seconds have passed"<<std::endl;
        }else{ 
          virtual_node=octree->updateNode(qualityPoints[i].key,true);
          if(virtual_node) virtual_node->setColor(0,255,0);
        }
      }
    }
}
  //std::cout<<"The number of points in the octomap is: "<<octree->getNumLeafNodes()<<std::endl;
  
void pub_camera_pose(const geometry_msgs::msg::PoseStamped::SharedPtr poseStamped){
    geometry_msgs::msg::Pose::SharedPtr pose = std::make_shared<geometry_msgs::msg::Pose>(poseStamped->pose);
    const auto& position = pose->position;
    const auto& quaternion = pose->orientation;
    Eigen::Quaterniond q(quaternion.w, quaternion.x, quaternion.y, quaternion.z);
    Eigen::Matrix3d rotation_matrix = q.toRotationMatrix();

    Eigen::Vector3d camera_direction = rotation_matrix.col(2).normalized();
    //std::cout<<std::endl<< "Position of the camera: X: "<< position.x<< " Y: "<< position.y<<" Z: "<<position.z<<std::endl;
    geometry_msgs::msg::PoseStamped camera_direction_pose;
    camera_direction_pose.header.stamp = this->now();
    camera_direction_pose.header.frame_id = poseStamped->header.frame_id;
    double raybot_height=position.z;
    double point_of_focus=1+raybot_height*0.2;
    camera_direction_pose.pose.position.x = position.x + camera_direction.x()*(point_of_focus);
    camera_direction_pose.pose.position.y = position.y + camera_direction.y()*(point_of_focus);
    camera_direction_pose.pose.position.z = position.z + camera_direction.z()*(point_of_focus);
    Eigen::Quaterniond camera_direction_quaternion = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(), camera_direction);
    camera_direction_pose.pose.orientation.x = camera_direction_quaternion.x();
    camera_direction_pose.pose.orientation.y = camera_direction_quaternion.y();
    camera_direction_pose.pose.orientation.z = camera_direction_quaternion.z();
    camera_direction_pose.pose.orientation.w = camera_direction_quaternion.w();
    pub_cam_pose->publish(camera_direction_pose);
}
void publishCameraCube(){
  visualization_msgs::msg::Marker cube_msg = visualization_msgs::msg::Marker();
  cube_msg.header.frame_id = "world";
  cube_msg.header.stamp = this->now();
  cube_msg.id = 0;
  cube_msg.type = visualization_msgs::msg::Marker::CUBE;
  cube_msg.action = visualization_msgs::msg::Marker::ADD;
  cube_msg.pose.position.x = focal_point.x();
  cube_msg.pose.position.y = focal_point.y();
  cube_msg.pose.position.z =focal_point.z();
  cube_msg.pose.orientation.x = 0.0;
  cube_msg.pose.orientation.y = 0.0;
  cube_msg.pose.orientation.z = 0.0;
  cube_msg.pose.orientation.w = 1.0;
  cube_msg.scale.x = robot_length;
  cube_msg.scale.y = robot_length;
  cube_msg.scale.z = robot_length;
  cube_msg.color.r = 0.0;
  cube_msg.color.g = 0.0;
  cube_msg.color.b = 0.0;
  cube_msg.color.a = 0.5;
  cube_msg.lifetime = rclcpp::Duration(10,0);
  pub_camera_cube->publish(cube_msg);

}

void publishChosenCube(){

  visualization_msgs::msg::Marker cube_msg = visualization_msgs::msg::Marker();
  cube_msg.header.frame_id = "world";
  cube_msg.header.stamp = this->now();
  cube_msg.id = 0;
  cube_msg.type = visualization_msgs::msg::Marker::CUBE;
  cube_msg.action = visualization_msgs::msg::Marker::ADD;
  cube_msg.pose.position.x = focal_point.x() + ii*robot_length;
  cube_msg.pose.position.y = focal_point.y() + jj*robot_length;
  cube_msg.pose.position.z = focal_point.z() + kk*robot_length;
  cube_msg.pose.orientation.x = 0.0;
  cube_msg.pose.orientation.y = 0.0;
  cube_msg.pose.orientation.z = 0.0;
  cube_msg.pose.orientation.w = 1.0;
  cube_msg.scale.x = robot_length;
  cube_msg.scale.y = robot_length;
  cube_msg.scale.z = robot_length;
  cube_msg.color.r = 0.5;
  cube_msg.color.g = 0.0;
  cube_msg.color.b = 0.5;
  cube_msg.color.a = 0.5;
  cube_msg.lifetime = rclcpp::Duration(10,0);
  pub_next_cube->publish(cube_msg);

}

void publishCube(int i, int j, int k){
  visualization_msgs::msg::Marker cube_msg = visualization_msgs::msg::Marker();
  cube_msg.header.frame_id = "world";
  cube_msg.header.stamp = this->now();
  cube_msg.id = 0;
  cube_msg.type = visualization_msgs::msg::Marker::CUBE;
  cube_msg.action = visualization_msgs::msg::Marker::ADD;
  cube_msg.pose.position.x = focal_point.x()+ i*robot_length;
  cube_msg.pose.position.y = focal_point.y()+j*robot_length;
  cube_msg.pose.position.z =focal_point.z()+k*robot_length;
  cube_msg.pose.orientation.x = 0.0;
  cube_msg.pose.orientation.y = 0.0;
  cube_msg.pose.orientation.z = 0.0;
  cube_msg.pose.orientation.w = 1.0;
  cube_msg.scale.x = robot_length;
  cube_msg.scale.y = robot_length;
  cube_msg.scale.z = robot_length;
  cube_msg.color.r = 0.0;
  cube_msg.color.g = 0.0;
  cube_msg.color.b = 1.0;
  cube_msg.color.a = 0.5;
  cube_msg.lifetime = rclcpp::Duration(1,0);
  pub_full_cube->publish(cube_msg);

}
void getNextPosition(const geometry_msgs::msg::PoseStamped::SharedPtr poseStamped)
{
  geometry_msgs::msg::Pose::SharedPtr pose=std::make_shared<geometry_msgs::msg::Pose>(poseStamped->pose);
  const auto& position = pose->position;
  const auto& quaternion = pose->orientation;
  
  Eigen::Quaterniond q(quaternion.w, quaternion.x, quaternion.y, quaternion.z);
  Eigen::Matrix3d rotation_matrix = q.toRotationMatrix();
  Eigen::Vector3d camera_vector= rotation_matrix.col(2).normalized();
  double raybot_height=position.z;
  double focal_distance=1.0+raybot_height*0.2;
  //Point where the camera sees sharpest (from the camera position, in the direction of the camera (near the quay wall))
  //The focal point must be the center of the cube 
  focal_point.x()=position.x+camera_vector.x()*(focal_distance);
  focal_point.y()=position.y+camera_vector.y()*(focal_distance);
  focal_point.z()=position.z+camera_vector.z()*(focal_distance);
  std::cout<<std::endl<<"Focal point is: X: "<<focal_point.x()<<" Y: "<<focal_point.y()<< " Z: "<<focal_point.z()<<std::endl;
  
  octomap::point3d p1(
    focal_point.x()-robot_length/2,focal_point.y()-robot_length/2,focal_point.z()-robot_length/2);
  octomap::point3d p2(focal_point.x()+robot_length/2,focal_point.y()+robot_length/2,focal_point.z()+robot_length/2);
  int min_cube_quality=std::numeric_limits<int>::max();
  int max_cube_quality=0;
  
  for (int i=-1; i<2;i++)
  {
    for (int j= -1; j<2;j++)
    {
      for (int k= -1; k<2;k++)
      {
        if(i!=0 or j!=0 or k!=0)
        {
          if(i==0)
          {
            if((position.z > 2.7 and k==1) or (position.z< -0.5 and k==-1) or (position.y < -7 and j==-1) or (position.y> 17 and j==1)){
             // In order to not surpass the surface of the water or crash with the gound of the simulation. 
             //Limit the raybot to be in the limits of the wall
            }else
            {
              octomap::point3d q1(p1.x() + i* robot_length,p1.y()+j*robot_length, p1.z()+k*robot_length);
              octomap::point3d q2(p2.x() + i* robot_length,p2.y()+j*robot_length, p2.z()+k*robot_length);
              //Now we have defined the sides of the cubes
              octree->setBBXMin(q1);
              octree->setBBXMax(q2);
              int number_points=0;
              int cube_quality=0;
              for (auto it = octree->begin_leafs_bbx(q1,q2); it != octree->end_leafs_bbx(); ++it) 
              {
                // Do something with the node
                if (octree->isNodeOccupied(*it))
                {
                    octomap::ColorOcTreeNode node=(*it);
                    octomap::ColorOcTreeNode::Color color = node.getColor();
                    const OcTreeKey key=it.getKey();
                    point3d node_pos=octree->keyToCoord(key);
                    if(color.r==255 and color.g==0){
                      //RED
                      cube_quality=cube_quality +1;
                    }else if (color.r==255 and color.g==128){
                      //ORANGE
                      cube_quality=cube_quality +3;
                    }else if(color.r==255 and color.g==255){
                      cube_quality=cube_quality +6;
                      //YELLOW
                    }else if(color.r==0 and color.g==255){
                      cube_quality=cube_quality +10;
                      //GREEN
                    }
                    number_points++;
                  }
              }
              //publishCameraCube();
              //publishCube(i,j,k);
              //CUBE WITH THE BIGGEST QUALITY
              if(cube_quality>max_cube_quality){
                max_cube_quality=cube_quality;
              }
              //CUBE WITH THE LEAST QUALITY
              if(cube_quality<min_cube_quality){
                min_cube_quality=cube_quality;
                ii=i;
                jj=j;
                kk=k;
              }
            }
          }
        }
      }
    }
    
  }

  publishChosenCube();
  publishCameraCube();
  next_point.x=position.x+ ii*robot_length;
  next_point.y=position.y+ jj*robot_length;
  next_point.z=position.z+ kk*robot_length;
  //Publish pose command:
  auto message = geometry_msgs::msg::Twist();
  float vx=ii/4.0;
  float vy=jj/4.0;
  float vz=-kk/4.0;
  message.linear.x = vx;
  message.linear.y = vy;
  message.linear.z = vz;
  pub_raybot_next_position->publish(message);
  //Variable for looking if a velocity has being sent and get the time when was sent
  velocity_sent=true;
  lastMapCheck = std::chrono::high_resolution_clock::now();
}
void circular_movement(){
  //Generate random circular movement around the point
}
void controlPosition(const geometry_msgs::msg::PoseStamped::SharedPtr poseStamped){
  //Put a timer counter the first time this function is called (everytime a new cube goal has been set), if for more than five or ten seconds, this function is still called with the same next point
  // Set many virtual points in the next point so that position is not intended to be reached again
  const auto& position = poseStamped->pose.position;
  auto message = geometry_msgs::msg::Twist();
  bool velocity_x=true, velocity_y=true,velocity_z=true;

  if(position.x-0.05>next_point.x or position.x+0.05<next_point.x){
    message.linear.x=next_point.x-position.x;
  }else{
    velocity_x=false;
  }
  if(position.y-0.05>next_point.y or position.y+0.05<next_point.y){
    message.linear.y=(next_point.y-position.y);
  }else{
    velocity_y=false;
  }
  if(position.z-0.05>next_point.z or position.z+0.05<next_point.z){
    message.linear.z=(next_point.z- position.z);
  }else{
    velocity_z=false;
  }

  if(velocity_x!=false or velocity_y!=false or velocity_z!=false)
  {
    pub_raybot_next_position->publish(message);
    velocity_sent=true;
  }else{
    publishOctomap();
    circular_movement();
    velocity_sent=false;
  }

}

  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr poseStamped)
  {
    pub_camera_pose(poseStamped);
    //geometry_msgs::msg::Pose::SharedPtr pose=std::make_shared<geometry_msgs::msg::Pose>(poseStamped->pose);
   if (!octree) {
    //std::cout << "Octomap not received yet" << std::endl;
    return;
   }

    //Octree is not empty
    actualTime = std::chrono::high_resolution_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::seconds>(actualTime - lastMapCheck); // calculate elapsed time

    if (elapsedTime.count() >= 1 and velocity_sent==false) { // if 2 seconds have passed
      if (!octree) {
        return;
      }
      getNextPosition(poseStamped);
    }else{
      publishChosenCube();
      publishCameraCube();
      controlPosition(poseStamped);
    }
  }


  std::shared_ptr<octomap::ColorOcTree> octree;
  int ii,jj,kk;

  Point next_point;

  rclcpp::TimerBase::SharedPtr timer;

  float robot_length=0.4;
  octomap::point3d focal_point;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_full_cube;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_next_cube;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_camera_cube;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_cam_pose;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr sub_octomap;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_raybot_next_position;
  rclcpp::Publisher<MarkerArray>::SharedPtr marker_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
  bool velocity_sent=false;

  // Vector for storing high quality points

  std::vector<qualityPoint> qualityPoints;
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