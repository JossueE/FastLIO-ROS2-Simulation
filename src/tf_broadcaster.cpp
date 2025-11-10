/*
@description:
In order to obtain a correct visualization in Rviz, this node is a tf broadcaster 
to provide transformation between frames odom and base_footprint.

@author: C. Mauricio Arteaga-Escamilla from "Robotica Posgrado (YouTube channel)"
*/

#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>

using namespace std::chrono_literals;
using std::placeholders::_1;


class TfBroadcaster : public rclcpp::Node
{
  public:
    TfBroadcaster() : Node("tf_broadcaster_odom")
    {
      // timer_ = this->create_wall_timer(20ms, std::bind(&LFController::loop, this)); //20ms = 50 Hz
      
      odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 
            2, std::bind(&TfBroadcaster::odom_callback, this, _1));

      // Initialize the transform broadcaster
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

      RCLCPP_INFO(this->get_logger(), "Node initialized");
    }
    
    ~TfBroadcaster() //Class destructor definition
    {
      RCLCPP_INFO(this->get_logger(), "Node TfBroadcaster has been terminated"); 
    }
    
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
      double x,y;
      x = msg->pose.pose.position.x;
      y = msg->pose.pose.position.y;
      
      //GET THE EULER ANGLES FROM THE QUATERNION
      tf2::Quaternion q( msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

      //Create and fill out the transform broadcaster
      tranStamp_.header.stamp = msg->header.stamp; //this->get_clock()->now();
      tranStamp_.header.frame_id = "odom"; //msg->header.frame_id; //ns_+frame_id_;
      tranStamp_.child_frame_id = "base_footprint";

      tranStamp_.transform.translation.x = x;
      tranStamp_.transform.translation.y = y;

      tranStamp_.transform.rotation.x = q.x();
      tranStamp_.transform.rotation.y = q.y();
      tranStamp_.transform.rotation.z = q.z();
      tranStamp_.transform.rotation.w = q.w();

      tf_broadcaster_->sendTransform(tranStamp_); // Send the transformation
    }

  private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    std::string ns_; 
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    geometry_msgs::msg::TransformStamped tranStamp_; 
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TfBroadcaster>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
