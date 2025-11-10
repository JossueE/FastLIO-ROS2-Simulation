#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl_conversions/pcl_conversions.h>
#include "pcl_conversions.hpp"  // helper de conversiones

class Pc2ToXYZI : public rclcpp::Node {
  private:

    std::string lidar_topic_in_ = "/none";
    std::string lidar_topic_out_ = "/none";

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub;

  public:

    Pc2ToXYZI() : Node("pc2_to_xyzi") {

      this->declare_parameter("lidar_topic_in_", std::string("/lidar/points_ign"));
      this->declare_parameter("lidar_topic_out_", std::string("/lidar/points_pcl"));

      this->get_parameter("lidar_topic_in_", lidar_topic_in_);
      this->get_parameter("lidar_topic_out_", lidar_topic_out_);

      sub = create_subscription<sensor_msgs::msg::PointCloud2>(lidar_topic_in_, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg){
          pcl::PointCloud<pcl::PointXYZI> cloud;
          pcl_df::fromROSMsg(*msg, cloud);                // ← convert to PCL (XYZI)
          sensor_msgs::msg::PointCloud2 out;
          pcl::toROSMsg(cloud, out);                   // ← return to PointCloud2
          out.header = msg->header;                    // mantein frame y stamp
          pub->publish(out);
        });

      pub = create_publisher<sensor_msgs::msg::PointCloud2>(lidar_topic_out_, rclcpp::SensorDataQoS());
    }

    ~Pc2ToXYZI() //Class destructor definition
    {
      RCLCPP_INFO(this->get_logger(), "Node Pc2ToXYZI has been terminated"); 
    }
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Pc2ToXYZI>());
  rclcpp::shutdown();
  return 0;
}


