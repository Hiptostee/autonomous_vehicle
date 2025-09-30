#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>



namespace lidar {

class LidarScanToCloud : public rclcpp::Node{

  public:
    explicit LidarScanToCloud(const rclcpp::NodeOptions& options): rclcpp::
    Node("scan_to_cloud", options){

      tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

      publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("/points", rclcpp::SystemDefaultsQoS());  
      subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SystemDefaultsQoS(),
        std::bind(&LidarScanToCloud::Callback, this, std::placeholders::_1));
    }

    private:

    laser_geometry::LaserProjection projector_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    void Callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_in){
      sensor_msgs::msg::PointCloud2 cloud;
      try{
        projector_.transformLaserScanToPointCloud(
          "base_link", *scan_in, cloud, *tf_buffer_);
        publisher_->publish(cloud);
      }
      catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Transform error: %s", ex.what());
      }
    }
  };
} 
RCLCPP_COMPONENTS_REGISTER_NODE(lidar::LidarScanToCloud)