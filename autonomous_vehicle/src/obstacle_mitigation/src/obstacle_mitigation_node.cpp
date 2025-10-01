#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>


namespace lidar {

class ObstacleMitigation : public rclcpp::Node{

  public:
    explicit ObstacleMitigation(const rclcpp::NodeOptions& options)
      : rclcpp::Node("obstacle_mitigation", options)
    {
      publisher_ = create_publisher<geometry_msgs::msg::Twist>(
        "/diff_drive_controller/cmd_vel_unstamped",
        rclcpp::SystemDefaultsQoS());

      subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/points",
        rclcpp::SystemDefaultsQoS(),
        std::bind(&ObstacleMitigation::Callback, this, std::placeholders::_1));
    }

  private:

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    bool obstacle_detected_ = false;

    double frontal_x_threshold_ = 0.75;    
    double frontal_y_tolerance_ = 0.2;   

    void Callback(const sensor_msgs::msg::PointCloud2::SharedPtr points){
      obstacle_detected_ = false;
 
      try {
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*points, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*points, "y");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y) {
          float x = *iter_x;
          float y = *iter_y;
       
          if (x > 0.0f && x < static_cast<float>(frontal_x_threshold_) &&
              std::abs(y) < static_cast<float>(frontal_y_tolerance_)) {
            obstacle_detected_ = true;
            break;
          }
        }
      } catch (const std::exception &e) {
        RCLCPP_WARN(this->get_logger(), "PointCloud2 iteration failed: %s", e.what());
      }

      geometry_msgs::msg::Twist cmd_vel;
      if (obstacle_detected_) {
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
      } else {
        cmd_vel.linear.x = 0.5; 
        cmd_vel.angular.z = 0.5; 
      }
      publisher_->publish(cmd_vel);
    }
  };
} 
RCLCPP_COMPONENTS_REGISTER_NODE(lidar::ObstacleMitigation)