#ifndef SCAN_TO_CLOUD_NODE_HPP_
#define SCAN_TO_CLOUD_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <laser_geometry/laser_geometry.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

namespace lidar {

class LidarScanToCloud : public rclcpp::Node {
public:
    explicit LidarScanToCloud(const rclcpp::NodeOptions& options);

private:
    laser_geometry::LaserProjection laser_projector_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
};

} 

#endif
