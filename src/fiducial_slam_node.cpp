// fiducial_slam_node.cpp
// A simple stub implementation for the fiducial_slam_ros2 node

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <fiducial_msgs/msg/fiducial_transform_array.hpp>
#include <fiducial_slam_ros2/srv/add_fiducial.hpp>
#include <fiducial_slam_ros2/transform_with_variance.hpp>
#include <fiducial_slam_ros2/helpers.hpp>
#include <fiducial_slam_ros2/map.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

class FiducialSlam : public rclcpp::Node
{
public:
  FiducialSlam()
  : Node("fiducial_slam")
  {
    RCLCPP_INFO(this->get_logger(), "Starting fiducial_slam_ros2 node");
    
    // Initialize parameters
    map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
    
    RCLCPP_INFO(this->get_logger(), "Using map_frame: %s", map_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "Using odom_frame: %s", odom_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "Using base_frame: %s", base_frame_.c_str());
    
    // Create the map instance
    map_ = std::make_shared<Map>(shared_from_this());
    
    // Simple subscription to the fiducial_transforms topic
    fiducial_sub_ = this->create_subscription<fiducial_msgs::msg::FiducialTransformArray>(
      "fiducial_transforms", 1,
      std::bind(&FiducialSlam::fiducialCallback, this, std::placeholders::_1));
  }

private:
  // Callback for handling fiducial transform messages
  void fiducialCallback(const fiducial_msgs::msg::FiducialTransformArray::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), 
                "Received %d fiducial transforms", 
                static_cast<int>(msg->transforms.size()));
    
    if (msg->transforms.empty()) {
      RCLCPP_DEBUG(this->get_logger(), "No fiducial transforms");
      return;
    }
    
    // Convert fiducial transforms to observations
    std::vector<Observation> observations;
    
    for (const auto &transform : msg->transforms) {
      // Create a TransformWithVariance for this fiducial
      TransformWithVariance tv(transform.transform, 0.1); // 0.1 is a placeholder variance
      
      tf2::Stamped<TransformWithVariance> camera_fiducial;
      camera_fiducial.frame_id_ = msg->header.frame_id;
      camera_fiducial.stamp_ = tf2::TimePointZero;
      camera_fiducial.setData(tv);
      
      // Create an observation
      observations.emplace_back(transform.fiducial_id, camera_fiducial);
      
      RCLCPP_DEBUG(this->get_logger(), "Adding observation for fiducial %d", transform.fiducial_id);
    }
    
    // Update the map with these observations
    map_->update(observations, this->now());
  }
  
  // Member variables
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  std::shared_ptr<Map> map_;
  
  // Subscribers
  rclcpp::Subscription<fiducial_msgs::msg::FiducialTransformArray>::SharedPtr fiducial_sub_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FiducialSlam>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
