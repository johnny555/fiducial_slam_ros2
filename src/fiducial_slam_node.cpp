// fiducial_slam_node.cpp
// A simple stub implementation for the fiducial_slam_ros2 node

#include <memory>
#include <string> // Required for std::string
#include <vector> // Required for std::vector
#include <cstdlib> // Required for std::getenv

#include <rclcpp/rclcpp.hpp>
#include <fiducial_msgs/msg/fiducial_transform_array.hpp>
#include <fiducial_slam_ros2/srv/add_fiducial.hpp>
#include <fiducial_slam_ros2/transform_with_variance.hpp>
#include <fiducial_slam_ros2/helpers.hpp>
#include <fiducial_slam_ros2/map.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class FiducialSlam : public rclcpp::Node
{
public:
  FiducialSlam()
  : Node("fiducial_slam")
  {
    RCLCPP_INFO(this->get_logger(), "FiducialSlam constructor: Starting");
    
    // Let the Map class declare and manage all parameters to avoid conflicts
    // We'll get the parameter values after the Map is initialized

    // Initialize TF buffer and listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  void initialize(std::shared_ptr<FiducialSlam> self) {
    // Create the map instance after the object is fully constructed
    // The Map class will declare all the necessary parameters
    map_ = std::make_shared<Map>(self);

    // Now get the parameter values that the Map class has declared
    map_frame_ = this->get_parameter("map_frame").as_string();
    odom_frame_ = this->get_parameter("odom_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();

    // Set camera frame if not already declared by Map
    try {
      camera_frame_ = this->get_parameter("camera_frame").as_string();
    } catch (const rclcpp::exceptions::ParameterNotDeclaredException&) {
      camera_frame_ = this->declare_parameter<std::string>("camera_frame", "realsense_camera_link");
    }
    
    // Declare parameters that are used by the Map class
    this->declare_parameter<bool>("publish_tf", true);
    this->declare_parameter<double>("tf_publish_interval", 1.0);

    const char* home = std::getenv("HOME");
    std::string home_path = home ? std::string(home) : std::string("/root");
    std::string default_map_path = home_path + "/.ros/slam/map.txt";
    this->declare_parameter<std::string>("map_file", default_map_path);
    this->declare_parameter<std::string>("initial_map_file", "");

    this->declare_parameter<bool>("publish_6dof_pose", false);
    this->declare_parameter<double>("systematic_error", 0.01);
    this->declare_parameter<double>("future_date_transforms", 0.1);
    this->declare_parameter<bool>("read_only_map", false);
    this->declare_parameter<std::string>("covariance_diagonal_str", "[0.1, 0.1, 0.1, 0.1, 0.1, 0.1]");
    this->declare_parameter<std::vector<double>>("covariance_diagonal", 
                                               std::vector<double>{0.1, 0.1, 0.1, 0.1, 0.1, 0.1});
    this->declare_parameter<double>("multi_error_threshold", -1.0); // Corrected typo from theshold

    RCLCPP_INFO(this->get_logger(), "Using map_frame: %s", map_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "Using odom_frame: %s", odom_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "Using base_frame: %s", base_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "Using camera_frame: %s", camera_frame_.c_str());

    // Simple subscription to the fiducial_transforms topic
    fiducial_sub_ = this->create_subscription<fiducial_msgs::msg::FiducialTransformArray>(
      "fiducial_transforms", 1,
      std::bind(&FiducialSlam::fiducialCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "fiducial_slam_ros2 node initialized successfully");
  }

private:
  // Callback for handling fiducial transform messages
  void fiducialCallback(const fiducial_msgs::msg::FiducialTransformArray::SharedPtr msg)
  {
    RCLCPP_DEBUG(this->get_logger(), 
                "Received %d fiducial transforms", 
                static_cast<int>(msg->transforms.size()));
    
    if (msg->transforms.empty()) {
      RCLCPP_DEBUG(this->get_logger(), "No fiducial transforms");
      return;
    }
    
    // Convert fiducial transforms to observations
    std::vector<Observation> observations;
    
    for (const auto &transform : msg->transforms) {
      try {
        // Create a TransformWithVariance for this fiducial
        // Use image quality to estimate variance (lower quality = higher variance)
        double variance = std::max(0.01, 1.0 / std::max(1.0, transform.image_error));
        TransformWithVariance tv(transform.transform, variance);
        
        tf2::Stamped<TransformWithVariance> camera_fiducial;
        camera_fiducial.frame_id_ = msg->header.frame_id;
        
        camera_fiducial.stamp_ = tf2_ros::fromMsg(msg->header.stamp);
        camera_fiducial.setData(tv);
        
        // Create an observation
        observations.emplace_back(transform.fiducial_id, camera_fiducial);
        
        RCLCPP_DEBUG(this->get_logger(), 
                    "Adding observation for fiducial %d with variance %.4f", 
                    transform.fiducial_id, variance);
      }
      catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), 
                   "Failed to process fiducial %d: %s", 
                   transform.fiducial_id, e.what());
      }
    }
    
    if (!observations.empty()) {
      // Update the map with these observations
      map_->update(observations, this->now());
      RCLCPP_DEBUG(this->get_logger(), "Updated map with %d observations", 
                  static_cast<int>(observations.size()));
    }
  }
  
  // Member variables
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string camera_frame_;
  std::shared_ptr<Map> map_;
  
  // TF components
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  // Subscribers
  rclcpp::Subscription<fiducial_msgs::msg::FiducialTransformArray>::SharedPtr fiducial_sub_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FiducialSlam>();


  // Initialize the node after construction is complete
  node->initialize(node);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
