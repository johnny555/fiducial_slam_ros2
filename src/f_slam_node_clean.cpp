// f_slam_node.cpp
// GTSAM-based fiducial SLAM implementation

#include <memory>
#include <chrono>
#include <unordered_map>
#include <fstream>
#include <sstream>

#include <rclcpp/rclcpp.hpp>
#include <fiducial_msgs/msg/fiducial_transform_array.hpp>
#include <fiducial_msgs/msg/fiducial_map_entry_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// GTSAM includes
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/linear/NoiseModel.h>

using namespace gtsam;

class FSlamNode : public rclcpp::Node
{
public:
  FSlamNode() : Node("f_slam")
  {
    RCLCPP_INFO(this->get_logger(), "Starting GTSAM-based fiducial SLAM node");
    
    // Declare parameters
    declare_parameter("map_frame", "map");
    declare_parameter("odom_frame", "odom");
    declare_parameter("base_frame", "base_link");
    declare_parameter("camera_frame", "camera_link");
    declare_parameter("mapping_mode", true);
    declare_parameter("fiducial_map_file", "fiducial_map.txt");
    declare_parameter("optimization_frequency", 2.0);
    
    // Get parameters
    map_frame_ = get_parameter("map_frame").as_string();
    odom_frame_ = get_parameter("odom_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    camera_frame_ = get_parameter("camera_frame").as_string();
    mapping_mode_ = get_parameter("mapping_mode").as_bool();
    fiducial_map_file_ = get_parameter("fiducial_map_file").as_string();
    optimization_frequency_ = get_parameter("optimization_frequency").as_double();
    
    RCLCPP_INFO(this->get_logger(), "Mode: %s", mapping_mode_ ? "MAPPING" : "LOCALIZATION");
    
    // Initialize TF
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    // Initialize GTSAM components
    initializeGTSAM();
    
    // Create subscribers
    fiducial_sub_ = create_subscription<fiducial_msgs::msg::FiducialTransformArray>(
      "fiducial_transforms", 10,
      std::bind(&FSlamNode::fiducialCallback, this, std::placeholders::_1));
      
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10,
      std::bind(&FSlamNode::odometryCallback, this, std::placeholders::_1));
    
    // Create publishers
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "robot_pose", 10);
    
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "fiducials", 100);
      
    map_pub_ = create_publisher<fiducial_msgs::msg::FiducialMapEntryArray>(
      "fiducial_map", 1);
    
    // Create optimization timer
    optimization_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / optimization_frequency_),
      std::bind(&FSlamNode::optimizationCallback, this));
      
    // Load existing fiducial map if in localization mode
    if (!mapping_mode_) {
      loadFiducialMap();
    }
    
    RCLCPP_INFO(this->get_logger(), "F-SLAM node initialized successfully");
  }
  
  ~FSlamNode() {
    if (mapping_mode_) {
      saveFiducialMap();
      RCLCPP_INFO(this->get_logger(), "Saved fiducial map to %s", fiducial_map_file_.c_str());
    }
  }

private:
  void initializeGTSAM() {
    // Create noise models
    // Odometry noise (x, y, z, roll, pitch, yaw)
    auto odom_noise_vector = (Vector(6) << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1).finished();
    odometry_noise_ = noiseModel::Diagonal::Variances(odom_noise_vector);
    
    // Fiducial observation noise
    auto fiducial_noise_vector = (Vector(6) << 0.05, 0.05, 0.05, 0.1, 0.1, 0.1).finished();
    fiducial_noise_ = noiseModel::Diagonal::Variances(fiducial_noise_vector);
    
    // Prior noise for first pose
    auto prior_noise_vector = (Vector(6) << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01).finished();
    prior_noise_ = noiseModel::Diagonal::Variances(prior_noise_vector);
    
    // Initialize first robot pose
    current_robot_key_ = Symbol('x', 0);
    Pose3 initial_pose = Pose3::Identity();
    
    // Add prior factor for first pose
    graph_.add(PriorFactor<Pose3>(current_robot_key_, initial_pose, prior_noise_));
    initial_estimate_.insert(current_robot_key_, initial_pose);
    
    pose_counter_ = 1;
    last_optimization_time_ = now();
  }
  
  void fiducialCallback(const fiducial_msgs::msg::FiducialTransformArray::SharedPtr msg) {
    if (msg->transforms.empty()) {
      return;
    }
    
    std::lock_guard<std::mutex> lock(graph_mutex_);
    
    for (const auto& transform : msg->transforms) {
      processFiducialObservation(transform, msg->header);
    }
  }
  
  void processFiducialObservation(const fiducial_msgs::msg::FiducialTransform& transform,
                                  const std_msgs::msg::Header& header) {
    int fiducial_id = transform.fiducial_id;
    Symbol fiducial_key = Symbol('l', fiducial_id);
    
    // Convert ROS transform to GTSAM Pose3
    Pose3 camera_to_fiducial = rosToPose3(transform.transform);
    
    // If this is a new fiducial and we're in mapping mode, initialize it
    if (mapping_mode_ && known_fiducials_.find(fiducial_id) == known_fiducials_.end()) {
      initializeNewFiducial(fiducial_id, camera_to_fiducial, header.stamp);
    }
    
    // Add observation factor if fiducial is known
    if (known_fiducials_.find(fiducial_id) != known_fiducials_.end()) {
      // Add factor: robot_pose + camera_to_fiducial = fiducial_pose
      graph_.add(BetweenFactor<Pose3>(current_robot_key_, fiducial_key, 
                                      camera_to_fiducial, fiducial_noise_));
      
      RCLCPP_DEBUG(this->get_logger(), "Added observation factor for fiducial %d", fiducial_id);
    }
  }
  
  void initializeNewFiducial(int fiducial_id, const Pose3& camera_to_fiducial, 
                           const rclcpp::Time& stamp) {
    // Only try to initialize if we have received odometry and have recent transforms
    if (!has_received_odom_) {
      RCLCPP_DEBUG(this->get_logger(), "Skipping fiducial initialization - no odometry received yet");
      return;
    }
    
    // Get robot pose in odom frame at the time of observation
    try {
      // Use current time if the requested time is too old
      rclcpp::Time lookup_time = stamp;
      rclcpp::Time current_time = now();
      
      // If the stamp is too old, use current time
      if ((current_time - stamp).nanoseconds() > 1000000000LL) {  // 1 second
        lookup_time = current_time;
        RCLCPP_DEBUG(this->get_logger(), "Using current time for transform lookup");
      }
      
      // Check if transform is available before attempting lookup
      if (!tf_buffer_->canTransform(odom_frame_, base_frame_, lookup_time, rclcpp::Duration::from_seconds(0.1))) {
        RCLCPP_DEBUG(this->get_logger(), "Transform not available yet for fiducial initialization");
        return;
      }
      
      auto robot_in_odom = tf_buffer_->lookupTransform(
        odom_frame_, base_frame_, lookup_time, rclcpp::Duration::from_seconds(0.1));
      
      Pose3 robot_pose = rosToPose3(robot_in_odom.transform);
      
      // Calculate fiducial position in odom frame
      // fiducial_in_odom = robot_in_odom * camera_to_fiducial
      Pose3 fiducial_pose = robot_pose * camera_to_fiducial;
      
      Symbol fiducial_key = Symbol('l', fiducial_id);
      initial_estimate_.insert(fiducial_key, fiducial_pose);
      known_fiducials_.insert(fiducial_id);
      
      RCLCPP_INFO(this->get_logger(), "Initialized new fiducial %d at position [%.2f, %.2f, %.2f]",
                  fiducial_id, fiducial_pose.x(), fiducial_pose.y(), fiducial_pose.z());
                  
    } catch (const tf2::TransformException& ex) {
      RCLCPP_DEBUG(this->get_logger(), "Could not get robot transform for fiducial initialization: %s", ex.what());
    } catch (const std::exception& ex) {
      RCLCPP_WARN(this->get_logger(), "Exception in fiducial initialization: %s", ex.what());
    }
  }
  
  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(graph_mutex_);
    
    if (last_odom_time_.nanoseconds() == 0) {
      last_odom_time_ = msg->header.stamp;
      last_odom_pose_ = rosToPose3(msg->pose.pose);
      has_received_odom_ = true;  // Mark that we've received odometry data
      return;
    }
    
    // Calculate odometry delta
    Pose3 current_odom_pose = rosToPose3(msg->pose.pose);
    Pose3 odom_delta = last_odom_pose_.inverse() * current_odom_pose;
    
    // Add new robot pose
    Symbol new_robot_key = Symbol('x', pose_counter_++);
    
    // Add odometry factor
    graph_.add(BetweenFactor<Pose3>(current_robot_key_, new_robot_key, 
                                    odom_delta, odometry_noise_));
    
    // Initialize new pose estimate
    if (current_estimate_.exists(current_robot_key_)) {
      Pose3 prev_pose = current_estimate_.at<Pose3>(current_robot_key_);
      initial_estimate_.insert(new_robot_key, prev_pose * odom_delta);
    } else {
      initial_estimate_.insert(new_robot_key, current_odom_pose);
    }
    
    current_robot_key_ = new_robot_key;
    last_odom_time_ = msg->header.stamp;
    last_odom_pose_ = current_odom_pose;
  }
  
  void optimizationCallback() {
    std::lock_guard<std::mutex> lock(graph_mutex_);
    
    RCLCPP_DEBUG(this->get_logger(), "Optimization callback started");
    
    // Only optimize if we have received at least one odometry message and have sufficient data
    if (graph_.empty() || initial_estimate_.empty() || !has_received_odom_) {
      RCLCPP_DEBUG(this->get_logger(), "Skipping optimization - insufficient data");
      return;
    }
    
    // Need at least one factor to optimize
    if (graph_.size() < 1) {
      RCLCPP_DEBUG(this->get_logger(), "Skipping optimization - not enough factors");
      return;
    }
    
    try {
      RCLCPP_DEBUG(this->get_logger(), "Starting optimization with %d factors", 
                   static_cast<int>(graph_.size()));
      
      // Merge initial estimate with current estimate for optimization
      Values combined_estimate = current_estimate_;
      
      for (const auto& key_value : initial_estimate_) {
        if (!combined_estimate.exists(key_value.key)) {
          combined_estimate.insert(key_value.key, key_value.value);
        }
      }
      
      // Check that we have estimates for all keys in the graph
      auto graph_keys = graph_.keys();
      
      for (const auto& key : graph_keys) {
        if (!combined_estimate.exists(key)) {
          RCLCPP_WARN(this->get_logger(), "Missing estimate for key %lu, skipping optimization", key);
          return;
        }
      }
      
      RCLCPP_DEBUG(this->get_logger(), "Combined estimate created with %d values", 
                   static_cast<int>(combined_estimate.size()));
      
      // Only optimize if we have meaningful data
      if (combined_estimate.size() < 2) {
        RCLCPP_DEBUG(this->get_logger(), "Skipping optimization - not enough estimates");
        return;
      }
      
      // Optimize the factor graph
      LevenbergMarquardtOptimizer optimizer(graph_, combined_estimate);
      Values optimized_result = optimizer.optimize();
      
      // Only update if optimization succeeded
      if (!optimized_result.empty()) {
        current_estimate_ = optimized_result;
        
        RCLCPP_DEBUG(this->get_logger(), "Optimization completed successfully");
        
        // Publish current robot pose
        publishRobotPose();
        
        // Publish fiducial markers and map
        publishFiducialMarkers();
        publishFiducialMap();
        
        // Clear initial estimate for next iteration
        initial_estimate_.clear();
      } else {
        RCLCPP_WARN(this->get_logger(), "Optimization returned empty result");
      }
      
    } catch (const std::exception& e) {
      RCLCPP_WARN(this->get_logger(), "Optimization failed: %s", e.what());
    }
  }
  
  void publishRobotPose() {
    if (!current_estimate_.exists(current_robot_key_)) {
      RCLCPP_DEBUG(this->get_logger(), "Robot pose not available in current estimate");
      return;
    }
    
    try {
      Pose3 robot_pose = current_estimate_.at<Pose3>(current_robot_key_);
      
      // Convert to ROS message
      auto pose_msg = std::make_unique<geometry_msgs::msg::PoseWithCovarianceStamped>();
      pose_msg->header.stamp = now();
      pose_msg->header.frame_id = map_frame_;
      
      pose_msg->pose.pose = pose3ToRos(robot_pose);
      
      // Set covariance (simplified)
      for (int i = 0; i < 36; ++i) {
        pose_msg->pose.covariance[i] = (i % 7 == 0) ? 0.1 : 0.0;
      }
      
      // Store timestamp before moving the message
      rclcpp::Time msg_stamp = pose_msg->header.stamp;
      
      pose_pub_->publish(std::move(pose_msg));
      
      // Publish map -> odom transform
      publishMapToOdomTransform(robot_pose, msg_stamp);
      
    } catch (const std::exception& e) {
      RCLCPP_WARN(this->get_logger(), "Failed to publish robot pose: %s", e.what());
    }
  }
  
  void publishFiducialMarkers() {
    for (int fiducial_id : known_fiducials_) {
      Symbol fiducial_key = Symbol('l', fiducial_id);
      if (current_estimate_.exists(fiducial_key)) {
        Pose3 fiducial_pose = current_estimate_.at<Pose3>(fiducial_key);
        publishFiducialMarker(fiducial_id, fiducial_pose);
      }
    }
  }
  
  void publishFiducialMarker(int fiducial_id, const Pose3& pose) {
    // Basic fiducial marker (cube)
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = map_frame_;
    marker.header.stamp = now();
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.id = fiducial_id;
    marker.ns = "fiducial";
    
    marker.pose = pose3ToRos(pose);
    marker.scale.x = 0.15;
    marker.scale.y = 0.15;
    marker.scale.z = 0.01;
    
    // Color: red for visible, gray for not currently visible
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;
    
    marker_pub_->publish(marker);
    
    // Text label
    visualization_msgs::msg::Marker text;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;
    text.header.frame_id = map_frame_;
    text.header.stamp = now();
    text.color.r = text.color.g = text.color.b = text.color.a = 1.0f;
    text.id = fiducial_id + 30000;
    text.scale.x = text.scale.y = text.scale.z = 0.1;
    text.pose = pose3ToRos(pose);
    text.pose.position.z += 0.2;  // Offset text above marker
    text.ns = "text";
    text.text = std::to_string(fiducial_id);
    
    marker_pub_->publish(text);
  }
  
  void publishFiducialMap() {
    auto msg = std::make_unique<fiducial_msgs::msg::FiducialMapEntryArray>();
    
    for (int fiducial_id : known_fiducials_) {
      Symbol fiducial_key = Symbol('l', fiducial_id);
      if (current_estimate_.exists(fiducial_key)) {
        Pose3 fiducial_pose = current_estimate_.at<Pose3>(fiducial_key);
        
        fiducial_msgs::msg::FiducialMapEntry entry;
        entry.fiducial_id = fiducial_id;
        entry.x = fiducial_pose.x();
        entry.y = fiducial_pose.y();
        entry.z = fiducial_pose.z();
        
        // Convert rotation to RPY
        Vector3 rpy = fiducial_pose.rotation().rpy();
        entry.rx = rpy(0);
        entry.ry = rpy(1);
        entry.rz = rpy(2);
        
        msg->fiducials.push_back(entry);
      }
    }
    
    map_pub_->publish(std::move(msg));
  }
  
  void publishMapToOdomTransform(const Pose3& robot_pose_in_map, const rclcpp::Time& stamp) {
    try {
      // Check if transform is available before attempting lookup
      if (!tf_buffer_->canTransform(odom_frame_, base_frame_, stamp, rclcpp::Duration::from_seconds(0.1))) {
        RCLCPP_DEBUG(this->get_logger(), "Cannot publish map->odom transform: odom->base_footprint not available");
        return;
      }
      
      // Get the current robot pose in odom frame
      auto robot_in_odom = tf_buffer_->lookupTransform(
        odom_frame_, base_frame_, stamp, rclcpp::Duration::from_seconds(0.1));
      
      Pose3 robot_pose_in_odom = rosToPose3(robot_in_odom.transform);
      
      // Calculate map -> odom transform
      // robot_in_map = map_to_odom * robot_in_odom
      // So: map_to_odom = robot_in_map * robot_in_odom.inverse()
      Pose3 map_to_odom = robot_pose_in_map * robot_pose_in_odom.inverse();
      
      // Create and publish the transform
      geometry_msgs::msg::TransformStamped map_to_odom_msg;
      map_to_odom_msg.header.stamp = stamp;
      map_to_odom_msg.header.frame_id = map_frame_;
      map_to_odom_msg.child_frame_id = odom_frame_;
      
      map_to_odom_msg.transform.translation.x = map_to_odom.x();
      map_to_odom_msg.transform.translation.y = map_to_odom.y();
      map_to_odom_msg.transform.translation.z = map_to_odom.z();
      
      auto quat = map_to_odom.rotation().toQuaternion();
      map_to_odom_msg.transform.rotation.w = quat.w();
      map_to_odom_msg.transform.rotation.x = quat.x();
      map_to_odom_msg.transform.rotation.y = quat.y();
      map_to_odom_msg.transform.rotation.z = quat.z();
      
      tf_broadcaster_->sendTransform(map_to_odom_msg);
      
      RCLCPP_DEBUG(this->get_logger(), "Published map->odom transform");
      
    } catch (const tf2::TransformException& ex) {
      RCLCPP_DEBUG(this->get_logger(), "Could not publish map->odom transform: %s", ex.what());
    } catch (const std::exception& ex) {
      RCLCPP_WARN(this->get_logger(), "Exception in transform publishing: %s", ex.what());
    }
  }
  
  void loadFiducialMap() {
    std::ifstream file(fiducial_map_file_);
    if (!file.is_open()) {
      RCLCPP_WARN(this->get_logger(), "Could not open fiducial map file: %s", fiducial_map_file_.c_str());
      return;
    }
    
    std::string line;
    while (std::getline(file, line)) {
      std::istringstream iss(line);
      int fiducial_id;
      double x, y, z, rx, ry, rz;
      
      if (iss >> fiducial_id >> x >> y >> z >> rx >> ry >> rz) {
        // Create pose from position and RPY
        Pose3 fiducial_pose = Pose3(Rot3::RzRyRx(rz, ry, rx), Point3(x, y, z));
        
        Symbol fiducial_key = Symbol('l', fiducial_id);
        initial_estimate_.insert(fiducial_key, fiducial_pose);
        known_fiducials_.insert(fiducial_id);
        
        RCLCPP_INFO(this->get_logger(), "Loaded fiducial %d from map file", fiducial_id);
      }
    }
    
    file.close();
    RCLCPP_INFO(this->get_logger(), "Loaded %zu fiducials from map file", known_fiducials_.size());
  }
  
  void saveFiducialMap() {
    std::ofstream file(fiducial_map_file_);
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Could not open fiducial map file for writing: %s", fiducial_map_file_.c_str());
      return;
    }
    
    for (int fiducial_id : known_fiducials_) {
      Symbol fiducial_key = Symbol('l', fiducial_id);
      if (current_estimate_.exists(fiducial_key)) {
        Pose3 fiducial_pose = current_estimate_.at<Pose3>(fiducial_key);
        
        // Convert rotation to RPY
        Vector3 rpy = fiducial_pose.rotation().rpy();
        
        file << fiducial_id << " " 
             << fiducial_pose.x() << " " 
             << fiducial_pose.y() << " " 
             << fiducial_pose.z() << " "
             << rpy(0) << " " 
             << rpy(1) << " " 
             << rpy(2) << std::endl;
      }
    }
    
    file.close();
    RCLCPP_INFO(this->get_logger(), "Saved %zu fiducials to map file", known_fiducials_.size());
  }

  // Utility functions for coordinate conversions
  Pose3 rosToPose3(const geometry_msgs::msg::Transform& transform) {
    Point3 position(transform.translation.x, transform.translation.y, transform.translation.z);
    Quaternion quaternion(transform.rotation.w, transform.rotation.x, 
                         transform.rotation.y, transform.rotation.z);
    return Pose3(Rot3(quaternion), position);
  }
  
  Pose3 rosToPose3(const geometry_msgs::msg::Pose& pose) {
    Point3 position(pose.position.x, pose.position.y, pose.position.z);
    Quaternion quaternion(pose.orientation.w, pose.orientation.x, 
                         pose.orientation.y, pose.orientation.z);
    return Pose3(Rot3(quaternion), position);
  }
  
  geometry_msgs::msg::Pose pose3ToRos(const Pose3& pose) {
    geometry_msgs::msg::Pose ros_pose;
    ros_pose.position.x = pose.x();
    ros_pose.position.y = pose.y();
    ros_pose.position.z = pose.z();
    
    auto quat = pose.rotation().toQuaternion();
    ros_pose.orientation.w = quat.w();
    ros_pose.orientation.x = quat.x();
    ros_pose.orientation.y = quat.y();
    ros_pose.orientation.z = quat.z();
    
    return ros_pose;
  }
  
  // Member variables
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string camera_frame_;
  bool mapping_mode_;
  std::string fiducial_map_file_;
  double optimization_frequency_;
  
  // TF components
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  
  // GTSAM components
  NonlinearFactorGraph graph_;
  Values initial_estimate_;
  Values current_estimate_;
  std::mutex graph_mutex_;
  
  // Noise models
  noiseModel::Diagonal::shared_ptr odometry_noise_;
  noiseModel::Diagonal::shared_ptr fiducial_noise_;
  noiseModel::Diagonal::shared_ptr prior_noise_;
  
  // State tracking
  Symbol current_robot_key_;
  int pose_counter_;
  std::set<int> known_fiducials_;
  
  // Odometry tracking
  rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};
  Pose3 last_odom_pose_;
  rclcpp::Time last_optimization_time_;
  bool has_received_odom_ = false;
  
  // ROS components
  rclcpp::Subscription<fiducial_msgs::msg::FiducialTransformArray>::SharedPtr fiducial_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<fiducial_msgs::msg::FiducialMapEntryArray>::SharedPtr map_pub_;
  rclcpp::TimerBase::SharedPtr optimization_timer_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FSlamNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
