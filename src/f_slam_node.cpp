// f_slam_node.cpp
// GTSAM-based fiducial SLAM implementation

#include <boost/make_shared.hpp>
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
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/linear/NoiseModel.h>
// Add this include for custom factors
#include <gtsam/nonlinear/NonlinearFactor.h>

// Custom factor to constrain roll and pitch
class RollPitchConstraintFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
private:
  double max_roll_pitch_; // Maximum allowed roll/pitch in radians

public:
  RollPitchConstraintFactor(gtsam::Key key, double max_roll_pitch_degrees, 
                           const gtsam::SharedNoiseModel& noise_model)
    : NoiseModelFactor1<gtsam::Pose3>(noise_model, key),
      max_roll_pitch_(max_roll_pitch_degrees * M_PI / 180.0) {}

  gtsam::Vector evaluateError(const gtsam::Pose3& pose,
                             boost::optional<gtsam::Matrix&> H = boost::none) const override {
    // Extract roll, pitch, yaw from the pose
    gtsam::Vector3 rpy = pose.rotation().rpy();
    double roll = rpy(0);
    double pitch = rpy(1);
    
    // Create error vector - penalize roll and pitch when they exceed limits
    gtsam::Vector2 error;
    
    // Soft constraint using exponential penalty beyond the limit
    if (std::abs(roll) > max_roll_pitch_) {
      error(0) = std::abs(roll) - max_roll_pitch_;
      if (roll < 0) error(0) = -error(0);
    } else {
      error(0) = 0.0;
    }
    
    if (std::abs(pitch) > max_roll_pitch_) {
      error(1) = std::abs(pitch) - max_roll_pitch_;
      if (pitch < 0) error(1) = -error(1);
    } else {
      error(1) = 0.0;
    }
    
    // Compute Jacobian if requested
    if (H) {
      // Simplified Jacobian - you might want to compute the exact one
      *H = gtsam::Matrix::Zero(2, 6);
      (*H)(0, 3) = 1.0; // d(error_roll)/d(roll)
      (*H)(1, 4) = 1.0; // d(error_pitch)/d(pitch)
    }
    
    return error;
  }
};

// Custom factor to constrain Z position relative to initial height
class ZConstraintFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
private:
  double reference_z_;     // Reference Z position (typically initial robot Z)
  double max_z_deviation_; // Maximum allowed deviation in meters

public:
  ZConstraintFactor(gtsam::Key key, double reference_z, double max_z_deviation_meters, 
                   const gtsam::SharedNoiseModel& noise_model)
    : NoiseModelFactor1<gtsam::Pose3>(noise_model, key),
      reference_z_(reference_z),
      max_z_deviation_(max_z_deviation_meters) {}

  gtsam::Vector evaluateError(const gtsam::Pose3& pose,
                             boost::optional<gtsam::Matrix&> H = boost::none) const override {
    double current_z = pose.z();
    double z_error = current_z - reference_z_;
    
    // Create error vector - penalize Z when it exceeds limits
    gtsam::Vector1 error;
    
    // Soft constraint using penalty beyond the limit
    if (std::abs(z_error) > max_z_deviation_) {
      error(0) = std::abs(z_error) - max_z_deviation_;
      if (z_error < 0) error(0) = -error(0);
    } else {
      error(0) = 0.0;
    }
    
    // Compute Jacobian if requested
    if (H) {
      *H = gtsam::Matrix::Zero(1, 6);
      (*H)(0, 2) = 1.0; // d(error_z)/d(z)
    }
    
    return error;
  }
};

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
    declare_parameter("optimization_frequency", 1.0);
    declare_parameter("max_roll_pitch_degrees", 2.0);
    declare_parameter("max_z_deviation_meters", 0.01);
    declare_parameter("fiducial_position_noise", 0.1);
    declare_parameter("fiducial_rotation_noise", 0.2);
    declare_parameter("fiducial_prior_position_noise", 0.05);
    declare_parameter("fiducial_prior_rotation_noise", 0.1);
    declare_parameter("min_observations_for_init", 3);  // Require multiple observations before trusting
    
    // Get parameters
    map_frame_ = get_parameter("map_frame").as_string();
    odom_frame_ = get_parameter("odom_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    camera_frame_ = get_parameter("camera_frame").as_string();
    mapping_mode_ = get_parameter("mapping_mode").as_bool();
    fiducial_map_file_ = get_parameter("fiducial_map_file").as_string();
    optimization_frequency_ = get_parameter("optimization_frequency").as_double();
    max_roll_pitch_degrees_ = get_parameter("max_roll_pitch_degrees").as_double();
    max_z_deviation_meters_ = get_parameter("max_z_deviation_meters").as_double();
    double fiducial_pos_noise = get_parameter("fiducial_position_noise").as_double();
    double fiducial_rot_noise = get_parameter("fiducial_rotation_noise").as_double();
    double fiducial_prior_pos_noise = get_parameter("fiducial_prior_position_noise").as_double();
    double fiducial_prior_rot_noise = get_parameter("fiducial_prior_rotation_noise").as_double();
    min_observations_for_init_ = get_parameter("min_observations_for_init").as_int();
    
    // Store noise parameters
    fiducial_pos_noise_ = fiducial_pos_noise;
    fiducial_rot_noise_ = fiducial_rot_noise;
    fiducial_prior_pos_noise_ = fiducial_prior_pos_noise;
    fiducial_prior_rot_noise_ = fiducial_prior_rot_noise;
    
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
    // Create noise models - tightened for better stability
    // Odometry noise (x, y, z, roll, pitch, yaw)
    auto odom_noise_vector = (Vector(6) << 0.05, 0.05, 0.05, 0.05, 0.05, 0.05).finished();
    odometry_noise_ = noiseModel::Diagonal::Variances(odom_noise_vector);
    
    // Fiducial observation noise - use configurable parameters
    auto fiducial_noise_vector = (Vector(6) << 
      fiducial_pos_noise_, fiducial_pos_noise_, fiducial_pos_noise_,
      fiducial_rot_noise_, fiducial_rot_noise_, fiducial_rot_noise_).finished();
    fiducial_noise_ = noiseModel::Diagonal::Variances(fiducial_noise_vector);
    
    // Prior noise for first pose - use loose noise until map is initialized
    auto prior_noise_vector = (Vector(6) << 1.0, 1.0, 1.0, 0.1, 0.1, 1.0).finished();
    prior_noise_ = noiseModel::Diagonal::Variances(prior_noise_vector);
    
    // Fiducial anchor noise - use configurable parameters
    auto fiducial_prior_vector = (Vector(6) << 
      fiducial_prior_pos_noise_, fiducial_prior_pos_noise_, fiducial_prior_pos_noise_,
      fiducial_prior_rot_noise_, fiducial_prior_rot_noise_, fiducial_prior_rot_noise_).finished();
    fiducial_prior_noise_ = noiseModel::Diagonal::Variances(fiducial_prior_vector);
    
    // Roll/pitch constraint noise - strong constraint
    auto roll_pitch_noise_vector = (Vector(2) << 0.1, 0.1).finished();
    roll_pitch_constraint_noise_ = noiseModel::Diagonal::Variances(roll_pitch_noise_vector);
    
    // Z constraint noise - strong constraint for flat surface
    auto z_constraint_noise_vector = (Vector(1) << 0.01).finished();
    z_constraint_noise_ = noiseModel::Diagonal::Variances(z_constraint_noise_vector);
    
    // Initialize first robot pose - don't fix it until we see the first fiducial
    current_robot_key_ = Symbol('x', 0);
    pose_counter_ = 1;
    last_optimization_time_ = now();
    map_initialized_ = false;
    
    // Don't add the robot pose to the graph yet - wait for first fiducial observation
    // This allows the map origin to be established at the robot's location when it first sees a landmark
    
    RCLCPP_INFO(this->get_logger(), "GTSAM initialized - waiting for first fiducial observation to establish map origin");
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
    
    // Track observations for each fiducial
    fiducial_observations_[fiducial_id]++;
    
    // If the map hasn't been initialized yet, we can't process observations
    if (!map_initialized_) {
      // Check if this is a new fiducial that we should initialize the map with
      if (mapping_mode_ && known_fiducials_.find(fiducial_id) == known_fiducials_.end()) {
        if (fiducial_observations_[fiducial_id] >= min_observations_for_init_) {
          initializeNewFiducial(fiducial_id, camera_to_fiducial, header.stamp);
        } else {
          RCLCPP_DEBUG(this->get_logger(), "Fiducial %d seen %d/%d times before map initialization", 
                       fiducial_id, fiducial_observations_[fiducial_id], min_observations_for_init_);
        }
      }
      return;  // Don't process observations until map is initialized
    }
    
    // If this is a new fiducial and we're in mapping mode
    if (mapping_mode_ && known_fiducials_.find(fiducial_id) == known_fiducials_.end()) {
      // Only initialize after seeing the fiducial multiple times to reduce noise impact
      if (fiducial_observations_[fiducial_id] >= min_observations_for_init_) {
        initializeNewFiducial(fiducial_id, camera_to_fiducial, header.stamp);
      } else {
        RCLCPP_DEBUG(this->get_logger(), "Fiducial %d seen %d/%d times before initialization", 
                     fiducial_id, fiducial_observations_[fiducial_id], min_observations_for_init_);
        return;  // Don't add observation factor yet
      }
    }
    
    // Add observation factor if fiducial is known
    if (known_fiducials_.find(fiducial_id) != known_fiducials_.end()) {
      // Get transform from base_link to camera
      Pose3 base_to_camera = Pose3::Identity();
      
      try {
        if (tf_buffer_->canTransform(base_frame_, camera_frame_, rclcpp::Time(0), rclcpp::Duration::from_seconds(0.1))) {
          auto base_to_camera_tf = tf_buffer_->lookupTransform(
            base_frame_, camera_frame_, rclcpp::Time(0), rclcpp::Duration::from_seconds(0.1));
          base_to_camera = rosToPose3(base_to_camera_tf.transform);
        }
      } catch (const tf2::TransformException& ex) {
        RCLCPP_DEBUG(this->get_logger(), "Using identity for base->camera transform: %s", ex.what());
      }
      
      // Adaptive noise based on distance (farther detections are noisier)
      double distance = camera_to_fiducial.translation().norm();
      // Scale factor to achieve ~0.3m noise at 3m distance (0.3/0.1 = 3.0 factor at 3m)
      double distance_factor = std::max(1.0, 1.0 + (distance - 1.0) * 0.67);  // Linear scaling from 1.0 at 1m to 3.0 at 3m
      
      // Create adaptive noise model
      auto adaptive_noise_vector = (Vector(6) << 
        fiducial_pos_noise_ * distance_factor, 
        fiducial_pos_noise_ * distance_factor, 
        fiducial_pos_noise_ * distance_factor,
        fiducial_rot_noise_ * distance_factor, 
        fiducial_rot_noise_ * distance_factor, 
        fiducial_rot_noise_ * distance_factor).finished();
      auto adaptive_noise = noiseModel::Diagonal::Variances(adaptive_noise_vector);
      
      Pose3 base_to_fiducial = base_to_camera * camera_to_fiducial;
      
      // Add factor with adaptive noise
      graph_.add(BetweenFactor<Pose3>(current_robot_key_, fiducial_key, 
                                      base_to_fiducial, adaptive_noise));
      
      RCLCPP_DEBUG(this->get_logger(), "Added observation factor for fiducial %d (distance: %.2f, noise factor: %.2f)", 
                   fiducial_id, distance, distance_factor);
    }
  }
  
  void addRollPitchConstraint(const Symbol& robot_key) {
    auto constraint = boost::make_shared<RollPitchConstraintFactor>(
      robot_key, max_roll_pitch_degrees_, roll_pitch_constraint_noise_);
    graph_.add(constraint);
    
    RCLCPP_DEBUG(this->get_logger(), "Added roll/pitch constraint for pose %zu", static_cast<size_t>(robot_key));
  }
  
  void addZConstraint(const Symbol& robot_key) {
    auto constraint = boost::make_shared<ZConstraintFactor>(
      robot_key, reference_z_, max_z_deviation_meters_, z_constraint_noise_);
    graph_.add(constraint);
    
    RCLCPP_DEBUG(this->get_logger(), "Added Z constraint for pose %zu (ref: %.3f, max dev: %.3f)", 
                 static_cast<size_t>(robot_key), reference_z_, max_z_deviation_meters_);
  }
  
  void initializeNewFiducial(int fiducial_id, const Pose3& camera_to_fiducial, 
                           const rclcpp::Time& /* stamp */) {
    try {
      // If this is the first fiducial we're seeing, establish the map origin at the current robot location
      if (!map_initialized_) {
        initializeMapOrigin(fiducial_id, camera_to_fiducial);
        return;
      }
      
      // For subsequent fiducials, initialize them relative to the current robot pose estimate
      Pose3 robot_pose_in_map;
      
      if (current_estimate_.exists(current_robot_key_)) {
        // Use the current optimized robot pose
        robot_pose_in_map = current_estimate_.at<Pose3>(current_robot_key_);
      } else if (initial_estimate_.exists(current_robot_key_)) {
        // Fall back to initial estimate if not optimized yet
        robot_pose_in_map = initial_estimate_.at<Pose3>(current_robot_key_);
      } else {
        RCLCPP_WARN(this->get_logger(), "Robot pose not available for fiducial initialization, using last odometry");
        robot_pose_in_map = last_odom_pose_;
      }
      
      // Get camera pose relative to base_link
      Pose3 base_to_camera = Pose3::Identity();
      
      try {
        if (tf_buffer_->canTransform(base_frame_, camera_frame_, rclcpp::Time(0), rclcpp::Duration::from_seconds(0.1))) {
          auto base_to_camera_tf = tf_buffer_->lookupTransform(
            base_frame_, camera_frame_, rclcpp::Time(0), rclcpp::Duration::from_seconds(0.1));
          base_to_camera = rosToPose3(base_to_camera_tf.transform);
        }
      } catch (const tf2::TransformException& ex) {
        RCLCPP_DEBUG(this->get_logger(), "Using identity for base->camera transform: %s", ex.what());
      }
      
      // Calculate fiducial position in map frame
      Pose3 fiducial_pose = robot_pose_in_map * base_to_camera * camera_to_fiducial;
      
      Symbol fiducial_key = Symbol('l', fiducial_id);
      initial_estimate_.insert(fiducial_key, fiducial_pose);
      known_fiducials_.insert(fiducial_id);
      
      RCLCPP_INFO(this->get_logger(), "Initialized new fiducial %d at position [%.2f, %.2f, %.2f]",
                  fiducial_id, fiducial_pose.x(), fiducial_pose.y(), fiducial_pose.z());
                  
    } catch (const std::exception& ex) {
      RCLCPP_WARN(this->get_logger(), "Exception in fiducial initialization: %s", ex.what());
    }
  }
  
  void initializeMapOrigin(int first_fiducial_id, const Pose3& camera_to_fiducial) {
    RCLCPP_INFO(this->get_logger(), "Initializing map origin with first fiducial %d", first_fiducial_id);
    
    // Get the current robot pose from odometry if available
    Pose3 robot_pose_at_map_origin = Pose3::Identity();
    
    if (has_received_odom_) {
      robot_pose_at_map_origin = last_odom_pose_;
      RCLCPP_INFO(this->get_logger(), "Using odometry pose as map origin: [%.2f, %.2f, %.2f]", 
                  robot_pose_at_map_origin.x(), robot_pose_at_map_origin.y(), robot_pose_at_map_origin.z());
    } else {
      RCLCPP_INFO(this->get_logger(), "No odometry available, using identity as map origin");
    }
    
    // Store reference Z for constraints
    reference_z_ = robot_pose_at_map_origin.z();
    
    // Add the robot pose to the graph with a strong prior (this establishes the map origin)
    auto tight_prior_noise = noiseModel::Diagonal::Variances(
      (Vector(6) << 0.001, 0.001, 0.001, 0.001, 0.001, 0.001).finished());
    graph_.add(PriorFactor<Pose3>(current_robot_key_, robot_pose_at_map_origin, tight_prior_noise));
    current_estimate_.insert(current_robot_key_, robot_pose_at_map_origin);
    
    // Add constraints for the robot pose
    addRollPitchConstraint(current_robot_key_);
    addZConstraint(current_robot_key_);
    
    // Get camera pose relative to base_link
    Pose3 base_to_camera = Pose3::Identity();
    
    try {
      if (tf_buffer_->canTransform(base_frame_, camera_frame_, rclcpp::Time(0), rclcpp::Duration::from_seconds(0.1))) {
        auto base_to_camera_tf = tf_buffer_->lookupTransform(
          base_frame_, camera_frame_, rclcpp::Time(0), rclcpp::Duration::from_seconds(0.1));
        base_to_camera = rosToPose3(base_to_camera_tf.transform);
      }
    } catch (const tf2::TransformException& ex) {
      RCLCPP_DEBUG(this->get_logger(), "Using identity for base->camera transform: %s", ex.what());
    }
    
    // Calculate first fiducial position in the new map frame
    Pose3 fiducial_pose = robot_pose_at_map_origin * base_to_camera * camera_to_fiducial;
    
    Symbol fiducial_key = Symbol('l', first_fiducial_id);
    initial_estimate_.insert(fiducial_key, fiducial_pose);
    current_estimate_.insert(fiducial_key, fiducial_pose);
    known_fiducials_.insert(first_fiducial_id);
    
    // Add a strong prior for the first fiducial to anchor the map
    graph_.add(PriorFactor<Pose3>(fiducial_key, fiducial_pose, fiducial_prior_noise_));
    
    map_initialized_ = true;
    
    RCLCPP_INFO(this->get_logger(), "Map origin established! Robot at [%.2f, %.2f, %.2f], first fiducial %d at [%.2f, %.2f, %.2f]",
                robot_pose_at_map_origin.x(), robot_pose_at_map_origin.y(), robot_pose_at_map_origin.z(),
                first_fiducial_id, fiducial_pose.x(), fiducial_pose.y(), fiducial_pose.z());
  }
  
  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(graph_mutex_);
    
    if (last_odom_time_.nanoseconds() == 0) {
      last_odom_time_ = msg->header.stamp;
      last_odom_pose_ = rosToPose3(msg->pose.pose);
      has_received_odom_ = true;
      
      // Update reference Z from first odometry message if available and map not yet initialized
      if (reference_z_ == 0.0) {
        reference_z_ = last_odom_pose_.z();
        RCLCPP_INFO(this->get_logger(), "Set reference Z to %.3f from first odometry", reference_z_);
      }
      return;
    }
    
    // If map hasn't been initialized yet, just track odometry but don't add to graph
    if (!map_initialized_) {
      last_odom_time_ = msg->header.stamp;
      last_odom_pose_ = rosToPose3(msg->pose.pose);
      RCLCPP_DEBUG(this->get_logger(), "Tracking odometry, waiting for map initialization");
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
    
    // Add constraints for the new pose
    addRollPitchConstraint(new_robot_key);
    addZConstraint(new_robot_key);
    
    // Initialize new pose estimate
    Pose3 prev_pose;
    if (current_estimate_.exists(current_robot_key_)) {
      prev_pose = current_estimate_.at<Pose3>(current_robot_key_);
    } else if (initial_estimate_.exists(current_robot_key_)) {
      // Fall back to initial estimate if not in current estimate yet
      prev_pose = initial_estimate_.at<Pose3>(current_robot_key_);
    } else {
      // This shouldn't happen if map is initialized, but handle gracefully
      RCLCPP_WARN(this->get_logger(), "Previous robot pose not found in estimates, using current odometry");
      prev_pose = last_odom_pose_;
    }
    
    initial_estimate_.insert(new_robot_key, prev_pose * odom_delta);
    
    current_robot_key_ = new_robot_key;
    last_odom_time_ = msg->header.stamp;
    last_odom_pose_ = current_odom_pose;
  }
  
  void optimizationCallback() {
    std::lock_guard<std::mutex> lock(graph_mutex_);
    
    RCLCPP_DEBUG(this->get_logger(), "Optimization callback started");
    
    // Only optimize if the map has been initialized
    if (!map_initialized_) {
      RCLCPP_DEBUG(this->get_logger(), "Skipping optimization - map not yet initialized");
      return;
    }
    
    // Only optimize if we have received at least one odometry message and have sufficient data
    if (graph_.empty() || current_estimate_.empty() || !has_received_odom_) {
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
      
      // Only add values from initial_estimate_ that don't already exist in current_estimate_
      for (const auto& key_value : initial_estimate_) {
        if (!combined_estimate.exists(key_value.key)) {
          combined_estimate.insert(key_value.key, key_value.value);
        } else {
          // Update existing values from initial_estimate_ (for new poses)
          combined_estimate.update(key_value.key, key_value.value);
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
      
      // Optimize the factor graph with more conservative settings
      LevenbergMarquardtParams params;
      params.maxIterations = 50;  // Limit iterations to prevent instability
      params.relativeErrorTol = 1e-5;
      params.absoluteErrorTol = 1e-5;
      
      LevenbergMarquardtOptimizer optimizer(graph_, combined_estimate, params);
      Values optimized_result = optimizer.optimize();
      
      // Only update if optimization succeeded and didn't diverge
      if (!optimized_result.empty()) {
        // Check for reasonable pose changes to detect divergence
        bool optimization_stable = true;
        
        if (current_estimate_.exists(current_robot_key_) && optimized_result.exists(current_robot_key_)) {
          Pose3 old_pose = current_estimate_.at<Pose3>(current_robot_key_);
          Pose3 new_pose = optimized_result.at<Pose3>(current_robot_key_);
          
          double position_change = (new_pose.translation() - old_pose.translation()).norm();
          if (position_change > 1.0) {  // More than 1 meter change indicates instability
            RCLCPP_WARN(this->get_logger(), "Large pose change detected (%.2f m), rejecting optimization", position_change);
            optimization_stable = false;
          }
        }
        
        if (optimization_stable) {
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
          RCLCPP_WARN(this->get_logger(), "Optimization rejected due to instability");
        }
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
        
        // In localization mode, add fiducials to current_estimate_ with strong priors
        current_estimate_.insert(fiducial_key, fiducial_pose);
        known_fiducials_.insert(fiducial_id);
        
        // Add strong prior factor to fix these fiducials in place during localization
        // This prevents them from being optimized and causing instabilities
        auto fixed_fiducial_noise = noiseModel::Diagonal::Variances(
          (Vector(6) << 0.001, 0.001, 0.001, 0.001, 0.001, 0.001).finished());
        graph_.add(PriorFactor<Pose3>(fiducial_key, fiducial_pose, fixed_fiducial_noise));
        
        RCLCPP_INFO(this->get_logger(), "Loaded fiducial %d from map file with strong prior", fiducial_id);
      }
    }
    
    file.close();
    
    if (!known_fiducials_.empty()) {
      // In localization mode, the map is considered initialized when we load existing fiducials
      map_initialized_ = true;
      
      // Still need to initialize the robot pose when first odometry/fiducial observation comes in
      // But we can add a loose prior for the robot at origin until then
      Pose3 initial_pose = Pose3::Identity();
      current_robot_key_ = Symbol('x', 0);
      
      // Add loose prior for robot pose - will be updated when first observation comes in
      auto loose_prior_noise = noiseModel::Diagonal::Variances(
        (Vector(6) << 1.0, 1.0, 1.0, 0.1, 0.1, 1.0).finished());
      graph_.add(PriorFactor<Pose3>(current_robot_key_, initial_pose, loose_prior_noise));
      current_estimate_.insert(current_robot_key_, initial_pose);
      
      // Add constraints for the initial pose
      reference_z_ = initial_pose.z();
      addRollPitchConstraint(current_robot_key_);
      addZConstraint(current_robot_key_);
      
      RCLCPP_INFO(this->get_logger(), "Loaded %zu fiducials from map file - localization mode ready", known_fiducials_.size());
    } else {
      RCLCPP_WARN(this->get_logger(), "No fiducials loaded from map file");
    }
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
  double max_roll_pitch_degrees_;
  double max_z_deviation_meters_;
  double reference_z_ = 0.0;  // Reference Z height for constraints
  bool map_initialized_ = false;  // Track whether map origin has been established
  
  // Noise parameters
  double fiducial_pos_noise_;
  double fiducial_rot_noise_;
  double fiducial_prior_pos_noise_;
  double fiducial_prior_rot_noise_;
  int min_observations_for_init_;
  
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
  noiseModel::Diagonal::shared_ptr fiducial_prior_noise_;
  noiseModel::Diagonal::shared_ptr roll_pitch_constraint_noise_;
  noiseModel::Diagonal::shared_ptr z_constraint_noise_;
  
  // State tracking
  Symbol current_robot_key_;
  int pose_counter_;
  std::set<int> known_fiducials_;
  std::unordered_map<int, int> fiducial_observations_;  // Track observation count per fiducial
  
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
