/*
 * Copyright (c) 2017-9, Ubiquity Robotics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of the FreeBSD Project.
 *
 */

#include <fiducial_slam_ros2/helpers.hpp>
#include <fiducial_slam_ros2/map.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <string>
#include <cmath>

#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/string.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <filesystem>
#include <cstdio> // For C-style file I/O (fopen, fprintf, etc.)
#include <sstream> // For std::istringstream

// Constructor for observation
Observation::Observation(int fid, const tf2::Stamped<TransformWithVariance> &camFid) {
    this->fid = fid;

    T_camFid = camFid;
    T_fidCam = T_camFid;
    T_fidCam.transform = T_camFid.transform.inverse();
}

// Update a fiducial position in map with a new estimate
void Fiducial::update(const tf2::Stamped<TransformWithVariance> &newPose) {
    pose.update(newPose);
    numObs++;
}

// Create a fiducial from an estimate of its position in the map
Fiducial::Fiducial(int id, const tf2::Stamped<TransformWithVariance> &pose) {
    this->id = id;
    this->pose = pose;
    this->lastPublished = rclcpp::Time(0);
    this->numObs = 0;
    this->visible = false;
}

// Constructor for map
Map::Map(rclcpp::Node::SharedPtr node) : node(node), logger(node->get_logger()) {
    frameNum = 0;
    initialFrameNum = 0;
    originFid = -1;
    isInitializingMap = false;
    havePose = false;
    fiducialToAdd = -1;

    // Initialize previous camera pose
    previousCameraPose.variance = 0.0;
    
    tfBuffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());

    listener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer, node); // Explicitly pass the node

    broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);

    robotPosePub = node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/fiducial_pose", 1);
    cameraPosePub = node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/fiducial_slam/camera_pose", 1);

    markerPub = node->create_publisher<visualization_msgs::msg::Marker>("/fiducials", 100);
    mapPub = node->create_publisher<fiducial_msgs::msg::FiducialMapEntryArray>("/fiducial_map", 1);

    clearSrv = node->create_service<std_srvs::srv::Empty>(
        "clear_map",
        std::bind(&Map::clearCallback, this, std::placeholders::_1, std::placeholders::_2));
    addSrv = node->create_service<fiducial_slam_ros2::srv::AddFiducial>(
        "add_fiducial",
        std::bind(&Map::addFiducialCallback, this, std::placeholders::_1, std::placeholders::_2));

    // Retrieve parameters that were declared in the FiducialSlam node
    mapFrame = node->get_parameter("map_frame").as_string();
    odomFrame = node->get_parameter("odom_frame").as_string();
    baseFrame = node->get_parameter("base_frame").as_string(); // Note: ROS1 default was base_footprint, ROS2 is base_link from FiducialSlam node

    tfPublishInterval = node->get_parameter("tf_publish_interval").as_double(); // ROS1 was float
    publishPoseTf = node->get_parameter("publish_tf").as_bool();
    map_systematic_error = node->get_parameter("systematic_error").as_double(); // Use the new member variable
    future_date_transforms = node->get_parameter("future_date_transforms").as_double();
    publish_6dof_pose = node->get_parameter("publish_6dof_pose").as_bool();
    readOnly = node->get_parameter("read_only_map").as_bool();

    // Handle covariance_diagonal string from launch file
    std::string covariance_diagonal_str = node->get_parameter("covariance_diagonal_str").as_string();
    if (!covariance_diagonal_str.empty()) {
        covarianceDiagonal = stringToDoubleVector(covariance_diagonal_str, logger);
        overridePublishedCovariance = !covarianceDiagonal.empty();
        if (overridePublishedCovariance) {
            if (covarianceDiagonal.size() != 6) {
                RCLCPP_WARN(logger, "ignoring covariance_diagonal because it has %ld elements, not 6", covarianceDiagonal.size());
                overridePublishedCovariance = false;
                covarianceDiagonal.clear();
            }
            // Check to make sure that the diagonal is non-zero
            for (auto variance : covarianceDiagonal) {
                if (variance == 0) {
                    RCLCPP_WARN(logger, "ignoring covariance_diagonal because it has 0 values");
                    covarianceDiagonal.clear();
                    overridePublishedCovariance = false;
                    break;
                }
            }
        }
    } else {
        // Fallback to deprecated direct parameter if string is empty
        std::vector<double> temp_covariance;
        temp_covariance = node->get_parameter("covariance_diagonal").as_double_array();
        covarianceDiagonal = temp_covariance;
        overridePublishedCovariance = !covarianceDiagonal.empty();
        
        if (overridePublishedCovariance) {
            if (covarianceDiagonal.size() != 6) {
                RCLCPP_WARN(logger, "ignoring covariance_diagonal because it has %ld elements, not 6", covarianceDiagonal.size());
                overridePublishedCovariance = false;
                covarianceDiagonal.clear();
            }
            // Check to make sure that the diagonal is non-zero
            for (auto variance : covarianceDiagonal) {
                if (variance == 0) {
                    RCLCPP_WARN(logger, "ignoring covariance_diagonal because it has 0 values");
                    covarianceDiagonal.clear();
                    overridePublishedCovariance = false;
                    break;
                }
            }
        }
    }

    // threshold of object error for using multi-fidicial pose
    // set -ve to never use
    multiErrorThreshold = node->get_parameter("multi_error_threshold").as_double(); // Corrected typo from theshold to threshold

    // Get home directory for default map location
    const char* home = std::getenv("HOME");
    std::string home_path = home ? std::string(home) : std::string("/root");
    std::string default_map_path = home_path + "/.ros/slam/map.txt";
    
    mapFilename = node->get_parameter("map_file").as_string();

    std::filesystem::path mapPath(mapFilename);
    std::filesystem::path dir = mapPath.parent_path();
    std::filesystem::create_directories(dir);

    std::string initialMap = node->get_parameter("initial_map_file").as_string();

    if (!initialMap.empty()) {
        loadMap(initialMap);
    } else {
        loadMap();
    }

    publishMarkers();
}

// Update map with a set of observations
void Map::update(std::vector<Observation> &obs, const rclcpp::Time &time) {
    frameNum++;

    if (obs.size() > 0 && fiducials.size() == 0) {
        isInitializingMap = true;
    }

    if (isInitializingMap) {
        autoInit(obs, time);
    } else {
        tf2::Stamped<TransformWithVariance> cameraPose;
        cameraPose.frame_id_ = mapFrame;
        cameraPose.stamp_ = tf2_ros::fromMsg(time);
        cameraPose.variance = 0.0;

        if (updatePose(obs, time, cameraPose) > 0 && obs.size() > 1 && !readOnly) {
            updateMap(obs, time, cameraPose);
            
            // Publish the camera's pose
            geometry_msgs::msg::PoseWithCovarianceStamped cameraPoseMsg;
            cameraPoseMsg.header.frame_id = mapFrame;
            cameraPoseMsg.header.stamp = time; // Use the observation time
            cameraPoseMsg.pose.pose = toPose(cameraPose.transform);
            
            // Convert variance to covariance matrix
            for (int i = 0; i < 36; i++) {
                cameraPoseMsg.pose.covariance[i] = 0;
            }
            cameraPoseMsg.pose.covariance[0] = cameraPoseMsg.pose.covariance[7] = 
                cameraPoseMsg.pose.covariance[14] = 
                cameraPoseMsg.pose.covariance[21] = cameraPoseMsg.pose.covariance[28] = 
                    cameraPoseMsg.pose.covariance[35] = cameraPose.variance;
            
            cameraPosePub->publish(cameraPoseMsg);

            // Compute robot pose
            tf2::Transform T_baseCam;
            if (lookupTransform(baseFrame, obs[0].T_camFid.frame_id_, time, T_baseCam)) {
                tf2::Transform robot_pose_tf = cameraPose.transform * T_baseCam;
                
                // Create and publish robot pose message
                geometry_msgs::msg::PoseWithCovarianceStamped robotPoseMsg;
                robotPoseMsg.header.frame_id = mapFrame;
                robotPoseMsg.header.stamp = time; // Use the observation time
                robotPoseMsg.pose.pose = toPose(robot_pose_tf);
                
                // Set covariance
                for (int i = 0; i < 36; i++) {
                    robotPoseMsg.pose.covariance[i] = 0;
                }
                robotPoseMsg.pose.covariance[0] = robotPoseMsg.pose.covariance[7] = 
                    robotPoseMsg.pose.covariance[14] = 
                    robotPoseMsg.pose.covariance[21] = robotPoseMsg.pose.covariance[28] = 
                        robotPoseMsg.pose.covariance[35] = cameraPose.variance;
                
                if (overridePublishedCovariance) {
                    for (int i = 0; i < 6; i++) {
                        robotPoseMsg.pose.covariance[i * 7] = covarianceDiagonal[i];
                    }
                }

                robotPosePub->publish(robotPoseMsg);

                // TF publishing
                if (publishPoseTf) { 
                    geometry_msgs::msg::TransformStamped tf_msg;
                    // Stamp will be updated before sending by the periodic publishTf or the Map::update() loop
                    tf_msg.header.frame_id = mapFrame;
                    // Default to publishing map -> base_frame
                    tf2::Transform transform_to_publish = robot_pose_tf;
                    std::string child_frame_to_publish = baseFrame;

                    if (!odomFrame.empty()) {
                        tf2::Transform T_odomBase;
                        // Lookup transform from odom to base_frame at the current time
                        if (lookupTransform(odomFrame, baseFrame, time, T_odomBase)) {
                            // Transform map -> base_frame to map -> odom
                            // T_mapOdom = T_mapBase * T_baseOdom (where T_baseOdom is T_odomBase.inverse())
                            transform_to_publish = robot_pose_tf * T_odomBase.inverse();
                            child_frame_to_publish = odomFrame;
                        } else {
                            RCLCPP_WARN(logger, "Could not lookup %s to %s transform. Publishing map->%s instead of map->%s", 
                                        odomFrame.c_str(), baseFrame.c_str(), baseFrame.c_str(), odomFrame.c_str());
                            // Fallback to publishing map->base_frame if odom->base_frame is not available
                        }
                    }

                    // Make outgoing transform make sense - ie only consist of x, y, yaw
                    // This can be disabled via the publish_6dof_pose param, mainly for debugging
                    if (!publish_6dof_pose) {
                        tf2::Vector3 translation = transform_to_publish.getOrigin();
                        translation.setZ(0);
                        transform_to_publish.setOrigin(translation);
                        double roll, pitch, yaw;
                        transform_to_publish.getBasis().getRPY(roll, pitch, yaw);
                        transform_to_publish.getBasis().setRPY(0, 0, yaw);
                    }

                    tf_msg.child_frame_id = child_frame_to_publish;
                    tf_msg.transform = tf2::toMsg(transform_to_publish);
                    
                    poseTf = tf_msg; // Store the latest calculated TF
                    havePose = true;
                }
            }
        }
    }

    handleAddFiducial(obs, time);
    publishMap();
}

// Main update loop, called periodically by the node
void Map::update() {
    // Publish TF periodically if enabled and available
    if (publishPoseTf && havePose) {
        rclcpp::Time now = node->get_clock()->now();
        // Check if tfPublishInterval (converted to nanoseconds) has passed
        if ((now.nanoseconds() - tfPublishTime.nanoseconds()) >= static_cast<int64_t>(tfPublishInterval * 1e9)) {
            // Update timestamp to current time + future_date_transforms before publishing
            poseTf.header.stamp = now + rclcpp::Duration::from_seconds(future_date_transforms);
            broadcaster->sendTransform(poseTf);
            tfPublishTime = now; 
        }
    }
    publishMarkers(); // Continue to publish markers periodically
}

// Implementation for autoInit
void Map::autoInit(const std::vector<Observation> &obs, const rclcpp::Time &time) {
    (void)time; // Mark as unused parameter

    if (obs.empty()) {
        return;
    }

    // Find the fiducial with the lowest ID to use as the origin
    int smallestID = std::numeric_limits<int>::max();
    size_t smallestIdx = 0;

    for (size_t i = 0; i < obs.size(); ++i) {
        const Observation &o_loop = obs[i];
        if (o_loop.fid < smallestID) {
            smallestID = o_loop.fid;
            smallestIdx = i;
        }
    }

    // Use the fiducial with the smallest ID as the origin
    if (originFid == -1) {
        const Observation &o = obs[smallestIdx];
        originFid = o.fid;

        tf2::Stamped<TransformWithVariance> T_mapFid;
        T_mapFid.frame_id_ = mapFrame;
        T_mapFid.stamp_ = tf2_ros::fromMsg(time);

        // Initialize the map with this fiducial at the origin
        TransformWithVariance tv(tf2::Transform(tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(0, 0, 0)), 0.0);
        T_mapFid.setData(tv);
        T_mapFid.variance = 0.0;

        fiducials[o.fid] = Fiducial(o.fid, T_mapFid);

        RCLCPP_INFO(logger, "Created origin at fiducial %d", originFid);

        // If there are additional observations, add them to the map
        if (obs.size() > 1) {
            TransformWithVariance T_fidCamTransform = TransformWithVariance(o.T_fidCam.transform.inverse(), 0.0);
            tf2::Stamped<TransformWithVariance> T_mapCam = createStampedTransform(
                T_fidCamTransform, mapFrame, tf2::TimePointZero);

            // Add other observed fiducials to the map
            for (size_t i = 0; i < obs.size(); i++) {
                if (i != smallestIdx) { // Skip the origin fiducial
                    const Observation &o2 = obs[i];
                    // Create a new stamped transform using the operator* and set its frame and time
                    tf2::Stamped<TransformWithVariance> T_mapFid2 = T_mapCam * o2.T_camFid;
                    T_mapFid2.frame_id_ = mapFrame;
                    T_mapFid2.stamp_ = tf2_ros::fromMsg(time);

                    fiducials[o2.fid] = Fiducial(o2.fid, T_mapFid2);
                    RCLCPP_INFO(logger, "Added initial fiducial %d to map with pose %f %f %f",
                               o2.fid,
                               T_mapFid2.transform.getOrigin().x(),
                               T_mapFid2.transform.getOrigin().y(),
                               T_mapFid2.transform.getOrigin().z());
                }
            }
        }
        isInitializingMap = false;
    } // End if (originFid == -1)
} // End Map::autoInit

// Implementation for updatePose
int Map::updatePose(std::vector<Observation> &obs, const rclcpp::Time &time,
                   tf2::Stamped<TransformWithVariance> &cameraPose) {
    int numEsts = 0;
    tf2::Stamped<TransformWithVariance> T_camBase;
    tf2::Stamped<TransformWithVariance> T_baseCam;
    tf2::Stamped<TransformWithVariance> T_mapBase;

    if (obs.size() == 0) {
        return 0;
    }


    if (lookupTransform(obs[0].T_camFid.frame_id_, baseFrame, time, T_camBase.transform)) {
        tf2::Vector3 c = T_camBase.transform.getOrigin();
        T_camBase.variance = 1.0;
    } else {
        RCLCPP_ERROR(logger, "Cannot determine tf from camera to robot");
        return 0;
    }

    if (lookupTransform(baseFrame, obs[0].T_camFid.frame_id_, time, T_baseCam.transform)) {
        tf2::Vector3 c = T_baseCam.transform.getOrigin();
        T_baseCam.variance = 1.0;
    } else {
        RCLCPP_ERROR(logger, "Cannot determine tf from robot to camera");
        return 0;
    }

    for (Observation &o : obs) {
        if (fiducials.find(o.fid) != fiducials.end()) {
            const Fiducial &fid = fiducials[o.fid];

            // Create proper stamped transform
            tf2::Stamped<TransformWithVariance> p = fid.pose * o.T_fidCam;
            p.frame_id_ = mapFrame;
            p.stamp_ = o.T_fidCam.stamp_;
            
            // Apply the camera->base transform
            TransformWithVariance combined = TransformWithVariance(
                p.transform * T_camBase.transform,
                p.variance + T_camBase.variance
            );
            p.setData(combined);
            auto position = p.transform.getOrigin();
            double roll, pitch, yaw;
            p.transform.getBasis().getRPY(roll, pitch, yaw);

            // Create variance according to how well the robot is upright on the ground
            auto cam_f = o.T_camFid.transform.getOrigin();
            double s1 = std::pow(position.z() / cam_f.z(), 2) *
                        (std::pow(cam_f.x(), 2) + std::pow(cam_f.y(), 2));
            double s2 = position.length2() * std::pow(std::sin(roll), 2);
            double s3 = position.length2() * std::pow(std::sin(pitch), 2);
            p.variance = s1 + s2 + s3 + map_systematic_error; // Use member variable
            o.T_camFid.variance = p.variance;

            // Determine if this is a good estimate
            bool valid = true;
            if (std::isnan(position.x()) || std::isnan(position.y()) || std::isnan(position.z())) {
                valid = false;
                RCLCPP_WARN(logger, "Skipping NAN estimate");
            } else if (position.z() > 10.0) {
                valid = false;
                RCLCPP_WARN(logger, "Skipping estimate with high Z");
            //} else if (std::abs(std::abs(roll) - M_PI) > 1.5 || std::abs(std::abs(pitch) - M_PI) > 1.5) {
              //  valid = false;
              //  RCLCPP_WARN(logger, "Skipping estimate with high roll/pitch");
            } else if (p.variance > 10.0) {
                valid = false;
                RCLCPP_WARN(logger, "Skipping estimate with high variance: %f", p.variance);
            }

            if (valid) {
                // Convert the camera pose into the base pose
                tf2::Stamped<TransformWithVariance> basePose = p;
                numEsts++;

                if (cameraPose.variance == 0.0) {
                    cameraPose = p;
                } else {
                    // Calculate the transform between the current cameraPose and p
                    TransformWithVariance diff = TransformWithVariance(
                        cameraPose.transform.inverse() * p.transform,
                        cameraPose.variance + p.variance
                    );
                    cameraPose.setData(diff);
                    cameraPose.update(p);
                }
            }
        }
    }

    if (numEsts == 0) {
        return 0;
    }

    // Apply temporal smoothing to reduce pose jumps
    if (previousCameraPose.variance > 0.0) {
        double smoothingFactor = 0.0; // Adjust this for more/less smoothing (0.0 = no smoothing, 1.0 = no update)
        
        // Smooth position
        tf2::Vector3 currentPos = cameraPose.transform.getOrigin();
        tf2::Vector3 previousPos = previousCameraPose.transform.getOrigin();
        tf2::Vector3 smoothedPos = previousPos.lerp(currentPos, smoothingFactor);
        
        // Smooth rotation using slerp
        tf2::Quaternion currentRot = cameraPose.transform.getRotation();
        tf2::Quaternion previousRot = previousCameraPose.transform.getRotation();
        tf2::Quaternion smoothedRot = previousRot.slerp(currentRot, smoothingFactor);
        
        // Apply smoothed transform
        cameraPose.transform.setOrigin(smoothedPos);
        cameraPose.transform.setRotation(smoothedRot);
        
        RCLCPP_DEBUG(logger, "Applied pose smoothing");
    }
    
    // Store current pose as previous for next iteration
    previousCameraPose = cameraPose;

    // Publish updated transform and update the robot's position in the map
    return numEsts;
}

// Implementation for updateMap
void Map::updateMap(const std::vector<Observation> &obs, const rclcpp::Time &time,
                   const tf2::Stamped<TransformWithVariance> &cameraPose) {
    (void)time; // Mark as unused
    // Mark all fiducials as not visible in this frame
    for (auto &map_pair : fiducials) {
        Fiducial &f = map_pair.second;
        f.visible = false;
    }

    // Update fiducials in the map
    for (const Observation &o : obs) {
        // This should take into account the variances from both
        // Use the multiplication operator we defined for stamped transforms
        tf2::Stamped<TransformWithVariance> T_mapFid = cameraPose * o.T_camFid;
        T_mapFid.frame_id_ = mapFrame;

        tf2::Vector3 trans = T_mapFid.transform.getOrigin();

        if (std::isnan(trans.x()) || std::isnan(trans.y()) || std::isnan(trans.z())) {
            RCLCPP_WARN(logger, "Skipping NAN estimate");
            continue;
        }

        // If the fiducial is not in the map, add it
        if (fiducials.find(o.fid) == fiducials.end()) {
            fiducials[o.fid] = Fiducial(o.fid, T_mapFid);
        }
        
        // Update the fiducial's position
        Fiducial &f = fiducials[o.fid];
        f.visible = true;
        
        if (f.pose.variance != 0) {
            f.update(T_mapFid);
        }

        // Update links between fiducials
        for (const Observation &observation : obs) {
            int fid = observation.fid;
            if (f.id != fid) {
                f.links.insert(fid);
            }
        }
        
        publishMarker(fiducials[o.fid]);
    }
}

// Stub implementation for lookupTransform
bool Map::lookupTransform(const std::string &from, const std::string &to, 
                         const rclcpp::Time &time, tf2::Transform &T) const {
    geometry_msgs::msg::TransformStamped transform;

    try {
        transform = tfBuffer->lookupTransform(from, to, time, rclcpp::Duration::from_seconds(0.5));
        tf2::fromMsg(transform.transform, T);
        return true;
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(logger, "%s", ex.what());
        return false;
    }
}

// Implementation for handleAddFiducial
void Map::handleAddFiducial(const std::vector<Observation> &obs, const rclcpp::Time &time) {
    if (fiducialToAdd == -1) {
        return;
    }
    
    for (size_t i=0; i<obs.size(); i++) {
        const Observation &o = obs[i];
        if (o.fid == fiducialToAdd) {
            tf2::Stamped<TransformWithVariance> T_mapCam;
            
            // Try to estimate the camera pose
            tf2::Stamped<TransformWithVariance> cameraPose;
            cameraPose.frame_id_ = mapFrame;
            cameraPose.stamp_ = tf2_ros::fromMsg(time);
            cameraPose.variance = 0.0;
            
            if (updatePose(const_cast<std::vector<Observation>&>(obs), node->now(), cameraPose) > 0) {
                // Use the * operator for stamped transforms
                tf2::Stamped<TransformWithVariance> T_mapFid = cameraPose * o.T_camFid;
                T_mapFid.frame_id_ = mapFrame;
                
                if (fiducials.find(o.fid) == fiducials.end()) {
                    fiducials[o.fid] = Fiducial(o.fid, T_mapFid);
                }
            }
            break;
        }
    }
    
    fiducialToAdd = -1;
}

// Implementation for publishMarkers
void Map::publishMarkers() {
    rclcpp::Time now = node->now();
    for (auto &fiducial_pair : fiducials) {
        Fiducial &f = fiducial_pair.second;
        if ((now - f.lastPublished).seconds() > 1.0 || f.lastPublished.seconds() == 0) { // Publish if older than 1s or never
            publishMarker(f);
        }
    }
}

// Implementation for publishMarker
void Map::publishMarker(Fiducial &fid) {
    fid.lastPublished = node->now(); // Update lastPublished time
    // Basic marker
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = mapFrame;
    marker.header.stamp = node->now();
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.id = fid.id;
    marker.ns = "fiducial";
    
    marker.pose.position.x = fid.pose.transform.getOrigin().getX();
    marker.pose.position.y = fid.pose.transform.getOrigin().getY();
    marker.pose.position.z = fid.pose.transform.getOrigin().getZ();
    
    marker.pose.orientation.x = fid.pose.transform.getRotation().x();
    marker.pose.orientation.y = fid.pose.transform.getRotation().y();
    marker.pose.orientation.z = fid.pose.transform.getRotation().z();
    marker.pose.orientation.w = fid.pose.transform.getRotation().w();
    
    marker.scale.x = 0.15;
    marker.scale.y = 0.15;
    marker.scale.z = 0.01;
    
    if (fid.visible) {
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;
    }
    else {
        marker.color.r = 0.8f;
        marker.color.g = 0.8f;
        marker.color.b = 0.8f;
        marker.color.a = 0.5f;
    }
    
    markerPub->publish(marker);
    
    // Variance indicator
    visualization_msgs::msg::Marker cylinder;
    cylinder.type = visualization_msgs::msg::Marker::CYLINDER;
    cylinder.action = visualization_msgs::msg::Marker::ADD;
    cylinder.header.frame_id = mapFrame;
    cylinder.header.stamp = node->now();
    cylinder.color.r = 0.0f;
    cylinder.color.g = 0.0f;
    cylinder.color.b = 1.0f;
    cylinder.color.a = 0.5f;
    cylinder.id = fid.id + 10000;
    cylinder.ns = "sigma";
    cylinder.scale.x = cylinder.scale.y = std::max(std::sqrt(fid.pose.variance), 0.1);
    cylinder.scale.z = 0.01;
    cylinder.pose.position.x = marker.pose.position.x;
    cylinder.pose.position.y = marker.pose.position.y;
    cylinder.pose.position.z = marker.pose.position.z;
    cylinder.pose.position.z += (marker.scale.z / 2.0) + 0.05;
    
    markerPub->publish(cylinder);
    
    // Text
    visualization_msgs::msg::Marker text;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;
    text.header.frame_id = mapFrame;
    text.header.stamp = node->now();
    text.color.r = text.color.g = text.color.b = text.color.a = 1.0f;
    text.id = fid.id + 30000;
    text.scale.x = text.scale.y = text.scale.z = 0.1;
    text.pose.position.x = marker.pose.position.x;
    text.pose.position.y = marker.pose.position.y;
    text.pose.position.z = marker.pose.position.z;
    text.pose.position.z += (marker.scale.z / 2.0) + 0.1;
    text.ns = "text";
    text.text = std::to_string(fid.id);
    
    markerPub->publish(text);
    
    // Links
    visualization_msgs::msg::Marker links;
    links.type = visualization_msgs::msg::Marker::LINE_LIST;
    links.action = visualization_msgs::msg::Marker::ADD;
    links.header.frame_id = mapFrame;
    links.header.stamp = node->now();
    links.color.r = 0.0f;
    links.color.g = 0.0f;
    links.color.b = 1.0f;
    links.color.a = 1.0f;
    links.id = fid.id + 40000;
    links.ns = "links";
    links.scale.x = links.scale.y = links.scale.z = 0.02;
    links.pose.position.x = 0;
    links.pose.position.y = 0;
    links.pose.position.z = 0;
    
    geometry_msgs::msg::Point gp0, gp1;
    tf2::Vector3 p0 = fid.pose.transform.getOrigin();
    gp0.x = p0.x();
    gp0.y = p0.y();
    gp0.z = p0.z();
    
    for (const auto linked_fid : fid.links) {
        // Only draw links in one direction
        if (fid.id < linked_fid) {
            if (fiducials.find(linked_fid) != fiducials.end()) {
                tf2::Vector3 p1 = fiducials[linked_fid].pose.transform.getOrigin();
                gp1.x = p1.x();
                gp1.y = p1.y();
                gp1.z = p1.z();
                links.points.push_back(gp0);
                links.points.push_back(gp1);
            }
        }
    }
    
    markerPub->publish(links);
}

// Stub implementation for publishMap
void Map::publishMap() {
    auto msg = std::make_unique<fiducial_msgs::msg::FiducialMapEntryArray>();
    // ROS 2 FiducialMapEntryArray does not have a header.
    // msg->header.stamp = node->now();
    // msg->header.frame_id = mapFrame;

    for (const auto &map_pair : fiducials) {
        const Fiducial &f = map_pair.second;

        fiducial_msgs::msg::FiducialMapEntry fme;
        fme.fiducial_id = f.id;

        tf2::Vector3 t = f.pose.transform.getOrigin();
        fme.x = t.x();
        fme.y = t.y();
        fme.z = t.z();

        double rx, ry, rz;
        f.pose.transform.getBasis().getRPY(rx, ry, rz);
        fme.rx = rx;
        fme.ry = ry;
        fme.rz = rz;
        // FiducialMapEntry might not have a variance field.
        // fme.variance = f.pose.variance; 

        msg->fiducials.push_back(fme);
    }

    mapPub->publish(std::move(msg));
}

// Implementation for publishTf
// This function can be called if an immediate TF publish is needed outside the regular Map::update cycle.
// However, the main periodic publishing is handled by Map::update().
void Map::publishTf() {
    if (!havePose || !publishPoseTf) { 
        return;
    }
    rclcpp::Time now = node->get_clock()->now();
    // Update timestamp to current time + future_date_transforms before publishing
    poseTf.header.stamp = now + rclcpp::Duration::from_seconds(future_date_transforms);
    broadcaster->sendTransform(poseTf);
}

// Implementation for drawLine
void Map::drawLine(const tf2::Vector3 &p0, const tf2::Vector3 &p1) {
    static int lid = 60000;
    visualization_msgs::msg::Marker line;
    line.type = visualization_msgs::msg::Marker::LINE_LIST;
    line.action = visualization_msgs::msg::Marker::ADD;
    line.header.frame_id = mapFrame;
    line.header.stamp = node->now();
    line.color.r = 1.0f;
    line.color.g = 0.0f;
    line.color.b = 0.0f;
    line.color.a = 1.0f;
    line.id = lid++;
    line.ns = "lines";
    line.scale.x = line.scale.y = line.scale.z = 0.01;
    line.pose.position.x = 0;
    line.pose.position.y = 0;
    line.pose.position.z = 0; // Set z to 0 for LINE_LIST in map frame

    geometry_msgs::msg::Point gp0, gp1;
    gp0.x = p0.x();
    gp0.y = p0.y();
    gp0.z = p0.z();
    gp1.x = p1.x();
    gp1.y = p1.y();
    gp1.z = p1.z();

    line.points.push_back(gp0);
    line.points.push_back(gp1);

    markerPub->publish(line);
}

// Implementation for map loading
bool Map::loadMap() {
    return loadMap(mapFilename);
}

// Implementation for map loading with filename
bool Map::loadMap(std::string filename) {
    int numRead = 0;

    FILE *fp = fopen(filename.c_str(), "r");
    if (fp == NULL) {
        RCLCPP_WARN(logger, "Could not open %s for read", filename.c_str());
        return false;
    }

    const int BUFSIZE = 2048;
    char linebuf[BUFSIZE];
    char linkbuf[BUFSIZE]; // Buffer for reading links

    while (!feof(fp) && fgets(linebuf, BUFSIZE - 1, fp) != NULL) {
        int id;
        double tx, ty, tz, rx, ry, rz, var;
        int numObs = 0;

        linkbuf[0] = '\0'; // Initialize linkbuf
        // Try to parse with links first
        int nElems = sscanf(linebuf, "%d %lf %lf %lf %lf %lf %lf %lf %d%[^\n]", &id, &tx, &ty,
                            &tz, &rx, &ry, &rz, &var, &numObs, linkbuf);

        if (nElems >= 9) { // 9 for no links, 10 or more if links are present (space separated)
            tf2::Vector3 tvec(tx, ty, tz);
            tf2::Quaternion q;
            q.setRPY(deg2rad(rx), deg2rad(ry), deg2rad(rz));

            auto twv = TransformWithVariance(tvec, q, var);
            // Convert rclcpp::Time to tf2::TimePoint
            tf2::TimePoint time_point = tf2::TimePoint(std::chrono::nanoseconds(node->now().nanoseconds()));
            Fiducial f =
                Fiducial(id, tf2::Stamped<TransformWithVariance>(twv, time_point, mapFrame));
            f.numObs = numObs;

            if (nElems > 9 && linkbuf[0] != '\0') { // If there's content in linkbuf
                 std::string links_str(linkbuf);
                 std::istringstream ss(links_str);
                 std::string s_id;
                 while (ss >> s_id) {
                     try {
                        f.links.insert(std::stoi(s_id));
                     } catch (const std::invalid_argument& ia) {
                        RCLCPP_WARN(logger, "Invalid link ID format: %s for fiducial %d", s_id.c_str(), id);
                     } catch (const std::out_of_range& oor) {
                        RCLCPP_WARN(logger, "Link ID out of range: %s for fiducial %d", s_id.c_str(), id);
                     }
                 }
            }

            fiducials[id] = f;
            numRead++;
        } else {
            RCLCPP_WARN(logger, "Invalid line (nelems %d): %s", nElems, linebuf);
        }
    }

    fclose(fp);
    publishMarkers(); // Publish markers after loading map
    publishMap();     // Publish map after loading
    return true;
}

// Implementation for map saving
bool Map::saveMap() {
    return saveMap(mapFilename);
}

// Implementation for map saving with filename
bool Map::saveMap(std::string filename) {
    FILE *fp = fopen(filename.c_str(), "w");
    if (fp == NULL) {
        RCLCPP_WARN(logger, "Could not open %s for write", filename.c_str());
        return false;
    }

    for (auto &map_pair : fiducials) {
        Fiducial &f = map_pair.second;
        tf2::Vector3 trans = f.pose.transform.getOrigin();
        double rx, ry, rz;
        f.pose.transform.getBasis().getRPY(rx, ry, rz);

        fprintf(fp, "%d %lf %lf %lf %lf %lf %lf %lf %d", f.id, trans.x(), trans.y(), trans.z(),
                rad2deg(rx), rad2deg(ry), rad2deg(rz), f.pose.variance, f.numObs);

        for (const auto linked_fid : f.links) {
            fprintf(fp, " %d", linked_fid);
        }
        fprintf(fp, "\n");
    }
    fclose(fp);
    return true;
}

// Service callback for clearing the map
bool Map::clearCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> req,
    std::shared_ptr<std_srvs::srv::Empty::Response> res)
{
    (void)req; // Mark as unused
    (void)res; // Mark as unused
    fiducials.clear();
    originFid = -1;
    isInitializingMap = false;
    return true;
}

// Service callback for adding a fiducial to the map
bool Map::addFiducialCallback(
    const std::shared_ptr<fiducial_slam_ros2::srv::AddFiducial::Request> req,
    std::shared_ptr<fiducial_slam_ros2::srv::AddFiducial::Response> res)
{
    (void)res; // Mark as unused
    fiducialToAdd = req->fiducial_id;
    return true;
}
