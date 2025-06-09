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

static double systematic_error = 0.01;

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
    listener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);
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

    mapFrame = node->declare_parameter<std::string>("map_frame", "map");
    odomFrame = node->declare_parameter<std::string>("odom_frame", "odom");
    baseFrame = node->declare_parameter<std::string>("base_frame", "base_footprint");

    tfPublishInterval = node->declare_parameter<float>("tf_publish_interval", 1.0);
    publishPoseTf = node->declare_parameter<bool>("publish_tf", true);
    systematic_error = node->declare_parameter<double>("systematic_error", 0.01);
    future_date_transforms = node->declare_parameter<double>("future_date_transforms", 0.1);
    publish_6dof_pose = node->declare_parameter<bool>("publish_6dof_pose", false);
    readOnly = node->declare_parameter<bool>("read_only_map", false);

    std::vector<double> temp_covariance;
    temp_covariance = node->declare_parameter<std::vector<double>>("covariance_diagonal", std::vector<double>{});
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

    // threshold of object error for using multi-fidicial pose
    // set -ve to never use
    multiErrorThreshold = node->declare_parameter<double>("multi_error_theshold", -1.0);

    // Get home directory for default map location
    const char* home = std::getenv("HOME");
    std::string home_path = home ? std::string(home) : std::string("/root");
    std::string default_map_path = home_path + "/.ros/slam/map.txt";
    
    mapFilename = node->declare_parameter<std::string>("map_file", default_map_path);

    std::filesystem::path mapPath(mapFilename);
    std::filesystem::path dir = mapPath.parent_path();
    std::filesystem::create_directories(dir);

    std::string initialMap = node->declare_parameter<std::string>("initial_map_file", "");

    if (!initialMap.empty()) {
        loadMap(initialMap);
    } else {
        loadMap();
    }

    publishMarkers();
}

// Update map with a set of observations
void Map::update(std::vector<Observation> &obs, const rclcpp::Time &time) {
    RCLCPP_INFO(logger, "Updating map with %d observations. Map has %d fiducials", 
             static_cast<int>(obs.size()), static_cast<int>(fiducials.size()));

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
            cameraPoseMsg.header.stamp = time;
            cameraPose.transform.getOrigin();
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
                tf2::Transform robotPose = cameraPose.transform * T_baseCam;
                
                // Create and publish robot pose message
                geometry_msgs::msg::PoseWithCovarianceStamped robotPoseMsg;
                robotPoseMsg.header.frame_id = mapFrame;
                robotPoseMsg.header.stamp = time;
                robotPoseMsg.pose.pose = toPose(robotPose);
                
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
                if (publishPoseTf && (frameNum % (int)(tfPublishInterval * 10) == 0)) {
                    geometry_msgs::msg::TransformStamped robotTransform;
                    robotTransform.header.stamp = time + rclcpp::Duration::from_seconds(future_date_transforms);
                    robotTransform.header.frame_id = mapFrame;
                    robotTransform.child_frame_id = odomFrame;
                    robotTransform.transform = tf2::toMsg(robotPose);
                    
                    poseTf = robotTransform;
                    havePose = true;
                    publishTf();
                }
            }
        }
    }

    handleAddFiducial(obs, time);
    publishMap();
}

// Empty update function
void Map::update() {
    // Nothing to do in this stub implementation
}

// Implementation for autoInit
void Map::autoInit(const std::vector<Observation> &obs, const rclcpp::Time &time) {
    if (obs.size() < 1) {
        RCLCPP_INFO(logger, "Cannot auto init map with zero observations");
        return;
    }

    RCLCPP_INFO(logger, "Auto init map from %d observations", static_cast<int>(obs.size()));
    
    // Find the fiducial with the lowest ID to use as the origin
    int smallestID = std::numeric_limits<int>::max();
    size_t smallestIdx = 0;
    
    for (size_t i = 0; i < obs.size(); i++) {
        const Observation &o = obs[i];
        if (o.fid < smallestID) {
            smallestID = o.fid;
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
    }
}

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
        RCLCPP_INFO(logger, "camera->base %lf %lf %lf", c.x(), c.y(), c.z());
        T_camBase.variance = 1.0;
    } else {
        RCLCPP_ERROR(logger, "Cannot determine tf from camera to robot");
        return 0;
    }

    if (lookupTransform(baseFrame, obs[0].T_camFid.frame_id_, time, T_baseCam.transform)) {
        tf2::Vector3 c = T_baseCam.transform.getOrigin();
        RCLCPP_INFO(logger, "base->camera %lf %lf %lf", c.x(), c.y(), c.z());
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
            p.variance = s1 + s2 + s3 + systematic_error;
            o.T_camFid.variance = p.variance;

            RCLCPP_INFO(logger, "Pose %d %lf %lf %lf %lf %lf %lf %lf", o.fid, position.x(), position.y(),
                    position.z(), roll, pitch, yaw, p.variance);

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
                RCLCPP_DEBUG(logger, "Estimate good");
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
        RCLCPP_INFO(logger, "No good estimates");
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
    RCLCPP_INFO(logger, "Found %d good estimates", numEsts);
    return numEsts;
}

// Implementation for updateMap
void Map::updateMap(const std::vector<Observation> &obs, const rclcpp::Time &time,
                   const tf2::Stamped<TransformWithVariance> &cameraPose) {
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

        // Logging
        tf2::Vector3 trans = T_mapFid.transform.getOrigin();
        RCLCPP_INFO(logger, "Estimate of %d %lf %lf %lf var %lf %lf", o.fid, trans.x(), trans.y(),
                 trans.z(), o.T_camFid.variance, T_mapFid.variance);

        if (std::isnan(trans.x()) || std::isnan(trans.y()) || std::isnan(trans.z())) {
            RCLCPP_WARN(logger, "Skipping NAN estimate");
            continue;
        }

        // If the fiducial is not in the map, add it
        if (fiducials.find(o.fid) == fiducials.end()) {
            RCLCPP_INFO(logger, "New fiducial %d", o.fid);
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
    
    RCLCPP_INFO(logger, "Adding fiducial %d to map", fiducialToAdd);
    
    for (size_t i=0; i<obs.size(); i++) {
        const Observation &o = obs[i];
        if (o.fid == fiducialToAdd) {
            RCLCPP_INFO(logger, "Found fiducial %d", o.fid);
            tf2::Stamped<TransformWithVariance> T_mapCam;
            
            // Try to estimate the camera pose
            tf2::Stamped<TransformWithVariance> cameraPose;
            cameraPose.frame_id_ = mapFrame;
            cameraPose.stamp_ = tf2_ros::fromMsg(time);
            cameraPose.variance = 0.0;
            
            if (updatePose(const_cast<std::vector<Observation>&>(obs), node->now(), cameraPose) > 0) {
                RCLCPP_INFO(logger, "Got camera pose in map");
                // Use the * operator for stamped transforms
                tf2::Stamped<TransformWithVariance> T_mapFid = cameraPose * o.T_camFid;
                T_mapFid.frame_id_ = mapFrame;
                
                if (fiducials.find(o.fid) == fiducials.end()) {
                    fiducials[o.fid] = Fiducial(o.fid, T_mapFid);
                    RCLCPP_INFO(logger, "Added fiducial %d to map at %lf %lf %lf",
                                o.fid,
                                T_mapFid.transform.getOrigin().x(),
                                T_mapFid.transform.getOrigin().y(),
                                T_mapFid.transform.getOrigin().z());
                }
            }
            else {
                RCLCPP_INFO(logger, "Could not get camera pose in map");
            }
            break;
        }
    }
    
    fiducialToAdd = -1;
}

// Implementation for publishMarkers
void Map::publishMarkers() {
    RCLCPP_DEBUG(logger, "Publishing %d fiducial markers", static_cast<int>(fiducials.size()));
    
    for (auto &fiducial_pair : fiducials) {
        publishMarker(fiducial_pair.second);
    }
}

// Implementation for publishMarker
void Map::publishMarker(Fiducial &fid) {
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
    RCLCPP_DEBUG(logger, "publishMap stub: Would publish map here");
    
    auto msg = std::make_unique<fiducial_msgs::msg::FiducialMapEntryArray>();
    
    for (auto &entry : fiducials) {
        fiducial_msgs::msg::FiducialMapEntry fme;
        fme.fiducial_id = entry.first;
        
        tf2::Vector3 trans = entry.second.pose.transform.getOrigin();
        fme.x = trans.x();
        fme.y = trans.y();
        fme.z = trans.z();
        
        double roll, pitch, yaw;
        entry.second.pose.transform.getBasis().getRPY(roll, pitch, yaw);
        fme.rx = roll;
        fme.ry = pitch;
        fme.rz = yaw;
        
        msg->fiducials.push_back(fme);
    }
    
    mapPub->publish(*msg);
}

// Implementation for publishTf
void Map::publishTf() {
    if (!havePose) {
        return;
    }
    
    broadcaster->sendTransform(poseTf);
    RCLCPP_DEBUG(logger, "Published transform map->odom with time %d.%d", 
                poseTf.header.stamp.sec, poseTf.header.stamp.nanosec);
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
    line.pose.position.z = 0;
    
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

// Stub implementation for map loading
bool Map::loadMap() {
    return loadMap(mapFilename);
}

// Stub implementation for map loading with filename
bool Map::loadMap(std::string filename) {
    RCLCPP_INFO(logger, "loadMap stub: Would load map from %s here", filename.c_str());
    return true;
}

// Stub implementation for map saving
bool Map::saveMap() {
    return saveMap(mapFilename);
}

// Stub implementation for map saving with filename
bool Map::saveMap(std::string filename) {
    RCLCPP_INFO(logger, "saveMap stub: Would save map to %s here", filename.c_str());
    return true;
}

// Service callback for clearing the map
bool Map::clearCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> req,
    std::shared_ptr<std_srvs::srv::Empty::Response> res)
{
    RCLCPP_INFO(logger, "Clearing map");
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
    fiducialToAdd = req->fiducial_id;
    RCLCPP_INFO(logger, "Request to add fiducial %d to map", fiducialToAdd);
    return true;
}
