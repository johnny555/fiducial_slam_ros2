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

// Update map with a set of observations (stub implementation)
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
        // Stub implementation
        RCLCPP_INFO(logger, "Map update: Would update pose and map here");
    }

    handleAddFiducial(obs);
    publishMap();
}

// Empty update function
void Map::update() {
    // Nothing to do in this stub implementation
}

// Stub implementation for autoInit
void Map::autoInit(const std::vector<Observation> &obs, const rclcpp::Time &time) {
    if (obs.size() < 1) {
        RCLCPP_INFO(logger, "Cannot auto init map with zero observations");
        return;
    }

    RCLCPP_INFO(logger, "Auto init map from %d observations", static_cast<int>(obs.size()));
    
    // Just use the first observation as our initial map in this stub
    if (obs.size() > 0 && originFid == -1) {
        const Observation &o = obs[0];
        originFid = o.fid;
        
        tf2::Stamped<TransformWithVariance> T_mapFid;
        T_mapFid.frame_id_ = mapFrame;
        T_mapFid.stamp_ = tf2::TimePointZero;
        
        // Initialize the map with this fiducial at the origin
        TransformWithVariance tv(tf2::Transform(tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(0, 0, 0)), 0.0);
        T_mapFid.setData(tv);
        T_mapFid.variance = 0.0;
        
        fiducials[o.fid] = Fiducial(o.fid, T_mapFid);
        
        RCLCPP_INFO(logger, "Created origin at fiducial %d", originFid);
        isInitializingMap = false;
    }
}

// Stub implementation for updatePose
int Map::updatePose(std::vector<Observation> &obs, const rclcpp::Time &time,
                   tf2::Stamped<TransformWithVariance> &cameraPose) {
    RCLCPP_INFO(logger, "updatePose stub: Would calculate robot pose here");
    return 0;
}

// Stub implementation for updateMap
void Map::updateMap(const std::vector<Observation> &obs, const rclcpp::Time &time,
                   const tf2::Stamped<TransformWithVariance> &cameraPose) {
    RCLCPP_INFO(logger, "updateMap stub: Would update map with new observations here");
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

// Stub implementation for handleAddFiducial
void Map::handleAddFiducial(const std::vector<Observation> &obs) {
    if (fiducialToAdd == -1) {
        return;
    }
    
    RCLCPP_INFO(logger, "handleAddFiducial stub: Would add fiducial %d to map here", fiducialToAdd);
    fiducialToAdd = -1;
}

// Stub implementation for publishMarkers
void Map::publishMarkers() {
    RCLCPP_DEBUG(logger, "publishMarkers stub: Would publish visualization markers here");
}

// Stub implementation for publishMarker
void Map::publishMarker(Fiducial &fid) {
    RCLCPP_DEBUG(logger, "publishMarker stub: Would publish marker for fiducial %d here", fid.id);
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

// Stub implementation for publishTf
void Map::publishTf() {
    RCLCPP_DEBUG(logger, "publishTf stub: Would publish TF here");
}

// Stub implementation for drawLine
void Map::drawLine(const tf2::Vector3 &p0, const tf2::Vector3 &p1) {
    RCLCPP_DEBUG(logger, "drawLine stub: Would draw line for visualization here");
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
