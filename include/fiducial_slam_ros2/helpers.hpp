#ifndef HELPERS_HPP
#define HELPERS_HPP

#include <cmath>
#include <memory>
#include <string> // Required for std::string, std::stod, etc.
#include <vector> // Required for std::vector
#include <sstream> // Required for std::istringstream
#include <algorithm> // Required for std::transform
#include <iterator> // Required for std::istream_iterator

#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp> // Required for RCLCPP_INFO, etc.

// Degrees to radians
constexpr double deg2rad(double deg) { return deg * M_PI / 180.0; }

// Radians to degrees
constexpr double rad2deg(double rad) { return rad * 180.0 / M_PI; }

// Convert a tf2::Transform to a geometry_msgs::Pose
inline geometry_msgs::msg::Pose toPose(const tf2::Transform& transform) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = transform.getOrigin().x();
    pose.position.y = transform.getOrigin().y();
    pose.position.z = transform.getOrigin().z();
    
    tf2::Quaternion q = transform.getRotation();
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    
    return pose;
}

// Helper function to convert a string of space-separated doubles to a vector of doubles
inline std::vector<double> stringToDoubleVector(const std::string& str, rclcpp::Logger logger) {
    std::vector<double> result;
    std::istringstream iss(str);
    double val;
    while (iss >> val) {
        result.push_back(val);
    }
    if (iss.fail() && !iss.eof()) { // Check for conversion errors that are not end-of-file
        RCLCPP_WARN(logger, "Failed to parse part of the covariance_diagonal string: %s", str.c_str());
    }
    return result;
}

#endif // HELPERS_HPP
