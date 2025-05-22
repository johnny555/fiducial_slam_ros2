#ifndef HELPERS_HPP
#define HELPERS_HPP

#include <cmath>
#include <memory>
#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/msg/pose.hpp>

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

#endif // HELPERS_HPP
