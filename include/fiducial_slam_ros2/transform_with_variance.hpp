#ifndef TRANSFORM_WITH_VARIANCE_HPP
#define TRANSFORM_WITH_VARIANCE_HPP

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <limits>

/**
 * @brief A class that represents a transform with associated variance
 * 
 * This class combines a tf2 transform with a variance value that
 * represents the uncertainty in the transform.
 */
class TransformWithVariance {
public:
    tf2::Transform transform;
    double variance;

    // Default constructor
    TransformWithVariance() = default;

    // Constructors that make a transform out of the different implementations
    TransformWithVariance(const tf2::Transform& t, double var) : transform(t), variance(var) {}
    
    TransformWithVariance(const geometry_msgs::msg::Transform& t, double var) : variance(var) {
        tf2::fromMsg(t, transform);
    }
    
    TransformWithVariance(const tf2::Vector3& tvec, const tf2::Quaternion& q, double var)
        : transform(q, tvec), variance(var) {}

    // Update this transform with a new one, with variances as weights
    void update(const TransformWithVariance& newT);

    // Converts this transform to a geometry_msgs PoseWithCovarianceStamped
    geometry_msgs::msg::PoseWithCovarianceStamped toPose(
        const std::string& frame, const rclcpp::Time& stamp) const;

    // Returns the inverse transform
    TransformWithVariance inverse() const;

    // Used to combine this transform with another one increasing total variance
    TransformWithVariance& operator*=(const TransformWithVariance& rhs) {
        // Update this transform, increasing variance
        transform *= rhs.transform;

        // Do simple addition of the variances
        // In a multi-variate case RMS may be more correct
        variance += rhs.variance;
        return *this;
    }
    
    friend TransformWithVariance operator*(TransformWithVariance lhs,
                                           const TransformWithVariance& rhs) {
        lhs *= rhs;
        return lhs;
    }
    
    // Used to combine this transform with another one keeping variance the same
    TransformWithVariance& operator*=(const tf2::Transform& rhs) {
        transform *= rhs;
        // No need to change the variance, we are assuming that rhs has variance of 0
        return *this;
    }
    
    friend TransformWithVariance operator*(TransformWithVariance lhs, const tf2::Transform& rhs) {
        lhs *= rhs;
        return lhs;
    }
};

// Weighted average of 2 transforms
TransformWithVariance averageTransforms(const TransformWithVariance& t1,
                                        const TransformWithVariance& t2);

// Use the tf2::Stamped template provided by tf2 library

// Global operators for stamped transformations
// These are defined outside of any namespace to avoid conflicts

// Create a TransformWithVariance from a basic Transform
inline TransformWithVariance toTransformWithVariance(const tf2::Transform& t, double variance = 0.0) {
    return TransformWithVariance(t, variance);
}

// Helper method to create a stamped transform
inline tf2::Stamped<TransformWithVariance> createStampedTransform(
    const TransformWithVariance& transform,
    const std::string& frame_id,
    const tf2::TimePoint& stamp = tf2::TimePointZero) {
    tf2::Stamped<TransformWithVariance> stamped;
    stamped.frame_id_ = frame_id;
    stamped.stamp_ = stamp;
    stamped.setData(transform);
    return stamped;
}

// Multiplying a stamped transform with another stamped transform
inline tf2::Stamped<TransformWithVariance> operator*(
    const tf2::Stamped<TransformWithVariance>& lhs,
    const tf2::Stamped<TransformWithVariance>& rhs) {
    tf2::Stamped<TransformWithVariance> result;
    result.frame_id_ = lhs.frame_id_;
    result.stamp_ = lhs.stamp_;
    
    // Combine the transforms and add their variances
    TransformWithVariance combined(lhs.transform * rhs.transform, lhs.variance + rhs.variance);
    result.setData(combined);
    
    return result;
}

#endif // TRANSFORM_WITH_VARIANCE_HPP
