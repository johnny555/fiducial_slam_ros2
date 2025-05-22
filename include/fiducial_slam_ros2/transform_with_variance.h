#ifndef TRANSFORM_VARIANCE_H
#define TRANSFORM_VARIANCE_H

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class TransformWithVariance {
public:
    tf2::Transform transform;
    double variance;

    TransformWithVariance() = default;

    // Constructors that make a transform out of the different implementations
    TransformWithVariance(const tf2::Transform& t, double var) : transform(t), variance(var) {};
    
    TransformWithVariance(const geometry_msgs::msg::Transform& t, double var) : variance(var) {
        tf2::fromMsg(t, transform);
    };
    
    TransformWithVariance(const tf2::Vector3& tvec, const tf2::Quaternion& q, double var)
        : transform(q, tvec), variance(var) {};

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
    
    friend tf2::Stamped<TransformWithVariance> operator*(
        tf2::Stamped<TransformWithVariance> lhs, const tf2::Stamped<TransformWithVariance>& rhs) {
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

    // Converts this transform to a geometry_msgs PoseWithCovarianceStamped.
    // The 6x6 covariance matrix  is filled as [variance 0 0 0 0 0; 0 variance 0 0 0 0; ...]
    geometry_msgs::msg::PoseWithCovarianceStamped toPose(
        const std::string& frame, const rclcpp::Time& stamp) const;

    // Returns the inverse transform.  
    // With the current implementation of the equations, the variance doesn't change
    TransformWithVariance inverse() const;
};

#endif // TRANSFORM_VARIANCE_H
