#include <fiducial_slam_ros2/transform_with_variance.hpp>
#include <cmath>

// Private helper functions
namespace {
    // Takes 2 variances and gives the kalman gain figure
    // Kalman gain represents how much to trust the new measurement compared to the existing one
    inline double kalman_gain(double var1, double var2) {
        return var1 / (var1 + var2);
    }

    // Returns the probability density at a point given the mean and variance of the distribution
    double probabilityAtPoint(const double x, const double u, const double var) {
        return (1.0 / (std::sqrt(var) * std::sqrt(2.0 * M_PI))) *
               std::exp(-((x - u) * (x - u)) / (2.0 * var));
    }

    // Takes in 2 estimates and the combined estimate and calculates the
    // variance of the new estimate to normalize it
    double normalizeDavid(const double newMean, const double mean1, double var1,
                          const double mean2, double var2) {
        // Find the probabilities of the new mean in both original gaussians
        double prob1_at_newMean = probabilityAtPoint(newMean, mean1, var1);
        double prob2_at_newMean = probabilityAtPoint(newMean, mean2, var2);
        // We use the sum in quadrature of these 2 probabilities
        double prob_at_newMean = sqrt(pow(prob1_at_newMean, 2) + pow(prob2_at_newMean, 2));

        double newVar = std::pow(1.0 / (prob_at_newMean * sqrt(2.0 * M_PI)), 2);

        // Bound the variance to prevent blow up
        newVar = std::min(newVar, 1e3);
        newVar = std::max(newVar, 1e-8);

        return newVar;
    }
}

// Converts this transform to a geometry_msgs PoseWithCovarianceStamped
geometry_msgs::msg::PoseWithCovarianceStamped TransformWithVariance::toPose(
    const std::string& frame, const rclcpp::Time& stamp) const {
    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    
    pose.header.frame_id = frame;
    pose.header.stamp = stamp;
    
    // Copy the transform
    tf2::toMsg(transform, pose.pose.pose);
    
    // Set the diagonal elements of the covariance matrix to the variance
    for (int i = 0; i < 6; i++) {
        pose.pose.covariance[i * 7] = variance; // Set diagonal elements only
    }
    
    return pose;
}

// Returns the inverse transform
TransformWithVariance TransformWithVariance::inverse() const {
    TransformWithVariance inv;
    inv.transform = transform.inverse();
    inv.variance = variance; // Variance stays the same for inverse
    return inv;
}

// Update this transform with a new one, with variances as weights
void TransformWithVariance::update(const TransformWithVariance& newT) {
    tf2::Vector3 p1 = transform.getOrigin();
    tf2::Quaternion q1 = transform.getRotation();
    double var1 = variance;

    tf2::Vector3 p2 = newT.transform.getOrigin();
    tf2::Quaternion q2 = newT.transform.getRotation();
    double var2 = newT.variance;

    // Calculate new mean for the position using Kalman gain
    double k = kalman_gain(var1, var2);
    transform.setOrigin(p1 + k * (p2 - p1));

    // Calculate new mean for the orientation using slerp
    transform.setRotation(q1.slerp(q2, k).normalized());

    // Calculate the new variance
    double mean1 = 0.0;
    double mean2 = (p2 - p1).length();
    double mean = (transform.getOrigin() - p1).length();

    // Normalize the variances
    variance = normalizeDavid(mean, mean1, var1, mean2, var2);
}

// Weighted average of 2 transforms
TransformWithVariance averageTransforms(const TransformWithVariance& t1,
                                        const TransformWithVariance& t2) {
    TransformWithVariance out = t1;
    out.update(t2);
    return out;
}
