#ifndef UW_PARTICLE_LOCALIZATION_CONFIG_HPP
#define UW_PARTICLE_LOCALIZATION_CONFIG_HPP

#include <vector>
#include <boost/assert.hpp>

namespace uw_localization {


template <typename M>
M convertProperty(const std::vector<double>& v) {
    M result;

    BOOST_ASSERT(result.rows() * result.cols() == (int) v.size());

    unsigned s = 0;

    for(unsigned i = 0; i < result.rows(); i++) {
        for(unsigned j = 0; j < result.cols(); j++) {
            result(i, j) = v[s++];
        }
    }

    return result;
}

struct FilterConfig {
    // General properties
    int particle_number;
    int perception_history_number;
    double hough_interspersal_ratio;
    double effective_sample_size_threshold;
    int minimum_perceptions;
    bool pure_random_motion;

    base::Vector3d init_position;
    base::Vector3d init_variance;

    // Sensor uncertainty
    double sonar_maximum_distance;
    double sonar_minimum_distance;
    double sonar_covariance;
    double pipeline_covariance;

    Eigen::Matrix3d static_motion_covariance;
};


}

#endif
