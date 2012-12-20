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
    
    double utm_relative_angle;
    double gps_covarianz;
    double gps_interspersal_ratio;

    // Sensor uncertainty
    double sonar_maximum_distance;
    double sonar_minimum_distance;
    double sonar_covariance;
    double pipeline_covariance;

    Eigen::Matrix3d static_motion_covariance;    
    
    //Motion model Properties
    double param_length;
    double param_radius;
    double param_mass;
    std::vector<double> param_thrusterCoefficient;
    double param_thrusterVoltage;
    std::vector<double> param_TCM;
    std::vector<double> param_dampingX;
    std::vector<double> param_dampingY;
    std::vector<double> param_dampingZ;
    bool param_floating;
};


}

#endif
