#ifndef UW_PARTICLE_LOCALIZATION_CONFIG_HPP
#define UW_PARTICLE_LOCALIZATION_CONFIG_HPP

#include <vector>
#include <string>
#include <boost/assert.hpp>
#include <uw_localization/types/environment.hpp>

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
    unsigned int particle_number;
    unsigned int perception_history_number;
    double hough_interspersal_ratio;
    double effective_sample_size_threshold;
    int minimum_perceptions;
    bool pure_random_motion;

    base::Vector3d init_position;
    base::Vector3d init_variance;
    
    bool use_markov;
    bool avg_particle_position;
    bool use_best_feature_only;

    // Sensor uncertainty
    double sonar_maximum_distance;
    double sonar_minimum_distance;
    
    double sonar_vertical_angle;
    double sonar_covariance_reflection_factor;
    double sonar_covariance_corner_factor;

    Eigen::Matrix3d static_motion_covariance;
    Eigen::Matrix3d static_speed_covariance;
    
    double sonar_covariance;
    double usbl_range_variance;
    double usbl_angle_variance;
    
    //Sensor transformations
    Eigen::Translation3d sonarToAvalon;

    Eigen::Quaterniond dvlRotation;
    
    Eigen::Affine3d usbl2world;
    
    double yaw_offset;
    
    bool useMap;
    
    bool filterZeros;
    
    bool advanced_motion_model;
    double max_velocity_drift;
    Environment* env;
    
    std::vector<std::string> joint_names;
    
    
};

}

#endif
