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
    
    bool use_markov;
    bool avg_particle_position;
    bool use_best_feature_only;

    // Sensor uncertainty
    double sonar_maximum_distance;
    double sonar_minimum_distance;
    double sonar_covariance;
    double pipeline_covariance;
    double buoy_covariance;
    
    double sonar_vertical_angle;
    double sonar_covariance_reflection_factor;
    double sonar_covariance_corner_factor;

    Eigen::Matrix3d static_motion_covariance;
    
    //Sensor transformations
    Eigen::Translation3d sonarToAvalon;
    Eigen::Vector3d pipelineToAvalon;
    Eigen::Vector3d gpsToAvalon;
    
    Eigen::Vector3d buoyCamPosition;
    Eigen::Quaternion<double> buoyCamRotation;
    
    double yaw_offset;
    
    bool useMap;
    
    bool filterZeros;
    
    //Motion model Properties
    double param_length;
    double param_radius;
    double param_mass;
    std::vector<double> param_thrusterCoefficient;
    std::vector<double> param_linearThrusterCoefficient;
    std::vector<double> param_squareThrusterCoefficient;
    double param_thrusterVoltage;
    std::vector<double> param_TCM;
    base::Matrix6d param_linDamp;
    base::Matrix6d param_sqDamp;
    base::Matrix6d param_linDampNeg;
    base::Matrix6d param_sqDampNeg;
    base::Vector3d param_centerOfGravity;
    base::Vector3d param_centerOfBuoyancy;
    bool param_floating;
    
    bool advanced_motion_model;
    double max_velocity_drift;
    Environment* env;
    
    std::vector<std::string> joint_names;
    
    //slam stuff
    
    double feature_weight_reduction;
    double feature_observation_range;
    double feature_observation_minimum_range;
    double feature_grid_resolution;
    double feature_filter_threshold;
    double feature_confidence;
    bool use_slam;
    bool use_mapping_only;

    
};

}

#endif
