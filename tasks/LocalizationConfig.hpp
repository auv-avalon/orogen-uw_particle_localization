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
    FilterConfig() : 
        particle_number(50),
        yaw_offset(0.0),
        particle_interspersal_ratio(0.0),
	effective_sample_size_threshold(particle_number / 2.0),
	minimum_perceptions(2),
        sonar_maximum_distance(20.0),
        sonar_covariance(1.0),
        use_motion_covariance(false)
    {}

    // General properties
    int particle_number;
    double yaw_offset;
    double particle_interspersal_ratio;
    double effective_sample_size_threshold;
    int minimum_perceptions;
    bool pure_random_motion;

    base::Vector3d init_position;
    base::Vector3d init_variance;

    // Sensor uncertainty
    double sonar_maximum_distance;
    double sonar_covariance;

    // optional properties
    void use_static_motion_covariance(const Eigen::Matrix3d& covariance) {
        static_motion_covariance = covariance;
        use_motion_covariance = true;
    }

    void reset_static_motion_covariance() {
        use_motion_covariance = false;
    }

    bool has_static_motion_covariance() const { return use_motion_covariance; }
    const Eigen::Matrix3d& get_static_motion_covariance() const { return static_motion_covariance; }

 private: 
    Eigen::Matrix3d static_motion_covariance;
    bool use_motion_covariance;
};


}

#endif
