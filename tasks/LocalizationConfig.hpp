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
        particle_interspersal_ratio(0.0),
        init_position(0.0, 0.0, 0.0), 
        init_covariance(Eigen::Matrix3d::Zero()),
        sonar_maximum_distance(20.0),
        sonar_covariance(Eigen::Matrix3d::Zero()),
        use_motion_covariance(false)
    {}

    // General properties
    int particle_number;
    double particle_interspersal_ratio;

    // Initialization
    Eigen::Vector3d init_position;
    Eigen::Matrix3d init_covariance;

    // Sensor uncertainty
    double sonar_maximum_distance;
    Eigen::Matrix3d sonar_covariance;

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
