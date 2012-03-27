#include "ParticleLocalization.hpp"
#include <base/pose.h>

using namespace machine_learning;

namespace uw_localization {

ParticleLocalization::ParticleLocalization(const FilterConfig& config) 
    : filter_config(config), 
    StaticSpeedNoise(Random::multi_gaussian<3>())
{
    if(config.has_static_motion_covariance()) {
        StaticSpeedNoise = Random::multi_gaussian(Eigen::Vector3d(0.0, 0.0, 0.0), config.get_static_motion_covariance()); 
    }
}

ParticleLocalization::~ParticleLocalization()
{}

double ParticleLocalization::dynamic(PoseParticle& X, const base::samples::RigidBodyState& U)
{
    MultiNormalRandom<3> SpeedNoise = Random::multi_gaussian(Eigen::Vector3d(0.0, 0.0, 0.0), U.cov_velocity);
    PoseParticle Xt;

    base::Vector3d v_noisy;

    if(filter_config.has_static_motion_covariance())
        v_noisy = U.velocity + StaticSpeedNoise();
    else
        v_noisy = U.velocity + SpeedNoise();
    
    base::Vector3d v_avg = (X.velocity + v_noisy) / 2.0;

    if( !X.timestamp.isNull() ) {
        double dt = (U.time - X.timestamp).toSeconds();

        Xt.position = X.position + vehicle_orientation * (v_avg * dt);
    }

    Xt.velocity = v_noisy;
    Xt.timestamp = U.time;
    Xt.confidence = X.confidence;
 
    return 0;
}

double ParticleLocalization::perception(PoseParticle& state, const base::samples::LaserScan& scan, const StochasticMap& map)
{
    return 0;
}
  
base::Vector3d ParticleLocalization::position(const PoseParticle& state) const
{
    return state.position;
}


double ParticleLocalization::yaw(const PoseParticle& state) const
{
    return base::getYaw(vehicle_orientation);
}


double ParticleLocalization::get_weight(const PoseParticle& state) const
{
    return state.confidence;
}



void ParticleLocalization::set_weight(PoseParticle& state, double value)
{
    state.confidence = value;
}


  
base::samples::RigidBodyState ParticleLocalization::estimate() const
{
    base::samples::RigidBodyState state;

    return state;
}

}
