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


void ParticleLocalization::initialize(std::vector<PoseParticle>& set, int numbers, const Eigen::Vector3d& pos, const Eigen::Matrix3d& cov, double yaw, double yaw_cov)
{

}



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

        Xt.position = X.position + vehicle_pose.orientation * (v_avg * dt);
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


base::Orientation ParticleLocalization::orientation(const PoseParticle& state) const
{
    return vehicle_pose.orientation;
}


double ParticleLocalization::getWeight(const PoseParticle& state) const
{
    return state.confidence;
}



void ParticleLocalization::setWeight(PoseParticle& state, double value)
{
    state.confidence = value;
}

void ParticleLocalization::preprocessing(const base::samples::RigidBodyState& speed)
{
    vehicle_pose.time = speed.time;
    vehicle_pose.velocity = speed.velocity;
    vehicle_pose.cov_velocity = speed.cov_velocity;
}


void ParticleLocalization::setCurrentOrientation(const base::samples::RigidBodyState& orientation)
{
    vehicle_pose.orientation = orientation.orientation;
    vehicle_pose.cov_orientation = orientation.cov_orientation;
    vehicle_pose.angular_velocity = orientation.angular_velocity;
    vehicle_pose.cov_angular_velocity = orientation.cov_angular_velocity;
    z_sample = orientation.position.z();
}

base::samples::RigidBodyState& ParticleLocalization::estimate()
{
    // use position estimate retrieved from particle filter
    vehicle_pose.position = mean_position;
    vehicle_pose.cov_position = cov_position;

    // use z samples from depth reader
    vehicle_pose.position.z() = z_sample;

    return vehicle_pose;
}


  
}
