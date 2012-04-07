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

    initialize(config.particle_number, config.init_position, config.init_covariance, 0.0, 0.0);
}

ParticleLocalization::~ParticleLocalization()
{}


void ParticleLocalization::initialize(int numbers, const Eigen::Vector3d& pos, const Eigen::Matrix3d& cov, double yaw, double yaw_cov)
{
    MultiNormalRandom<3> initializer = Random::multi_gaussian(pos, cov);

    for(unsigned i = 0; i < numbers; i++) {
        PoseParticle pp;
        pp.position = initializer();
        pp.velocity = base::Vector3d(0.0, 0.0, 0.0);
        pp.confidence = 1.0 / numbers;
        
        particles.push_back(pp);
    }
}



void ParticleLocalization::dynamic(PoseParticle& X, const base::samples::RigidBodyState& U)
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
}


const base::Time& ParticleLocalization::getTimestamp(const base::samples::RigidBodyState& U)
{
    return U.time;
}


double ParticleLocalization::perception(const PoseParticle& X, const base::samples::LaserScan& Z, const NodeMap& M)
{
    double angle = Z.start_angle;
    double yaw = base::getYaw(vehicle_pose.orientation);
    double measure_distance = Z.ranges[0] / 1000.0;
    
    Eigen::AngleAxis<double> sonar_yaw(angle, Eigen::Vector3d::UnitZ()); 
    Eigen::AngleAxis<double> abs_yaw(yaw, Eigen::Vector3d::UnitZ());
    Eigen::Affine3d SonarToAvalon(Eigen::Translation3d(-1.0, 0.0, 0.0));

    Eigen::Vector3d RelativeZ = sonar_yaw * SonarToAvalon * base::Vector3d(measure_distance, 0.0, 0.0);
    Eigen::Vector3d AbsZ = (abs_yaw * RelativeZ) + X.position;

    boost::tuple<Node*, double> distance = M.getNearestDistance("root.wall", AbsZ, X.position);

    double probability = gaussian1d(distance.get<1>(), 0.0, filter_config.sonar_covariance);

    return probability;
}


bool ParticleLocalization::isMaximumRange(const base::samples::LaserScan& Z)
{
    double range = Z.ranges[0];

    return (range == base::samples::TOO_FAR || range >= filter_config.sonar_maximum_distance) ;
}


bool ParticleLocalization::belongsToWorld(const PoseParticle& X, const NodeMap& M)
{
    return M.belongsToWorld(X.position);
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

void ParticleLocalization::setCurrentSpeed(const base::samples::RigidBodyState& speed)
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
