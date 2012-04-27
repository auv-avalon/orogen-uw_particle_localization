#include "ParticleLocalization.hpp"
#include <base/pose.h>

using namespace machine_learning;

namespace uw_localization {

base::samples::RigidBodyState* PoseParticle::pose = 0;

ParticleLocalization::ParticleLocalization(const FilterConfig& config) 
    : filter_config(config), 
    StaticSpeedNoise(Random::multi_gaussian(Eigen::Vector3d(0.0, 0.0, 0.0), config.get_static_motion_covariance())),
    sonar_debug(0)
{
    initialize(config.particle_number, config.init_position, config.init_variance, 0.0, 0.0);
}

ParticleLocalization::~ParticleLocalization()
{}


void ParticleLocalization::initialize(int numbers, const Eigen::Vector3d& pos, const Eigen::Vector3d& var, double yaw, double yaw_cov)
{
    UniformRealRandom pos_x = Random::uniform_real(pos.x(), var.x());
    UniformRealRandom pos_y = Random::uniform_real(pos.y(), var.y());
    UniformRealRandom pos_z = Random::uniform_real(pos.z(), var.z());

    particles.clear();

    for(unsigned i = 0; i < numbers; i++) {
        PoseParticle pp;
        pp.p_position = base::Vector3d(pos_x(), pos_y(), pos_z());
        pp.p_velocity = base::Vector3d(0.0, 0.0, 0.0);
        pp.main_confidence = 1.0 / numbers;
        pp.part_confidences = Eigen::Vector3d(0.0, 0.0, 0.0);
        
        particles.push_back(pp);
    }

    PoseParticle::pose = &vehicle_pose;
}



void ParticleLocalization::dynamic(PoseParticle& X, const base::samples::RigidBodyState& U)
{
    MultiNormalRandom<3> SpeedNoise = Random::multi_gaussian(Eigen::Vector3d(0.0, 0.0, 0.0), U.cov_velocity);

    base::Vector3d v_noisy;
    base::Vector3d u_velocity;

    if(filter_config.pure_random_motion)
        u_velocity = base::Vector3d(0.0, 0.0, 0.0);
    else
        u_velocity = U.velocity;

    if(filter_config.has_static_motion_covariance()) {
        v_noisy = u_velocity + StaticSpeedNoise();
    } else {
        v_noisy = u_velocity + SpeedNoise();
    }
    
    base::Vector3d v_avg = (X.p_velocity + v_noisy) / 2.0;

    if( !X.timestamp.isNull() ) {
        double dt = (U.time - X.timestamp).toSeconds();

        X.p_position = X.p_position + vehicle_pose.orientation * (v_avg * dt);
    }

    X.p_velocity = v_noisy;
    X.timestamp = U.time;
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
    Eigen::Vector3d AbsZ = (abs_yaw * RelativeZ) + X.p_position;

    boost::tuple<Node*, double, Eigen::Vector3d> distance = M.getNearestDistance("root.wall", AbsZ, X.p_position);

    double probability = gaussian1d(0.0, filter_config.sonar_covariance, distance.get<1>());

    if(sonar_debug != 0) {
        uw_localization::debug::SonarPerception s;
        s.particle = X.p_position;
        s.obstacle = AbsZ;
        s.expected_obstacle = distance.get<2>();
        s.laser_distance = distance.get<1>();
        s.perception_confidence = probability;
        s.timestamp = X.timestamp;
        sonar_debug->write(s);
    }

    return probability;
}


bool ParticleLocalization::isMaximumRange(const base::samples::LaserScan& Z)
{
    double range = Z.ranges[0] / 1000.0;

    return (range == base::samples::TOO_FAR || range >= filter_config.sonar_maximum_distance) ;
}


void ParticleLocalization::setCurrentSpeed(const base::samples::RigidBodyState& speed)
{
    vehicle_pose.time = speed.time;
    vehicle_pose.velocity = speed.velocity;
    vehicle_pose.cov_velocity = speed.cov_velocity;
}


bool ParticleLocalization::isParticleInWorld(const PoseParticle& X, const NodeMap& M) {
    return M.belongsToWorld(X.p_position);
}


void ParticleLocalization::teleportParticles(const base::samples::RigidBodyState& pose)
{
    for(unsigned i = 0; i < particles.size(); i++) {
        particles[i].p_position = pose.position;
        particles[i].main_confidence = 1.0 / particles.size();
        particles[i].part_confidences = Eigen::Vector3d(0.0, 0.0, 0.0);
    }
}


void ParticleLocalization::setCurrentOrientation(const base::samples::RigidBodyState& orientation)
{
    vehicle_pose.time = timestamp;
    vehicle_pose.orientation = orientation.orientation * Eigen::AngleAxis<double>(filter_config.yaw_offset, Eigen::Vector3d::UnitZ());
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
