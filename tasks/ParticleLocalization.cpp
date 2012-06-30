#include "ParticleLocalization.hpp"
#include <base/pose.h>

using namespace machine_learning;

namespace uw_localization {

base::samples::RigidBodyState* PoseParticle::pose = 0;

ParticleLocalization::ParticleLocalization(const FilterConfig& config) 
    : ParticleFilter<PoseParticle, NodeMap>(), filter_config(config), 
    motion_model(VehicleParameter()),
    StaticSpeedNoise(Random::multi_gaussian(Eigen::Vector3d(0.0, 0.0, 0.0), config.static_motion_covariance)),
    sonar_debug(0)
{
    first_perception_received = false;
    PoseParticle::pose = &vehicle_pose;
}

ParticleLocalization::~ParticleLocalization()
{}


UwVehicleParameter ParticleLocalization::VehicleParameter() const
{
    UwVehicleParameter p;

    p.Length = 1.4;
    p.Radius = 0.15;
    p.Mass = 63;

//    p.InertiaTensor << 0.5 * p.Mass * (p.Radius * p.Radius), 0.0, 0.0, 
//        0.0, (1.0 / 12.0) * p.Mass * (3.0 * p.Radius * p.Radius + p.Length * p.Length), 0.0,
//        0.0, 0.0, (1.0 / 12.0) * p.Mass * (3.0 * p.Radius * p.Radius + p.Length * p.Length);

    p.ThrusterCoefficient << 0.000, 0.000, -0.005, -0.005, 0.005, -0.005;
    p.ThrusterVoltage = 25.4;

    p.TCM << 0.0, 0.0, 1.0, // 0.0, -0.92, 0.0, // HEAVE
             0.0, 0.0, 1.0, //0.0, 0.205, 0.0, // HEAVE
             1.0, 0.0, 0.0, //0.0, 0.0, -0.17, // SURGE
             1.0, 0.0, 0.0, //0.0, 0.0, 0.17, // SURGE
             0.0, 1.0, 0.0, //0.0, 0.0, -0.81, // SWAY
             0.0, 1.0, 0.0; //0.0, 0.0, 0.04;  // SWAY

    p.DampingX << 8.0, 0.05;//6.836, 0.761;
    p.DampingY << 58.28, 1.599;
    p.DampingZ << 0.0, -23.8;
    p.floating = true;

    return p;
}



void ParticleLocalization::initialize(int numbers, const Eigen::Vector3d& pos, const Eigen::Vector3d& var, double yaw, double yaw_cov)
{
    UniformRealRandom pos_x = Random::uniform_real(pos.x() - var.x() * 0.5, pos.x() + var.x() * 0.5 );
    UniformRealRandom pos_y = Random::uniform_real(pos.y() - var.y() * 0.5, pos.y() + var.y() * 0.5 );
    UniformRealRandom pos_z = Random::uniform_real(pos.z() - var.z() * 0.5, pos.z() + var.z() * 0.5 );

    particles.clear();

    for(int i = 0; i < numbers; i++) {
        PoseParticle pp;
        pp.p_position = base::Vector3d(pos_x(), pos_y(), pos_z());
        pp.p_velocity = base::Vector3d(0.0, 0.0, 0.0);
        pp.main_confidence = 1.0 / numbers;
        
        particles.push_back(pp);
    }

    generation++;

    motion_pose.position = pos;
}




void ParticleLocalization::dynamic(PoseParticle& X, const base::samples::RigidBodyState& U)
{
    base::Vector3d v_noisy;
    base::Vector3d u_velocity;

    if(filter_config.pure_random_motion)
        u_velocity = base::Vector3d(0.0, 0.0, 0.0);
    else
        u_velocity = U.velocity;

    v_noisy = u_velocity + StaticSpeedNoise();
    
    base::Vector3d v_avg = (X.p_velocity + v_noisy) / 2.0;

    if( !X.timestamp.isNull() ) {
        double dt = (U.time - X.timestamp).toSeconds();

        X.p_position = X.p_position + vehicle_pose.orientation * (v_avg * dt);
    }

    X.p_velocity = v_noisy;
    X.timestamp = U.time;
    X.p_position.z() = vehicle_pose.position.z();
}

void ParticleLocalization::dynamic(PoseParticle& X, const base::actuators::Status& Ut)
{
    Vector6d Xt;

    if( !X.timestamp.isNull() ) {
        double dt = (Ut.time - X.timestamp).toSeconds();

        base::Vector3d v_noisy;
        base::Vector3d u_velocity;

        if(filter_config.pure_random_motion) {
            u_velocity = base::Vector3d(0.0, 0.0, 0.0);
        }  else {
            Xt << X.p_velocity.x(), X.p_velocity.y(), X.p_velocity.z(), 
               X.p_position.x(), X.p_position.y(), X.p_position.z();

            Vector6d U = motion_model.transition(Xt, dt, Ut);

            u_velocity = U.block<3, 1>(0, 0);

        }

        v_noisy = u_velocity + StaticSpeedNoise();

        base::Vector3d v_avg = (X.p_velocity + v_noisy) / 2.0;

        X.p_position = X.p_position + vehicle_pose.orientation * (v_avg * dt);
        X.p_velocity = v_noisy;
        X.p_position.z() = vehicle_pose.position.z();
    } 
    
    X.timestamp = Ut.time;
}

void ParticleLocalization::update_dead_reckoning(const base::actuators::Status& Ut)
{
    if( !motion_pose.time.isNull() ) {
        Vector6d Xt;

        double dt = (Ut.time - motion_pose.time).toSeconds();
        Xt.block<3,1>(0,0) = motion_pose.velocity;
        Xt.block<3,1>(3,0) = base::Vector3d(0.0, 0.0, 0.0);

        Vector6d U = motion_model.transition(Xt, dt, Ut);
        base::Vector3d u_t1 = U.block<3,1>(0,0);

        base::Vector3d v_avg = (motion_pose.velocity + u_t1) / 2.0;

        motion_pose.position = motion_pose.position + vehicle_pose.orientation * (v_avg * dt);
        motion_pose.velocity = u_t1;
    } 

    motion_pose.time = Ut.time;

    motion_pose.orientation = vehicle_pose.orientation;
    motion_pose.position.z() = vehicle_pose.position.z();
}




const base::Time& ParticleLocalization::getTimestamp(const base::samples::RigidBodyState& U)
{
    return U.time;
}

const base::Time& ParticleLocalization::getTimestamp(const base::actuators::Status& U)
{
    return U.time;
}

double ParticleLocalization::observeAndDebug(const base::samples::LaserScan& z, const NodeMap& m, double importance)
{
    double effective_sample_size = observe(z, m, importance);

    best_sonar_measurement.time = z.time;

    sonar_debug->write(best_sonar_measurement);

    best_sonar_measurement.confidence = -1.0;

    return effective_sample_size;
}


double ParticleLocalization::perception(const PoseParticle& X, const base::samples::LaserScan& Z, const NodeMap& M)
{
    uw_localization::PointInfo info;
    info.time = X.timestamp;

    double angle = Z.start_angle;
    double yaw = base::getYaw(vehicle_pose.orientation);
    double z_distance = Z.ranges[0] / 1000.0;

    // check if this particle is still part of the world
    if(!M.belongsToWorld(X.p_position)) {
        debug(z_distance, X.p_position, 0.0, NOT_IN_WORLD); 
        return 0.0;
    }

    // check if current laser scan is in a valid range
    if(Z.ranges[0] == base::samples::TOO_FAR 
            || z_distance > filter_config.sonar_maximum_distance 
            || z_distance < filter_config.sonar_minimum_distance)
    {
        double p = 1.0 / (filter_config.sonar_maximum_distance - filter_config.sonar_minimum_distance);
        debug(z_distance, X.p_position, p, OUT_OF_RANGE);
        return p;
    }
   
    // check current measurement with map
    Eigen::AngleAxis<double> sonar_yaw(angle, Eigen::Vector3d::UnitZ()); 
    Eigen::AngleAxis<double> abs_yaw(yaw, Eigen::Vector3d::UnitZ());
    Eigen::Affine3d SonarToAvalon(Eigen::Translation3d(-0.5, 0.0, 0.0));

    Eigen::Vector3d RelativeZ = sonar_yaw * SonarToAvalon * base::Vector3d(z_distance, 0.0, 0.0);
    Eigen::Vector3d AbsZ = (abs_yaw * RelativeZ) + X.p_position;

    boost::tuple<Node*, double, Eigen::Vector3d> distance = M.getNearestDistance("root.wall", AbsZ, X.p_position);

    double probability = gaussian1d(0.0, filter_config.sonar_covariance, distance.get<1>());

    debug(z_distance, distance.get<2>(), AbsZ, X.p_position, probability);

    first_perception_received = true;

    return probability;
}


void ParticleLocalization::interspersal(const base::samples::RigidBodyState& p, const NodeMap& m)
{
    reduceParticles(1.0 - filter_config.hough_interspersal_ratio);

    PoseParticle best = particles.front();
    PoseParticle last = particles.back();

    MultiNormalRandom<3> Pose = Random::multi_gaussian(p.position, p.cov_position);

    for(size_t i = particles.size(); i < filter_config.particle_number; i++) {
        PoseParticle pp;
        pp.p_position = Pose();
        pp.p_velocity = best.p_velocity;
        pp.main_confidence = best.main_confidence - 0.001;

        particles.push_back(pp);
    }

    normalizeParticles();
}


void ParticleLocalization::debug(double distance, const base::Vector3d& location, double conf, PointStatus status)
{
    if(best_sonar_measurement.confidence < conf) {
        uw_localization::PointInfo info;
        info.distance = distance;
        info.desire_point = base::Vector3d(0.0, 0.0, 0.0);
        info.real_point = base::Vector3d(0.0, 0.0, 0.0);
        info.location = location;
        info.confidence = conf;
        info.status = status;

        best_sonar_measurement = info;
    }
}

void ParticleLocalization::debug(double distance, const base::Vector3d& desire, const base::Vector3d& real, const base::Vector3d& loc, double conf)
{
    if(best_sonar_measurement.confidence < conf) {
        uw_localization::PointInfo info;
        info.distance = distance;
        info.desire_point = desire;
        info.real_point = real;
        info.location = loc;
        info.confidence = conf;
        info.status = OKAY;

        best_sonar_measurement = info;
    }
}


void ParticleLocalization::teleportParticles(const base::samples::RigidBodyState& pose)
{
    std::list<PoseParticle>::iterator it;
    for(it = particles.begin(); it != particles.end(); ++it) {
        it->p_position = pose.position;
        it->main_confidence = 1.0 / particles.size();
    }
}


void ParticleLocalization::setCurrentOrientation(const base::samples::RigidBodyState& orientation)
{
    vehicle_pose.time = timestamp;
    vehicle_pose.orientation = orientation.orientation;
    vehicle_pose.cov_orientation = orientation.cov_orientation;
    vehicle_pose.position = orientation.position;
    vehicle_pose.angular_velocity = orientation.angular_velocity;
}



  
}
