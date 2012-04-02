/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include "ParticleLocalization.hpp"
#include <aggregator/StreamAligner.hpp>

using namespace uw_particle_localization;
using namespace uw_localization;

Task::Task(std::string const& name, TaskCore::TaskState initial_state)
    : TaskBase(name, initial_state)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : TaskBase(name, engine, initial_state)
{
}

Task::~Task()
{
}


// bool Task::configureHook()
// {
//     if (! TaskBase::configureHook())
//         return false;
//     return true;
// }


bool Task::startHook()
{
     if (! TaskBase::startHook())
         return false;

     aggr = new aggregator::StreamAligner;
     aggr->setTimeout(base::Time::fromSeconds(_max_sample_delay.value()));

     const double size_factor = 2.0;

     laser_sid = aggr->registerStream<base::samples::LaserScan>(
             boost::bind(&Task::callbackLaser, this, _1, _2),
             size_factor * _max_sample_delay.value() / _laser_period.value(),
             base::Time::fromSeconds(_laser_period.value()));

     orientation_sid = aggr->registerStream<base::samples::RigidBodyState>(
             boost::bind(&Task::callbackOrientation, this, _1, _2),
             size_factor * _max_sample_delay.value() / _orientation_period.value(),
             base::Time::fromSeconds(_orientation_period.value()));

     speed_sid = aggr->registerStream<base::samples::RigidBodyState>(
             boost::bind(&Task::callbackSpeed, this, _1, _2),
             size_factor * _max_sample_delay.value() / _speed_period.value(),
             base::Time::fromSeconds(_speed_period.value()));

     FilterConfig config;
     config.particle_number = _particle_number.value();
     config.particle_interspersal_ratio = _particle_interspersal_ratio.value();
     config.init_position = convertProperty<Eigen::Vector3d>(_init_position.value());
     config.init_covariance = convertProperty<Eigen::Matrix3d>(_init_covariance.value());
     config.sonar_maximum_distance = _sonar_maximum_distance.value();
     config.sonar_covariance = convertProperty<Eigen::Matrix3d>(_sonar_covariance.value());
     
     if(_static_motion_covariance.value().size() > 0) {
         config.use_static_motion_covariance(convertProperty<Eigen::Matrix3d>(_static_motion_covariance.value()));
     }

     localizer = new ParticleLocalization(config);

     Eigen::Vector3d a(0.0, 0.0, -1.0);
     Eigen::Vector3d b(5.0, 0.0, -1.0);

     Eigen::Matrix3d a_v(3,3);

     a_v << 0.1, 0.0, 0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 1.0;

     Node* group = new Node("root");
     group->addChild(new LandmarkNode(a, a_v));
     group->addChild(new LandmarkNode(b, a_v));

     Eigen::Vector3d limits(10.0, 10.0, 10.0);
     Eigen::Translation3d t(0.0, 0.0, 0.0);

     map = new StochasticMap(limits, t, group);

     return true;
}


void Task::updateHook()
{
     TaskBase::updateHook();

     base::samples::RigidBodyState orientation;
     base::samples::RigidBodyState speed;
     base::samples::LaserScan laser;

     while(_orientation_samples.read(orientation, false) == RTT::NewData) {
         aggr->push(orientation_sid, orientation.time, orientation);                 
     }

     while(_speed_samples.read(speed, false) == RTT::NewData) {
         aggr->push(speed_sid, speed.time, speed);
     }

     while(_laser_samples.read(laser, false) == RTT::NewData) {
         aggr->push(laser_sid, laser.time, laser);
     }

     _streamaligner_status.write(aggr->getStatus());

     LandmarkMap marks = map->getMap();

     _landmarks.write(marks);
     _particles.write(localizer->getParticleSet());
}


void Task::callbackLaser(base::Time ts, const base::samples::LaserScan& scan)
{
}



void Task::callbackOrientation(base::Time ts, const base::samples::RigidBodyState& rbs)
{
}



void Task::callbackSpeed(base::Time ts, const base::samples::RigidBodyState& rbs)
{
}



void Task::stopHook()
{
     TaskBase::stopHook();

     delete aggr;
     delete localizer;
     delete map;
}


// void Task::errorHook()
// {
//     TaskBase::errorHook();
// }


// void Task::cleanupHook()
// {
//     TaskBase::cleanupHook();
// }

