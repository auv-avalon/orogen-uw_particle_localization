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
     config.sonar_covariance = _sonar_covariance.value();

     current_depth = 0;

     number_sonar_perceptions = 0;
    

     if(_static_motion_covariance.value().size() > 0) {
         config.use_static_motion_covariance(convertProperty<Eigen::Matrix3d>(_static_motion_covariance.value()));
         std::cout << "use static motion cov" << std::endl;
     }

     std::cout << "ele: " << _static_motion_covariance.value().size() << std::endl;

     localizer = new ParticleLocalization(config);
     map = new NodeMap(_yaml_map.value());

     localizer->setSonarDebug(this);

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

     while(current_depth < _minimum_depth.value() && _laser_samples.read(laser, false) == RTT::NewData) {
         aggr->push(laser_sid, laser.time, laser);
     }

     while( aggr->step() );

     _streamaligner_status.write(aggr->getStatus());

     MixedMap localization_map = map->getMap();

     _mixedmap.write(localization_map);
     _particles.write(localizer->getParticleSet());
}


void Task::callbackLaser(base::Time ts, const base::samples::LaserScan& scan)
{
    base::Vector3d ratio(_perception_ratio.value(), _noise_ratio.value(), _max_distance_ratio.value());

    double Neff = localizer->observe(scan, *map, ratio, 1.0 / _sonar_maximum_distance.value());

    number_sonar_perceptions++;

    _effective_sample_size.write(Neff);

    if(number_sonar_perceptions >= _minimum_perceptions.value() 
            && Neff < _effective_sample_size_threshold.value()) {
        localizer->resample();
        number_sonar_perceptions = 0;
    }
}



void Task::callbackOrientation(base::Time ts, const base::samples::RigidBodyState& rbs)
{
    localizer->setCurrentOrientation(rbs);
    current_depth = rbs.position.z();
}



void Task::callbackSpeed(base::Time ts, const base::samples::RigidBodyState& rbs)
{
    localizer->update(rbs);
}



void Task::stopHook()
{
     TaskBase::stopHook();

     delete aggr;
     delete localizer;
     delete map;
}


void Task::write(const uw_localization::debug::SonarPerception& sample)
{
    _sonar_perception.write(sample);
}

// void Task::errorHook()
// {
//     TaskBase::errorHook();
// }


// void Task::cleanupHook()
// {
//     TaskBase::cleanupHook();
// }

