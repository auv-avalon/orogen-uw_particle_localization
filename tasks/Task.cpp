/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include "ParticleLocalization.hpp"
#include "Fir.hpp"
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

     hough_sid = aggr->registerStream<base::samples::RigidBodyState>(
             boost::bind(&Task::callbackSpeed, this, _1, _2),
             size_factor * _max_sample_delay.value() / _hough_period.value(),
             base::Time::fromSeconds(_hough_period.value()));

     gt_sid = aggr->registerStream<base::samples::RigidBodyState>(
             boost::bind(&Task::callbackGroundtruth, this, _1, _2),
             size_factor * _max_sample_delay.value() / _groundtruth_period.value(),
             base::Time::fromSeconds(_groundtruth_period.value()));

     FilterConfig config;
     config.particle_number = _particle_number.value();
     config.particle_interspersal_ratio = _particle_interspersal_ratio.value();
     config.sonar_maximum_distance = _sonar_maximum_distance.value();
     config.sonar_minimum_distance = _sonar_minimum_distance.value();
     config.sonar_covariance = _sonar_covariance.value();
     config.yaw_offset = _yaw_offset.value();
     config.pure_random_motion = _pure_random_motion.value();

     weights = MovingAverage(_aliasing_buffer_size.value());

     if(!_init_position.value().empty() && !_init_variance.value().empty()) {
         config.init_position = convertProperty<Eigen::Vector3d>(_init_position.value());
         config.init_variance = convertProperty<Eigen::Vector3d>(_init_variance.value());
     }

     current_depth = 0;

     number_sonar_perceptions = 0;

     if(_static_motion_covariance.value().size() > 0) {
         config.use_static_motion_covariance(convertProperty<Eigen::Matrix3d>(_static_motion_covariance.value()));
     }

     localizer = new ParticleLocalization(config);
     map = new NodeMap(_yaml_map.value());

     if(_init_position.value().empty() || _init_variance.value().empty()) 
        localizer->initialize(_particle_number.value(), base::Vector3d(0.0, 0.0, 0.0), map->getLimitations(), 
                0.0, 0.0); 
     else
        localizer->initialize(_particle_number.value(), config.init_position, config.init_variance, 0.0, 0.0);

     localizer->setSonarDebug(this);

     return true;
}


void Task::step(const base::samples::RigidBodyState& sample)
{
    if(!_aliasing_buffer_size.value() > 0)
        _pose_samples.write(sample);

    buffer.push_front(sample);

    if(buffer.size() == static_cast<size_t>(_aliasing_buffer_size.value())) {
        base::samples::RigidBodyState& back = buffer.back();

        unsigned i = 0;
        std::list<base::samples::RigidBodyState>::const_iterator it = buffer.begin();

        base::Vector3d position = base::Vector3d::Zero();

        while(it != buffer.end()) {
            position += weights[i] * it->position;
            ++i;
            ++it;
        }

        back.position = position;

        if(state() == RUNNING)
            _pose_samples.write(back);

        buffer.pop_back();
    }
}


void Task::updateHook()
{
     TaskBase::updateHook();
   
     base::samples::RigidBodyState orientation;
     base::samples::RigidBodyState speed;
     base::samples::RigidBodyState hough;
     base::samples::RigidBodyState gt;
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

     while(_pose_update.read(hough, false) == RTT::NewData) {
         aggr->push(hough_sid, hough.time, hough);
     }

     while(_ground_truth.read(gt, false) == RTT::NewData) {
         aggr->push(gt_sid, gt.time, gt);
     }


     while( aggr->step() );

     _streamaligner_status.write(aggr->getStatus());

     _environment.write(map->getEnvironment());
     
     base::samples::RigidBodyState pose = localizer->estimate();

     _particles.write(localizer->getParticleSet());

     if(!pose.time.isNull()) {
         step(pose);
     }
}


void Task::callbackLaser(base::Time ts, const base::samples::LaserScan& scan)
{
    double Neff = localizer->observeAndDebug(scan, *map, _sonar_importance.value());

    number_sonar_perceptions++;

    if(number_sonar_perceptions >= static_cast<size_t>(_minimum_perceptions.value()) 
            && Neff < _effective_sample_size_threshold.value()) {
        localizer->resample();
        number_sonar_perceptions = 0;
    }

    last_perception = ts;
}



void Task::callbackOrientation(base::Time ts, const base::samples::RigidBodyState& rbs)
{
    localizer->setCurrentOrientation(rbs);
    current_depth = rbs.position.z();

    if(start_time.isNull()) {
        start_time = ts;
    }

    if((ts - start_time).toSeconds() > _waiting_time.value())
        state(RUNNING);
    else 
        state(WAITING);

    if(!last_perception.isNull() && (ts - last_perception).toSeconds() > _reset_timeout.value()) {
        localizer->initialize(_particle_number.value(), base::Vector3d(0.0, 0.0, 0.0), map->getLimitations(), 
                base::getYaw(rbs.orientation), 0.0); 
        last_perception = ts;
        start_time = ts;
    }
}


void Task::callbackGroundtruth(base::Time ts, const base::samples::RigidBodyState& rbs)
{
    localizer->teleportParticles(rbs);
}


void Task::callbackHough(base::Time ts, const base::samples::RigidBodyState& rbs)
{
    last_perception = ts;
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


void Task::write(const uw_localization::ParticleInfo& sample)
{
    _debug_sonar.write(sample);
}

// void Task::errorHook()
// {
//     TaskBase::errorHook();
// }


// void Task::cleanupHook()
// {
//     TaskBase::cleanupHook();
// }

