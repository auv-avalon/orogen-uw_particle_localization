/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include "ParticleLocalization.hpp"
#include <aggregator/StreamAligner.hpp>

using namespace uw_particle_localization;

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
}


// void Task::errorHook()
// {
//     TaskBase::errorHook();
// }


// void Task::cleanupHook()
// {
//     TaskBase::cleanupHook();
// }

