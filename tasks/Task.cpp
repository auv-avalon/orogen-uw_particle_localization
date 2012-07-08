/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include "ParticleLocalization.hpp"
#include "Fir.hpp"
#include <aggregator/StreamAligner.hpp>

using namespace uw_particle_localization;
using namespace uw_localization;

Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
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

     
     std::cout << "Setup NodeMap" << std::endl;
     map = new NodeMap(_yaml_map.value());
     
     FilterConfig config;
     config.particle_number = _particle_number.value();
     config.perception_history_number = _perception_history_number.value();
     config.sonar_maximum_distance = _sonar_maximum_distance.value();
     config.sonar_minimum_distance = _sonar_minimum_distance.value();
     config.sonar_covariance = _sonar_covariance.value();
     config.pure_random_motion = _pure_random_motion.value();

     if(!_init_position.value().empty() && !_init_variance.value().empty()) {
         config.init_position = convertProperty<Eigen::Vector3d>(_init_position.value());
         config.init_variance = convertProperty<Eigen::Vector3d>(_init_variance.value());
     } else {
         config.init_position = base::Vector3d(0.0, 0.0, 0.0);
         config.init_variance = map->getLimitations();
     }

     current_depth = 0;

     number_sonar_perceptions = 0;
     number_rejected_samples = 0;

     std::cout << "Setup Static motion covariance" << std::endl;

     if(_static_motion_covariance.value().size() > 0) {
         config.static_motion_covariance = convertProperty<Eigen::Matrix3d>(_static_motion_covariance.value());
     } else {
         std::cout << "No static motion covariance assigned. Use standard matrix" << std::endl;
         Eigen::Matrix3d id = Eigen::Matrix3d::Identity();
         id(2,2) = 0.0;
         config.static_motion_covariance = id;
         
         std::vector<double> value;
         for(unsigned i = 0; i < config.static_motion_covariance.rows(); i++) {
             for(unsigned j = 0; j < config.static_motion_covariance.cols(); j++) {
                 value.push_back(config.static_motion_covariance(i,j));
             }
         }

         _static_motion_covariance.set(value);
     }

     localizer = new ParticleLocalization(config);
     localizer->initialize(config.particle_number, config.init_position, config.init_variance, 0.0, 0.0);
     
     localizer->setSonarDebug(this);

     return true;
}

void Task::updateHook()
{
     TaskBase::updateHook();
   
     _environment.write(map->getEnvironment());
     
     base::samples::RigidBodyState pose = localizer->estimate();
     base::samples::RigidBodyState motion = localizer->dead_reckoning();

     _particles.write(localizer->getParticleSet());

     if(!pose.time.isNull()) 
        _pose_samples.write(pose);

     if(!motion.time.isNull())
       _dead_reckoning_samples.write(motion);
}


void Task::laser_samplesCallback(const base::Time& ts, const base::samples::LaserScan& scan)
{
    last_perception = ts;

    if(number_rejected_samples < static_cast<unsigned>(_init_sample_rejection.value())) {
        number_rejected_samples++;
        return;
    }

    double Neff = localizer->observeAndDebug(scan, *map, _sonar_importance.value());

    if(localizer->hasStats()) {
        _stats.write(localizer->getStats());
    }

    number_sonar_perceptions++;

    if(number_sonar_perceptions >= static_cast<size_t>(_minimum_perceptions.value()) 
            && Neff < _effective_sample_size_threshold.value()) {
        localizer->resample();
        number_sonar_perceptions = 0;
    }
}


void Task::pipeline_samplesCallback(const base::Time& ts, const controlData::Pipeline& pipeline) 
{

}

void Task::orientation_samplesCallback(const base::Time& ts, const base::samples::RigidBodyState& rbs)
{
    localizer->setCurrentOrientation(rbs);
    current_depth = rbs.position.z();

    if(start_time.isNull()) {
        start_time = ts;
    }

    if(!last_perception.isNull() && (ts - last_perception).toSeconds() > _reset_timeout.value()) {
        localizer->initialize(_particle_number.value(), base::Vector3d(0.0, 0.0, 0.0), map->getLimitations(), 
                base::getYaw(rbs.orientation), 0.0); 
        last_perception = ts;
        start_time = ts;
    }
}


void Task::pose_updateCallback(const base::Time& ts, const base::samples::RigidBodyState& rbs)
{
    last_perception = ts;

    localizer->interspersal(rbs, *map, _hough_interspersal_ratio.value());

    number_sonar_perceptions = 0;
}


void Task::speed_samplesCallback(const base::Time& ts, const base::samples::RigidBodyState& rbs)
{
    localizer->update(rbs);
}


void Task::thruster_samplesCallback(const base::Time& ts, const base::actuators::Status& status)
{
    localizer->update(status);
    localizer->update_dead_reckoning(status);
}


void Task::stopHook()
{
     TaskBase::stopHook();

     //delete aggr;
     delete localizer;
     delete map;
}


void Task::write(const uw_localization::PointInfo& sample)
{
    _debug_sonar_beam.write(sample);
}

// void Task::errorHook()
// {
//     TaskBase::errorHook();
// }


// void Task::cleanupHook()
// {
//     TaskBase::cleanupHook();
// }

