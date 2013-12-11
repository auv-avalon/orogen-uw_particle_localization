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
  Eigen::Matrix3d m;
  m.setZero();
  _param_sqDamp.set(m);
  _param_sqDampNeg.set(m);
  m(0,0) = 8.203187564;
  m(1,1) = 24.94216;
  _param_linDamp.set(m);
  _param_linDampNeg.set(m);
  
  Eigen::Vector3d v;
  v.setZero();
  _param_centerOfBuoyancy.set(v);
  _param_centerOfGravity.set(v);
  
  Eigen::Vector3d v_sonar, v_gps, v_buoy_cam, v_buoy_rotation, v_pipeline;
  v_sonar << -0.5, 0.0, 0.0;
  _sonar_position.set(v_sonar);
  v_gps.setZero();
  _gps_position.set(v_gps);
  v_buoy_cam << 0.7, 0.0, 0.0;
  _buoy_cam_position.set(v_buoy_cam);
  v_buoy_rotation.setZero();
  _buoy_cam_rotation.set(v_buoy_rotation);
  v_pipeline << -0.7, 0.0, -2.0;
  _pipeline_position.set(v_pipeline);  
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
}

Task::~Task()
{
  //delete localizer;
  //delete map;
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
      
     FilterConfig config;
     
     if(_yaml_map.value().empty()){
       std::cout << "ERROR: No yaml-map given" << std::endl;
       return false;
     }else{  
      std::cout << "Setup NodeMap" << std::endl;
      map = new NodeMap(_yaml_map.value());
      env = map->getEnvironment();
      config.env = &env;
      config.useMap = true;
     } 
     
     
     config.particle_number = _particle_number.value();
     config.perception_history_number = _perception_history_number.value();
     config.sonar_maximum_distance = _sonar_maximum_distance.value();
     config.sonar_minimum_distance = _sonar_minimum_distance.value();
     config.sonar_covariance = _sonar_covariance.value();
     config.pipeline_covariance = _pipeline_covariance.value();
     config.pure_random_motion = _pure_random_motion.value();
     
     config.sonar_vertical_angle = _sonar_vertical_angle.value();
     config.sonar_covariance_reflection_factor = _sonar_covariance_reflection_factor.value();
     config.sonar_covariance_corner_factor = _sonar_covariance_corner_factor.value();
     
     config.utm_relative_angle = _utm_relative_angle.value();
     config.gps_covarianz = _gps_covarianz.value();
     config.gps_interspersal_ratio = _gps_interspersal_ratio.value();

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
     number_gps_perceptions = 0;

     std::cout << "Setup static motion covariance" << std::endl;

     if(_static_motion_covariance.value().size() > 0) {
         config.static_motion_covariance = convertProperty<Eigen::Matrix3d>(_static_motion_covariance.value());
     } else {
         std::cout << "No static motion covariance assigned. Use standard matrix" << std::endl;
         Eigen::Matrix3d id = Eigen::Matrix3d::Identity();         
         config.static_motion_covariance = id;
         
         std::vector<double> value;
         for(unsigned i = 0; i < config.static_motion_covariance.rows(); i++) {
             for(unsigned j = 0; j < config.static_motion_covariance.cols(); j++) {
                 value.push_back(config.static_motion_covariance(i,j));
             }
         }

         _static_motion_covariance.set(value);
     }
      
    config.param_length = _param_length.value();
    config.param_radius = _param_radius.value();;
    config.param_mass = _param_mass.value();
    config.param_centerOfGravity = _param_centerOfGravity.value();
    config.param_centerOfBuoyancy = _param_centerOfBuoyancy.value();
    
    if(_param_thrusterCoefficient.value().size() < 18){
	std::cout << "No valid thruster coefficients assigned. Use standart coefficients" << std::endl;  
	//config.param_thrusterCoefficient.clear();
	//config.param_thrusterCoefficient.resize(6,0.005);
	
	double values[] = {0.000, 0.000, -0.005, -0.005, 0.005, -0.005};
	config.param_thrusterCoefficient = std::vector<double>(values, values + sizeof(values)/sizeof(double)) ;
	
	config.param_linearThrusterCoefficient.clear();
	config.param_linearThrusterCoefficient.insert(config.param_linearThrusterCoefficient.begin(), 6, 0.0);
	
	config.param_squareThrusterCoefficient.clear();
	config.param_squareThrusterCoefficient.insert(config.param_squareThrusterCoefficient.begin(), 6, 0.0);
	
    }else{
	config.param_thrusterCoefficient = _param_thrusterCoefficient.value();
	
	config.param_thrusterCoefficient.clear();
	config.param_thrusterCoefficient.insert( config.param_thrusterCoefficient.begin(), 
						_param_thrusterCoefficient.value().begin(), _param_thrusterCoefficient.value().begin() + 6);
	
	config.param_linearThrusterCoefficient.clear();
	config.param_linearThrusterCoefficient.insert( config.param_linearThrusterCoefficient.begin(),
						      _param_thrusterCoefficient.value().begin() + 6, _param_thrusterCoefficient.value().begin() + 12);
	
	config.param_squareThrusterCoefficient.clear();
	config.param_squareThrusterCoefficient.insert( config.param_squareThrusterCoefficient.begin(),
						      _param_thrusterCoefficient.value().begin() + 12, _param_thrusterCoefficient.value().end());
    }
      
    
    config.param_thrusterVoltage = _param_thrusterVoltage.value();
    
    if(_param_TCM.value().size() < 36){
      std::cout << "No valid TCM assigned. Use standard TCM" << std::endl;
      double values[] = { 0.0, 0.0, 1.0, 1.0, 0.0, 0.0,
			    0.0, 0.0, 0.0, 0.0, 1.0, 1.0,
			    1.0, 1.0, 0.0, 0.0, 0.0, 0.0,
			    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
			    -0.92, 0.205, 0.0, 0.0, 0.0, 0.0,
			    0.0, 0.0, -0.17, 0.17, -0.81, 0.04};
	config.param_TCM = std::vector<double>(values, values + sizeof(values)/sizeof(double));		    
    }else{
      config.param_TCM = _param_TCM.value();
    }
    
    
    if(_param_linDamp.get().rows() != 6 && _param_linDamp.get().cols() != 6 &&
      _param_linDampNeg.get().rows() != 6 && _param_linDampNeg.get().cols() != 6 &&
      _param_sqDamp.get().rows() != 6 && _param_sqDamp.get().cols() != 6 &&
      _param_sqDampNeg.get().rows() != 6 && _param_sqDampNeg.get().cols() != 6){
      
	std::cout << "Error: All damping matrix need to have 6 dimensions!" << std::endl;
	return false;
    }
    
    //Dirty, converting dynamic matrix to static matrix
    config.param_linDamp = _param_linDamp.get().block<6,6>(0,0);
    config.param_sqDamp = _param_sqDamp.get().block<6,6>(0,0);
    config.param_linDampNeg = _param_linDampNeg.get().block<6,6>(0,0);
    config.param_sqDampNeg = _param_sqDampNeg.get().block<6,6>(0,0);

    config.param_floating = _param_floating.value(); 
    
    config.advanced_motion_model = _advanced_motion_model.value();
    
    config.sonarToAvalon = Eigen::Translation3d(_sonar_position.get());
    config.pipelineToAvalon = _pipeline_position.get();
    config.gpsToAvalon = _gps_position.get();
    
    config.buoyCamPosition = _buoy_cam_position.get();
    config.buoyCamRotation = eulerToQuaternion( _buoy_cam_rotation.get());
    
    orientation_sample_recieved = false;
          
     //delete localizer;
     localizer = new ParticleLocalization(config);
     localizer->initialize(config.particle_number, config.init_position, config.init_variance, 0.0, 0.0);
          
     localizer->setSonarDebug(this);

     return true;
}

void Task::updateHook()
{
     TaskBase::updateHook();
   
     if(_debug.value() && !_yaml_map.value().empty())
       _environment.write(map->getEnvironment());
     
     base::samples::RigidBodyState pose = localizer->estimate();
     base::samples::RigidBodyState motion = localizer->dead_reckoning();
     base::samples::RigidBodyState full_motion = localizer->full_dead_reckoning();
     pose.angular_velocity = motion.angular_velocity;
     //pose.velocity[0]=0.0;
     //pose.velocity[1]=0.0;
     //pose.angular_velocity[2]=0.0;
     
     if(_debug.value())
        _particles.write(localizer->getParticleSet());

     //if(!pose.time.isNull())
	pose.position[2] = current_depth;
	pose.velocity[2] = motion.velocity[2];
        _pose_samples.write(pose);     

     if(!motion.time.isNull())
       _dead_reckoning_samples.write(motion);
     
     if(!full_motion.time.isNull() && _advanced_motion_model)
       _full_dead_reckoning.write(full_motion);
     
     battery_management::batteryInformation batteryInfo;
     /* //TODO RE-Add
     while(_battery_status.read(batteryInfo)==RTT::NewData){
       double voltage = 0.0;
       
       for(int i =0; i<8; i++){
	 voltage += batteryInfo.cellVoltage[i];
       }
       localizer->setThrusterVoltage(voltage);
       
     }  
     */
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
    last_perception = ts;

    switch(pipeline.inspection_state) {
        case controlData::FOUND_PIPE:
        case controlData::FOLLOW_PIPE:
        case controlData::END_OF_PIPE:
        case controlData::ALIGN_AUV:
            localizer->observe(pipeline, *map, _pipeline_importance.value());
            break;
        default:
            break;
    }
}

void Task::orientation_samplesCallback(const base::Time& ts, const base::samples::RigidBodyState& rbs)
{   
    orientation_sample_recieved = true;
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
  if(orientation_sample_recieved){    
    localizer->update(status);    
    localizer->update_dead_reckoning(status);
  }else{
    std::cout << "No initial orientation-sample recieved" << std::endl;
  }  
    
}

void Task::thruster_commandsCallback(const base::Time& ts, const base::actuators::Command& rbs){
  base::actuators::Status status;
  status.time = rbs.time;
  status.resize(rbs.mode.size());
  
  for(int i=0; i < rbs.mode.size(); i++){
     status.states[i].pwm = rbs.target[i]; 
  }
  
  thruster_samplesCallback(ts, status);
  
}

void Task::gps_pose_samplesCallback(const base::Time& ts, const base::samples::RigidBodyState& rbs){
    
  double Neff = localizer->observeAndDebug(rbs,*map,_gps_importance.value());
  
  number_gps_perceptions++;
  
  if(number_gps_perceptions >= _minimum_perceptions.value()) {
        localizer->resample();
        number_gps_perceptions = 0;
    }  
}


void Task::buoy_samplesCallback(const base::Time& ts, const avalon::feature::Buoy& buoy){
  last_perception = ts;
  
  double effective_sample_size = localizer->observe(buoy, *map, _buoy_importance.value());
  
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
    if(_debug.value())
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

