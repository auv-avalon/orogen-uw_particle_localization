/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include "ParticleLocalization.hpp"
#include "Fir.hpp"
#include <aggregator/StreamAligner.hpp>
#include <Eigen/Core>

using namespace uw_particle_localization;
using namespace uw_localization;

Task::Task(std::string const& name)
    : TaskBase(name)
{
  map = 0;

  
  Eigen::Vector3d v_sonar, v_gps, v_buoy_cam, v_buoy_rotation, v_pipeline, v_dvl_rotation;;
  v_sonar << -0.5, 0.0, 0.0;
  _sonar_position.set(v_sonar);
  v_gps.setZero();
  _gps_position.set(v_gps);
  v_dvl_rotation << 0.0, 0.0, 0.25 * M_PI;
  _dvl_rotation.set(v_dvl_rotation);
  
  localizer = 0;
  map = 0;

}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
  
  map = 0;

  
  Eigen::Vector3d v_sonar, v_gps, v_buoy_cam, v_buoy_rotation, v_pipeline, v_dvl_rotation;;
  v_sonar << -0.5, 0.0, 0.0;
  _sonar_position.set(v_sonar);
  v_gps.setZero();
  _gps_position.set(v_gps);
  v_dvl_rotation << 0.0, 0.0, 0.25 * M_PI;
  _dvl_rotation.set(v_dvl_rotation);
  
  localizer = 0;
  map = 0;
  
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
      
 
     if(_yaml_map.value().empty()){
       std::cout << "ERROR: No yaml-map given" << std::endl;
       return false;
     }else{  
      std::cout << "Setup NodeMap" << std::endl;
      map = new NodeMap();
      if(!map->fromYaml(_yaml_map.value())){
        std::cerr << "ERROR: No map could be load " << _yaml_map.value().c_str() << std::endl;
        return false;
      }
     
      env = map->getEnvironment();      
      
      config.env = &env;
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
     
     config.yaw_offset = _orientation_offset.get();

     if(!_init_position.value().empty() && !_init_variance.value().empty()) {
         config.init_position = convertProperty<Eigen::Vector3d>(_init_position.value());
         config.init_variance = convertProperty<Eigen::Vector3d>(_init_variance.value());
     } else {
         config.init_position = base::Vector3d(0.0, 0.0, 0.0);
         config.init_variance = map->getLimitations();
     }

     current_depth = 0.0;
     current_ground = -8.0;

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

    
    config.sonarToAvalon = Eigen::Translation3d(_sonar_position.get());

    config.gpsToAvalon = _gps_position.get();
   
    config.dvlRotation = eulerToQuaternion( _dvl_rotation.get()); 
    
    config.filterZeros = _filter_zeros.get();
    
    config.use_markov = _use_markov.get();
    config.avg_particle_position = _avg_particle_position.get();
    config.use_best_feature_only = _use_best_feature_only.get();   
 
    
    orientation_sample_recieved = false;
          
     //delete localizer;
     localizer = new ParticleLocalization(config);
     localizer->initialize(config.particle_number, config.init_position, config.init_variance, 0.0, 0.0);
 
          
     localizer->setSonarDebug(this);
     
     last_hough_timeout = base::Time::fromMicroseconds(0);
     last_speed_time = base::Time::fromMicroseconds(0);

     position_jump_detected = false;
     last_scan_angle = 0.0;
     
     return true;
}

void Task::updateHook()
{
     TaskBase::updateHook();
     
     //Read pose sample updates
     base::samples::RigidBodyState rbs;
     while(_pose_update.read(rbs) == RTT::NewData){
       
       pose_updateCallback(rbs.time, rbs);       
     }

   
     if(_debug.value() && !_yaml_map.value().empty()){
             
       if(base::Time::now().toSeconds() - last_map_update.toSeconds() > 0.1){

          updateConfig();
          _environment.write(map->getEnvironment());
          _particles.write(localizer->getParticleSet());            

          last_map_update = base::Time::now();
       }
       
     }     
     
     base::samples::RigidBodyState pose;
     if(_avg_particle_position.get()){
        pose = localizer->estimate_middle();
     }
     else{
       pose = localizer->estimate();
     }
     
     
     base::samples::RigidBodyState motion = localizer->dead_reckoning();
     base::samples::RigidBodyState full_motion = localizer->full_dead_reckoning();
     pose.angular_velocity = motion.angular_velocity;
     //pose.velocity[0]=0.0;
     //pose.velocity[1]=0.0;
     //pose.angular_velocity[2]=0.0;
     
     //Convert velocities to world frame
     pose.velocity = pose.orientation * pose.velocity;
     motion.velocity = motion.orientation * motion.velocity;
     full_motion.velocity = full_motion.orientation * full_motion.velocity;

     //Corect position covariance by a threshold
     double sigma_square = std::pow(_position_covariance_threshold.value(), 2);
     for(int i = 0; i < 3; i++){
     
        if(pose.cov_position(i,i) < sigma_square){
          pose.cov_position(i,i) = sigma_square;
        }       
     }     
     
     
     if(!pose.time.isNull()){

        _pose_samples.write(pose);
        lastRBS = pose;
     }
      
     if(!motion.time.isNull())
       _dead_reckoning_samples.write(motion);
     
     if(!full_motion.time.isNull())
       _full_dead_reckoning.write(full_motion);
     
     /* //TODO RE-Add
     battery_management::batteryInformation batteryInfo;
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
  //base::Time temp = base::Time::now();
  
  double scan_diff = std::fabs(last_scan_angle - scan.start_angle);
  
  while(scan_diff > M_PI)
    scan_diff -= 2.0 * M_PI;
  
  last_scan_angle = scan.start_angle;
  sum_scan += std::fabs(scan_diff);  
    
  if(perception_state_machine(ts)){
  
    //Observe data for all particels
    double Neff = localizer->observeAndDebug(scan, *map, _sonar_importance.value());

    if(localizer->hasStats()) {
      _stats.write(localizer->getStats());
    }

    number_sonar_perceptions++;

    //If we had enough observations and a good particle diffusion -> resample
    if(number_sonar_perceptions >= static_cast<size_t>(_minimum_perceptions.value()) 
            && Neff < _effective_sample_size_threshold.value()) {
      localizer->resample();
      validate_particles();
      number_sonar_perceptions = 0;
     
    } 
    
  }

  
  //std::cout << "Calc-time lasersamples: " << base::Time::now().toSeconds() - temp.toSeconds() << std::endl;
  
}



void Task::orientation_samplesCallback(const base::Time& ts, const base::samples::RigidBodyState& rbs)
{   
    orientation_sample_recieved = true;
    localizer->setCurrentOrientation(rbs);
    localizer->setCurrentAngularVelocity(rbs);
    localizer->setCurrentZVelocity(rbs);
    localizer->setCurrentDepth(rbs);
    
    current_depth = rbs.position.z();

    if(start_time.isNull()) {
        start_time = ts;
    }

    if(!last_perception.isNull() && (ts - last_perception).toSeconds() > _reset_timeout.value()) {
        localizer->initialize(_particle_number.value(), config.init_position, map->getLimitations(), 
                base::getYaw(rbs.orientation), 0.0);
	std::cout << "Initialize" << std::endl;
        last_perception = ts;
        start_time = ts;
	changeState(NO_SONAR);
    }
}


void Task::pose_updateCallback(const base::Time& ts, const base::samples::RigidBodyState& rbs)
{
    last_hough = ts;
    
    if(map->belongsToWorld(rbs.position)){
    
      //There is a jump in the position. we need at least a half scan to validate the new particles!
      if( (lastRBS.position - rbs.position).norm() > 3.0){
        std::cout << "Detected position jump: " << (lastRBS.position - rbs.position).norm() << "m" << std::endl;
        position_jump_detected = true;
        sum_scan = 0.0;
      }
      
      localizer->interspersal(rbs, *map, _hough_interspersal_ratio.value(), false, position_jump_detected);

      number_sonar_perceptions = 0;
    }
    else{
      std::cout << "Hough outside of world: " << rbs.position.transpose() << std::endl;
    }
}


void Task::speed_samplesCallback(const base::Time& ts, const base::samples::RigidBodyState& rbs)
{
    //base::Time temp = base::Time::now();
    if(base::samples::RigidBodyState::isValidValue(rbs.velocity)){
  
      base::samples::RigidBodyState state = rbs;
      state.velocity = config.dvlRotation * rbs.velocity;
      
      localizer->setCurrentVelocity(state);
    
      if(orientation_sample_recieved){
        localizer->update(state, *map);
              
      }else{
        changeState(NO_ORIENTATION);
      }
        
      last_speed_time = ts;
      last_motion = ts;
    }
    //std::cout << "Calc time speed-samples: " << base::Time::now().toSeconds() - temp.toSeconds() << std::endl;
}


void Task::gps_pose_samplesCallback(const base::Time& ts, const base::samples::RigidBodyState& rbs){
    
  localizer->observeAndDebug(rbs,*map,_gps_importance.value());
  
  number_gps_perceptions++;
  
  if(number_gps_perceptions >= _minimum_perceptions.value()) {
        localizer->resample();
        localizer->setParticlesValid();
        number_gps_perceptions = 0;
    }  
}


void Task::stopHook()
{
     TaskBase::stopHook();

     //delete aggr;
     delete localizer;
     delete map;
     
     localizer = 0;
     map = 0;

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




bool Task::perception_state_machine(const base::Time& ts)
{
    if(orientation_sample_recieved){ //we have a valid orientation
      
      if(current_depth < _minimum_depth.get()){ //we have a valid depth
        
        if(last_motion.isNull() || ts.toSeconds() - last_motion.toSeconds() > _reset_timeout.get()) //Joint timeout
           changeState(NO_JOINTS_NO_DVL);
        else if(last_hough.isNull() || ts.toSeconds() - last_hough.toSeconds() > _hough_timeout.get()){ //Hough timeout
          changeState(NO_HOUGH);
          
          //get timedifference to last hpugh timeout, and intersper random particles after a given time
          if(ts.toSeconds() - last_hough.toSeconds() > _hough_timeout.get()){
            
            if(last_hough_timeout.isNull()){
              last_hough_timeout = ts;
            }
            else if(ts.toSeconds() - last_hough_timeout.toSeconds() > _hough_timeout.get()){
              
              last_hough_timeout = ts;
              localizer->interspersal(base::samples::RigidBodyState(), *map, _hough_timeout_interspersal.get(), true, true);              
            }        
            
          }          
          
        }
        else //Everything is fine :-)
          changeState(LOCALIZING);
        
        
        last_perception = ts;

        if(number_rejected_samples < static_cast<unsigned>(_init_sample_rejection.value())) {
            number_rejected_samples++;
            return false;
        }
        
        return true;     
      
        
      }else{ //Invalid depth
        changeState(ABOVE_SURFACE);
      }
    }
    else{ //Invalid or no orientation
      changeState(NO_ORIENTATION);
    }
    return false;
}

void Task::changeState(States new_state){
  
  if(new_state != state()){
    state(new_state);
  }
  
}

void Task::validate_particles(){
  
  //If we had detected a position jump, we want at least a half scan to validate the particles
  if(position_jump_detected){
    if(sum_scan < M_PI)
      return;
    
  }
  
  position_jump_detected = false;
  localizer->setParticlesValid();
}




void Task::updateConfig(){
 
  
    config.sonar_maximum_distance = _sonar_maximum_distance.value();
    config.sonar_minimum_distance = _sonar_minimum_distance.value();
    config.sonar_covariance = _sonar_covariance.value();
    config.pipeline_covariance = _pipeline_covariance.value();  
    
    config.sonar_vertical_angle = _sonar_vertical_angle.value();
    config.sonar_covariance_reflection_factor = _sonar_covariance_reflection_factor.value();
    config.sonar_covariance_corner_factor = _sonar_covariance_corner_factor.value();     

    config.gps_covarianz = _gps_covarianz.value();
    config.gps_interspersal_ratio = _gps_interspersal_ratio.value();    
    
    localizer->updateConfig(config);
}
  

