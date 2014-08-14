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
  grid_map = 0;
  base::Matrix6d m;
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
  
  localizer = 0;
  map = 0;
  grid_map = 0;

}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
  
  map = 0;
  grid_map = 0;  
  base::Matrix6d m;
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
  
  localizer = 0;
  map = 0;
  grid_map = 0;  
  
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
      grid_map = new DepthObstacleGrid( base::Vector2d(-map->getTranslation().x(), -map->getTranslation().y() ),
                              1.2 * base::Vector2d(map->getLimitations().x(), map->getLimitations().y() ), _feature_grid_resolution.get());
      grid_map->initGrid();
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
     
     if(_static_speed_covariance.value().size() > 0) {
         config.static_speed_covariance = convertProperty<Eigen::Matrix3d>(_static_speed_covariance.value());
     } else {
         std::cout << "No static speed covariance assigned. Use static motion matrix" << std::endl;
         config.static_speed_covariance = config.static_motion_covariance;
         
         std::vector<double> value;
         for(unsigned i = 0; i < config.static_speed_covariance.rows(); i++) {
             for(unsigned j = 0; j < config.static_speed_covariance.cols(); j++) {
                 value.push_back(config.static_speed_covariance(i,j));
             }
         }

         _static_speed_covariance.set(value);         
         
     }     
     
     
      
    if (!initMotionConfig())
      return false;
    
    config.advanced_motion_model = _advanced_motion_model.value();
    config.max_velocity_drift = _max_velocity_drift.value();
    
    config.sonarToAvalon = Eigen::Translation3d(_sonar_position.get());
    config.pipelineToAvalon = _pipeline_position.get();
    config.gpsToAvalon = _gps_position.get();
    
    config.buoyCamPosition = _buoy_cam_position.get();
    config.buoyCamRotation = eulerToQuaternion( _buoy_cam_rotation.get());    
     
    config.filterZeros = _filter_zeros.get();
    
    config.use_markov = _use_markov.get();
    config.avg_particle_position = _avg_particle_position.get();
    config.use_best_feature_only = _use_best_feature_only.get();
    
    config.use_slam = _use_slam.get();
    config.use_mapping_only = _use_mapping_only.get();
    config.feature_grid_resolution = _feature_grid_resolution.get();
    config.feature_weight_reduction = _feature_weight_reduction.get();
    config.feature_observation_range = _feature_observation_range.get();
    config.feature_observation_minimum_range = _feature_observation_minimum_range.get();
    config.feature_filter_threshold = _feature_filter_threshold.get();
    config.feature_confidence = _feature_confidence.get();
    config.feature_empty_cell_confidence = _feature_empty_cell_confidence.get();
    config.feature_confidence_threshold = _feature_confidence_threshold.get();
    config.feature_output_confidence_threshold = _feature_output_confidence_threshold.get();
    config.feature_observation_count_threshold = _feature_observation_count_threshold.get();
    config.echosounder_variance = _echosounder_variance.get();
    
    orientation_sample_recieved = false;
          
     //delete localizer;
     localizer = new ParticleLocalization(config);
     localizer->initialize(config.particle_number, config.init_position, config.init_variance, 0.0, 0.0);
     
     if(_use_slam && map){
       
       localizer->init_slam(map);
       
     }
          
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
       _environment.write(map->getEnvironment());
      
       if(base::Time::now().toSeconds() - last_map_update.toSeconds() > 2.0){
        
          base::samples::Pointcloud pc;
          
          if(_use_slam.get()){
            pc = localizer->getPointCloud();
          }
          else{
            pc = grid_map->getCloud();
          }
          
            
          pc.time = base::Time::now();
          _depth_grid.write( pc);
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
    
     
     if(_debug.value())
        _particles.write(localizer->getParticleSet());

     if(!pose.time.isNull()){

        _pose_samples.write(pose);
        lastRBS = pose;
     }
      
      
     if(!motion.time.isNull())
       _dead_reckoning_samples.write(motion);
     
     if(!full_motion.time.isNull() && _advanced_motion_model)
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
  double scan_diff = std::fabs(last_scan_angle - scan.start_angle);
  
  while(scan_diff > M_PI)
    scan_diff -= 2.0 * M_PI;
  
  last_scan_angle = scan.start_angle;
  sum_scan += std::fabs(scan_diff);  
    
  if(perception_state_machine(ts)){
  
    double Neff = localizer->observeAndDebug(scan, *map, _sonar_importance.value());

    if(localizer->hasStats()) {
      _stats.write(localizer->getStats());
    }

    number_sonar_perceptions++;

    if(number_sonar_perceptions >= static_cast<size_t>(_minimum_perceptions.value()) 
            && Neff < _effective_sample_size_threshold.value()) {
      localizer->resample();
      validate_particles();
      number_sonar_perceptions = 0;
     
    } 
    
  }

}

void Task::obstacle_samplesCallback(const base::Time& ts, const sonar_detectors::ObstacleFeatures& sample)
{
  sonar_detectors::ObstacleFeatures features = sample;
  
  filter_sample(features);
  
  double scan_diff = std::fabs(last_scan_angle - features.angle);
  
  while(scan_diff > M_PI)
    scan_diff -= 2.0 * M_PI;
  
  last_scan_angle = features.angle;
  sum_scan += std::fabs(scan_diff);   
  
  if(perception_state_machine(ts)){

      double Neff = localizer->observeAndDebug(features, *map, _sonar_importance.value());

      if(localizer->hasStats()) {
      _stats.write(localizer->getStats());
      }

      number_sonar_perceptions++;

      if(number_sonar_perceptions >= static_cast<size_t>(_minimum_perceptions.value()) 
            && Neff < _effective_sample_size_threshold.value()) {
        localizer->resample();
        validate_particles();
        number_sonar_perceptions = 0;

      }
      
      if(!base::isNaN( lastRBS.cov_position(0,0)) && !base::isInfinity( lastRBS.cov_position(0,0)) )  
        localizer->setObstacles(features, *grid_map, lastRBS);
        
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
    localizer->setCurrentAngularVelocity(rbs);
    localizer->setCurrentZVelocity(rbs);
    localizer->setCurrentDepth(rbs);
    
    current_depth = rbs.position.z();

    if(start_time.isNull()) {
        start_time = ts;
    }

    if(!last_perception.isNull() && (ts - last_perception).toSeconds() > _reset_timeout.value()) {
        localizer->initialize(_particle_number.value(), base::Vector3d(0.0, 0.0, 0.0), map->getLimitations(), 
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
  
    if(base::samples::RigidBodyState::isValidValue(rbs.velocity)){
  
      localizer->setCurrentVelocity(rbs);
    
      if(orientation_sample_recieved){
        localizer->update(rbs, *map);
              
      }else{
        changeState(NO_ORIENTATION);
      }
        
      last_speed_time = ts;
      last_motion = ts;
    }
}


void Task::thruster_samplesCallback(const base::Time& ts, const base::samples::Joints& status)
{  
  
  if(last_speed_time.isNull() || ts.toSeconds() - last_speed_time.toSeconds() > _speed_samples_timeout.get() ){
  
    
    base::samples::Joints j = status;
    last_motion = ts; 

    if(status.hasNames()){
    
      for(int i = 0; i < status.size() && i < config.joint_names.size(); i++){
        
          try{
            j.elements[i] = status[config.joint_names[i]]; 
          }
          catch(...){
          }	
      }  
    }

    if(orientation_sample_recieved){
      localizer->update_dead_reckoning(j);
      localizer->update(j, *map);
      
    }else{
      changeState(NO_ORIENTATION);
    }
  }
    
}

void Task::gps_pose_samplesCallback(const base::Time& ts, const base::samples::RigidBodyState& rbs){
    
  double Neff = localizer->observeAndDebug(rbs,*map,_gps_importance.value());
  
  number_gps_perceptions++;
  
  if(number_gps_perceptions >= _minimum_perceptions.value()) {
        localizer->resample();
        localizer->setParticlesValid();
        number_gps_perceptions = 0;
    }  
}


void Task::buoy_samplesCallback(const base::Time& ts, const avalon::feature::Buoy& buoy){
  last_perception = ts;
  
  //double effective_sample_size = localizer->observe(buoy, *map, _buoy_importance.value());
  
}

void Task::echosounder_samplesCallback(const base::Time& ts, const base::samples::RigidBodyState& rbs){

  
  if(rbs.position[2] > 0.0){
    
    if(orientation_sample_recieved){
      
      //std::cout << "Observe ground: " << rbs.position[2] << " depth: " << current_depth << " sum: " << current_depth - rbs.position[2] << std::endl;
      
      if(_use_markov.get())
        localizer->observe_markov(current_depth - rbs.position[2], *grid_map, 1.0);
      else
        localizer->observe(current_depth - rbs.position[2], *grid_map, 1.0);
      
      if(!_use_slam.get()){
        grid_map->setDepth(lastRBS.position.x(), lastRBS.position.y(), current_depth - rbs.position[2], lastRBS.cov_position(0,0) );  
      }
        
      current_ground = current_depth - rbs.position[2];
    }
    
  }
    
}

void Task::stopHook()
{
     TaskBase::stopHook();

     //delete aggr;
     delete localizer;
     delete map;
     delete grid_map;
     
     localizer = 0;
     map = 0;
     grid_map = 0;
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

bool Task::initMotionConfig()
{
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
      double values[] = { 0.0, 0.0, -1.0, -1.0, 0.0, 0.0,
			    0.0, 0.0, 0.0, 0.0, 1.0, -1.0,
			    1.0, -1.0, 0.0, 0.0, 0.0, 0.0,
			    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
			    -0.92, 0.205, 0.0, 0.0, 0.0, 0.0,
			    0.0, 0.0, -0.17, 0.17, -0.81, 0.04};
	config.param_TCM = std::vector<double>(values, values + sizeof(values)/sizeof(double));		    
    }else{
      config.param_TCM = _param_TCM.value();
    }

    if(_param_linDamp.get().rows() != 6 && _param_linDamp.get().cols() != 6){
      
	std::cout << "Error: linDamp-Matrix should be 6D. Real dimensions: " << _param_linDamp.get().rows() << " , " << _param_linDamp.get().cols() << std::endl;
	return false;
    }
    if(_param_linDampNeg.get().rows() != 6 && _param_linDampNeg.get().cols() != 6){
      
	std::cout << "Error: linDampNeg-Matrix should be 6D. Real dimensions: " << _param_linDampNeg.get().rows() << " , " << _param_linDampNeg.get().cols() << std::endl;
	return false;
    }
    if(_param_sqDamp.get().rows() != 6 && _param_sqDamp.get().cols() != 6){
      
	std::cout << "Error: sqDamp-Matrix should be 6D. Real dimensions: " << _param_sqDamp.get().rows() << " , " << _param_sqDamp.get().cols() << std::endl;
	return false;
    }
    if(_param_sqDampNeg.get().rows() != 6 && _param_sqDampNeg.get().cols() != 6){
      
	std::cout << "Error: sqDampNeg-Matrix should be 6D. Real dimensions: " << _param_sqDampNeg.get().rows() << " , " << _param_sqDampNeg.get().cols() << std::endl;
	return false;
    }    
    //Dirty, converting dynamic matrix to static matrix
    config.param_linDamp = _param_linDamp.get().block<6,6>(0,0);
    config.param_sqDamp = _param_sqDamp.get().block<6,6>(0,0);
    config.param_linDampNeg = _param_linDampNeg.get().block<6,6>(0,0);
    config.param_sqDampNeg = _param_sqDampNeg.get().block<6,6>(0,0);

    config.param_floating = _param_floating.value();
        
    std::vector<std::string> names;
    if(_joint_names.get().size() == 6){
      names = _joint_names.get();
    }else{
      std::string name_values[] =  {std::string("right"), std::string("left"), std::string("dive"), std::string("pitch"), std::string("strave"), std::string("yaw")};
      std::cout << "No joint names set. Using default" << std::endl;
      names = std::vector<std::string>(name_values, name_values + sizeof(name_values) / sizeof(std::string));
    }
    
    config.joint_names = names; 
    
    return true;
}


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


void Task::filter_sample(sonar_detectors::ObstacleFeatures& sample){
  
  //std::cout << "Filter features - before: " << sample.features.size();
  
  //calculate reflection of the ground
  double diff_ground = std::fabs( current_ground - current_depth);
  double dist_groundreflection = diff_ground/sin(_sonar_vertical_angle.get()/2.0);
  double dist_surfacereflection = std::fabs(current_depth)/sin(_sonar_vertical_angle.get()/2.0);
  //std::cout << "--------------" << std::endl;
  //std::cout << "Filter surface at " << dist_surfacereflection << std::endl;
  //std::cout << "Filter ground at " << dist_groundreflection << std::endl;
  
  uint32_t last_range = -1;
  double  last_confidence = NAN;
  
  //Search for duplicate features
  //we asume, that duplicate features succed to each other
  
  //std::cout << "Size before: " << sample.features.size() << std::endl;
  for(std::vector<sonar_detectors::ObstacleFeature>::iterator it = sample.features.begin(); it != sample.features.end(); ){
   
    double dist = it->range / 1000.0;
    
    
    //std::cout << "confidence: " << it->confidence << ", range: " << it->range <<  ", threshold: " << _feature_filter_threshold.get() << std::endl;
    
    //We have a duplicate -> remove feature with lower confidence
    if(it->range == last_range){
      //std::cout << "Duplicate" << std::endl;
      if(it->confidence <= last_confidence){        
        it = sample.features.erase(it);

      }else{

        if(it == sample.features.begin()){
          it = sample.features.erase(it - 1);
          last_confidence = it->confidence;
          ++it;
        }
      }
     
     //Feature is out of range -> remove it!
    }else if(dist <= 0 || dist < _sonar_minimum_distance.get() || dist > _sonar_maximum_distance.get()){
      //std::cout << "Out of range" << std::endl;
      it = sample.features.erase(it);
      
      //Feature could be a false reflection from the ground or surface
    }else if( std::fabs( dist - dist_groundreflection) < 0.5 || std::fabs( dist - dist_surfacereflection) < 0.5 ){ 
      //std::cout << "Reflection" << std::endl;
      it = sample.features.erase(it); //TODO do we need this?
      //++it;
      
      //Feature confidence is to low -> do not use it!
    }else if( _feature_filter_threshold.get() > 0.0 && it->confidence <= _feature_filter_threshold.get()){
      //std::cout << "Filter because low confidence" << std::endl;
      it = sample.features.erase(it);      
    
    }else{
      last_range = it->range;
      last_confidence = it->confidence;
      ++it;
      //std::cout << "No filtering" << std::endl;
    }
    
    if(it == sample.features.end()){
      //std::cout << "Break loop" << std::endl;
      break;
    }
    
  }
  //std::cout << "Size after: " << sample.features.size() << std::endl;
  _debug_filtered_obstacles.write(sample);
  //std::cout << " after: " << sample.features.size();
  
}

