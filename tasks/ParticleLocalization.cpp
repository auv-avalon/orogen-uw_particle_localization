#include "ParticleLocalization.hpp"
#include <base/pose.h>
#include <list>
//#include <stdexcept>

using namespace machine_learning;

namespace uw_localization {

base::samples::RigidBodyState* PoseSlamParticle::pose = 0;

ParticleLocalization::ParticleLocalization(const FilterConfig& config) 
    : ParticleFilter<PoseSlamParticle>(), filter_config(config),
    StaticSpeedNoise(Random::multi_gaussian(Eigen::Vector3d(0.0, 0.0, 0.0), config.static_speed_covariance)),
    StaticMotionNoise(Random::multi_gaussian(Eigen::Vector3d(0.0, 0.0, 0.0), config.static_motion_covariance)),
    perception_history_sum(0.0),
    sonar_debug(0)
{
    first_perception_received = false;
    PoseSlamParticle::pose = &vehicle_pose;
    utm_origin = base::Vector3d::Zero();
    utm_origin[0] = -1;
    dynamic_model = 0;    
}

ParticleLocalization::~ParticleLocalization()
{ 
  if(filter_config.advanced_motion_model){
    delete dynamic_model;
    dynamic_model = 0;
  }
}


UwVehicleParameter ParticleLocalization::VehicleParameter(FilterConfig filter_config)
{
    UwVehicleParameter p;

    p.Length = filter_config.param_length;
    p.Radius = filter_config.param_radius;
    p.Mass = filter_config.param_mass;

    p.ThrusterCoefficient << filter_config.param_thrusterCoefficient[0], filter_config.param_thrusterCoefficient[1], filter_config.param_thrusterCoefficient[2],
		  filter_config.param_thrusterCoefficient[3], filter_config.param_thrusterCoefficient[4], filter_config.param_thrusterCoefficient[5];
		  
    p.LinearThrusterCoefficient << filter_config.param_linearThrusterCoefficient[0], filter_config.param_linearThrusterCoefficient[1],
				    filter_config.param_linearThrusterCoefficient[2],  filter_config.param_linearThrusterCoefficient[3],
				    filter_config.param_linearThrusterCoefficient[4], filter_config.param_linearThrusterCoefficient[5];
		  
    p.SquareThrusterCoefficient << filter_config.param_squareThrusterCoefficient[0], filter_config.param_squareThrusterCoefficient[1],
				    filter_config.param_squareThrusterCoefficient[2],  filter_config.param_squareThrusterCoefficient[3],
				    filter_config.param_squareThrusterCoefficient[4],  filter_config.param_squareThrusterCoefficient[5];
    p.ThrusterVoltage = filter_config.param_thrusterVoltage;
    
    p.TCM = MatrixTCM::Zero();    
    for(int i=0; i<3;i++){
      for(int j=0; j<6;j++){
	p.TCM(j,i) = filter_config.param_TCM[i*6+j];
      }
    }     

    /*
    p.TCM << 0.0, 0.0, 1.0, // 0.0, -0.92, 0.0, // HEAVE
             0.0, 0.0, 1.0, //0.0, 0.205, 0.0, // HEAVE
             1.0, 0.0, 0.0, //0.0, 0.0, -0.17, // SURGE
             1.0, 0.0, 0.0, //0.0, 0.0, 0.17, // SURGE
             0.0, 1.0, 0.0, //0.0, 0.0, -0.81, // SWAY
             0.0, 1.0, 0.0; //0.0, 0.0, 0.04;  // SWAY
    */
             
    p.DampingX << filter_config.param_sqDamp(0,0) , filter_config.param_linDamp(0,0);
    p.DampingY << filter_config.param_sqDamp(1,1) , filter_config.param_linDamp(1,1);
    p.DampingZ << filter_config.param_sqDamp(2,2) , filter_config.param_linDamp(2,2);
    p.floating = filter_config.param_floating;
        
    return p;
}

void ParticleLocalization::init_slam(NodeMap *map){
  
  dp_slam.init( base::Vector2d(-map->getTranslation().x(), -map->getTranslation().y() ),
                              1.2 * base::Vector2d(map->getLimitations().x(), map->getLimitations().y() ),
                filter_config.feature_grid_resolution , filter_config);
  
  dp_slam.initalize_statics(map);
  
  
}


void ParticleLocalization::initialize(int numbers, const Eigen::Vector3d& pos, const Eigen::Vector3d& var, double yaw, double yaw_cov)
{  
    
    Eigen::Vector3d var1 = var;
    
    //This fixes a boost-bug in uniform_real, which causes a loop, when min=max
    for(int i=0; i<3; i++){
      if(var1[i] <= 0.0)
	var1[i] = 0.0001;
    }  
  
    UniformRealRandom pos_x = Random::uniform_real(pos.x() - var1.x() * 0.5, pos.x() + var1.x() * 0.5 );
    UniformRealRandom pos_y = Random::uniform_real(pos.y() - var1.y() * 0.5, pos.y() + var1.y() * 0.5 );
    UniformRealRandom pos_z = Random::uniform_real(pos.z() - var1.z() * 0.5, pos.z() + var1.z() * 0.5 );

    particles.clear();
    perception_history.clear();
    perception_history_sum = 0.0;

    for(int i = 0; i < numbers; i++) {
        PoseSlamParticle pp;
        pp.p_position = base::Vector3d(pos_x(), pos_y(), pos_z());
        pp.p_velocity = base::Vector3d(0.0, 0.0, 0.0);
        pp.main_confidence = 1.0 / numbers;
        pp.valid = true;
        particles.push_back(pp);
    }

    generation++;

    motion_pose.position = pos;
    motion_pose.velocity << 0.0,0.0,0.0;
    motion_pose.angular_velocity << 0.0,0.0,0.0;
    motion_pose.time = base::Time::now();
    
    full_motion_pose.position = pos;
    full_motion_pose.velocity << 0.0, 0.0, 0.0;
    full_motion_pose.angular_velocity << 0.0, 0.0, 0.0;
    full_motion_pose.time = base::Time::now();
    
    vehicle_pose.position = pos;
    vehicle_pose.velocity << 0.0, 0.0, 0.0;
    vehicle_pose.angular_velocity << 0.0, 0.0 , 0.0;
    vehicle_pose.time = base::Time::now();
    
    best_sonar_measurement.confidence = 0.0;
    lastActuatorTime = base::Time();
    
    if(filter_config.advanced_motion_model){
      
      underwaterVehicle::Parameters params = initializeDynamicModel(VehicleParameter(filter_config), filter_config);
      dynamic_model = new underwaterVehicle::DynamicModel(0.1, 5, 0.0);
      dynamic_model->init_param(params);
      dynamic_model_params = params;
    }else{      
      motion_model.init(VehicleParameter(filter_config));
    }
}

underwaterVehicle::Parameters ParticleLocalization::initializeDynamicModel(UwVehicleParameter p, FilterConfig filter_config){
  
  underwaterVehicle::Parameters params;
  
  params.uwv_mass = p.Mass;
  params.uwv_volume = M_PI*std::pow(p.Radius,2.0)*p.Length;
  params.uwv_float =filter_config.param_floating;
  
  params.distance_body2centerofbuoyancy = filter_config.param_centerOfBuoyancy;
  params.distance_body2centerofgravity = filter_config.param_centerOfGravity;
  
  params.waterDensity = kWaterDensity;
  params.gravity = kGravity;
  
  params.thrusterVoltage = p.ThrusterVoltage;
  
  params.thruster_coefficients_pwm = filter_config.param_thrusterCoefficient;
  params.linear_thruster_coefficients_pwm = filter_config.param_linearThrusterCoefficient;
  params.square_thruster_coefficients_pwm = filter_config.param_squareThrusterCoefficient;
  
  //double values[] = {7.48,0,0,0,0,0,  0,16.29,0,0,0,0,  0,0,16.29,0,0,0,  0,0,0,0.61,0,0,  0,0,0,0,1.67,0,  0,0,0,0,0,1.67};
  //params.mass_matrix = std::vector<double>(values, values + sizeof(values)/sizeof(double));
  
  base::Matrix6d mass_matrix = base::Matrix6d::Zero(); 
    
  //Setting index (1,1) = inertia mass + added mass
  //inertia mass = mass; added mass = 0.1 * mass
  mass_matrix(0,0) = filter_config.param_mass + 0.1 * filter_config.param_mass; 
  
  //Setting index (2,2)
  //inertia mass = mass; added mass = waterdensity * PI * radius^2 * length 
  mass_matrix(1,1) = filter_config.param_mass + M_PI * std::pow(filter_config.param_radius,2.0) * kWaterDensity * filter_config.param_length;
  
  //Setting index (3,3)
  //inertia mass = mass; added mass = waterdensity * PI * radius^2 * length 
  mass_matrix(2,2) = filter_config.param_mass + M_PI * std::pow(filter_config.param_radius,2.0) * kWaterDensity * filter_config.param_length;
  
  //Setting index (4,4) -> Roll
  //inertia mass = mass; added mass = 0 
  mass_matrix(3,3) = filter_config.param_mass;
  
  //Setting index (5,5) -> Pitch
  //inertia mass = mass; added mass = 1/12 * pi * waterdensity * radius^2 * length^3
  mass_matrix(4,4) =  filter_config.param_mass + (1.0/12.0) * M_PI * kWaterDensity * std::pow(filter_config.param_radius,2.0) * std::pow(filter_config.param_length,3.0); 

    //Setting index (6,6) -> Yaw
  //inertia mass = mass; added mass = 1/12 * pi * waterdensity * radius^2 * length^3
  mass_matrix(5,5) =  filter_config.param_mass + (1.0/12.0) * M_PI * kWaterDensity * std::pow(filter_config.param_radius,2.0) * std::pow(filter_config.param_length,3.0);
  
  filter_config.param_mass = p.Mass; 
  params.massMatrix = mass_matrix;
  params.massMatrixNeg = mass_matrix;

  //Damping Coefficients
  params.linDampMatrix = base::Matrix6d::Zero();
  params.linDampMatrixNeg = base::Matrix6d::Zero();
  params.quadDampMatrix = base::Matrix6d::Zero();
  params.quadDampMatrixNeg = base::Matrix6d::Zero();
  
  params.linDampMatrix = filter_config.param_linDamp;
  params.linDampMatrixNeg = filter_config.param_linDampNeg;
  params.quadDampMatrix = filter_config.param_sqDamp;
  params.quadDampMatrixNeg = filter_config.param_sqDampNeg;  
    
  //Initialize Thruster mapping
  params.number_of_thrusters = 6;
  
  params.thruster_control_matrix.clear();
  params.thruster_control_matrix.resize(36,0.0);
  //Filling thruster controll matrix
  //Change coefficients from line-wise to col-wise
  for(int i=0; i<6; i++){
    for(int j=0; j<6; j++){
      params.thruster_control_matrix[i*6+j] = filter_config.param_TCM[j*6+i];
    }
  }  
  
  params.sim_per_cycle = 5;
  params.plant_order = 12; //3 directions, 3 orientation angles, 3 linear velocities, 3 angular velocities
  params.ctrl_order = 6; //6 thruster inputs
  params.samplingtime = 0.1;  
  
  //Setting inital position
  params.initial_condition[0] = filter_config.init_position[0];
  params.initial_condition[1] = filter_config.init_position[1];
  params.initial_condition[2] = filter_config.init_position[2];
  
  //Initial orientation and speed is 0
  for(int i=3;i<12;i++)
    params.initial_condition[i] = 0.0;
  
  params.initial_time = 0.0;
  
  return params;  
}

void ParticleLocalization::dynamic(PoseSlamParticle& X, const base::samples::RigidBodyState& U, const NodeMap& map)
{
    base::Vector3d v_noisy;
    base::Vector3d u_velocity;

    if(filter_config.pure_random_motion)
        u_velocity = base::Vector3d(0.0, 0.0, 0.0);
    else
        u_velocity = U.velocity;
    
    if( !X.timestamp.isNull() ) {
      double dt = (U.time - X.timestamp).toSeconds();
      v_noisy = u_velocity + (StaticSpeedNoise() * dt);

      base::Vector3d v_avg = (X.p_velocity + v_noisy) / 2.0;
      
      base::Vector3d pos = X.p_position + vehicle_pose.orientation * (v_avg * dt);
      
      if(map.belongsToWorld(pos)){      
        X.p_position = pos;
      }
      else{
        X.valid = false; //Particle left world, something went wrong
      }
      X.p_velocity = v_noisy;
    }

    X.timestamp = U.time;
    X.p_velocity[2] = vehicle_pose.velocity[2];
    X.p_position.z() = vehicle_pose.position.z();
}

void ParticleLocalization::dynamic(PoseSlamParticle& X, const base::samples::Joints& Ut, const NodeMap& map)
{
    Vector6d Xt;
    base::Time sample_time = Ut.time;
    
    if(sample_time.isNull())
      sample_time=base::Time::now();
    
    if( !X.timestamp.isNull() ) {
        double dt = (sample_time - X.timestamp).toSeconds();
	
	if(dt < 5.0){ //Only use dynamic_model for small timesteps, to prevent an overflow
	
	  base::Vector3d v_noisy;
	  base::Vector3d u_velocity;

	    if(filter_config.pure_random_motion) {
		u_velocity = base::Vector3d(0.0, 0.0, 0.0);
	    }else if(filter_config.advanced_motion_model){	  	  

              //UPdate motion model
	      dynamic_model->setPosition(X.p_position);
	      dynamic_model->setLinearVelocity(X.p_velocity);
	      dynamic_model->setSamplingtime(dt);
	      dynamic_model->setPWMLevels(Ut);	
	      
	      u_velocity = dynamic_model->getLinearVelocity();
	      
	    }else{  
	      Xt << X.p_velocity.x(), X.p_velocity.y(), X.p_velocity.z(), 
		  X.p_position.x(), X.p_position.y(), X.p_position.z();

		Vector6d V = motion_model.transition(Xt, dt, Ut);

		u_velocity = V.block<3, 1>(0, 0);	   
	    }   
	  
	  //Motion noise. Noise depends on the delta-time. For a long time intervall, there is more noise
	  v_noisy = u_velocity + (StaticMotionNoise() * dt); 

	  base::Vector3d v_avg = (X.p_velocity + v_noisy) / 2.0;
	  
	  if(vehicle_pose.hasValidOrientation() && !std::isnan(v_avg[0]) && !std::isnan(v_avg[1]) 
		  && base::samples::RigidBodyState::isValidValue(v_noisy) ){
            
            base::Vector3d pos = X.p_position + vehicle_pose.orientation * (v_avg * dt);
          
            //Only exept new position, if new position is part of world
            if(map.belongsToWorld(pos)){
              X.p_position = pos;
            }
            else{
              X.valid = false; //Particle left world, something went wrong
            }
            
	    X.p_velocity = v_noisy;
          
          //Cut off velocity ddift
          for(int i = 0; i < 3; i++){
            
            if(X.p_velocity[i] < motion_pose.velocity[i] - filter_config.max_velocity_drift)
              X.p_velocity[i] = motion_pose.velocity[i] - filter_config.max_velocity_drift;
            
            if(X.p_velocity[i] > motion_pose.velocity[i] + filter_config.max_velocity_drift)
              X.p_velocity[i] = motion_pose.velocity[i] + filter_config.max_velocity_drift;
            
          }        
          
          
	  }
      }
    }
    
    X.p_position.z() = vehicle_pose.position.z();
    X.timestamp = sample_time;
}
 

void ParticleLocalization::update_dead_reckoning(const base::samples::Joints& Ut)
{   
    base::Time sample_time = Ut.time;
    if(sample_time.isNull())
      sample_time = base::Time::now(); 
    
    if( !lastActuatorTime.isNull() ) {
        Vector6d Xt;
	base::Vector3d u_t1;
	double dt = (Ut.time - lastActuatorTime).toSeconds();
	
	if(dt < 5.0 && dt >= 0.0){
	  
	  if(filter_config.advanced_motion_model){
		      
	    //std::cout << "Velocity: " << motion_pose.velocity[0] << " " << motion_pose.velocity[1] << std::endl;
	    dynamic_model->setPosition(motion_pose.position);
	    dynamic_model->setLinearVelocity(motion_pose.velocity);
	    dynamic_model->setAngularVelocity(base::Vector3d::Zero());
	    dynamic_model->setOrientation(vehicle_pose.orientation);
	    dynamic_model->setSamplingtime(dt);
	    
	    dynamic_model->setPWMLevels(Ut);	   
	    
	    u_t1 = dynamic_model->getLinearVelocity();	
	    
	    //Full dead reckoning
	    dynamic_model->setPosition(full_motion_pose.position);
	    dynamic_model->setLinearVelocity(full_motion_pose.velocity);
	    dynamic_model->setAngularVelocity(full_motion_pose.angular_velocity);
	    dynamic_model->setOrientation(full_motion_pose.orientation);
	    
	    dynamic_model->setPWMLevels(Ut);
	    
	    full_motion_pose.position = dynamic_model->getPosition();
	    full_motion_pose.velocity = dynamic_model->getLinearVelocity();
	    full_motion_pose.angular_velocity = dynamic_model->getAngularVelocity();
	    full_motion_pose.orientation = dynamic_model->getOrientation_in_Quat();
	    full_motion_pose.time = Ut.time;
	    
	    
	    //std::cout << "new Velocity: " << u_t1[0] << " " << u_t1[1] << std::endl ;
	  }else{
	    
	    Xt.block<3,1>(0,0) = motion_pose.velocity;
	    Xt.block<3,1>(3,0) = base::Vector3d(0.0, 0.0, 0.0);
	    
	    Vector6d V = motion_model.transition(Xt, dt, Ut);
	    u_t1 = V.block<3,1>(0,0);	   
	  }

	  base::Vector3d v_avg = (motion_pose.velocity + u_t1) / 2.0;
	  
	  if(base::samples::RigidBodyState::isValidValue(u_t1) && vehicle_pose.hasValidOrientation() && vehicle_pose.hasValidVelocity()){
	    
	    motion_pose.position = motion_pose.position + vehicle_pose.orientation * (v_avg * dt);
	    motion_pose.velocity = u_t1;	 
	  
	  }else{
	    std::cout << "Error in motion_model. Velocity or orientation is unset." << std::endl;
	    if(!vehicle_pose.hasValidOrientation())
	      std::cout << "Invalid orientation" << std::endl;
	  }
	}else{
          std::cout << "Timestampdifference between thruster-samples to big or negative" << std::endl;
          std::cout << "Last: " << lastActuatorTime << " actual: " << Ut.time << std::endl; 
	}
	
    } 
    
    lastActuatorTime = sample_time;
    motion_pose.time = base::Time::now();
    
    motion_pose.velocity.z() = vehicle_pose.velocity.z();
    vehicle_pose.velocity = motion_pose.velocity;
    
}




const base::Time& ParticleLocalization::getTimestamp(const base::samples::RigidBodyState& U)
{
    return U.time;
}

const base::Time& ParticleLocalization::getTimestamp(const base::samples::Joints& U)
{
    return U.time;
}

double ParticleLocalization::observeAndDebug(const base::samples::LaserScan& z, NodeMap& m, double importance)
{
    zeroConfidenceCount = 0;
    measurement_incomplete = false;
    
    double effective_sample_size;
    
    if(filter_config.use_markov)
      effective_sample_size = observe_markov(z, m, importance);
    else      
      effective_sample_size = observe(z, m, importance);
    
    best_sonar_measurement.time = z.time;

    sonar_debug->write(best_sonar_measurement);

    if(best_sonar_measurement.status == OKAY)
        addHistory(best_sonar_measurement);

    best_sonar_measurement.confidence = -1.0;
    
    if(zeroConfidenceCount > 0 && filter_config.filterZeros)
      filterZeros();

    if(measurement_incomplete)
      return INFINITY; //If we have no complete meassurement, we do not want to resample!!!
    
    return effective_sample_size;
}


double ParticleLocalization::observeAndDebug(const sonar_detectors::ObstacleFeatures& z, NodeMap& m, double importance)
{
    zeroConfidenceCount = 0;
    measurement_incomplete = false;
    
    double effective_sample_size;
    
    if(filter_config.use_markov)
      effective_sample_size = observe_markov(z, m, importance);
    else      
      effective_sample_size = observe(z, m, importance);
    
    best_sonar_measurement.time = z.time;

    sonar_debug->write(best_sonar_measurement);

    if(best_sonar_measurement.status == OKAY)
        addHistory(best_sonar_measurement);

    best_sonar_measurement.confidence = -1.0;
    
    if(zeroConfidenceCount > 0 && filter_config.filterZeros)
      filterZeros();

    if(measurement_incomplete)
      return INFINITY; //If we have no complete meassurement, we do not want to resample!!!
    
    return effective_sample_size;
}



double ParticleLocalization::observeAndDebug(const base::samples::RigidBodyState& z, NodeMap& m, double importance)
{	
    if(utm_origin[0]==-1){
	utm_origin << z.position[0], z.position[1], 0.0;
    }
    
    //Converts the utm-coordinate to our world coordinate system
    base::Vector3d pose;
    pose[0] = cos(filter_config.utm_relative_angle)*(z.position[0]-utm_origin[0])
	  - sin(filter_config.utm_relative_angle)*(z.position[1]-utm_origin[1])
	  + filter_config.init_position[0];
    pose[1] = cos(filter_config.utm_relative_angle)*(z.position[1]-utm_origin[1])
	  + sin(filter_config.utm_relative_angle)*(z.position[0]-utm_origin[0])
	  + filter_config.init_position[1];    
    
    //Sets the covarianz matrix of the rbs
    base::samples::RigidBodyState newRBS = z;
    base::Matrix3d cov = base::Matrix3d::Zero();
    cov << filter_config.gps_covarianz, 0, 0,
	0, filter_config.gps_covarianz, 0,
	0, 0, filter_config.gps_covarianz;
    newRBS.cov_position = cov;
    
    newRBS.position[0]=pose[0];
    newRBS.position[1]=pose[1];
    
    //Creates new particle at the gps-position
    interspersal(newRBS, m, filter_config.gps_interspersal_ratio, false);	  
    
    double effective_sample_size = observe(pose, m, importance);   
    
    best_sonar_measurement.time = z.time;
    
    if(best_sonar_measurement.status == OKAY)
      addHistory(best_sonar_measurement);
    
    best_sonar_measurement.confidence = -1.0;
    timestamp = base::Time::now();
    
    return effective_sample_size;
}  



double ParticleLocalization::perception(PoseSlamParticle& X, const base::samples::LaserScan& Z, NodeMap& M)
{
    uw_localization::PointInfo info;
    info.time = X.timestamp;

    double angle = Z.start_angle;
    double yaw = base::getYaw(vehicle_pose.orientation);
    double z_distance = Z.ranges[0] / 1000.0;

    // check if this particle is still part of the world
    if(!M.belongsToWorld(X.p_position)) {
        debug(z_distance, X.p_position, 0.0, NOT_IN_WORLD);
        zeroConfidenceCount++;
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
    Eigen::Affine3d SonarToAvalon(filter_config.sonarToAvalon);

    Eigen::Vector3d RelativeZ = sonar_yaw * SonarToAvalon * base::Vector3d(z_distance, 0.0, 0.0);
    Eigen::Vector3d AbsZ = (abs_yaw * RelativeZ) + X.p_position;

    boost::tuple<Node*, double, Eigen::Vector3d> distance = M.getNearestDistance("root.wall", AbsZ, X.p_position);
    boost::tuple<Node*, double, Eigen::Vector3d> distance_box = M.getNearestDistance("root.box",
				  Eigen::Vector3d(0.0, filter_config.sonar_vertical_angle/2.0, yaw + angle), X.p_position);


    double dst = distance.get<1>();
    double dst_box = distance_box.get<1>();
          
    double diff_dst = std::fabs( dst - z_distance);
    double diff_dst_box = std::fabs( dst_box - z_distance);
    bool box = false;
    
    if(dst_box != INFINITY){
      //std::cout << "Dst: " << dst << " - dst box: " << dst_box << std::endl;
    }
    
    
    if(diff_dst > diff_dst_box){
      dst = dst_box;
      distance = distance_box;
      box = true;
      
    }

    if(dst == INFINITY){
      measurement_incomplete = true;
      debug(z_distance, X.p_position, X.main_confidence, MAP_INVALID);
      return X.main_confidence;
    }    
    
    double covar = filter_config.sonar_covariance;
    
    if(dst > vehicle_pose.position[2]/sin(filter_config.sonar_vertical_angle/2.0))
      covar = covar * filter_config.sonar_covariance_reflection_factor;    

    if(angleDiffToCorner(angle+yaw, X.p_position, filter_config.env) < 0.1)
      covar = covar * filter_config.sonar_covariance_corner_factor;    

    double probability = gaussian1d(0.0, covar, dst - z_distance);
    
    if(box){
      debug(z_distance, dst ,angle + yaw  ,distance.get<2>(), AbsZ, X.p_position, probability, OBSTACLE);
    }else{    
      debug(z_distance, dst ,angle + yaw  ,distance.get<2>(), AbsZ, X.p_position, probability);
    }
    //std::cout << distance.get<1>() << std::endl;    
    
    first_perception_received = true;

    return probability;
}

double ParticleLocalization::perception(PoseSlamParticle& X, const sonar_detectors::ObstacleFeatures& Z, NodeMap& M){
 
    //Check if particle is part of the map
    if(!M.belongsToWorld(X.p_position)) {
        debug(0.0, X.p_position, 0.0, NOT_IN_WORLD);
        zeroConfidenceCount++;
        return 0.0;
    }  
  
  //Check if there are valid features
  if(Z.features.empty()){
   
     double p = X.main_confidence;
     debug(0.0, X.p_position, p, OUT_OF_RANGE);
     return p;
  }
  
  if(filter_config.use_slam){
    double val = dp_slam.observe(X, Z, base::getYaw(vehicle_pose.orientation));
    std::cout << "Pos: " << X.p_position.transpose() << " - confidence: " << val << std::endl;
    
    if(!filter_config.use_mapping_only){
    
      if(val == 0.0)
        return X.main_confidence;
      
      return val;
    }
  }
  
  
    double angle = Z.angle;
    double yaw = base::getYaw(vehicle_pose.orientation);
    // Sonar transformations
    Eigen::AngleAxis<double> sonar_yaw(angle, Eigen::Vector3d::UnitZ()); 
    Eigen::AngleAxis<double> abs_yaw(yaw, Eigen::Vector3d::UnitZ());    
    Eigen::Affine3d SonarToAvalon(filter_config.sonarToAvalon);  
  
  
  bool valid_range = false;
  bool valid_map = false;
  std::list<double> distance_diffs;
  std::list<PointStatus> states;
  std::list< boost::tuple<Node*, double, Eigen::Vector3d> > distances;
  std::list<double> z_distances;
  std::list<base::Vector3d> z_points;
  
  //Calculate perception for every feature
  for(std::vector<sonar_detectors::ObstacleFeature>::const_iterator it = Z.features.begin(); it != Z.features.end(); it++){
    
    //If the confidence is zero, ignore feature
    if(it->confidence <= 0.0)
      continue;
    
    double z_distance = it->range / 1000.0;
    
    // check if current laser scan is in a valid range
    if(z_distance == base::samples::TOO_FAR 
            || z_distance > filter_config.sonar_maximum_distance 
            || z_distance < filter_config.sonar_minimum_distance)
    {
        continue;
    }    
    
    //At least, on feature is in valid range
    valid_range = true;
    
    Eigen::Vector3d RelativeZ = sonar_yaw * SonarToAvalon * base::Vector3d(z_distance, 0.0, 0.0);
    Eigen::Vector3d AbsZ = (abs_yaw * RelativeZ) + X.p_position;
    
    //Calculate perception model
    boost::tuple<Node*, double, Eigen::Vector3d> distance = M.getNearestDistance("root.wall", AbsZ, X.p_position);
    boost::tuple<Node*, double, Eigen::Vector3d> distance_box = M.getNearestDistance("root.box",
                                  Eigen::Vector3d(0.0, filter_config.sonar_vertical_angle/2.0, yaw + angle), X.p_position);
    
    double dist_diff = std::fabs(z_distance - distance.get<1>());
    double dist_diff_box = std::fabs(z_distance - distance_box.get<1>());
    
    if(distance.get<1>() == INFINITY && distance_box.get<1>() == INFINITY)
      continue;
    
    //At least one feature can be modeled
    valid_map = true;
    
    //Select best modeled perception
    if(dist_diff > dist_diff_box){
      distance_diffs.push_back(dist_diff_box);
      states.push_back(OBSTACLE);
      distances.push_back(distance_box);
    }
    else{
      distance_diffs.push_back(dist_diff);
      states.push_back(OKAY);
      distances.push_back(distance);
    }      

      z_distances.push_back(z_distance);
      z_points.push_back(AbsZ);

  }
  
  //There were no valid features
  if(!valid_range){
     double p = X.main_confidence;
     debug(0.0, X.p_position, p, OUT_OF_RANGE);
     return p;    
  }
  
  //No features could be modeled
  if(!valid_map){
      measurement_incomplete = true;
      debug(0.0, X.p_position, X.main_confidence, MAP_INVALID);
      return X.main_confidence;    
    
  }  
  
  double best_diff = INFINITY;
  PointStatus best_state;
  boost::tuple<Node*, double, Eigen::Vector3d> best_distance;
  double best_z;
  base::Vector3d best_zPoint;
  
  //Select feature with the lowest modeled difference
  std::list<PointStatus>::iterator state_it = states.begin();
  std::list< boost::tuple<Node*, double, Eigen::Vector3d> >::iterator dist_it= distances.begin();
  std::list< double>::iterator it_z = z_distances.begin();
  std::list<base::Vector3d>::iterator it_zpoint = z_points.begin();
  for(std::list<double>::iterator it = distance_diffs.begin(); it != distance_diffs.end(); it++, state_it++, dist_it++, it_z++, it_zpoint++){
    if(*it < best_diff){
      best_diff = *it;
      best_state = *state_it;
      best_distance = *dist_it;
      best_z = *it_z;
      best_zPoint = *it_zpoint;
    }
    
  }  
  
  //Rate the best feature
  double probability;
  
  if(filter_config.use_best_feature_only){ //Rate only the best feature
    probability = gaussian1d(0.0, filter_config.sonar_covariance, best_diff);
  }  
  else{ //Rate all features, multiply probabilities
    
    probability = 1.0;
    
    for(std::list<double>::iterator it = distance_diffs.begin(); it != distance_diffs.end(); it++){
      probability *= gaussian1d(0.0, filter_config.sonar_covariance, *it);      
    }    
  }
    
  debug(best_z, best_distance.get<1>() ,angle + yaw  ,best_distance.get<2>(), best_zPoint, X.p_position, probability, best_state);
  
  first_perception_received = true;
  
 return probability; 
}



double ParticleLocalization::perception(PoseSlamParticle& X, const controlData::Pipeline& Z, NodeMap& M) 
{
    double yaw = base::getYaw(vehicle_pose.orientation);
    Eigen::AngleAxis<double> abs_yaw(yaw, Eigen::Vector3d::UnitZ());

    Eigen::Vector3d AbsZ = (abs_yaw * filter_config.pipelineToAvalon) + X.p_position;

    boost::tuple<Node*, double, Eigen::Vector3d> distance;
    if(Z.inspection_state == controlData::END_OF_PIPE){
        distance = M.getNearestDistance("root.end_of_pipe", AbsZ, X.p_position);
	
    }else if(Z.inspection_state == controlData::FOUND_PIPE || Z.inspection_state == controlData::FOLLOW_PIPE || Z.inspection_state){
        distance = M.getNearestDistance("root.pipeline", AbsZ, X.p_position);
    }

    double probability = gaussian1d(0.0, filter_config.pipeline_covariance, distance.get<1>());
    
    first_perception_received = true;

    return probability;
}


double ParticleLocalization::perception(PoseSlamParticle& X, const base::Vector3d& Z, NodeMap& M)
{
    Eigen::Matrix<double,2,1> pos;
    pos << X.p_position[0] , X.p_position[1];
    Eigen::Matrix<double,2,2> covar;
    covar << filter_config.gps_covarianz, 0, 0, filter_config.gps_covarianz;
    Eigen::Matrix<double,2,1> gps; 
    gps << Z[0], Z[1];    
    
    //check if this particle is part of the world
    if(filter_config.useMap && !M.belongsToWorld(X.p_position)) {
        debug(Z, 0.0, NOT_IN_WORLD); 
        return 0.0;
    }
    
    //double propability = calc_gaussian(pos, covar, gps);
    
    double diff=std::sqrt(std::pow(X.p_position[0]-Z[0], 2.0) + std::pow(X.p_position[1]-Z[1], 2.0));
    double probability = gaussian1d(0, filter_config.gps_covarianz, diff);
    
    debug(Z,probability,OKAY);
    
    first_perception_received = true;
    
    return probability;
}

double ParticleLocalization::perception(PoseSlamParticle& X, const avalon::feature::Buoy& Z, NodeMap& M){
  
  Eigen::Vector3d cameraInWorld = X.p_position + (vehicle_pose.orientation * filter_config.buoyCamPosition);
  Eigen::Vector3d buoyToCam = vehicle_pose.orientation * (filter_config.buoyCamRotation * Z.world_coord);
  Eigen::Vector3d buoyInWorld = cameraInWorld + buoyToCam;
  
  double distance = M.getNearestDistance("root.buoy", buoyInWorld, X.p_position).get<1>();
  
  double probability = gaussian1d(0.0, filter_config.buoy_covariance, distance);
  
  first_perception_received = true;
  
  return probability;
}


double ParticleLocalization::perception(PoseSlamParticle& X, const double& Z, DepthObstacleGrid& M){

  if(first_perception_received)
    M.setDepth(X.p_position.x(), X.p_position.y(), Z, X.main_confidence);  
  
  return X.main_confidence;
  
}  
  
void ParticleLocalization::addHistory(const uw_localization::PointInfo& info)
{
    if(perception_history.size() >= filter_config.perception_history_number) {
        perception_history_sum -= perception_history.front();
        perception_history.pop_front();
    }

    perception_history.push_back(info.confidence);
    perception_history_sum += perception_history.back();
}

bool ParticleLocalization::hasStats() const
{
    return (perception_history.size() == filter_config.perception_history_number) && !timestamp.isNull();
}


uw_localization::Stats ParticleLocalization::getStats() const
{
    uw_localization::Stats stats;
    stats.timestamp = timestamp;
    stats.uncertainty_degree = perception_history_sum / filter_config.perception_history_number;
    stats.effective_sample_size = effective_sample_size;
    stats.particle_generation = generation;

    return stats;
}

void ParticleLocalization::interspersal(const base::samples::RigidBodyState& p, const NodeMap& m, double ratio, bool random_uniform)
{
    reduceParticles(1.0 - ratio);

    PoseSlamParticle best = particles.front();
    PoseSlamParticle worst = particles.back();
    
    base::Vector3d limit = m.getLimitations();
    MultiNormalRandom<3> Pose = Random::multi_gaussian(p.position, p.cov_position);
    UniformRealRandom pos_x = Random::uniform_real(-(limit.x() / 2.0) , (limit.x() / 2.0) );
    UniformRealRandom pos_y = Random::uniform_real(-(limit.y() / 2.0) , (limit.y() / 2.0) );    
    int count = 0;
    
    for(size_t i = particles.size(); i < filter_config.particle_number; i++) {
        PoseSlamParticle pp;
        
        if(random_uniform){
          pp.p_position[0] = pos_x();
          pp.p_position[1] = pos_y();
        }
        else{        
          pp.p_position = Pose();
        }
        
        pp.p_velocity = best.p_velocity;
        pp.p_position[2] = best.p_position[2];
        pp.main_confidence = worst.main_confidence / 10000.0; //Choose a realy small value
        pp.valid = false;
        
        count++;
        particles.push_back(pp);
    }
    std::cout << "Interspersal. Created " << count << " new particles." << std::endl;
    normalizeParticles();
}


void ParticleLocalization::debug(double distance, const base::Vector3d& location, double conf, PointStatus status)
{
    if(best_sonar_measurement.confidence < conf) {
        uw_localization::PointInfo info;
        info.distance = distance;
	info.desire_distance = 0.0;
        info.desire_point = base::Vector3d(0.0, 0.0, 0.0);
        info.real_point = base::Vector3d(0.0, 0.0, 0.0);
	info.angle = 0.0;
        info.location = location;
        info.confidence = conf;
        info.status = status;

        best_sonar_measurement = info;
    }
}

void ParticleLocalization::debug(double distance, double desire_distance, double angle, const base::Vector3d& desire, const base::Vector3d& real, const base::Vector3d& loc, double conf, PointStatus status)
{
    if(best_sonar_measurement.confidence < conf) {
        uw_localization::PointInfo info;
        info.distance = distance;
        info.desire_distance = desire_distance;
        info.desire_point = desire;
        info.real_point = real;
        info.angle = angle;
        info.location = loc;
        info.confidence = conf;
        info.status = status;

        best_sonar_measurement = info;
    }
}

void ParticleLocalization::debug(double distance, double desire_distance, double angle, const base::Vector3d& desire, const base::Vector3d& real, const base::Vector3d& loc, double conf)
{
    if(best_sonar_measurement.confidence < conf) {
        uw_localization::PointInfo info;
        info.distance = distance;
	info.desire_distance = desire_distance;
        info.desire_point = desire;
        info.real_point = real;
	info.angle = angle;
        info.location = loc;
        info.confidence = conf;
        info.status = OKAY;

        best_sonar_measurement = info;
    }
}

void ParticleLocalization::debug(const base::Vector3d& pos, double conf, PointStatus status)
{
    if(best_sonar_measurement.confidence < conf){
	uw_localization::PointInfo info;
	info.distance = 0.0;
	info.desire_distance = 0.0;
	info.angle = 0.0;
	info.desire_point = base::Vector3d(0.0,0.0,0.0);
	info.real_point = base::Vector3d(0.0,0.0,0.0);
	info.location = base::Vector3d(pos[0],pos[1],0.0);
	info.confidence = conf;
	info.status = status;
    }  
  
}  

void ParticleLocalization::teleportParticles(const base::samples::RigidBodyState& pose)
{
    std::list<PoseSlamParticle>::iterator it;
    for(it = particles.begin(); it != particles.end(); ++it) {
        it->p_position = pose.position;
        it->main_confidence = 1.0 / particles.size();
    }
}


void ParticleLocalization::setCurrentOrientation(const base::samples::RigidBodyState& orientation)
{
  
    if(timestamp.toSeconds() > orientation.time.toSeconds()){
      vehicle_pose.time = timestamp;
    }else{
      vehicle_pose.time = orientation.time;
    }
    
    if(base::samples::RigidBodyState::isValidValue(orientation.orientation)){
    
      vehicle_pose.orientation = Eigen::AngleAxis<double>(filter_config.yaw_offset, Eigen::Vector3d::UnitZ()) * orientation.orientation;
      vehicle_pose.cov_orientation = orientation.cov_orientation;

      motion_pose.orientation = vehicle_pose.orientation;
      motion_pose.cov_orientation = vehicle_pose.cov_orientation;
    }
    
}

void ParticleLocalization::setCurrentVelocity(const base::samples::RigidBodyState& speed)
{
  
  if(base::samples::RigidBodyState::isValidValue(speed.velocity)){
  
    vehicle_pose.velocity = speed.velocity;
    motion_pose.velocity = speed.velocity;
  }
  
}

void ParticleLocalization::setCurrentAngularVelocity(const base::samples::RigidBodyState& speed){
  
  if(base::samples::RigidBodyState::isValidValue(speed.angular_velocity)){
  
    vehicle_pose.angular_velocity = speed.angular_velocity;
    motion_pose.angular_velocity = speed.angular_velocity;
  }
}

void ParticleLocalization::setCurrentZVelocity(const base::samples::RigidBodyState& speed){
  
  if(!std::isnan(speed.velocity.z() )){
    vehicle_pose.velocity.z() = speed.velocity.z();
    motion_pose.velocity.z() = speed.velocity.z();
  }
  
}

void ParticleLocalization::setCurrentDepth(const base::samples::RigidBodyState& depth){
  
  if(!std::isnan(depth.position.z() )){
    vehicle_pose.position.z() = depth.position.z();
    motion_pose.position.z() = depth.position.z();
  }
  
}

void ParticleLocalization::setThrusterVoltage(double voltage){
  if(filter_config.advanced_motion_model)
    dynamic_model->setThrusterVoltage(voltage);
  else
    motion_model.setThrusterVoltage(voltage); 
}


double ParticleLocalization::angleDiffToCorner(double sonar_orientation, base::Vector3d position, Environment* env){
  //Environment env = m.getEnvironment();
  double minAngleDiff = M_PI/2.0;
  
  for (std::vector<Plane>::iterator it = env->planes.begin() ; it != env->planes.end(); it++){
      double angle = atan2(it->position[1] - position[1], it->position[0] - position[0]);
      double angleDiff = fabs(sonar_orientation - angle) < M_PI ? fabs(sonar_orientation - angle) : (2.0*M_PI)-fabs(sonar_orientation - angle); 
      
      if(angleDiff < minAngleDiff)
	minAngleDiff = angleDiff;
      
      angle = atan2((it->position + it->span_horizontal)[1] - position[0],
		    (it->position + it->span_horizontal)[0] - position[0]);
      angleDiff = fabs(sonar_orientation - angle) < M_PI ? fabs(sonar_orientation - angle) : (2.0*M_PI)-fabs(sonar_orientation - angle);
      
      if(angleDiff < minAngleDiff)
	minAngleDiff = angleDiff;    
  }  
  
  return minAngleDiff;
}


void ParticleLocalization::filterZeros(){

    base::Vector3d var = filter_config.init_variance;
    base::Vector3d pos = filter_config.init_position;
  
    for(int i=0; i<3; i++){
      if(var[i] <= 0.0)
        var[i] = 0.0001;
    }  
  
    UniformRealRandom pos_x = Random::uniform_real(pos.x() - var.x() * 0.5, pos.x() + var.x() * 0.5 );
    UniformRealRandom pos_y = Random::uniform_real(pos.y() - var.y() * 0.5, pos.y() + var.y() * 0.5 );
    UniformRealRandom pos_z = Random::uniform_real(pos.z() - var.z() * 0.5, pos.z() + var.z() * 0.5 );  
  
    int count = 0;
    std::list<PoseSlamParticle>::iterator it;
    for(it = particles.begin(); it != particles.end(); ++it) {
        
      //if particle is outside the map, calculate new random position
        if(it->main_confidence == 0.0 || std::isnan(it->main_confidence)){
          it->p_position = base::Vector3d(pos_x(), pos_y(), pos_z());
          it->p_velocity = base::Vector3d(0.0, 0.0, 0.0);
          it->main_confidence = 1.0 / particles.size();
          count++;
        }      
    }
  
}

void ParticleLocalization::setParticlesValid(){

  for( std::list<PoseSlamParticle>::iterator it = particles.begin(); it != particles.end(); it++){
    it->valid = true;
  }
  
  
}

void ParticleLocalization::setObstacles(const sonar_detectors::ObstacleFeatures& z, DepthObstacleGrid& m, const base::samples::RigidBodyState& rbs){
  
  Eigen::AngleAxis<double> sonar_yaw(z.angle, Eigen::Vector3d::UnitZ()); 
  Eigen::AngleAxis<double> abs_yaw(rbs.getYaw(), Eigen::Vector3d::UnitZ());
  Eigen::Affine3d SonarToAvalon(filter_config.sonarToAvalon);
  
  std::vector<Eigen::Vector2d> grid_cells = m.getGridCells( Eigen::Vector2d( rbs.position.x(), rbs.position.y()), z.angle + rbs.getYaw()
                                                          , filter_config.sonar_minimum_distance, filter_config.feature_observation_range);
  
  //std::cout << "Got " << grid_cells.size() << " grid cells" << std::endl;
  for(std::vector<sonar_detectors::ObstacleFeature>::const_iterator it = z.features.begin(); it != z.features.end(); it++){
    
    double distance = it->range / 1000.0;
    
    if(filter_config.feature_filter_threshold > 0.0 && it->confidence < filter_config.feature_filter_threshold)
      continue;
    
    if(distance > filter_config.sonar_maximum_distance || distance < filter_config.sonar_minimum_distance
      || distance < filter_config.feature_observation_minimum_range)
      continue;
    
    Eigen::Vector3d RelativeZ = sonar_yaw * SonarToAvalon * base::Vector3d(distance , 0.0, 0.0);
    Eigen::Vector3d AbsZ = (abs_yaw * RelativeZ) + rbs.position;
    
    double confidence;
    
    if(filter_config.feature_filter_threshold > 0.0)
      confidence = it->confidence;
    else
      confidence = 1.0;
    
    m.setObstacle(AbsZ.x(), AbsZ.y(), true, confidence);
    base::Vector2d z_temp = m.getGridCoord(AbsZ.x(), AbsZ.y()); //Grid cell of the observation
    
    //Remove all corresponding cells
    for(std::vector<Eigen::Vector2d>::iterator it_grid = grid_cells.begin(); it_grid != grid_cells.end(); it_grid++){
      
      if(z_temp == *it_grid){
        it_grid = grid_cells.erase(it_grid);
        
        if(it_grid == grid_cells.end())
          break;
        
      }
      
    } 
  }
    
  //Remove not observed obstacles from map
  //std::cout << grid_cells.size() << " grid cells left after filtering" << std::endl;
  for(std::vector<Eigen::Vector2d>::iterator it_grid = grid_cells.begin(); it_grid != grid_cells.end(); it_grid++){
    
    m.setObstacle(it_grid->x(), it_grid->y(), false, filter_config.feature_weight_reduction); 
    
  }  
  
  //m.reduce_weights(1.0 / (360.0* 8.0));
}

base::samples::Pointcloud ParticleLocalization::getPointCloud(){
  
  base::samples::Pointcloud pc;
  
  if(filter_config.use_slam){
    
    double best_conf = 0.0;
    std::list<PoseSlamParticle>::iterator best_it;
    
    //Search for best particle
    for(std::list<PoseSlamParticle>::iterator it = particles.begin(); it != particles.end(); it++){
      
      if(it->main_confidence > best_conf && it->valid){
        best_conf = it->main_confidence;
        best_it = it;
      }
      
    }
    
    //We need a valid particle
    if(best_conf > 0.0){
      //std::cout << "GetCloud" << std::endl;
      //std::cout << "Depth: " << best_it->depth_cells.size() << " ,Obstacles: " << best_it->obstacle_cells.size() << std::endl;
      pc = dp_slam.getCloud(*best_it);
      std::cout << "Found best particle" << std::endl;
    }
    else{
      std::cout << "Found no particle to create map" << std::endl;
    }
    
  }
  
  return pc; 
  
}


}
