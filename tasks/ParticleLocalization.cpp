#include "ParticleLocalization.hpp"
#include <base/pose.h>
//#include <stdexcept>

using namespace machine_learning;

namespace uw_localization {

base::samples::RigidBodyState* PoseParticle::pose = 0;

ParticleLocalization::ParticleLocalization(const FilterConfig& config) 
    : ParticleFilter<PoseParticle, NodeMap>(), filter_config(config), 
    motion_model(VehicleParameter()),
    StaticSpeedNoise(Random::multi_gaussian(Eigen::Vector3d(0.0, 0.0, 0.0), config.static_motion_covariance)),
    perception_history_sum(0.0),
    sonar_debug(0)
{
    first_perception_received = false;
    PoseParticle::pose = &vehicle_pose;
    utm_origin = std::make_pair(-1,-1);
}

ParticleLocalization::~ParticleLocalization()
{ 
  if(filter_config.advanced_motion_model)
    delete dynamic_model;
}


UwVehicleParameter ParticleLocalization::VehicleParameter() const
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
        PoseParticle pp;
        pp.p_position = base::Vector3d(pos_x(), pos_y(), pos_z());
        pp.p_velocity = base::Vector3d(0.0, 0.0, 0.0);
        pp.main_confidence = 1.0 / numbers;
        
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
    
    if(filter_config.advanced_motion_model)
      initializeDynamicModel(VehicleParameter());
}

void ParticleLocalization::initializeDynamicModel(UwVehicleParameter p){
  dynamic_model = new underwaterVehicle::DynamicModel(0.1,5,0.0);
  //dynamic_model = new underwaterVehicle::DynamicModel();
  
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
  params.thrusters.resize(6);
  params.thrusters.thruster_mapped_names.clear();
  
  for(int i=0; i<6; i++){
      if(filter_config.param_TCM[i] > 0.0)
	params.thrusters.thruster_mapped_names.push_back(underwaterVehicle::SURGE);
      else if(filter_config.param_TCM[6+i] > 0.0)
	params.thrusters.thruster_mapped_names.push_back(underwaterVehicle::SWAY);
      else if(filter_config.param_TCM[12+i] > 0.0)
	params.thrusters.thruster_mapped_names.push_back(underwaterVehicle::HEAVE);
      else
	params.thrusters.thruster_mapped_names.push_back(underwaterVehicle::UNINITIALISED);
  }    
  
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
  
  dynamic_model->init_param(params);
  dynamic_model_params=params;
  
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
    X.p_velocity[2] = vehicle_pose.velocity[2];
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
        }else if(filter_config.advanced_motion_model){	  	  

	  underwaterVehicle::ThrusterMapping input_thruster_data;
	  //input_thruster_data.thruster_mapped_names = input_uwv_parameters.thrusters.thruster_mapped_names;
	  input_thruster_data.thruster_value.resize(5);
	  for(unsigned int i=0;i<Ut.states.size();i++){
	    input_thruster_data.thruster_value[i] = Ut.states[i].pwm;
	  } 
	    	  
	  dynamic_model->setPosition(X.p_position);
	  dynamic_model->setLinearVelocity(X.p_velocity);
	  dynamic_model->setSamplingtime(dt);
	  dynamic_model->setPWMLevels(input_thruster_data);	
	  
	  u_velocity = dynamic_model->getLinearVelocity();
	  
	}else{  
          Xt << X.p_velocity.x(), X.p_velocity.y(), X.p_velocity.z(), 
               X.p_position.x(), X.p_position.y(), X.p_position.z();

            Vector6d U = motion_model.transition(Xt, dt, Ut);

            u_velocity = U.block<3, 1>(0, 0);	   
	}   


        v_noisy = u_velocity + StaticSpeedNoise();
        
        if(v_noisy.x() > 0.55)
            v_noisy.x() = 0.55;
        else if(v_noisy.x() < -0.55)
            v_noisy.x() = -0.55;

        base::Vector3d v_avg = (X.p_velocity + v_noisy) / 2.0;
	
	if(vehicle_pose.hasValidOrientation() && vehicle_pose.hasValidVelocity()){
	  X.p_position = X.p_position + vehicle_pose.orientation * (v_avg * dt);
	  X.p_velocity = v_noisy;	  
	}
    }
    
    X.p_position.z() = vehicle_pose.position.z();
    X.timestamp = Ut.time;
}
 

void ParticleLocalization::update_dead_reckoning(const base::actuators::Status& Ut)
{
    if( !lastActuatorTime.isNull() ) {
        Vector6d Xt;
	base::Vector3d u_t1;
	double dt = (Ut.time - lastActuatorTime).toSeconds();
	
	if(filter_config.advanced_motion_model){
	  	    
	  underwaterVehicle::ThrusterMapping input_thruster_data;
	  //input_thruster_data.thruster_mapped_names = input_uwv_parameters.thrusters.thruster_mapped_names;
	  input_thruster_data.resize(6);
	  for(unsigned int i=0;i<Ut.states.size();i++){
	    input_thruster_data.thruster_value[i] = Ut.states[i].pwm;
	  }
	  input_thruster_data.thruster_mapped_names = dynamic_model_params.thrusters.thruster_mapped_names;	  
	  
	  //std::cout << "Velocity: " << motion_pose.velocity[0] << " " << motion_pose.velocity[1] << std::endl;
	  dynamic_model->setPosition(motion_pose.position);
	  dynamic_model->setLinearVelocity(motion_pose.velocity);
	  dynamic_model->setAngularVelocity(base::Vector3d::Zero());
	  dynamic_model->setOrientation(vehicle_pose.orientation);
	  dynamic_model->setSamplingtime(dt);
	  
	  dynamic_model->setPWMLevels(input_thruster_data);	   
	  
	  u_t1 = dynamic_model->getLinearVelocity();	
	  
	  //Full dead reckoning
	  dynamic_model->setPosition(full_motion_pose.position);
	  dynamic_model->setLinearVelocity(full_motion_pose.velocity);
	  dynamic_model->setAngularVelocity(full_motion_pose.angular_velocity);
	  dynamic_model->setOrientation(full_motion_pose.orientation);
	  
	  dynamic_model->setPWMLevels(input_thruster_data);
	  
	  full_motion_pose.position = dynamic_model->getPosition();
	  full_motion_pose.velocity = dynamic_model->getLinearVelocity();
	  full_motion_pose.angular_velocity = dynamic_model->getAngularVelocity();
	  full_motion_pose.orientation = dynamic_model->getOrientation_in_Quat();
	  full_motion_pose.time = Ut.time;
	  
	  
	  //std::cout << "new Velocity: " << u_t1[0] << " " << u_t1[1] << std::endl ;
	}else{
	  
	  Xt.block<3,1>(0,0) = motion_pose.velocity;
	  Xt.block<3,1>(3,0) = base::Vector3d(0.0, 0.0, 0.0);
	  
	  Vector6d U = motion_model.transition(Xt, dt, Ut);
	  u_t1 = U.block<3,1>(0,0);	   
	}

        base::Vector3d v_avg = (motion_pose.velocity + u_t1) / 2.0;
        
	if((!base::isUnset<double>(u_t1[0])) && (!base::isUnset<double>(u_t1[1])) && (!base::isUnset<double>(u_t1[2]))
	  && vehicle_pose.hasValidOrientation() && vehicle_pose.hasValidVelocity()){
	  motion_pose.position = motion_pose.position + vehicle_pose.orientation * (v_avg * dt);
	  motion_pose.velocity = u_t1;	 
	
	}else{
	  std::cout << "Error in motion_model. Velocity or orientation is unset." << std::endl;
	  if(!vehicle_pose.hasValidOrientation())
	    std::cout << "Invalid orientation" << std::endl;
	}  
	
    } 
    
    lastActuatorTime = Ut.time;
    motion_pose.time = base::Time::now();
    motion_pose.velocity[2] = vehicle_pose.velocity[2];
    motion_pose.angular_velocity = vehicle_pose.angular_velocity;
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

    if(best_sonar_measurement.status == OKAY)
        addHistory(best_sonar_measurement);

    best_sonar_measurement.confidence = -1.0;

    return effective_sample_size;
}

double ParticleLocalization::observeAndDebug(const base::samples::RigidBodyState& z, const NodeMap& m, double importance)
{	
    if(utm_origin.first==-1){
	utm_origin = std::make_pair(z.position[0], z.position[1]);
    }
    
    //Converts the utm-coordinate to our world coordinate system
    std::pair<double, double> pose;
    pose.first = cos(filter_config.utm_relative_angle)*(z.position[0]-utm_origin.first)
	  - sin(filter_config.utm_relative_angle)*(z.position[1]-utm_origin.second)
	  + filter_config.init_position[0];
    pose.second = cos(filter_config.utm_relative_angle)*(z.position[1]-utm_origin.second)
	  + sin(filter_config.utm_relative_angle)*(z.position[0]-utm_origin.first)
	  + filter_config.init_position[1];    
    
    //Sets the covarianz matrix of the rbs
    base::samples::RigidBodyState newRBS = z;
    base::Matrix3d cov = base::Matrix3d::Zero();
    cov << filter_config.gps_covarianz, 0, 0,
	0, filter_config.gps_covarianz, 0,
	0, 0, filter_config.gps_covarianz;
    newRBS.cov_position = cov;
    
    newRBS.position[0]=pose.first;
    newRBS.position[1]=pose.second;
    
    //Creates new particle at the gps-position
    interspersal(newRBS, m, filter_config.gps_interspersal_ratio);	  
    
    double effective_sample_size = observe(pose, m, importance);   
    
    best_sonar_measurement.time = z.time;
    
    if(best_sonar_measurement.status == OKAY)
      addHistory(best_sonar_measurement);
    
    best_sonar_measurement.confidence = -1.0;
    timestamp = base::Time::now();
    
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
    boost::tuple<Node*, double, Eigen::Vector3d> distance_box = M.getNearestDistance("root.box",
				  Eigen::Vector3d(0.0, filter_config.sonar_vertical_angle/2.0, yaw + angle), X.p_position);
				  
    if(distance.get<1>() < distance_box.get<1>() - z_distance){
      distance_box.get<1>() = distance_box.get<1>() - z_distance;
      distance = distance_box;
    }
    
    double covar = filter_config.sonar_covariance;
    
    if(distance.get<1>() > vehicle_pose.position[2]/sin(filter_config.sonar_vertical_angle/2.0))
      covar = covar * filter_config.sonar_covariance_reflection_factor;
    

    if(angleDiffToCorner(angle+yaw, X.p_position, filter_config.env) < 0.1)
      covar = covar * filter_config.sonar_covariance_corner_factor;
    
    double probability = gaussian1d(0.0, covar, distance.get<1>());
    
    debug(z_distance, distance.get<2>(), AbsZ, X.p_position, probability);

    first_perception_received = true;

    return probability;
}

double ParticleLocalization::perception(const PoseParticle& X, const controlData::Pipeline& Z, const NodeMap& M) 
{
    double yaw = base::getYaw(vehicle_pose.orientation);
    Eigen::AngleAxis<double> abs_yaw(yaw, Eigen::Vector3d::UnitZ());

    Eigen::Vector3d RelZ(-0.7, 0.0, -2.0);
    Eigen::Vector3d AbsZ = (abs_yaw * RelZ) + X.p_position;

    boost::tuple<Node*, double, Eigen::Vector3d> distance;
    if(Z.inspection_state == controlData::END_OF_PIPE)
        distance = M.getNearestDistance("root.end_of_pipe", AbsZ, X.p_position);
    else
        distance = M.getNearestDistance("root.pipeline", AbsZ, X.p_position);

    double probability = gaussian1d(0.0, filter_config.pipeline_covariance, distance.get<1>());
    
    first_perception_received = true;

    return probability;
}


double ParticleLocalization::perception(const PoseParticle& X, const std::pair<double,double>& Z, const NodeMap& M)
{
    Eigen::Matrix<double,2,1> pos;
    pos << X.p_position[0] , X.p_position[1];
    Eigen::Matrix<double,2,2> covar;
    covar << filter_config.gps_covarianz, 0, 0, filter_config.gps_covarianz;
    Eigen::Matrix<double,2,1> gps; 
    gps << Z.first, Z.second;    
    
    //check if this particle is part of the world
    if(filter_config.useMap && !M.belongsToWorld(X.p_position)) {
        debug(Z, 0.0, NOT_IN_WORLD); 
        return 0.0;
    }
    
    //double propability = calc_gaussian(pos, covar, gps);
    
    double diff=std::sqrt(std::pow(X.p_position[0]-Z.first, 2.0) + std::pow(X.p_position[1]-Z.second, 2.0));
    double propability = gaussian1d(0, filter_config.gps_covarianz, diff);
    
    debug(Z,propability,OKAY);
    
    first_perception_received = true;
    
    return propability;
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

void ParticleLocalization::interspersal(const base::samples::RigidBodyState& p, const NodeMap& m, double ratio)
{
    reduceParticles(1.0 - ratio);

    PoseParticle best = particles.front();

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

void ParticleLocalization::debug(std::pair<double, double> pos, double conf, PointStatus status)
{
    if(best_sonar_measurement.confidence < conf){
	uw_localization::PointInfo info;
	info.distance = 0.0;
	info.desire_point = base::Vector3d(0.0,0.0,0.0);
	info.real_point = base::Vector3d(0.0,0.0,0.0);
	info.location = base::Vector3d(pos.first,pos.second,0.0);
	info.confidence = conf;
	info.status = status;
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
    vehicle_pose.velocity[2] = orientation.velocity[2];
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

}
