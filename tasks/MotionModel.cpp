/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "MotionModel.hpp"
#include "ParticleLocalization.hpp"

using namespace uw_particle_localization;

MotionModel::MotionModel(std::string const& name, TaskCore::TaskState initial_state)
    : MotionModelBase(name)
{
  
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
}

MotionModel::MotionModel(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : MotionModelBase(name, engine)
{
  
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
}

MotionModel::~MotionModel()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See MotionModel.hpp for more detailed
// documentation about them.

bool MotionModel::configureHook()
{
    if (! MotionModelBase::configureHook())
        return false;
    return true;
}
bool MotionModel::startHook()
{
    if (! MotionModelBase::startHook())
        return false;
    initMotionConfig();
    
    advanced_model = _advanced_motion_model;
    uw_localization::UwVehicleParameter uw_param = uw_localization::ParticleLocalization::VehicleParameter(config);
    
    if(advanced_model){
      underwaterVehicle::Parameters params = uw_localization::ParticleLocalization::initializeDynamicModel(uw_param, config);
      dynamic_model = new underwaterVehicle::DynamicModel(0.1, 5, 0.0, NULL, 12, 6);
      dynamic_model->init_param(params);      
    }
    else{
      motion_model.init(uw_param);
    }    
    
    motion_pose.position = base::Vector3d::Zero();
    motion_pose.velocity = base::Vector3d::Zero();
    motion_pose.cov_velocity = _velocity_covariance.value().asDiagonal();
    motion_pose.cov_position = _velocity_covariance.value().asDiagonal();
    
    return true;
}

void MotionModel::orientation_samplesCallback(const base::Time& time, const base::samples::RigidBodyState& rbs){
  
  motion_pose.time = time;
  last_orientation = rbs;
}

void MotionModel::thruster_samplesCallback(const base::Time& ts, const base::samples::Joints& joint){
    
    motion_pose.time = ts;  

      if(last_orientation.hasValidOrientation()){
  
        base::samples::Joints j = joint;
        
        //If we have no 6 thruster, fill up the joints with zeros
        if(j.size() < 6){
          
          unsigned int size = j.size();
          j.elements.resize(6);
          j.names.resize(6);
          
          for(; size < 6; size++){
            
            j.elements[size].raw = 0.0;
            
          }
          
        }       
        
        if(joint.hasNames()){
        
          for(unsigned int i = 0; i < joint.size() && i < config.joint_names.size(); i++){
            
              try{
                j.elements[i] = joint[config.joint_names[i]]; 
              }
              catch(...){
              }   
          }  
        }
        
      if(!last_orientation.time.isNull() && !lastThrusterTime.isNull() ) {
          base::Vector6d Xt;
          base::Vector3d u_t1;
          double dt = (j.time - lastThrusterTime).toSeconds();
          
          if(dt < 5.0 && dt >= 0.0){
            
            if(advanced_model){
                        
              //std::cout << "Velocity: " << motion_pose.velocity[0] << " " << motion_pose.velocity[1] << std::endl;
              dynamic_model->setPosition(motion_pose.position);
              dynamic_model->setLinearVelocity(motion_pose.velocity);
              dynamic_model->setAngularVelocity(base::Vector3d::Zero());
              dynamic_model->setOrientation(last_orientation.orientation);
              dynamic_model->setSamplingtime(dt);
              
              dynamic_model->setPWMLevels(j);        
              
              u_t1 = dynamic_model->getLinearVelocity();  
  ;
            }else{
              
              Xt.block<3,1>(0,0) = motion_pose.velocity;
              Xt.block<3,1>(3,0) = base::Vector3d(0.0, 0.0, 0.0);
              
              base::Vector6d V = motion_model.transition(Xt, dt, j);
              u_t1 = V.block<3,1>(0,0);      
            }

            base::Vector3d v_avg = (motion_pose.velocity + u_t1) / 2.0;            

            if(base::samples::RigidBodyState::isValidValue(u_t1) && last_orientation.hasValidOrientation() ){
              
              motion_pose.position = motion_pose.position + last_orientation.orientation * (v_avg * dt);
              motion_pose.velocity = u_t1;
              motion_pose.cov_velocity = _velocity_covariance.value().asDiagonal();
              motion_pose.cov_position = motion_pose.cov_position + motion_pose.cov_velocity * dt;              
            
            }else{
              std::cout << "Error in motion_model. Velocity or orientation is unset." << std::endl;
              if(!last_orientation.hasValidOrientation())
                std::cout << "Invalid orientation" << std::endl;
            }

          }else{
            std::cout << "Timestampdifference between thruster-samples to big" << std::endl;
            std::cout << lastThrusterTime.toString() << std::endl;
            std::cout << j.time.toString() << std::endl;
          }
          
      }     
        
        lastThrusterTime = joint.time;      
        motion_pose.orientation = last_orientation.orientation;
      }       
}

void MotionModel::updateHook()
{
    MotionModelBase::updateHook();
    
    if(!last_orientation.time.isNull())
      _pose_samples.write(motion_pose);
    
}

bool MotionModel::initMotionConfig()
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
