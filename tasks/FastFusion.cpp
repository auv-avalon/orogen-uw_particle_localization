/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "FastFusion.hpp"

using namespace uw_particle_localization;

FastFusion::FastFusion(std::string const& name, TaskCore::TaskState initial_state)
    : FastFusionBase(name, initial_state)
{
}

FastFusion::FastFusion(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : FastFusionBase(name, engine, initial_state)
{
}

FastFusion::~FastFusion()
{
}



bool FastFusion::startHook()
{
    if (! FastFusionBase::startHook())
        return false;
    return true;
}
void FastFusion::updateHook()
{
    FastFusionBase::updateHook();
    
    base::samples::RigidBodyState rbs;
    
    //Update vehicle position
    while(_position_samples.read(rbs) == RTT::NewData){
      
      if(base::samples::RigidBodyState::isValidValue(rbs.position)){
        
        actualPose.position.x() = rbs.position.x();
        actualPose.position.y() = rbs.position.y();
        
        //We never recieved velocity samples or we have a timeout -> use this samples insted!
        if(lastVelocity.isNull() || (rbs.time.toSeconds() - lastVelocity.toSeconds()) > _velocity_timeout.get() ){
          actualPose.velocity.x() = rbs.velocity.x();
          actualPose.velocity.y() = rbs.velocity.y();
        }
        
        
        //Do we have a newer timestamp?
        if(actualPose.time.toSeconds() < rbs.time.toSeconds()){
          actualPose.time = rbs.time;
        }
        
      }
      
    }
    
    //Update vehicle depth
    while(_depth_samples.read(rbs) == RTT::NewData){
      
      if( (!std::isnan(rbs.position.z()) ) && (!std::isnan(rbs.velocity.z() ) ) ) {
        
        actualPose.position.z() = rbs.position.z();
        actualPose.velocity.z() = rbs.velocity.z();
        
        //Do we have a newer timestamp?
        if(actualPose.time.toSeconds() < rbs.time.toSeconds()){
          actualPose.time = rbs.time;
        }
        
      }
    
    }
    
    //Update vehicle orientation
    while(_orientation_samples.read(rbs) == RTT::NewData){
      
      if(base::samples::RigidBodyState::isValidValue(rbs.orientation) &&
        base::samples::RigidBodyState::isValidValue(rbs.angular_velocity) ){
        
        actualPose.orientation = rbs.orientation;
        actualPose.angular_velocity = rbs.angular_velocity;
      
        //Do we have a newer timestamp?
        if(actualPose.time.toSeconds() < rbs.time.toSeconds()){
          actualPose.time = rbs.time;
          
        }
      
      }           
      
    }
    
    //Update vehicle velocity
    while(_velocity_samples.read(rbs) == RTT::NewData){
      
      //We need an valid orientation for body-to-world-transformation
      if(base::samples::RigidBodyState::isValidValue(rbs.velocity) && base::samples::RigidBodyState::isValidValue(actualPose.orientation) ){
        
        //Convert velocity to world frame
        Eigen::Vector3d vel = actualPose.orientation * rbs.velocity;
        
        actualPose.velocity.x() = vel.x();
        actualPose.velocity.y() = vel.y();
        lastVelocity = rbs.time;
        
        //Do we have a newer timestamp?
        if(actualPose.time.toSeconds() < rbs.time.toSeconds()){
          actualPose.time = rbs.time;
        }
        
        
      }
      
      
    }
    
    _pose_samples.write(actualPose);
    
}

