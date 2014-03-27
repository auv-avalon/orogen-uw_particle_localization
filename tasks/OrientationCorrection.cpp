/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "OrientationCorrection.hpp"

using namespace uw_particle_localization;

OrientationCorrection::OrientationCorrection(std::string const& name, TaskCore::TaskState initial_state)
    : OrientationCorrectionBase(name, initial_state)
{
  
}

OrientationCorrection::OrientationCorrection(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : OrientationCorrectionBase(name, engine, initial_state)
{
  
}

OrientationCorrection::~OrientationCorrection()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See OrientationCorrection.hpp for more detailed
// documentation about them.

bool OrientationCorrection::configureHook()
{
    if (! OrientationCorrectionBase::configureHook())
        return false;
    return true;
}
bool OrientationCorrection::startHook()
{
    if (! OrientationCorrectionBase::startHook())
        return false;
    
    offset_buffer.resize(_buffer_size.get());
    
    lastOrientation.time = base::Time::fromSeconds(0);
    lastIMU.time = base::Time::fromSeconds(0);
    offset_recieved = 0;
    
    return true;
}
void OrientationCorrection::updateHook()
{
    OrientationCorrectionBase::updateHook();
    
    base::samples::RigidBodyState ori;
    
    while(_orientation_input.read(ori) == RTT::NewData){
      lastOrientation = ori;
      base::samples::RigidBodyState ori_imu_corrected = ori;
      
      if(!lastIMU.time.isNull()){
	
	ori_imu_corrected.orientation = actNorthOffset * ori.orientation;
	
	if(offset_recieved){
	  ori.orientation = actOffset * ori.orientation;
	}
	else{
	  ori = ori_imu_corrected;
	}	
	
      }
	
      _orientation_output.write(ori);
    }
    
    base::samples::RigidBodyState imu;
    
    while(_absolute_orientation.read(imu) == RTT::NewData){
      
      if(!lastOrientation.time.isNull()){
	lastIMU = imu;
	
	double north_offset = base::getYaw(lastIMU.orientation) - _north_offset.get() - base::getYaw(lastOrientation.orientation);
	
	while(north_offset > M_PI){
	  north_offset -= 2.0 * M_PI;
	}
	
	while(north_offset < M_PI){
	  north_offset += 2.0 * M_PI;
	}
	  
	actNorthOffset = Eigen::AngleAxis<double>(north_offset, Eigen::Vector3d::UnitZ());
	
      }
      
      lastIMU = imu;    
      
    }
    
    double offset;    
    while(_orientation_offset.read(offset) == RTT::NewData){
      
      offset_buffer.push_back(offset);
      
      if(offset_buffer.size() >= _min_buffer_size.get()){
	
	double sonar_offset = calcMedian(offset_buffer);	
	actOffset = actNorthOffset * Eigen::AngleAxis<double>(sonar_offset, Eigen::Vector3d::UnitZ());
      
	offset_recieved = 1;
	
      }
      else{
	actOffset = actNorthOffset;
      }      
    }    
}


double OrientationCorrection::calcMedian(boost::circular_buffer<double> buffer){
  
  if(!buffer.empty()){
   
    std::list<double> sorted_list;
    
    //Sort the buffered offsets
    for(boost::circular_buffer<double>::iterator it = buffer.begin(); it != buffer.end(); it++){
      
      if(sorted_list.empty()){
	sorted_list.push_back(*it);
      }else{
	
	std::list<double>::iterator jt = sorted_list.begin();
		
	while( jt != sorted_list.end() && *jt < *it)
	  jt++;
	
	if(jt != sorted_list.end()){
	  sorted_list.insert(jt, *it);
	}
	else{
	  sorted_list.push_back(*it);
	}
	
      }     
    
    }
    
    //Return the middle element of the list
    std::list<double>::iterator jt = sorted_list.begin();
    
    //since there is no random access to lists, iterate to the middle element
    for(int i = 0; i < sorted_list.size()/2; i++)
      jt++;
    
    return *jt;    
  }
  
  return 0.0;  
}

