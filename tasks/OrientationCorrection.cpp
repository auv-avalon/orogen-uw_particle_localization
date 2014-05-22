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
    actNorthOffset = Eigen::AngleAxis<double>(0.0, Eigen::Vector3d::UnitZ());
    
    return true;
}
void OrientationCorrection::updateHook()
{
    OrientationCorrectionBase::updateHook();
    
    base::samples::RigidBodyState ori;
    
    while(_orientation_input.read(ori) == RTT::NewData){
      lastOrientation = ori;
      base::samples::RigidBodyState ori_corrected = ori;
      
	ori_corrected.orientation = actNorthOffset * ori.orientation;
	
	if(offset_recieved){
	  ori.orientation = actOffset * ori.orientation;
	}
	else{
	  ori = ori_corrected;
	}	
	
      _orientation_output.write(ori_corrected);
      _orientation_offset_corrected.write(ori);
    }
        
    sonar_wall_hough::PositionQuality offset;    
    while(_orientation_offset.read(offset) == RTT::NewData){
      
      offset_buffer.push_back(offset.orientation_drift);
      
      if(offset_buffer.size() >= _min_buffer_size.get()){
	
	double sonar_offset = calcMedian(offset_buffer);
        actOffsetVal = -sonar_offset;
	actOffset = actNorthOffset * Eigen::AngleAxis<double>(-sonar_offset, Eigen::Vector3d::UnitZ());
      
	offset_recieved = 1;
	
      }
      else{
	actOffset = actNorthOffset;
      }      
    }
    
    if(!lastResetRecieved.isNull() && (base::Time::now().toSeconds() - lastResetRecieved.toSeconds()) > 10.0)
      state(RUNNING);
    
}


bool OrientationCorrection::reset(double angle){
  std::cout << "Reset Called" << std::endl;
  if(lastOrientation.time.isNull()){
    std::cout << "No initial Orientation recieved" << std::endl;
    return false;
  }
  
  actNorthOffset = Eigen::AngleAxis<double>( angle - base::getYaw(lastOrientation.orientation) , Eigen::Vector3d::UnitZ());
  state(RESET);  
  
  lastResetRecieved = base::Time::now();
  
  return true;
}

double OrientationCorrection::calcMedian(boost::circular_buffer<double> &buffer){
  
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
    for(int i = 0; i < sorted_list.size()/2; i++){
      jt++;
      
    }
        
    return *jt;    
  }
  
  return 0.0;  
}

void OrientationCorrection::middleOffsets(boost::circular_buffer<double> &buffer){
  
  if(!buffer.empty()){
    
    for(boost::circular_buffer<double>::iterator it = buffer.begin(); it != buffer.end(); it++){
      
      *it -= actOffsetVal;
      
    }   
    
  } 
  
}

