#include "DPSlam.hpp"

using namespace uw_localization;

DPSlam::DPSlam(){
 
  map = 0;
}

DPSlam::~DPSlam(){
   delete map;
   map = 0;
}

void DPSlam::init(base::Vector2d position, base::Vector2d span, double resolution, const FilterConfig config){
  
  this->config = config;
  delete map;
  
  map = new DPMap(position, span, resolution);  
  map->initGrid();
}

double DPSlam::observe(PoseSlamParticle &X, const double &depth){
  
  base::Vector2d pos = map->getGridCoord(X.p_position.x(), X.p_position.y());
  
  //Search for correspondig cell
  for(std::list<std::pair<Eigen::Vector2d,int64_t > >::iterator it = X.depth_cells.begin(); it != X.depth_cells.end(); it++){
    
    //We found a match
    if(it->first == pos){
      int64_t id = map->setDepth(pos.x(), pos.y(), depth, X.main_confidence, it->second);
      
      if(id != 0 && id != it->second)
        it->second = id;
      
      
      return X.main_confidence;
    }
    
  }
  
  int64_t id = map->setDepth(pos.x(), pos.y(), depth, X.main_confidence, 0);
 
  if(id != 0)
    X.depth_cells.push_back( std::make_pair(pos, id) );
  
  return X.main_confidence;
  
}


double DPSlam::observe(PoseSlamParticle &X, const sonar_detectors::ObstacleFeatures& Z, double vehicle_yaw){
  std::vector<base::Vector2d> cells = map->getGridCells( base::Vector2d(X.p_position.x(), X.p_position.y()), Z.angle,
                                                         config.feature_observation_minimum_range, config.feature_observation_range);
  
  Eigen::AngleAxis<double> abs_yaw(Z.angle, Eigen::Vector3d::UnitZ());
  
 for(std::vector<sonar_detectors::ObstacleFeature>::const_iterator it_f = Z.features.begin(); it_f != Z.features.end(); it_f++){
      
      double dist = it_f->range / 1000.0;
      base::Vector3d real_pos = X.p_position + (abs_yaw * base::Vector3d(dist, 0.0, 0.0 ) );
      
      base::Vector2d feature_discrete = map->getGridCoord(real_pos.x(), real_pos.y() );
      
      //Search for coresponding grid cells
      for(std::vector<base::Vector2d>::iterator it_c = cells.begin(); it_c != cells.end(); it_c++){
        
        if(feature_discrete == *it_c){
          
          cells.erase(it_c);
          break;       
          
        }        
      }     
      
      bool found_match = false;
      
      for(std::list<std::pair<Eigen::Vector2d,int64_t > >::iterator it = X.obstacle_cells.begin(); it != X.obstacle_cells.end(); it++){
        
        if(feature_discrete == it->first){
          int64_t id = map->setObstacle(feature_discrete.x(), feature_discrete.y(), true, X.main_confidence, it->second);
          
          if(id != 0)
            it->second = id;
          
          found_match = true;
          break;
        }
        
        //TODO rate partivle
        
      } 
      
      if(!found_match){
      
        int64_t id = map->setObstacle(feature_discrete.x(), feature_discrete.y(), true, X.main_confidence, 0);
        
        if(id != 0)
          X.obstacle_cells.push_back(std::make_pair(feature_discrete, id));
      }
  }
  
  //TODO handle empty cells
  
  
  return X.main_confidence;
  
}

base::samples::Pointcloud DPSlam::getCloud(PoseSlamParticle &X){
  
  return map->getCloud(X.depth_cells, X.obstacle_cells);
  
}


