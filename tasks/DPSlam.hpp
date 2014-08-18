#ifndef UW_LOCALIZATION_DPSLAM_DPSLAM_HPP
#define UW_LOCALIZATION_DPSLAM_DPSLAM_HPP

#include "LocalizationConfig.hpp"
#include <machine_learning/GaussianParameters.hpp>
#include <uw_localization/dp_slam/dp_map.hpp>
#include <uw_localization/dp_slam/dp_types.hpp>
#include <uw_localization/maps/node_map.hpp>
#include <sonar_detectors/SonarDetectorTypes.hpp>
#include <cmath>

namespace uw_localization{
  
  class DPSlam{
    
  private:
    DPMap* map;
    NodeMap* node_map;
    FilterConfig config;
    double lastAngle;
    double sumAngle;
    
  public:
    
    DPSlam();
    
    ~DPSlam();
    
    void init(base::Vector2d position, base::Vector2d span, double resolution, FilterConfig config);
    
    void initalize_statics(NodeMap *map);
    
    /**
     * Observe the depth for one particle
     * @param X: the position and map particle
     * @param depth: Depth measurement
     * @return: Perception confidence
     */
    double observe(PoseSlamParticle &X, const double &depth);
    
    /**
     * Observe the sonar measurement for one particle
     */
    double observe(PoseSlamParticle &X, const sonar_detectors::ObstacleFeatures& Z, double vehicle_yaw, double vehicle_depth);
    
    /**
     * Rate a given particle
     * @param distances: list of observed laser distances
     * @param distances_cells: list of simulated map distances
     * @return: probalility of the messurement
     */
    double rateParticle(std::list<double> &distances, std::list< std::pair<double,double > > &distances_cells);
    
    base::samples::Pointcloud getCloud(PoseSlamParticle &X);
    
    /**
     * Reduces the weight of the particles
     * The redusing event is triggered, when the sum of the scan angle reaches max_sum
     */
    void reduceFeatures(double angle, double max_sum = 4 * M_PI);
    
  };
  
  
}



#endif