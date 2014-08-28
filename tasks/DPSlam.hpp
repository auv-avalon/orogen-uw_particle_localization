#ifndef UW_LOCALIZATION_DPSLAM_DPSLAM_HPP
#define UW_LOCALIZATION_DPSLAM_DPSLAM_HPP

#include "LocalizationConfig.hpp"
#include <machine_learning/GaussianParameters.hpp>
#include <uw_localization/dp_slam/dp_map.hpp>
#include <uw_localization/dp_slam/dp_types.hpp>
#include <uw_localization/maps/node_map.hpp>
#include <uw_localization/types/map.hpp>
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
    
    /**
     * Initialize the map of the dp-slam
     * @param position: relaive position of the coordinate-center
     * @param span: size of the map
     * @param resolution: size of one cell
     * @param config: filter-config
     */
    void init(base::Vector2d position, base::Vector2d span, double resolution, FilterConfig config);
    
    /**
     * Set static elements in the grid-map
     */
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
     * Depth-observation for a generall, non-particle map
     */
    double observeDepth(const base::Vector3d &pos, const base::Matrix3d &pos_covar, const double &depth);
    
    /**
     * Rate a given particle
     * @param distances: list of observed laser distances
     * @param distances_cells: list of simulated map distances
     * @return: probalility of the messurement
     */
    double rateParticle(std::list<double> &distances, std::list< std::pair<double,double > > &distances_cells);
    
    /**
     * Get a pointcloud-representation of one particle-map
     */
    base::samples::Pointcloud getCloud(PoseSlamParticle &X);
    void getSimpleGrid(PoseSlamParticle &X, uw_localization::SimpleGrid &grid);
    
    /**
     * Reduces the weight of the particles
     * The reduce-event is triggered, when the sum of the scan angle reaches max_sum
     */
    void reduceFeatures(double angle, double max_sum = 4 * M_PI);
    
  };
  
  
}



#endif