#ifndef UW_LOCALIZATION_DPSLAM_DPSLAM_HPP
#define UW_LOCALIZATION_DPSLAM_DPSLAM_HPP

#include "LocalizationConfig.hpp"
#include <uw_localization/dp_slam/dp_map.hpp>
#include <uw_localization/dp_slam/dp_types.hpp>
#include <sonar_feature_estimator/FeatureEstimationDebugTypes.hpp>

namespace uw_localization{
  
  class DPSlam{
    
  private:
    DPMap* map;
    FilterConfig config;
    
  public:
    
    DPSlam();
    
    ~DPSlam();
    
    void init(base::Vector2d position, base::Vector2d span, double resolution, FilterConfig config);
    
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
    double observe(PoseSlamParticle &X, const sonar_detectors::ObstacleFeatures& Z, double vehicle_yaw);
    
    base::samples::Pointcloud getCloud(PoseSlamParticle &X);
    
  };
  
  
}



#endif