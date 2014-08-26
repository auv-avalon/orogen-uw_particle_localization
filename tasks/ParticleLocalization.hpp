/* ----------------------------------------------------------------------------
 * avalon/orogen/uw_particle_localization/ParticleLocalization.hpp
 * written by Christoph Mueller, Oct 2011
 * University of Bremen
 * ----------------------------------------------------------------------------
*/

#ifndef UW_LOCALIZATION__PARTICLE_LOCALIZATION_HPP
#define UW_LOCALIZATION__PARTICLE_LOCALIZATION_HPP

#include <base/eigen.h>
#include <base/samples/rigid_body_state.h>
#include <base/samples/Joints.hpp>
#include <base/samples/laser_scan.h>
#include <machine_learning/RandomNumbers.hpp>
#include <uw_localization/filters/particle_filter.hpp>
#include <uw_localization/model/uw_motion_model.hpp>
#include <uw_localization/maps/node_map.hpp>
#include <uw_localization/maps/grid_map.hpp>
#include <uw_localization/maps/depth_obstacle_grid.hpp>
#include <uw_localization/types/info.hpp>
#include <uw_localization/types/map.hpp>
#include <uw_localization/dp_slam/dp_types.hpp>
#include <offshore_pipeline_detector/pipeline.h>
#include <uwv_dynamic_model/uwv_dynamic_model.h>
#include <sonar_detectors/SonarDetectorTypes.hpp>
#include <visual_detectors/Types.hpp>
#include "LocalizationConfig.hpp"
#include "Types.hpp"
#include "DPSlam.hpp"


namespace uw_localization {

class ParticleLocalization : public ParticleFilter<PoseSlamParticle>,
  public Dynamic<PoseSlamParticle, base::samples::Joints, NodeMap>,
  public Dynamic<PoseSlamParticle, base::samples::RigidBodyState, NodeMap>,
  public Perception<PoseSlamParticle, base::samples::LaserScan, NodeMap>,
  public Perception<PoseSlamParticle, sonar_detectors::ObstacleFeatures, NodeMap>,
  public Perception<PoseSlamParticle, controlData::Pipeline, NodeMap>,
  public Perception<PoseSlamParticle, std::pair<double,double>, NodeMap>,
  public Perception<PoseSlamParticle, avalon::feature::Buoy, NodeMap>,
  public Perception<PoseSlamParticle, double, DepthObstacleGrid>
{
public:
  ParticleLocalization(const FilterConfig& config);
  virtual ~ParticleLocalization();

  static UwVehicleParameter VehicleParameter(FilterConfig filter_config);

  void init_slam(NodeMap *map);
  virtual void initialize(int numbers, const Eigen::Vector3d& pos, const Eigen::Vector3d& cov, double yaw, double yaw_cov);
  static underwaterVehicle::Parameters initializeDynamicModel(UwVehicleParameter p, FilterConfig filter_config);

  virtual base::Position position(const PoseSlamParticle& X) const { return X.p_position; }
  virtual base::Vector3d velocity(const PoseSlamParticle& X) const { return X.pose->velocity; }
  virtual base::samples::RigidBodyState orientation(const PoseSlamParticle& X) const { return *(X.pose); }
  virtual bool isValid(const PoseSlamParticle& X) const {return X.valid; }

  virtual double confidence(const PoseSlamParticle& X) const { return X.main_confidence; }
  virtual void   setConfidence(PoseSlamParticle& X, double weight) { X.main_confidence = weight; }

  virtual void dynamic(PoseSlamParticle& x, const base::samples::RigidBodyState& u, const NodeMap& m);
  virtual void dynamic(PoseSlamParticle& x, const base::samples::Joints& u, const NodeMap& m);

  virtual const base::Time& getTimestamp(const base::samples::RigidBodyState& u);
  virtual const base::Time& getTimestamp(const base::samples::Joints& u);

  virtual double perception(PoseSlamParticle& x, const base::samples::LaserScan& z, NodeMap& m);
  virtual double perception(PoseSlamParticle& x, const controlData::Pipeline& z, NodeMap& m);
  virtual double perception(PoseSlamParticle& x, const avalon::feature::Buoy& z, NodeMap& m);  
  
  /**
   * Calculates the propability of a particle using a recieved list of sonar features
   * @param X: a Particle
   * @param Z: perception of the sonar
   * @param M: the nodemap
   * @return: propability of the particle
   */
  virtual double perception(PoseSlamParticle& x, const sonar_detectors::ObstacleFeatures& z, NodeMap& m);  
    
 /**
 * Calculates the propability of a particle using a received gps-position
 * @param X: a Particle
 * @param T: the perception as a gps-position
 * @param M: the nodemap
 * @return the propability of the particle
 */ 
  virtual double perception(PoseSlamParticle& x, const base::Vector3d& z, NodeMap& m);

  
  /**
   * Calculated the position propability using a depth sample
   * @param x: a position particle
   * @param z: depth sample
   * @param M: the gridmap
   */  
  virtual double perception(PoseSlamParticle& x, const double& z, DepthObstacleGrid& m);
  
  /**
   * Delete a amount of particles and insert randomly new articles
   * @param pos: state of the vehicle, with position and position_covariance
   * @param m: map of the enviroment
   * @param ratio: amount of particles, which will be deleted, in percent
   * @param random_uniform: ignore the position_covariance and interspere random over the environment 
   * @param invalidate_particles: set new particles to invalid -> particles need to be meassured to be valid
   */
  void interspersal(const base::samples::RigidBodyState& pos, const NodeMap& m, double ratio, bool random_uniform, bool invalidate_particles);
  

  double observeAndDebug(const base::samples::LaserScan& z, NodeMap& m, double importance = 1.0);
  
  double observeAndDebug(const sonar_detectors::ObstacleFeatures& z, NodeMap& m, double importance = 1.0);
  
  /**
   * Receives a perception as a gps-position and updates the current particle-set
   * @param z: Perception as an utm-coordinate
   * @param m: nodemap of the enviroment, where the perception takes place
   * @param importance: importace factor of the perception
   * @return: the effectiv sample size (the average square weight)
   */
  double observeAndDebug(const base::samples::RigidBodyState& z, NodeMap& m, double importance = 1.0);

  void debug(double distance, double desire_distance, double angle, const base::Vector3d& desire, const base::Vector3d& real, const base::Vector3d& loc, double conf);
  void debug(double distance, double desire_distance, double angle, const base::Vector3d& desire, const base::Vector3d& real, const base::Vector3d& loc, double conf, PointStatus Status);
  void debug(double distance,  const base::Vector3d& loc, double conf, PointStatus status);
  void debug(const base::Vector3d& pos, double conf, PointStatus status);
  
  void addHistory(const PointInfo& status);

  bool hasStats() const;
  uw_localization::Stats getStats() const;

  void setCurrentOrientation(const base::samples::RigidBodyState& orientation);  
  void setCurrentVelocity(const base::samples::RigidBodyState& speed);
  void setCurrentAngularVelocity(const base::samples::RigidBodyState& speed);
  void setCurrentZVelocity(const base::samples::RigidBodyState& speed);
  void setCurrentDepth(const base::samples::RigidBodyState& depth);

  void update_dead_reckoning(const base::samples::Joints& u);
  const base::samples::RigidBodyState& dead_reckoning() const { return motion_pose; }
  const base::samples::RigidBodyState& full_dead_reckoning() const { return full_motion_pose;}

  void teleportParticles(const base::samples::RigidBodyState& position);

  void setSonarDebug(DebugWriter<uw_localization::PointInfo>* debug) {
      sonar_debug = debug;
  }
  
  void setThrusterVoltage(double voltage);
  
  /**
   * Calculates the angle-difference between the sonar_beam and the nearest corner of the pool
   * @sonar_orientation: the orientation angle of the sonar (yaw)
   * @position: the position of the auv
   * @m: node-map of the pool
   * @return: an absolute angle-difference in radian
   */
  double angleDiffToCorner(double sonar_orientation, base::Vector3d position, Environment* env);
  
  /**
   * Filters particles with zero-confidence
   */
  void filterZeros();
  
  /**
   * Sets all particles in valid-mode. This method should be called after a resampling
   */
  void setParticlesValid();
  
  void setObstacles(const sonar_detectors::ObstacleFeatures& z, DepthObstacleGrid& m, const base::samples::RigidBodyState& rbs);
  
  void observeDepth(const Eigen::Vector3d &pose, const Eigen::Matrix3d pos_covar, double depth);
  
  /**
   * Return the pointcloud-map of the best particle
   * This is only avaiable in use_slam-mode, (else an empty cloud will be returned)
   */
  base::samples::Pointcloud getPointCloud();
  uw_localization::SimpleGrid getSimpleGrid();

private:
  FilterConfig filter_config;
  UwMotionModel motion_model;
  underwaterVehicle::DynamicModel* dynamic_model;
  underwaterVehicle::Parameters dynamic_model_params;
  base::samples::RigidBodyState vehicle_pose;
  base::samples::RigidBodyState motion_pose;
  base::samples::RigidBodyState full_motion_pose;
  base::Time lastActuatorTime;
  DPSlam dp_slam;

  machine_learning::MultiNormalRandom<3> StaticSpeedNoise;
  machine_learning::MultiNormalRandom<3> StaticMotionNoise;

  uw_localization::PointInfo best_sonar_measurement;

  std::list<double> perception_history;
  double perception_history_sum;
  bool used_dvl;
  
  //the origin of the coordinate system as utm-coordinate
  base::Vector3d utm_origin;
  
  //Number of particles with zero confidence
  int zeroConfidenceCount;
  
  //True, if there are invalid measurements caused by incomplete map
  bool measurement_incomplete;

  /** observers */
  DebugWriter<uw_localization::PointInfo>* sonar_debug;
};


}

#endif
