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

#include <uw_localization/maps/node_map.hpp>

#include <uw_localization/types/info.hpp>
#include <uw_localization/types/map.hpp>

#include <offshore_pipeline_detector/pipeline.h>

#include <sonar_detectors/SonarDetectorTypes.hpp>

#include "LocalizationConfig.hpp"
#include "Types.hpp"


namespace uw_localization {

class ParticleLocalization : public ParticleFilter<PoseParticle>,
  public Dynamic<PoseParticle, base::samples::RigidBodyState, NodeMap>,
  public Perception<PoseParticle, base::samples::LaserScan, NodeMap>,
  public Perception<PoseParticle, base::samples::RigidBodyState, NodeMap>,
  public Perception<PoseParticle, uw_localization::AngleWithTimestamp, NodeMap>,
  public Perception<PoseParticle, base::Position, NodeMap>
{
public:
  ParticleLocalization(const FilterConfig& config);
  virtual ~ParticleLocalization();

  virtual void initialize(int numbers, const Eigen::Vector3d& pos, const Eigen::Vector3d& cov, double yaw, double yaw_cov);
  
  void updateConfig(const FilterConfig& config);

  virtual base::Position position(const PoseParticle& X) const { return X.p_position; }
  virtual base::Vector3d velocity(const PoseParticle& X) const { return X.pose->velocity; }
  virtual base::samples::RigidBodyState orientation(const PoseParticle& X) const { return *(X.pose); }
  virtual bool isValid(const PoseParticle& X) const {return X.valid; }
  virtual void setValid(PoseParticle &X, bool flag){ X.valid = flag; }

  virtual double confidence(const PoseParticle& X) const { return X.main_confidence; }
  virtual void   setConfidence(PoseParticle& X, double weight) { X.main_confidence = weight; }

  virtual void dynamic(PoseParticle& x, const base::samples::RigidBodyState& u, const NodeMap& m);

  virtual const base::Time& getTimestamp(const base::samples::RigidBodyState& u);
  virtual const base::Time& getTimestamp(const base::samples::Joints& u);
  base::Time getCurrentTimestamp();

  virtual double perception(PoseParticle& x, const base::samples::LaserScan& z, NodeMap& m);
  virtual double perception(PoseParticle& x, const base::samples::RigidBodyState& z, NodeMap& m);
  virtual double perception(PoseParticle& x, const uw_localization::AngleWithTimestamp& z, NodeMap& m);
  virtual double perception(PoseParticle& x, const base::Position& z, NodeMap&);
  

  
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
  double observeAndDebug(const base::samples::RigidBodyState& z, NodeMap& m, double importance);
  double observeAndDebug(const uw_localization::AngleWithTimestamp& z, NodeMap& m, double importance);
  double observeAndDebug(const base::Position& z, NodeMap& m, double importance);
  
  void debug(double distance, double desire_distance, double angle, const base::Vector3d& desire, const base::Vector3d& real, const base::Vector3d& loc, double conf);
  void debug(double distance, double desire_distance, double angle, const base::Vector3d& desire, const base::Vector3d& real, const base::Vector3d& loc, double conf, PointStatus Status);
  void debug(double distance,  const base::Vector3d& loc, double conf, PointStatus status);
  
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

  void setSonarDebug(DebugWriter<uw_localization::PointInfo>* debug) {
      sonar_debug = debug;
  }

  
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
  

private:
  FilterConfig filter_config;

  base::samples::RigidBodyState vehicle_pose;
  base::samples::RigidBodyState motion_pose;
  base::samples::RigidBodyState full_motion_pose;
  base::Time lastActuatorTime;


  machine_learning::MultiNormalRandom<3> StaticSpeedNoise;


  uw_localization::PointInfo best_sonar_measurement;

  std::list<double> perception_history;
  double perception_history_sum;
  bool used_dvl;
  
  
  //Number of particles with zero confidence
  int zeroConfidenceCount;
  
  //True, if there are invalid measurements caused by incomplete map
  bool measurement_incomplete;

  /** observers */
  DebugWriter<uw_localization::PointInfo>* sonar_debug;
};


}

#endif
