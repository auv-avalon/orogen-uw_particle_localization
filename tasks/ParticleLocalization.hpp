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
#include <base/samples/laser_scan.h>
#include <machine_learning/RandomNumbers.hpp>
#include <uw_localization/filters/particle_filter.hpp>
#include <uw_localization/maps/node_map.hpp>
#include <uw_localization/types/info.hpp>
#include "LocalizationConfig.hpp"
#include "Types.hpp"

namespace uw_localization {

struct PoseParticle {
  base::Position p_position;
  base::Vector3d p_velocity;
  base::Time timestamp;

  double main_confidence;

  static base::samples::RigidBodyState* pose;
};


class ParticleLocalization : public ParticleFilter<PoseParticle, base::samples::RigidBodyState, NodeMap>,
  public Perception<PoseParticle, base::samples::LaserScan, NodeMap>
{
public:
  ParticleLocalization(const FilterConfig& config);
  virtual ~ParticleLocalization();

  virtual void initialize(int numbers, const Eigen::Vector3d& pos, const Eigen::Vector3d& cov, double yaw, double yaw_cov);

  virtual base::Position position(const PoseParticle& X) const { return X.p_position; }
  virtual base::samples::RigidBodyState orientation(const PoseParticle& X) const { return *(X.pose); }

  virtual double confidence(const PoseParticle& X) const { return X.main_confidence; }
  virtual void   setConfidence(PoseParticle& X, double weight) { X.main_confidence = weight; }

  virtual void dynamic(PoseParticle& x, const base::samples::RigidBodyState& u);
  virtual const base::Time& getTimestamp(const base::samples::RigidBodyState& u);

  virtual double perception(const PoseParticle& x, const base::samples::LaserScan& z, const NodeMap& m);

  double observeAndDebug(const base::samples::LaserScan& z, const NodeMap& m, double importance = 1.0);

  void debug(double distance, const base::Vector3d& desire, const base::Vector3d& real, const base::Vector3d& loc, double conf);
  void debug(double distance,  const base::Vector3d& loc, double conf, PointStatus status);

  void setCurrentOrientation(const base::samples::RigidBodyState& orientation);
  void setCurrentSpeed(const base::samples::RigidBodyState& speed);

  void teleportParticles(const base::samples::RigidBodyState& position);

  void setSonarDebug(DebugWriter<uw_localization::PointInfo>* debug) {
      sonar_debug = debug;
  }

private:
  FilterConfig filter_config;
  base::samples::RigidBodyState vehicle_pose;
  machine_learning::MultiNormalRandom<3> StaticSpeedNoise;
  double z_sample;

  uw_localization::PointInfo best_sonar_measurement;

  /** observers */
  DebugWriter<uw_localization::PointInfo>* sonar_debug;
};


}

#endif
