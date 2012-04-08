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
#include "LocalizationConfig.hpp"

namespace uw_localization {

struct PoseParticle {
  base::Position position;
  base::Vector3d velocity;
  base::Time timestamp;
  double confidence;
};


class ParticleLocalization : public ParticleFilter<PoseParticle, base::samples::RigidBodyState, NodeMap>,
  public Perception<PoseParticle, base::samples::LaserScan, NodeMap>
{
public:
  ParticleLocalization(const FilterConfig& config);
  ~ParticleLocalization();

  virtual void initialize(int numbers, const Eigen::Vector3d& pos, const Eigen::Matrix3d& cov, double yaw, double yaw_cov);

  virtual void dynamic(PoseParticle& x, const base::samples::RigidBodyState& u);
  virtual const base::Time& getTimestamp(const base::samples::RigidBodyState& u);

  virtual double perception(const PoseParticle& x, const base::samples::LaserScan& z, const NodeMap& m);
  virtual bool isMaximumRange(const base::samples::LaserScan& z);
  
  virtual base::Vector3d position(const PoseParticle& state) const;
  virtual base::Orientation orientation(const PoseParticle& state) const;
  virtual bool belongsToWorld(const PoseParticle& x, const NodeMap& map);

  virtual double getWeight(const PoseParticle& state) const;
  virtual void setWeight(PoseParticle& state, double value);
  
  virtual base::samples::RigidBodyState& estimate();

  void setCurrentOrientation(const base::samples::RigidBodyState& orientation);
  void setCurrentSpeed(const base::samples::RigidBodyState& speed);

private:
  FilterConfig filter_config;
  base::samples::RigidBodyState vehicle_pose;
  machine_learning::MultiNormalRandom<3> StaticSpeedNoise;
  double z_sample;
};


}

#endif
