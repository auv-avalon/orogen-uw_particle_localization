/* ----------------------------------------------------------------------------
 * slam/sonar_particle_filter.h
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
#include <uw_localization/maps/stochastic_map.hpp>
#include "LocalizationConfig.hpp"

namespace uw_localization {

struct PoseParticle {
  base::Position position;
  base::Vector3d velocity;
  base::Time timestamp;
  double confidence;
};


class ParticleLocalization : public ParticleFilter<PoseParticle>,
  public Dynamic<PoseParticle, base::samples::RigidBodyState>,
  public Perception<PoseParticle, base::samples::LaserScan, StochasticMap>
{
public:
  ParticleLocalization(const FilterConfig& config);
  ~ParticleLocalization();

  virtual double dynamic(PoseParticle& state, const base::samples::RigidBodyState& speed);
  virtual double perception(PoseParticle& state, const base::samples::LaserScan& scan, const StochasticMap& map);
  
  virtual base::Vector3d position(const PoseParticle& state) const;
  virtual base::Orientation orientation(const PoseParticle& state) const;
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
