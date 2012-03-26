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
#include <uw_localization/filters/particle_filter.hpp>
#include <uw_localization/maps/stochastic_map.hpp>

namespace uw_localization {

struct PoseParticle {
  base::Position position;
  base::Vector3d velocity;
  base::Time timestamp;
  double confidence;
};


class ParticleLocalization : public ParticleFilter<PoseParticle, StochasticMap>,
  public Dynamic<PoseParticle, base::samples::RigidBodyState>,
  public Perception<PoseParticle, base::samples::LaserScan, StochasticMap>
{
public:
  virtual double dynamic(PoseParticle& state, const base::samples::RigidBodyState& speed);
  virtual double perception(PoseParticle& state, const base::samples::LaserScan& scan, const StochasticMap& map);
  
  virtual base::Vector3d position(const PoseParticle& state) const;
  virtual double yaw(const PoseParticle& state) const;
  virtual double get_weight(const PoseParticle& state) const;
  virtual void set_weight(PoseParticle& state, double value);
  
  virtual base::samples::RigidBodyState estimate() const;

private:
  base::Orientation vehicle_orientation;
};


}

#endif
