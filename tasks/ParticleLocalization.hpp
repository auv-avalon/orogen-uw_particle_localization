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

namespace uw_localization {

struct PoseParticle {
    base::Position position;
    base::Vector3d velocity;
    base::Time timestamp;
    double confidence;
};


}

#endif
