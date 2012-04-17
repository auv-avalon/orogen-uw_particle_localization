/* ----------------------------------------------------------------------------
 * Types.hpp
 * written by Christoph Mueller, Mar 2012
 * University of Bremen
 * ----------------------------------------------------------------------------
*/

#ifndef UW_PARTICLE_LOCALIZATION_TYPES_HPP_
#define UW_PARTICLE_LOCALIZATION_TYPES_HPP_

#include <base/eigen.h>
#include <base/time.h>

namespace uw_localization {
namespace debug {

struct SonarPerception {
   /** position of the current particle */
   base::Vector3d particle;

   /** chosen most proper obstacle point from sonar */
   base::Vector3d obstacle;

   /** chosen expected obstacle point from map */
   base::Vector3d expected_obstacle;

   /** Timestamp */
   base::Time timestamp;

   /** laser distance */
   double laser_distance;

   /** confidence for this perception */
   double perception_confidence;
};

}}

#endif
