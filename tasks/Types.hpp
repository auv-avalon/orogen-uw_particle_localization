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

struct Stats {
    /** current time */
    base::Time timestamp;

    /** computed degree of uncertainty over best measurement samples */
    double uncertainty_degree;

    /** effective_sample size for resampling timing */
    double effective_sample_size;

    /** particle generation */
    unsigned int particle_generation;
    
    /** Number of obstacle features per particle */
    double obstacle_features_per_particle;
    
    /** Number of depth features per particle */
    double depth_features_per_particle;
    
    //True, if the last used dynamic step was the dvl. false, if the motion model was used
    bool used_dvl;
};


}

#endif
