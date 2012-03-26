#include "ParticleLocalization.hpp"
#include <machine_learning/RandomNumbers.hpp>

using namespace machine_learning;

namespace uw_localization {

double ParticleLocalization::dynamic(PoseParticle& X, const base::samples::RigidBodyState& U)
{
    MultiNormalRandom<3> Noise = Random::multi_gaussian(U.velocity, U.cov_velocity);
    PoseParticle Xt;
    
    base::Vector3d v_noisy = Noise();
    base::Vector3d v_avg = (X.velocity + v_noisy) / 2.0;

    if( !X.timestamp.isNull() ) {
        double dt = (U.time - X.timestamp).toSeconds();

        Xt.position = X.position + vehicle_orientation * (v_avg * dt);
    }

    Xt.velocity = v_noisy;
    Xt.timestamp = U.time;
    Xt.confidence = X.confidence;
 
    return 0;
}

double ParticleLocalization::perception(PoseParticle& state, const base::samples::LaserScan& scan, const StochasticMap& map)
{
    return 0;
}
  
base::Vector3d ParticleLocalization::position(const PoseParticle& state) const
{
    return state.position;
}


double ParticleLocalization::yaw(const PoseParticle& state) const
{
    return 0;
}


double ParticleLocalization::get_weight(const PoseParticle& state) const
{
    return state.confidence;
}



void ParticleLocalization::set_weight(PoseParticle& state, double value)
{
    state.confidence = value;
}


  
base::samples::RigidBodyState ParticleLocalization::estimate() const
{
    base::samples::RigidBodyState state;

    return state;
}

}
