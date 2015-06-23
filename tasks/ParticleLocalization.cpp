#include "ParticleLocalization.hpp"
#include <base/pose.h>
#include <list>
//#include <stdexcept>

using namespace machine_learning;

namespace uw_localization {

base::samples::RigidBodyState* PoseParticle::pose = 0;

ParticleLocalization::ParticleLocalization(const FilterConfig& config) 
    : ParticleFilter<PoseParticle>(), filter_config(config),
    StaticSpeedNoise(Random::multi_gaussian(Eigen::Vector3d(0.0, 0.0, 0.0), config.static_motion_covariance)),
    perception_history_sum(0.0),
    sonar_debug(0)
{
    first_perception_received = false;
    PoseParticle::pose = &vehicle_pose;

}

ParticleLocalization::~ParticleLocalization()
{ 

}



void ParticleLocalization::initialize(int numbers, const Eigen::Vector3d& pos, const Eigen::Vector3d& var, double yaw, double yaw_cov)
{  
    
    Eigen::Vector3d var1 = var;
    
    //This fixes a boost-bug in uniform_real, which causes a loop, when min=max
    for(int i=0; i<3; i++){
      if(var1[i] <= 0.0)
	var1[i] = 0.0001;
    }  
  
    UniformRealRandom pos_x = Random::uniform_real(pos.x() - var1.x() * 0.5, pos.x() + var1.x() * 0.5 );
    UniformRealRandom pos_y = Random::uniform_real(pos.y() - var1.y() * 0.5, pos.y() + var1.y() * 0.5 );
    UniformRealRandom pos_z = Random::uniform_real(pos.z() - var1.z() * 0.5, pos.z() + var1.z() * 0.5 );

    bool first_init = true; //Is this the first time, the initialization is called
    PoseParticle best; // Best particle
    
    //We already have particles -> the filter was initialized before
    if(particles.size() > 0){
      first_init = false;
      
      particles.sort(compare_particles<PoseParticle>);
      best = particles.front();
     
    }
    
    particles.clear();
    perception_history.clear();
    perception_history_sum = 0.0;

    for(int i = 0; i < numbers; i++) {
        PoseParticle pp;
        pp.p_position = base::Vector3d(pos_x(), pos_y(), pos_z());
        pp.p_velocity = base::Vector3d(0.0, 0.0, 0.0);
        
        pp.main_confidence = 1.0 / numbers;
        pp.valid = true;
        particles.push_back(pp);
    }

    generation++;

    motion_pose.position = pos;
    motion_pose.velocity << 0.0,0.0,0.0;
    motion_pose.angular_velocity << 0.0,0.0,0.0;
    motion_pose.time = base::Time::now();
    
    full_motion_pose.position = pos;
    full_motion_pose.velocity << 0.0, 0.0, 0.0;
    full_motion_pose.angular_velocity << 0.0, 0.0, 0.0;
    full_motion_pose.time = base::Time::now();
    
    vehicle_pose.position = pos;
    vehicle_pose.velocity << 0.0, 0.0, 0.0;
    vehicle_pose.angular_velocity << 0.0, 0.0 , 0.0;
    vehicle_pose.time = base::Time::now();
    
    best_sonar_measurement.confidence = 0.0;
    lastActuatorTime = base::Time();
    
}


void ParticleLocalization::updateConfig(const FilterConfig& config){
  filter_config = config;
}

void ParticleLocalization::dynamic(PoseParticle& X, const base::samples::RigidBodyState& U, const NodeMap& map)
{
    base::Vector3d v_noisy;
    base::Vector3d u_velocity;

    used_dvl = true;
    
    if(filter_config.pure_random_motion)
        u_velocity = base::Vector3d(0.0, 0.0, 0.0);
    else
        u_velocity = U.velocity;
    
    if( !X.timestamp.isNull() ) {
      double dt = (U.time - X.timestamp).toSeconds();
      v_noisy = u_velocity + (StaticSpeedNoise() * dt);

      base::Vector3d v_avg = (X.p_velocity + v_noisy) / 2.0;
      
      base::Vector3d pos = X.p_position + (v_avg * dt);
      
      if(map.belongsToWorld(pos)){      
        X.p_position = pos;
      }
      else{
        X.valid = false; //Particle left world, something went wrong
      }
      X.p_velocity = v_noisy;
    }

    X.timestamp = U.time;
    X.p_velocity[2] = vehicle_pose.velocity[2];
    X.p_position.z() = vehicle_pose.position.z();
}






const base::Time& ParticleLocalization::getTimestamp(const base::samples::RigidBodyState& U)
{
    return U.time;
}

const base::Time& ParticleLocalization::getTimestamp(const base::samples::Joints& U)
{
    return U.time;
}

base::Time ParticleLocalization::getCurrentTimestamp(){
  
  return vehicle_pose.time;
  
}

double ParticleLocalization::observeAndDebug(const base::samples::LaserScan& z, NodeMap& m, double importance)
{
    zeroConfidenceCount = 0;
    measurement_incomplete = false;
    
    double effective_sample_size;
    
    if(filter_config.use_markov)
      effective_sample_size = observe_markov(z, m, importance);
    else      
      effective_sample_size = observe(z, m, importance);
    
    best_sonar_measurement.time = z.time;

    sonar_debug->write(best_sonar_measurement);

    if(best_sonar_measurement.status == OKAY)
        addHistory(best_sonar_measurement);

    best_sonar_measurement.confidence = -1.0;
    
    if(zeroConfidenceCount > 0 && filter_config.filterZeros)
      filterZeros();

    if(measurement_incomplete)
      return INFINITY; //If we have no complete meassurement, we do not want to resample!!!
    
    return effective_sample_size;
}

double ParticleLocalization::observeAndDebug(const base::samples::RigidBodyState& z, NodeMap& m, double importance){
  
  if(filter_config.use_markov)
    return observe_markov(z, m, importance);
  else
    return observe(z, m, importance);
  
}

double ParticleLocalization::observeAndDebug(const uw_localization::AngleWithTimestamp& z, NodeMap& m, double importance){
  
  if(filter_config.use_markov)
    return observe_markov(z, m, importance);
  else
    return observe(z, m, importance);
  
  
}



double ParticleLocalization::perception(PoseParticle& X, const base::samples::LaserScan& Z, NodeMap& M)
{
    uw_localization::PointInfo info;
    info.time = X.timestamp;

    double angle = Z.start_angle;
    double yaw = base::getYaw(vehicle_pose.orientation);
    double z_distance = Z.ranges[0] / 1000.0;

    // check if this particle is still part of the world
    if(!M.belongsToWorld(X.p_position)) {
        debug(z_distance, X.p_position, 0.0, NOT_IN_WORLD);
        zeroConfidenceCount++;
        return 0.0;
    }

    // check if current laser scan is in a valid range
    if(Z.ranges[0] == base::samples::TOO_FAR 
            || z_distance > filter_config.sonar_maximum_distance 
            || z_distance < filter_config.sonar_minimum_distance)
    {
        double p = 1.0 / (filter_config.sonar_maximum_distance - filter_config.sonar_minimum_distance);
        debug(z_distance, X.p_position, p, OUT_OF_RANGE);
        return p;
    }
   
    // check current measurement with map
    Eigen::AngleAxis<double> sonar_yaw(angle, Eigen::Vector3d::UnitZ()); 
    Eigen::AngleAxis<double> abs_yaw(yaw, Eigen::Vector3d::UnitZ());
    Eigen::Affine3d SonarToAvalon(filter_config.sonarToAvalon);

    Eigen::Vector3d RelativeZ = sonar_yaw * SonarToAvalon * base::Vector3d(z_distance, 0.0, 0.0);
    Eigen::Vector3d AbsZ = (abs_yaw * RelativeZ) + X.p_position;

    boost::tuple<Node*, double, Eigen::Vector3d> distance = M.getNearestDistance("root.wall", AbsZ, X.p_position);
    boost::tuple<Node*, double, Eigen::Vector3d> distance_box = M.getNearestDistance("root.box",
				  Eigen::Vector3d(0.0, filter_config.sonar_vertical_angle/2.0, yaw + angle), X.p_position);


    double dst = distance.get<1>();
    double dst_box = distance_box.get<1>();
          
    double diff_dst = std::fabs( dst - z_distance);
    double diff_dst_box = std::fabs( dst_box - z_distance);
    bool box = false;
    
    if(dst_box != INFINITY){
      //std::cout << "Dst: " << dst << " - dst box: " << dst_box << std::endl;
    }
    
    
    if(diff_dst > diff_dst_box){
      dst = dst_box;
      distance = distance_box;
      box = true;
      
    }

    if(dst == INFINITY){
      measurement_incomplete = true;
      debug(z_distance, X.p_position, X.main_confidence, MAP_INVALID);
      return X.main_confidence;
    }    
    
    double covar = filter_config.sonar_covariance;
    
    if(dst > vehicle_pose.position[2]/sin(filter_config.sonar_vertical_angle/2.0))
      covar = covar * filter_config.sonar_covariance_reflection_factor;    

    if(angleDiffToCorner(angle+yaw, X.p_position, filter_config.env) < 0.1)
      covar = covar * filter_config.sonar_covariance_corner_factor;    

    double probability = gaussian1d(0.0, covar, dst - z_distance);
    
    if(box){
      debug(z_distance, dst ,angle + yaw  ,distance.get<2>(), AbsZ, X.p_position, probability, OBSTACLE);
    }else{    
      debug(z_distance, dst ,angle + yaw  ,distance.get<2>(), AbsZ, X.p_position, probability);
    }
    //std::cout << distance.get<1>() << std::endl;    
    
    first_perception_received = true;

    return probability;
}


double ParticleLocalization::perception(PoseParticle& X, const base::samples::RigidBodyState& Z, NodeMap& M){
  
  base::Vector3d posZ = Z.position;
  posZ.z() = 0.0;
  base::Vector3d ppos2usbl = filter_config.usbl2world.inverse() * X.p_position;
  ppos2usbl.z() = 0.0;

  base::Vector2d x_polar( ppos2usbl.norm(), std::atan2( ppos2usbl.y(), ppos2usbl.x()) );
  base::Vector2d z_polar( posZ.norm(), std::atan2( posZ.y(), posZ.x()) );

  base::Matrix2d cov = base::Matrix2d::Identity();
  cov(0,0) = filter_config.usbl_range_variance;
  cov(1,1) = filter_config.usbl_angle_variance;
  
  return machine_learning::calc_gaussian<2>( z_polar, cov, x_polar);
  
}  


double ParticleLocalization::perception(PoseParticle& X, const uw_localization::AngleWithTimestamp& Z, NodeMap& M){
  
  base::Vector3d ppos2usbl = filter_config.usbl2world.inverse() * X.p_position;

  double x_angle = std::atan2( ppos2usbl.y(), ppos2usbl.x());
  
  return machine_learning::gaussian1d(0.0, filter_config.usbl_angle_variance, x_angle - Z.angle.rad) ;
  
} 

  
void ParticleLocalization::addHistory(const uw_localization::PointInfo& info)
{
    if(perception_history.size() >= filter_config.perception_history_number) {
        perception_history_sum -= perception_history.front();
        perception_history.pop_front();
    }

    perception_history.push_back(info.confidence);
    perception_history_sum += perception_history.back();
}

bool ParticleLocalization::hasStats() const
{
    return (perception_history.size() == filter_config.perception_history_number) && !timestamp.isNull();
}


uw_localization::Stats ParticleLocalization::getStats() const
{
    uw_localization::Stats stats;
    stats.timestamp = timestamp;
    stats.uncertainty_degree = perception_history_sum / filter_config.perception_history_number;
    stats.effective_sample_size = effective_sample_size;
    stats.particle_generation = generation;
    stats.used_dvl = used_dvl;
      
    return stats;
}

void ParticleLocalization::interspersal(const base::samples::RigidBodyState& p, const NodeMap& m, double ratio, bool random_uniform, bool invalidate_particles)
{
    reduceParticles(1.0 - ratio);

    PoseParticle best = particles.front();
    PoseParticle worst = particles.back();
    
    base::Vector3d limit = m.getLimitations();
    MultiNormalRandom<3> Pose = Random::multi_gaussian<3>(p.position, p.cov_position);
    UniformRealRandom pos_x = Random::uniform_real(-(limit.x() / 2.0) , (limit.x() / 2.0) );
    UniformRealRandom pos_y = Random::uniform_real(-(limit.y() / 2.0) , (limit.y() / 2.0) );    
    int count = 0;
    
    for(size_t i = particles.size(); i < filter_config.particle_number; i++) {
        PoseParticle pp;
        
        if(random_uniform){
          pp.p_position[0] = pos_x();
          pp.p_position[1] = pos_y();
        }
        else{        
          pp.p_position = Pose();
        }
        
        pp.p_velocity = best.p_velocity;
        pp.p_position[2] = best.p_position[2];
        
          
        pp.main_confidence = worst.main_confidence / 10000.0; //Choose a realy small value
        
        if(invalidate_particles){
          pp.valid = false;
        }
        else{
          pp.valid = true;
        }
        
        
        
        count++;
        particles.push_back(pp);
    }
    std::cout << "Interspersal. Created " << count << " new particles." << std::endl;
    normalizeParticles();
}


void ParticleLocalization::debug(double distance, const base::Vector3d& location, double conf, PointStatus status)
{
    if(best_sonar_measurement.confidence < conf) {
        uw_localization::PointInfo info;
        info.distance = distance;
        info.desire_distance = 0.0;
        info.desire_point = base::Vector3d(0.0, 0.0, 0.0);
        info.real_point = base::Vector3d(0.0, 0.0, 0.0);
        info.angle = 0.0;
        info.location = location;
        info.confidence = conf;
        info.status = status;

        best_sonar_measurement = info;
    }
}

void ParticleLocalization::debug(double distance, double desire_distance, double angle, const base::Vector3d& desire, const base::Vector3d& real, const base::Vector3d& loc, double conf, PointStatus status)
{
    if(best_sonar_measurement.confidence < conf) {
        uw_localization::PointInfo info;
        info.distance = distance;
        info.desire_distance = desire_distance;
        info.desire_point = desire;
        info.real_point = real;
        info.angle = angle;
        info.location = loc;
        info.confidence = conf;
        info.status = status;

        best_sonar_measurement = info;
    }
}

void ParticleLocalization::debug(double distance, double desire_distance, double angle, const base::Vector3d& desire, const base::Vector3d& real, const base::Vector3d& loc, double conf)
{
    if(best_sonar_measurement.confidence < conf) {
        uw_localization::PointInfo info;
        info.distance = distance;
	info.desire_distance = desire_distance;
        info.desire_point = desire;
        info.real_point = real;
	info.angle = angle;
        info.location = loc;
        info.confidence = conf;
        info.status = OKAY;

        best_sonar_measurement = info;
    }
}


void ParticleLocalization::setCurrentOrientation(const base::samples::RigidBodyState& orientation)
{
  
    if(timestamp.toSeconds() > orientation.time.toSeconds()){
      vehicle_pose.time = timestamp;
    }else{
      vehicle_pose.time = orientation.time;
    }
    
    if(base::samples::RigidBodyState::isValidValue(orientation.orientation)){
    
      vehicle_pose.orientation = Eigen::AngleAxis<double>(filter_config.yaw_offset, Eigen::Vector3d::UnitZ()) * orientation.orientation;
      vehicle_pose.cov_orientation = orientation.cov_orientation;

      motion_pose.orientation = vehicle_pose.orientation;
      motion_pose.cov_orientation = vehicle_pose.cov_orientation;
    }
    
}

void ParticleLocalization::setCurrentVelocity(const base::samples::RigidBodyState& speed)
{
  
  if(base::samples::RigidBodyState::isValidValue(speed.velocity)){
  
    vehicle_pose.velocity = speed.velocity;
    motion_pose.velocity = speed.velocity;
  }
  
}

void ParticleLocalization::setCurrentAngularVelocity(const base::samples::RigidBodyState& speed){
  
  if(base::samples::RigidBodyState::isValidValue(speed.angular_velocity)){
  
    vehicle_pose.angular_velocity = speed.angular_velocity;
    motion_pose.angular_velocity = speed.angular_velocity;
  }
}

void ParticleLocalization::setCurrentZVelocity(const base::samples::RigidBodyState& speed){
  
  if(!std::isnan(speed.velocity.z() )){
    vehicle_pose.velocity.z() = speed.velocity.z();
    motion_pose.velocity.z() = speed.velocity.z();
  }
  
}

void ParticleLocalization::setCurrentDepth(const base::samples::RigidBodyState& depth){
  
  if(!std::isnan(depth.position.z() )){
    vehicle_pose.position.z() = depth.position.z();
    motion_pose.position.z() = depth.position.z();
  }
  
}



double ParticleLocalization::angleDiffToCorner(double sonar_orientation, base::Vector3d position, Environment* env){
  //Environment env = m.getEnvironment();
  double minAngleDiff = M_PI/2.0;
  
  for (std::vector<Plane>::iterator it = env->planes.begin() ; it != env->planes.end(); it++){
      double angle = atan2(it->position[1] - position[1], it->position[0] - position[0]);
      double angleDiff = fabs(sonar_orientation - angle) < M_PI ? fabs(sonar_orientation - angle) : (2.0*M_PI)-fabs(sonar_orientation - angle); 
      
      if(angleDiff < minAngleDiff)
	minAngleDiff = angleDiff;
      
      angle = atan2((it->position + it->span_horizontal)[1] - position[0],
		    (it->position + it->span_horizontal)[0] - position[0]);
      angleDiff = fabs(sonar_orientation - angle) < M_PI ? fabs(sonar_orientation - angle) : (2.0*M_PI)-fabs(sonar_orientation - angle);
      
      if(angleDiff < minAngleDiff)
	minAngleDiff = angleDiff;    
  }  
  
  return minAngleDiff;
}


void ParticleLocalization::filterZeros(){

    base::Vector3d var = filter_config.init_variance;
    base::Vector3d pos = filter_config.init_position;
  
    for(int i=0; i<3; i++){
      if(var[i] <= 0.0)
        var[i] = 0.0001;
    }  
  
    UniformRealRandom pos_x = Random::uniform_real(pos.x() - var.x() * 0.5, pos.x() + var.x() * 0.5 );
    UniformRealRandom pos_y = Random::uniform_real(pos.y() - var.y() * 0.5, pos.y() + var.y() * 0.5 );
    UniformRealRandom pos_z = Random::uniform_real(pos.z() - var.z() * 0.5, pos.z() + var.z() * 0.5 );  
  
    int count = 0;
    std::list<PoseParticle>::iterator it;
    for(it = particles.begin(); it != particles.end(); ++it) {
        
      //if particle is outside the map, calculate new random position
        if(it->main_confidence == 0.0 || std::isnan(it->main_confidence)){
          it->p_position = base::Vector3d(pos_x(), pos_y(), pos_z());
          it->p_velocity = base::Vector3d(0.0, 0.0, 0.0);
          it->main_confidence = 1.0 / particles.size();
          count++;
        }      
    }
  
}




}
