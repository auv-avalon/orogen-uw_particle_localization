/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef UW_PARTICLE_LOCALIZATION_TASK_TASK_HPP
#define UW_PARTICLE_LOCALIZATION_TASK_TASK_HPP

#include "uw_particle_localization/TaskBase.hpp"
#include <uw_localization/filters/particle_filter.hpp>
//#include <battery_management/BatteryManagementTypes.hpp>
#include <vector>
#include <list>
#include "LocalizationConfig.hpp"
#include <uw_localization/maps/grid_map.hpp>
#include <uw_localization/maps/depth_obstacle_grid.hpp>
#include <uw_localization/types/map.hpp>

namespace aggregator {
    class StreamAligner;
}

namespace uw_localization {
    class ParticleLocalization;
    class NodeMap;
    class DepthObstacleGrid;
}

namespace uw_particle_localization {

    class Task : public TaskBase,
        public uw_localization::DebugWriter<uw_localization::PointInfo>
    {
	friend class TaskBase;
    protected:
          base::Time last_perception;
          base::Time last_motion;
          base::Time start_time;
          base::Time last_hough;
          base::Time last_hough_timeout;
          base::Time last_speed_time;
          base::Time last_echosounder;
          base::samples::RigidBodyState lastRBS;
          base::samples::RigidBodyState lastOrientation;
          base::Time last_map_update;

          double current_depth;
          double current_ground;
          bool orientation_sample_recieved;

          unsigned number_sonar_perceptions;
          unsigned number_rejected_samples;
          int number_gps_perceptions;
          bool position_jump_detected;
          double sum_scan;
          double last_scan_angle;
          bool found_buoy_white;
          bool found_buoy_orange;
           
          /**
           * Changes the state of the task
           * If the new state is equal to the old state, no action is performed
           */
          void changeState(States new_state);
          
          void step(const base::samples::RigidBodyState& sample);

          virtual void laser_samplesCallback(const base::Time& ts, const base::samples::LaserScan& scan);
          virtual void orientation_samplesCallback(const base::Time& ts, const base::samples::RigidBodyState& rbs);
          virtual void speed_samplesCallback(const base::Time& ts, const base::samples::RigidBodyState& rbs);
          virtual void thruster_samplesCallback(const base::Time& ts, const base::samples::Joints& rbs);
          virtual void pose_updateCallback(const base::Time& ts, const base::samples::RigidBodyState& rbs);
          virtual void pipeline_samplesCallback(const base::Time& ts, const controlData::Pipeline& pipeline);
          virtual void gps_pose_samplesCallback(const base::Time& ts, const base::samples::RigidBodyState& rbs);
          virtual void buoy_samplesCallback(const base::Time&, const avalon::feature::Buoy&);
          virtual void echosounder_samplesCallback(const base::Time&, const base::samples::RigidBodyState& rbs);
          virtual void obstacle_samplesCallback(const base::Time&, const sonar_detectors::ObstacleFeatures& features);
          virtual void structure_samplesCallback(const base::Time& ts, const bool& structure);

          uw_localization::ParticleLocalization* localizer;
          uw_localization::NodeMap* map;
          uw_localization::DepthObstacleGrid* grid_map;
          uw_localization::Environment env;
          uw_localization::FilterConfig config;
          
          void write(const uw_localization::PointInfo& sample);
          bool initMotionConfig();
          
          /**
           * Statemachine for perception states
           * @param ts: Timestamp of the perception
           * @return: True, if the perception should be observed
           */
          bool perception_state_machine(const base::Time& ts);
          
          /**
           * Sets all particles to valid
           * If there was a position jump, we only set the particles to valid, when we have at least a half scan
           * @param angle: angle of the last scan
           */
          void validate_particles();
          
          /**
           * Filter out duplicate features. If a feature is found multiple times, the feature with the highest confidence is choosen
           * We assume, that duplicate features are succeed to each other, to reduce computation time
           * @param sample: Features to be filtered
           */
          void filter_sample(sonar_detectors::ObstacleFeatures& sample);
          
          /**
           * Update the config-struct for changed properties
           * Change only the covariances, slam-properties 
           */
          void updateConfig();

    public:
        Task(std::string const& name = "uw_particle_localization::Task");

        Task(std::string const& name, RTT::ExecutionEngine* engine);

        /** Default deconstructor of Task
         */
	~Task();

        // bool configureHook();

        bool startHook();

        void updateHook();

        // void errorHook();

        void stopHook();

        // void cleanupHook();
    };
}

#endif

