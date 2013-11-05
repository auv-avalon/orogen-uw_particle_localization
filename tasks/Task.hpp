/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef UW_PARTICLE_LOCALIZATION_TASK_TASK_HPP
#define UW_PARTICLE_LOCALIZATION_TASK_TASK_HPP

#include "uw_particle_localization/TaskBase.hpp"
#include <uw_localization/filters/particle_filter.hpp>
#include <battery_management/BatteryManagementTypes.hpp>
#include <vector>
#include <list>

namespace aggregator {
    class StreamAligner;
}

namespace uw_localization {
    class ParticleLocalization;
    class NodeMap;
}

namespace uw_particle_localization {

    class Task : public TaskBase,
        public uw_localization::DebugWriter<uw_localization::PointInfo>
    {
	friend class TaskBase;
    protected:
        base::Time last_perception;
        base::Time start_time;

        int laser_sid;
        int orientation_sid;
        int speed_sid;
        int thruster_sid;
        int hough_sid;
        int gt_sid;
        double current_depth;
	 bool orientation_sample_recieved;

        std::vector<double> weights;
        std::list<base::samples::RigidBodyState> buffer;

        unsigned number_sonar_perceptions;
        unsigned number_rejected_samples;
	int number_gps_perceptions;

        void step(const base::samples::RigidBodyState& sample);

        virtual void laser_samplesCallback(const base::Time& ts, const base::samples::LaserScan& scan);
        virtual void orientation_samplesCallback(const base::Time& ts, const base::samples::RigidBodyState& rbs);
        virtual void speed_samplesCallback(const base::Time& ts, const base::samples::RigidBodyState& rbs);
        virtual void thruster_samplesCallback(const base::Time& ts, const base::actuators::Status& rbs);
	virtual void thruster_commandsCallback(const base::Time& ts, const base::actuators::Command& rbs);
        virtual void pose_updateCallback(const base::Time& ts, const base::samples::RigidBodyState& rbs);
        virtual void pipeline_samplesCallback(const base::Time& ts, const controlData::Pipeline& pipeline);
	virtual void gps_pose_samplesCallback(const base::Time& ts, const base::samples::RigidBodyState& rbs);

        uw_localization::ParticleLocalization* localizer;
        uw_localization::NodeMap* map;
	uw_localization::Environment env;
	
        void write(const uw_localization::PointInfo& sample);


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

