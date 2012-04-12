/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef UW_PARTICLE_LOCALIZATION_TASK_TASK_HPP
#define UW_PARTICLE_LOCALIZATION_TASK_TASK_HPP

#include "uw_particle_localization/TaskBase.hpp"
#include <uw_localization/filters/particle_filter.hpp>

namespace aggregator {
    class StreamAligner;
}

namespace uw_localization {
    class ParticleLocalization;
    class NodeMap;
}

namespace uw_particle_localization {

    class Task : public TaskBase,
        public uw_localization::DebugWriter<uw_localization::debug::SonarPerception>
    {
	friend class TaskBase;
    protected:
        aggregator::StreamAligner* aggr;

        int laser_sid;
        int orientation_sid;
        int speed_sid;
        double current_depth;

        unsigned number_sonar_perceptions;

        void callbackLaser(base::Time ts, const base::samples::LaserScan& scan);
        void callbackOrientation(base::Time ts, const base::samples::RigidBodyState& rbs);
        void callbackSpeed(base::Time ts, const base::samples::RigidBodyState& rbs);

        uw_localization::ParticleLocalization* localizer;
        uw_localization::NodeMap* map;

        void write(const uw_localization::debug::SonarPerception& sample);


    public:
        Task(std::string const& name = "uw_particle_localization::Task", TaskCore::TaskState initial_state = Stopped);

        Task(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state = Stopped);

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

