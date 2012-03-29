/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef UW_PARTICLE_LOCALIZATION_TASK_TASK_HPP
#define UW_PARTICLE_LOCALIZATION_TASK_TASK_HPP

#include "uw_particle_localization/TaskBase.hpp"

namespace aggregator {
    class StreamAligner;
}

namespace uw_localization {
    class ParticleLocalization;
    class StochasticMap;
}

namespace uw_particle_localization {

    class Task : public TaskBase
    {
	friend class TaskBase;
    protected:
        aggregator::StreamAligner* aggr;

        int laser_sid;
        int orientation_sid;
        int speed_sid;

        void callbackLaser(base::Time ts, const base::samples::LaserScan& scan);
        void callbackOrientation(base::Time ts, const base::samples::RigidBodyState& rbs);
        void callbackSpeed(base::Time ts, const base::samples::RigidBodyState& rbs);

        uw_localization::ParticleLocalization* localizer;
        uw_localization::StochasticMap* map;


    public:
        /** TaskContext constructor for Task
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        Task(std::string const& name = "uw_particle_localization::Task", TaskCore::TaskState initial_state = Stopped);

        /** TaskContext constructor for Task 
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices. 
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task. 
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
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

