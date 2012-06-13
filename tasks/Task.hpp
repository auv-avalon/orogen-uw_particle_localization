/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef UW_PARTICLE_LOCALIZATION_TASK_TASK_HPP
#define UW_PARTICLE_LOCALIZATION_TASK_TASK_HPP

#include "uw_particle_localization/TaskBase.hpp"
#include <uw_localization/filters/particle_filter.hpp>
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
        aggregator::StreamAligner* aggr;
        base::Time last_perception;
        base::Time start_time;

        int laser_sid;
        int orientation_sid;
        int speed_sid;
        int hough_sid;
        int gt_sid;
        double current_depth;

        std::vector<double> weights;
        std::list<base::samples::RigidBodyState> buffer;

        unsigned number_sonar_perceptions;

        void step(const base::samples::RigidBodyState& sample);

        void callbackLaser(base::Time ts, const base::samples::LaserScan& scan);
        void callbackOrientation(base::Time ts, const base::samples::RigidBodyState& rbs);
        void callbackGroundtruth(base::Time ts, const base::samples::RigidBodyState& rbs);
        void callbackHough(base::Time ts, const base::samples::RigidBodyState& rbs);
        void callbackSpeed(base::Time ts, const base::samples::RigidBodyState& rbs);

        uw_localization::ParticleLocalization* localizer;
        uw_localization::NodeMap* map;

        void write(const uw_localization::PointInfo& sample);


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

