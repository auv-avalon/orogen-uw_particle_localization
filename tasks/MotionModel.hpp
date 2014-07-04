/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef UW_PARTICLE_LOCALIZATION_MOTIONMODEL_TASK_HPP
#define UW_PARTICLE_LOCALIZATION_MOTIONMODEL_TASK_HPP

#include "uw_particle_localization/MotionModelBase.hpp"
#include <uwv_dynamic_model/uwv_dynamic_model.h>
#include <uw_localization/model/uw_motion_model.hpp>
#include <base/Eigen.hpp>

namespace uw_particle_localization {

    /*! \class MotionModel 
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     * import_types_from "battery_management"
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','uw_particle_localization::MotionModel')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument. 
     */
    class MotionModel : public MotionModelBase
    {
	friend class MotionModelBase;
    protected:
      
      bool initMotionConfig();
      
      uw_localization::FilterConfig config;
      
      
    private:
	uw_localization::UwMotionModel motion_model;
	underwaterVehicle::DynamicModel* dynamic_model;
	underwaterVehicle::Parameters dynamic_model_params;
	
	base::Time lastThrusterTime;
	base::samples::RigidBodyState motion_pose;
	base::samples::RigidBodyState last_orientation;
	bool advanced_model;


    public:
        /** TaskContext constructor for MotionModel
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        MotionModel(std::string const& name = "uw_particle_localization::MotionModel", TaskCore::TaskState initial_state = Stopped);

        /** TaskContext constructor for MotionModel 
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices. 
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task. 
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        MotionModel(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state = Stopped);

        /** Default deconstructor of MotionModel
         */
	~MotionModel();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         \verbatim
         task_context "TaskName" do
           needs_configuration
           ...
         end
         \endverbatim
         */
        bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
        bool startHook();

        /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called.
         *
         * The error(), exception() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeError, Exception and
         * FatalError states. 
         *
         * In the first case, updateHook() is still called, and recover() allows
         * you to go back into the Running state.  In the second case, the
         * errorHook() will be called instead of updateHook(). In Exception, the
         * component is stopped and recover() needs to be called before starting
         * it again. Finally, FatalError cannot be recovered.
         */
        void updateHook();
        
        virtual void orientation_samplesCallback(const base::Time& ts, const base::samples::RigidBodyState& rbs);
        virtual void thruster_samplesCallback(const base::Time& ts, const base::samples::Joints& joint);

    };
}

#endif

