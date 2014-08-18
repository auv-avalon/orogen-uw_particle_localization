/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef UW_PARTICLE_LOCALIZATION_FASTFUSION_TASK_HPP
#define UW_PARTICLE_LOCALIZATION_FASTFUSION_TASK_HPP

#include "uw_particle_localization/FastFusionBase.hpp"
#include <base/samples/RigidBodyState.hpp>

namespace uw_particle_localization {

    /*! \class FastFusion 
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     * 
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','uw_particle_localization::FastFusion')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument. 
     */
    class FastFusion : public FastFusionBase
    {
	friend class FastFusionBase;
    protected:



    public:
        /** TaskContext constructor for FastFusion
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        FastFusion(std::string const& name = "uw_particle_localization::FastFusion", TaskCore::TaskState initial_state = Stopped);

        /** TaskContext constructor for FastFusion 
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices. 
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task. 
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        FastFusion(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state = Stopped);

        /** Default deconstructor of FastFusion
         */
	~FastFusion();


        bool startHook();


        void updateHook();
        
        
    private:
      
      base::samples::RigidBodyState actualPose;
      base::Time lastVelocity;


    };
}

#endif

