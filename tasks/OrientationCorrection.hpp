/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef UW_PARTICLE_LOCALIZATION_ORIENTATIONCORRECTION_TASK_HPP
#define UW_PARTICLE_LOCALIZATION_ORIENTATIONCORRECTION_TASK_HPP

#include "uw_particle_localization/OrientationCorrectionBase.hpp"
#include <boost/circular_buffer.hpp>
#include <math.h>

namespace uw_particle_localization {

    /*! \class OrientationCorrection 
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     * 
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','uw_particle_localization::OrientationCorrection')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument. 
     */
    class OrientationCorrection : public OrientationCorrectionBase
    {
	friend class OrientationCorrectionBase;
    protected:



    public:
        /** TaskContext constructor for OrientationCorrection
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        OrientationCorrection(std::string const& name = "uw_particle_localization::OrientationCorrection", TaskCore::TaskState initial_state = Stopped);

        /** TaskContext constructor for OrientationCorrection 
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices. 
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task. 
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        OrientationCorrection(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state = Stopped);

        /** Default deconstructor of OrientationCorrection
         */
	~OrientationCorrection();

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
        
        void stopHook();
	
	virtual bool reset(double angle);
	
    private:
	
          base::samples::RigidBodyState lastOrientation;
          base::samples::RigidBodyState lastIMU;
          boost::circular_buffer<double> *offset_buffer;
          
          Eigen::AngleAxis<double> actOffset;
          double actOffsetVal;
          Eigen::AngleAxis<double> actNorthOffset;
          
          double calcMedian(boost::circular_buffer<double> *buffer);
          void middleOffsets(boost::circular_buffer<double> *buffer);
          
          int offset_recieved;
          base::Time lastResetRecieved;	
      
    };
}

#endif

