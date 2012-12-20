require 'vizkit'
include Orocos

Orocos.initialize

widget = Vizkit.load "simulator.ui"

log = Orocos::Log::Replay.open("AvalonSimulation.1.log")

Orocos.run "uw_particle_localization_test" ,:wait => 10000, :valgrind => false, :valgrind_options => ['--undef-value-errors=no'] do 
    
    actuators = log.task 'asv_actuators'
    sim = log.task 'avalon_simulation'
    
    localication = TaskContext.get 'uw_particle_localization'
    localication.debug = true
    localication.init_position = [20.0, 10.0, 0.0]
    localication.init_variance = [1.0, 1.0, 0.0]

    localication.static_motion_covariance = [0.00001,0.0,0.0, 0.0,0.00001,0.0, 0.0,0.0,0.0]
    localication.pure_random_motion = true
    localication.particle_number = 100
    localication.minimum_depth = 0.0
    localication.minimum_perceptions = 3
    localication.effective_sample_size_threshold = 0.9
    localication.hough_interspersal_ratio = 0.1
    localication.sonar_maximum_distance = 12.0
    localication.sonar_covariance = 4.0
    localication.yaml_map = File.join("nurc.yml")
    localication.gps_covarianz = 2.0
    
    
    #actuators.status.connect_to localication.thruster_samples
    actuators.pose_samples.connect_to localication.gps_pose_samples
    actuators.pose_samples.connect_to localication.orientation_samples
    actuators.pose_samples.connect_to localication.speed_samples
    #actuators.pose_samples.connect_to localication.pose_update
    localication.configure
    localication.start

    Vizkit.display sim
    Vizkit.display actuators
    Vizkit.display localication
    Vizkit.control log
    #widget.show 
    Vizkit.exec


end

