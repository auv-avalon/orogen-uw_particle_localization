require 'vizkit'
include Orocos

Orocos.initialize

#log = Orocos::Log::Replay.open("AvalonSimulation.0.log")
log = Orocos::Log::Replay.open("../../parameter_identification/scripts/AvalonSimulation.1.log")

#Orocos.run "AvalonSimulation" ,:wait => 10000, :valgrind => false, :valgrind_options => ['--undef-value-errors=no'] do 
Orocos.run "uw_particle_localization_test", "sonar_feature_estimator::Task"=> "sonar_feature_estimator",:wait => 10000, :valgrind => false, :valgrind_options => ['--undef-value-errors=no'] do 
  Orocos.log_all 
  
  simulation = log.task 'avalon_simulation'


    pos = TaskContext.get 'uw_particle_localization'
    feature = Orocos::TaskContext.get 'sonar_feature_estimator'

    feature.derivative_history_length = 1
    feature.plain_threshold = 0.5
    feature.signal_threshold = 0.7
    feature.enable_debug_output = true


    pos.debug = false
    pos.init_position = [20.0, 10.0, 0.0]
    pos.init_variance = [1.0, 1.0, 0.0]

    pos.static_motion_covariance = [0.00001,0.0,0.0, 0.0,0.00001,0.0, 0.0,0.0,0.0]
    pos.pure_random_motion = false
    pos.particle_number = 1
    pos.minimum_depth = 0.0
    pos.minimum_perceptions = 3
    pos.effective_sample_size_threshold = 0.9
    pos.hough_interspersal_ratio = 0.1
    pos.sonar_maximum_distance = 12.0
    pos.sonar_covariance = 4.0
    pos.yaml_map = File.join("/home/fabio/avalon/supervision/maps/studiobad.yml")
    pos.advanced_motion_model = false
    pos.param_TCM = [0.0,0.0,1.0,1.0,0.0,0.0,
                     0.0,0.0,0.0,0.0,1.0,1.0,
                     1.0,1.0,0.0,0.0,0.0,0.0,
                     0.0,0.0,0.0,0.0,0.0,0.0,
                     0.0,0.0,0.0,0.0,0.0,0.0,
                     0.0,0.0,0.0,0.0,0.0,0.0]
    pos.param_thrusterCoefficient = [0.005, -0.005,-0.005,-0.005,0.005,-0.005,
				      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
    pos.param_thrusterVoltage = 22.225
    #pos.param_dampingX = [8.203187564, 0.04959]
    #pos.param_dampingY = [24.94216, 0.042393]
    #pos.param_dampingZ = [0.0, 0.0]
    pos.param_floating = true 
    
    #pos.perception_ratio = 1.0
    #pos.noise_ratio = 0.0
    #pos.max_distance_ratio = 0.0

    actuators = log.task 'avalon_actuators'
    #asv_actuators = log.task 'asv_actuators'


    sonar = log.task 'sonar'    

    sonar_rear = log.task 'sonar_rear'

    state_estimator = log.task 'state_estimator'
    
    #sonar.sonar_beam.connect_to feature.sonar_input
    state_estimator.pose_samples.connect_to pos.orientation_samples
    #state_estimator.pose_samples.connect_to pos.speed_samples
    #feature.new_feature.connect_to pos.laser_samples
    actuators.status.connect_to pos.thruster_samples
    #feature.configure
    #feature.start
    pos.configure
    pos.start

    #Vizkit.display feature
    Vizkit.display pos
    Vizkit.control log
    #Vizkit.display actuators
    #Vizkit.display simulation
    Vizkit.exec


end
