require 'vizkit'
include Orocos

Orocos.initialize

#log = Orocos::Log::Replay.open("AvalonSimulation.0.log")
#log = Orocos::Log::Replay.open("/media/WINDOWS/LOGS/Micron.2.log", "/media/WINDOWS/LOGS/scripts/avalon_back_base_control.2.log")
                               #"/media/WINDOWS/LOGS/scripts/avalon_back_base_control.3.log")
#log = Orocos::Log::Replay.open("/media/WINDOWS/LOGS/testhalle_4_3_2014/20140304-1617/avalon_back_base_control.0.log",
#                              "/media/WINDOWS/LOGS/testhalle_4_3_2014/20140304-1617/sonar.0.log",
#                              "/media/WINDOWS/LOGS/testhalle_4_3_2014/20140304-1617/xsens.0.log")

log = Orocos::Log::Replay.open("/media/WINDOWS/LOGS/testhalle_25_3_2014/20140325-1622/avalon_back_base_control.0.log",
                               "/media/WINDOWS/LOGS/testhalle_25_3_2014/20140325-1622/sonar.0.log",
                               "/media/WINDOWS/LOGS/testhalle_25_3_2014/20140325-1622/xsens.0.log")

#Orocos.run "AvalonSimulation" ,:wait => 10000, :valgrind => false, :valgrind_options => ['--undef-value-errors=no'] do 
Orocos.run "uw_particle_localization_test", "orientation_correction_test", "sonar_wall_hough", "sonar_feature_estimator::Task"=> "sonar_feature_estimator",
#    :wait => 10000, :valgrind => ["orientation_correction_test","orientation_correction"]   , :valgrind_options => ['--undef-value-errors=no'] do
    :wait => 10000, :valgrind => false   , :valgrind_options => ['--undef-value-errors=no'] do
  #Orocos.log_all 
  puts "Begin"
  
  
    sonar = log.task 'sonar'
    ori = log.task "depth_orientation_fusion"
    actuators = log.task "motion_control"
    imu = log.task "xsens"

    pos = TaskContext.get 'uw_particle_localization'
    oriCor = TaskContext.get 'orientation_correction'
    feature = Orocos::TaskContext.get 'sonar_feature_estimator'
    
   
    feature.derivative_history_length = 1
    feature.plain_threshold = 0.5
    feature.signal_threshold = 0.7
    feature.enable_debug_output = true

#=begin
    pos.debug = true
    pos.init_position = [0.0, 0.0, 0.0]
    pos.init_variance = [24.0, 20.0, 1.0]

    pos.static_motion_covariance = [0.05,0.0,0.0, 0.0,0.05,0.0, 0.0,0.0,0.0]
    pos.pure_random_motion = false
    pos.particle_number = 500
    pos.minimum_depth = 0.0
    pos.minimum_perceptions = 5
    pos.effective_sample_size_threshold = 0.3
    pos.hough_interspersal_ratio = 0.01
    pos.sonar_maximum_distance = 25.0
    pos.sonar_covariance = 1.5
    pos.yaml_map = File.join("#{ENV['AUTOPROJ_PROJECT_BASE']}/avalon/orogen/uw_particle_localization/maps/testhalle.yml")
    pos.advanced_motion_model = false
    pos.param_TCM = [0.0,0.0,1.0,-1.0,0.0,0.0,
                     0.0,0.0,0.0,0.0,0.1,-1.0,
                     1.0,-1.0,0.0,0.0,0.0,0.0,
                     0.0,0.0,0.0,0.0,0.0,0.0,
                     0.0,0.0,0.0,0.0,0.0,0.0,
                     0.0,0.0,0.0,0.0,0.0,0.0]
    pos.joint_names = ["pitch", "dive", "right" , "left", "yaw", "strave"]
    pos.param_thrusterCoefficient = [0.005, 0.005,0.005,0.005,0.005,0.005,
				      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
    pos.param_thrusterVoltage = 25.4
    pos.param_length = 1.0 #1.4
    pos.param_radius = 0.1 #0.15
    pos.param_mass = 6.5 #65.0
    #pos.param_dampingX = [8.203187564, 0.04959]
    #pos.param_dampingY = [24.94216, 0.042393]
    #pos.param_dampingZ = [0.0, 0.0]
    pos.param_floating = true
    pos.sonar_covariance_reflection_factor = 1.0
    pos.sonar_covariance_corner_factor = 1.0
    pos.init_sample_rejection = 10
    pos.orientation_offset = -2.47 - (Math::PI / 2.0) #0.47951
    pos.sonar_position = [[0.0, 0.0, 0.0]]
    pos.laser_samples_period = 0.01
    
    #pos.apply_conf_file("#{ENV['AUTOPROJ_PROJECT_BASE']}/bundles/avalon/config/orogen/uw_particle_localization::Task.yml", ["maritime_hall"])
    
    
  hough = TaskContext.get 'sonar_wall_hough'
  hough.usePositionSamples = false
  hough.correctToFirstPosition = true
  hough.sensorAngularResolution = 1.8
  hough.sensorAngularTolerance = 18
  hough.filterThreshold = 40
  hough.withMinimumFilter = true
  hough.minLineVotesRatio = 0.05
  hough.show_debug = true
  hough.basinHeight = 19 #20
  hough.basinWidth = 23 #24
  hough.angleDelta = -142 - 90 #27.47
  hough.minDistance = 1.0
  hough.maxDistance = 500
  hough.continous_write = false
  
  #hough.apply_conf_file("#{ENV['AUTOPROJ_PROJECT_BASE']}/bundles/avalon/config/orogen/sonar_wall_hough::Task.yml", ["maritime_hall"])
  
  oriCor.buffer_size = 5
  oriCor.min_buffer_size = 2
  
  
  sonar.sonar_beam.connect_to hough.sonar_samples, :type => :buffer, :size => 100
  ori.pose_samples.connect_to hough.orientation_samples
  pos.dead_reckoning_samples.connect_to hough.pose_samples
  
#=end
    #pos.perception_ratio = 1.0
    #pos.noise_ratio = 0.0
    #pos.max_distance_ratio = 0.0

    #actuators = log.task 'avalon_actuators'
    #asv_actuators = log.task 'asv_actuators'
    #pos.thruster_commands_period = 0#99999999999999999999999
    
=begin
    log.force_torque_controller.motor_command.filter = Proc.new do |motor_command|
      motor_command.time = Time.now
      motor_command
    end
    
    log.Micron.sonar_beam.filter = Proc.new do |sonar_beam|
      sonar_beam.time = Time.now
      sonar_beam
    end
    
    log.depth_orientation_fusion.pose_samples.filter = Proc.new do |pose_samples|
      pose_samples.time = Time.now
      pose_samples
    end
=end
    
    sonar.sonar_beam.connect_to feature.sonar_input
    ori.pose_samples.connect_to pos.orientation_samples
    #ori.pose_samples.connect_to feature.orientation_sample
    feature.new_feature.connect_to pos.laser_samples
    actuators.joint_commands.connect_to pos.thruster_samples
    hough.position.connect_to pos.pose_update
    
    puts "Conection start"
    
    ori.pose_samples.connect_to oriCor.orientation_input
    hough.orientation_drift.connect_to oriCor.orientation_offset
    
    puts "Connection end"
    
    feature.configure
    feature.start
    pos.configure
    pos.start
    hough.configure
    hough.start
    #oriCor.configure
    #oriCor.start
    puts "started"
    
    
    #Vizkit.display feature
    #Vizkit.display pos.particles
    Vizkit.display pos
    #Vizkit.display hough
    #Vizkit.display hough.lines
    #Vizkit.display oriCor
    Vizkit.control log
    
    #Vizkit.display actuators
    #Vizkit.display simulation
    Vizkit.exec


end
