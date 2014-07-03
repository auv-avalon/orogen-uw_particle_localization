puts "start"

require 'vizkit'
include Orocos

puts "1"

Orocos.initialize

puts "2"

=begin
log_center = Orocos::Log::Replay.open("/media/WINDOWS/LOGS/SAUC-E12/20120706-2003_Day1_sonar_angle_5_0_gain_0_9_centered/sonar.0.log",
                               "/media/WINDOWS/LOGS/SAUC-E12/20120706-2003_Day1_sonar_angle_5_0_gain_0_9_centered/avalon_back_base_control.0.log",
                                "/media/WINDOWS/LOGS/SAUC-E12/20120706-2003_Day1_sonar_angle_5_0_gain_0_9_centered/sonar_rear.0.log")
=end

=begin
log_bottom = Orocos::Log::Replay.open("/media/WINDOWS/LOGS/SAUC-E12/20120706-1946_Day1_sonar_angle_2_5_gain_0_9_bottom_basin/sonar.0.log",
                               "/media/WINDOWS/LOGS/SAUC-E12/20120706-1946_Day1_sonar_angle_2_5_gain_0_9_bottom_basin/avalon_back_base_control.0.log",
                                "/media/WINDOWS/LOGS/SAUC-E12/20120706-1946_Day1_sonar_angle_2_5_gain_0_9_bottom_basin/sonar_rear.0.log")
=end                               
=begin
log_top = Orocos::Log::Replay.open("/media/WINDOWS/LOGS/SAUC-E12/20120706-1958_Day1_sonar_angle_2_5_gain_0_9_top_basin/sonar.0.log",
                               "/media/WINDOWS/LOGS/SAUC-E12/20120706-1958_Day1_sonar_angle_2_5_gain_0_9_top_basin/avalon_back_base_control.0.log",
                                "/media/WINDOWS/LOGS/SAUC-E12/20120706-1958_Day1_sonar_angle_2_5_gain_0_9_top_basin/sonar_rear.0.log")

=end

#=begin
log_traj = Orocos::Log::Replay.open("/media/WINDOWS/LOGS/SAUC-E12/20120707-1600_Day2_Manual_trajectory/sonar.0.log",
                                    "/media/WINDOWS/LOGS/SAUC-E12/20120707-1600_Day2_Manual_trajectory/avalon_back_base_control.0.log",
                                    "/media/WINDOWS/LOGS/SAUC-E12/20120707-1600_Day2_Manual_trajectory/sonar_rear.0.log")
#=end

log = log_traj

puts "3"

#Orocos.run "AvalonSimulation" ,:wait => 10000, :valgrind => false, :valgrind_options => ['--undef-value-errors=no'] do 
Orocos.run "uw_particle_localization_test","orientation_correction_test", "sonar_wall_hough",
    "sonar_feature_estimator::Task"=> "sonar_feature_estimator", "joint_converter::Task"  => "joint_converter",
#    :wait => 10000, :valgrind => ["orientation_correction_test","orientation_correction"]   , :valgrind_options => ['--undef-value-errors=no'] do
    :wait => 10000, :valgrind => false   , :valgrind_options => ['--undef-value-errors=no'] do
#     :wait => 10000, :gdb => ["uw_particle_localization_test", "uw_particle_localization"] do
  #Orocos.log_all 
  puts "Begin"
  
  
    sonar = log.task 'sonar'
    ori = log.task "depth_orientation_fusion"
    actuators = log.task "hbridge"
    echo = log.task "sonar_rear"
    #imu = log.task "xsens"

    pos = TaskContext.get 'uw_particle_localization'
    oriCor = TaskContext.get 'orientation_correction'
    feature = Orocos::TaskContext.get 'sonar_feature_estimator'
    joint_converter = Orocos::TaskContext.get 'joint_converter'
    
   
    feature.derivative_history_length = 3
    feature.plain_threshold = 0.9
    feature.signal_threshold = 1.5
    feature.enable_debug_output = true
    feature.proportional_value_threshold = 0.7
    feature.feature_threshold = 0.7
    
    actuators.status_motors.connect_to joint_converter.motor_status

    pos.debug = true
    pos.init_position = [0.0, 0.0, 0.0]
    pos.init_variance = [120.0, 50.0, 1.0]

    pos.static_motion_covariance = [0.2,0.0,0.0, 0.0,0.2,0.0, 0.0,0.0,0.0]
    pos.pure_random_motion = false
    pos.particle_number = 500
    pos.minimum_depth = 0.0
    pos.minimum_perceptions = 3
    pos.effective_sample_size_threshold = 0.8
    pos.hough_interspersal_ratio = 0.05
    pos.sonar_maximum_distance = 100.0
    pos.sonar_covariance = 1.0
    pos.yaml_map = File.join("#{ENV['AUTOPROJ_PROJECT_BASE']}/auv_avalon/orogen/uw_particle_localization/maps/nurc.yml")
    pos.advanced_motion_model = false
    pos.filter_zeros = true
    pos.param_TCM = [0.0,0.0,-1.0,-1.0,0.0,0.0,
                     0.0,0.0,0.0,0.0,0.1,-1.0,
                     1.0,-1.0,0.0,0.0,0.0,0.0,
                     0.0,0.0,0.0,0.0,0.0,0.0,
                     0.0,0.0,0.0,0.0,0.0,0.0,
                     0.0,0.0,0.0,0.0,0.0,0.0]
    pos.joint_names = ["pitch", "dive", "right" , "left", "yaw", "strave"]
    pos.param_thrusterCoefficient = [0.005, 0.005,0.005,0.005,0.005,0.005,
                                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
    pos.param_thrusterVoltage = 35
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
    pos.orientation_offset = 0#-2.47 - (Math::PI / 2.0) #0.47951
    pos.sonar_position = [[0.6, 0.0, 0.0]]
    pos.laser_samples_period = 0.01
    pos.echosounder_samples_period = 0.01
    
    #pos.apply_conf_file("#{ENV['AUTOPROJ_PROJECT_BASE']}/bundles/avalon/config/orogen/uw_particle_localization::Task.yml", ["default"])
    pos.yaml_map = File.join("#{ENV['AUTOPROJ_PROJECT_BASE']}/auv_avalon/orogen/uw_particle_localization/maps/nurc.yml")
    pos.orientation_offset = 0#-2.47 - (Math::PI / 2.0) #0.47951
    
    pos.hough_timeout = 20.0
    pos.hough_timeout_interspersal = 0.0
    
    pos.aggregator_max_latency = 0.5
    #pos.particle_number = 1
    
    
  hough = TaskContext.get 'sonar_wall_hough'
=begin  
  hough.usePositionSamples = false
  hough.correctToFirstPosition = true
  hough.sensorAngularResolution = 1.8
  hough.sensorAngularTolerance = 45#25 #18
  hough.filterThreshold = 20
  hough.withMinimumFilter = true
  hough.minLineVotesRatio = 0.01#0.05
  hough.minDistance = 1.0
  hough.maxDistance = 500
  hough.continous_write = false
  hough.anglesPerBin = 1#2
  hough.distancesPerBin = 1
=end
  hough.filterThreshold = 70
  hough.show_debug = true
  hough.basinHeight = 50 #20
  hough.basinWidth = 120 #24
  hough.angleDelta = 0#-142 - 70  #-142 - 90 #27.47  
  hough.withMinimumFilter = false
  hough.ignoreOrientation = true
  hough.sensorAngularTolerance = 10
  hough.continous_write = false
  
  #hough.apply_conf_file("#{ENV['AUTOPROJ_PROJECT_BASE']}/bundles/avalon/config/orogen/sonar_wall_hough::Task.yml", ["maritime_hall"])
  
  oriCor.buffer_size = 20
  oriCor.min_buffer_size = 6
    
  sonar.sonar_beam.connect_to hough.sonar_samples, :type => :buffer, :size => 100
  ori.pose_samples.connect_to hough.orientation_samples
  pos.dead_reckoning_samples.connect_to hough.pose_samples
  

  
    sonar.sonar_beam.connect_to feature.sonar_input
    
    ori.pose_samples.connect_to pos.orientation_samples do |sample|
      sample.velocity[2] = 0.0
      sample
    end

    #ori.pose_samples.connect_to feature.orientation_sample
    feature.new_feature.connect_to pos.laser_samples
    joint_converter.joints.connect_to pos.thruster_samples
    hough.position.connect_to pos.pose_update
    echo.ground_distance.connect_to pos.echosounder_samples
    
    puts "Conection start"
    
    ori.pose_samples.connect_to oriCor.orientation_input
    hough.position_quality.connect_to oriCor.orientation_offset
    
    puts "Connection end"
    
    feature.configure
    feature.start
    joint_converter.configure
    joint_converter.start    
    pos.configure
    pos.start
    hough.configure
    hough.start
    #oriCor.configure
    #oriCor.start
    puts "started"
    
    
    #Vizkit.display feature
    Vizkit.display pos
    #Vizkit.display joint_converter
    #Vizkit.display feature
    Vizkit.display hough
    #Vizkit.display hough.lines
    #Vizkit.display oriCor
    Vizkit.control log
    
    #Vizkit.display actuators
    #Vizkit.display simulation
    Vizkit.exec


end
