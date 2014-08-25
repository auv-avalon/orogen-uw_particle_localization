puts "start"

require 'vizkit'
include Orocos

puts "1"

Orocos.initialize

puts "2"



log = Orocos::Log::Replay.open("/media/WINDOWS/LOGS/lokalisierung_falsch/avalon_back_base_control.0.log",
                               "/media/WINDOWS/LOGS/lokalisierung_falsch/sonar.0.log",
                               "/media/WINDOWS/LOGS/lokalisierung_falsch/localization.0.log")



puts "3"


Orocos.run "sonar_wall_hough::Task" => "sonar_wall_hough", "uw_particle_localization::OrientationCorrection" => "orientation_correction",
    "uw_particle_localization::Task" => "uw_particle_localization2", 
    "sonar_feature_estimator::Task"=> "sonar_feature_estimator", "uw_particle_localization::MotionModel" => "motion_model",
#    :wait => 10000, :valgrind => ["orientation_correction_test", "orientation_correction"]   , :valgrind_options => ['--undef-value-errors=no'] do
    :wait => 10000, :valgrind => false   , :valgrind_options => ['--undef-value-errors=no'] do
  #Orocos.log_all 
  puts "Begin"
  
  
    sonar = log.task 'sonar'
    ori = log.task "depth_orientation_fusion"
    actuators = log.task "motion_control"
    loc = log.task "uw_particle_localization"



    pos = TaskContext.get 'uw_particle_localization2'
    oriCor = TaskContext.get 'orientation_correction'
    feature = Orocos::TaskContext.get 'sonar_feature_estimator'
    motion = Orocos::TaskContext.get 'motion_model'    
   
    feature.derivative_history_length = 1
    feature.plain_threshold = 0.9
    feature.signal_threshold = 0.7
    feature.enable_debug_output = true
    feature.feature_threshold = 0.5

#=begin

       

    pos.apply_conf_file("#{ENV['AUTOPROJ_CURRENT_ROOT']}/bundles/avalon/config/orogen/uw_particle_localization::Task.yml", ["default"])    
  
    motion.apply_conf_file("#{ENV['AUTOPROJ_CURRENT_ROOT']}/bundles/avalon/config/orogen/uw_particle_localization::MotionModel.yml", ["default"])
    #motion.apply_conf_file("/home/fabio/avalon/bundles/avalon/config/orogen/uw_particle_localization::MotionModel.yml", ["default"])

    
    
  hough = TaskContext.get 'sonar_wall_hough'
  hough.usePositionSamples = false
  hough.correctToFirstPosition = true
  hough.sensorAngularResolution = 1.8
  hough.sensorAngularTolerance = 45#25 #18
  hough.filterThreshold = 20
  hough.withMinimumFilter = true
  hough.minLineVotesRatio = 0.01#0.05
  hough.show_debug = true
  hough.basinHeight = 20 #20
  hough.basinWidth = 23 #24
  hough.angleDelta = 0#-142 - 70  #-142 - 90 #27.47
  hough.minDistance = 1.0
  hough.maxDistance = 500
  hough.continous_write = false
  hough.anglesPerBin = 1#2
  hough.distancesPerBin = 1
  hough.gain = 5
  

  #hough.apply_conf_file("#{ENV['AUTOPROJ_CURRENT_ROOT']}/bundles/avalon/config/orogen/sonar_wall_hough::Task.yml", ["maritime_hall"])

  hough.ignoreOrientation = true
  hough.sensorAngularTolerance = 10

  
  oriCor.buffer_size = 20
  oriCor.min_buffer_size = 6
    
  sonar.sonar_beam.connect_to hough.sonar_samples, :type => :buffer, :size => 100
  ori.pose_samples.connect_to hough.orientation_samples
  pos.dead_reckoning_samples.connect_to hough.pose_samples
  

    
    sonar.sonar_beam.connect_to feature.sonar_input
    
    #ori.pose_samples.connect_to pos.echosounder_samples
    #ori.pose_samples.connect_to feature.orientation_sample
    #feature.new_feature.connect_to pos.laser_samples  ## Old single features
    feature.features_out.connect_to pos.obstacle_samples ##New multiple features
    actuators.joint_commands.connect_to pos.thruster_samples
    actuators.joint_commands.connect_to motion.thruster_samples
    ori.pose_samples.connect_to pos.orientation_samples
    ori.pose_samples.connect_to motion.orientation_samples
    
    hough.position.connect_to pos.pose_update
    
    
    puts "Conection start"
    
    loc.pose_samples.connect_to oriCor.orientation_input
    
    ori.pose_samples.connect_to oriCor.orientation_input
    hough.position_quality.connect_to oriCor.orientation_offset
    
    puts "Connection end"
    
    feature.configure
    feature.start
    pos.configure
    pos.start
    #hough.configure
    #hough.start
    motion.configure
    motion.start
    #oriCor.configure
    #oriCor.start
    puts "started"
    
    
    #Vizkit.display feature
    #Vizkit.display pos.particles
    Vizkit.display pos
    #Vizkit.display hough
    Vizkit.display motion
    #Vizkit.display hough.lines
    #Vizkit.display oriCor
    Vizkit.control log
    

    Vizkit.exec


end
