puts "start"

require 'vizkit'
include Orocos

puts "1"

Orocos.initialize

puts "2"

#log = Orocos::Log::Replay.open("AvalonSimulation.0.log")
#log = Orocos::Log::Replay.open("/media/WINDOWS/LOGS/Micron.2.log", "/media/WINDOWS/LOGS/scripts/avalon_back_base_control.2.log")
                               #"/media/WINDOWS/LOGS/scripts/avalon_back_base_control.3.log")
#log = Orocos::Log::Replay.open("/media/WINDOWS/LOGS/testhalle_4_3_2014/20140304-1617/avalon_back_base_control.0.log",
#                              "/media/WINDOWS/LOGS/testhalle_4_3_2014/20140304-1617/sonar.0.log",
#                              "/media/WINDOWS/LOGS/testhalle_4_3_2014/20140304-1617/xsens.0.log")

#log = Orocos::Log::Replay.open("/media/WINDOWS/LOGS/testhalle_25_3_2014/20140325-1622/avalon_back_base_control.0.log",
#                               "/media/WINDOWS/LOGS/testhalle_25_3_2014/20140325-1622/sonar.0.log",
#                               "/media/WINDOWS/LOGS/testhalle_25_3_2014/20140325-1622/xsens.0.log")

log = Orocos::Log::Replay.open("/media/WINDOWS/LOGS/sonar_testhalle_boden/20140616-1617_sonar_am_boden/avalon_back_base_control.0.log",
                               "/media/WINDOWS/LOGS/sonar_testhalle_boden/20140616-1617_sonar_am_boden/sonar.0.log",
                               "/media/WINDOWS/LOGS/sonar_testhalle_boden/20140616-1617_sonar_am_boden/xsens.0.log")



puts "3"


Orocos.run "sonar_wall_hough::Task" => "sonar_wall_hough", "uw_particle_localization::OrientationCorrection" => "orientation_correction",
    "uw_particle_localization::Task" => "uw_particle_localization", "sonar_feature_detector::Task" => "sonar_feature_detector",
    "sonar_feature_estimator::Task"=> "sonar_feature_estimator", #"uw_particle_localization::MotionModel" => "motion_model",
#    :wait => 10000, :valgrind => ["orientation_correction_test", "orientation_correction"]   , :valgrind_options => ['--undef-value-errors=no'] do
    :wait => 10000, :valgrind => false   , :valgrind_options => ['--undef-value-errors=no'] do
  #Orocos.log_all 
  puts "Begin"
  
  
    sonar = log.task 'sonar'
    ori = log.task "depth_orientation_fusion"
    #actuators = log.task "motion_control"
    imu = log.task "xsens"

    pos = TaskContext.get 'uw_particle_localization'
    oriCor = TaskContext.get 'orientation_correction'
    feature = Orocos::TaskContext.get 'sonar_feature_estimator'
    sonar_detector = TaskContext.get 'sonar_feature_detector'
    #motion = Orocos::TaskContext.get 'motion_model'    
   
    feature.derivative_history_length = 1
    feature.plain_threshold = 0.9
    feature.signal_threshold = 0.7
    feature.enable_debug_output = true
    feature.feature_threshold = 0.5

#=begin
    pos.debug = true
    pos.init_position = [0.0, 0.0, 0.0]
    pos.init_variance = [24.0, 20.0, 1.0]

    pos.static_motion_covariance = [0.2,0.0,0.0, 0.0,0.2,0.0, 0.0,0.0,0.0]
    pos.pure_random_motion = false
    pos.particle_number = 500
    pos.minimum_perceptions = 3
    pos.effective_sample_size_threshold = 0.8
    pos.hough_interspersal_ratio = 0.05
    pos.sonar_maximum_distance = 25.0

    pos.sonar_covariance = 1.0

    pos.advanced_motion_model = false
    pos.filter_zeros = true
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
       

    pos.apply_conf_file("#{ENV['AUTOPROJ_CURRENT_ROOT']}/bundles/avalon/config/orogen/uw_particle_localization::Task.yml", ["default", "slam_testhalle"])            
    pos.yaml_map = File.join("#{ENV['AUTOPROJ_CURRENT_ROOT']}/auv_avalon/orogen/uw_particle_localization/maps/testhalle.yml")
    #pos.apply_conf_file("/home/fabio/avalon/bundles/avalon/config/orogen/uw_particle_localization::Task.yml", ["default"])
    pos.orientation_offset = Math::PI / 2.0 # 1.53 + (Math::PI / 4.0)# Math::PI / 2.0  #0# -2.47 - (Math::PI / 2.0) #0.47951
    
    pos.sonar_covariance = 0.5
    pos.hough_timeout = 20.0
    pos.hough_timeout_interspersal = 0.0
    
    pos.aggregator_max_latency = 0.5
    pos.minimum_depth = 50
    #pos.particle_number = 1
    pos.use_best_feature_only = true
    pos.max_velocity_drift = 0.1
=begin
    pos.feature_observation_range = 15
    pos.feature_weight_reduction = 0.9
    pos.use_slam = true
    pos.use_mapping_only = true
    pos.particle_number = 200 #500
    pos.feature_grid_resolution = 1.0
    pos.feature_confidence = 0.85
    pos.feature_empty_cell_confidence = 0.6
    pos.feature_observation_range = 15
    pos.feature_observation_minimum_range = 2    
    pos.feature_confidence_threshold = 0.3
    pos.feature_output_confidence_threshold = 0.5
    pos.feature_filter_threshold = 0.3
    pos.feature_observation_count_threshold = 7    
    pos.sonar_vertical_angle = 0.52
    pos.position_covariance_threshold = 1.0
=end  
  
    #motion.apply_conf_file("#{ENV['AUTOPROJ_CURRENT_ROOT']}/bundles/avalon/config/orogen/uw_particle_localization::MotionModel.yml", ["default"])
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
  
  sonar_detector.map_origin = [[11.5, 10.0]]
  sonar_detector.map_span = [[23, 20]]
  sonar_detector.minimum_wall_distance = 2.5
  sonar_detector.optimal_object_size = 2.0
  sonar_detector.minimum_object_cells = 2
    
  sonar.sonar_beam.connect_to hough.sonar_samples, :type => :buffer, :size => 100
  ori.pose_samples.connect_to hough.orientation_samples
  pos.dead_reckoning_samples.connect_to hough.pose_samples
  pos.grid_map.connect_to sonar_detector.grid_maps
  pos.pose_samples.connect_to sonar_detector.pose_samples
    
    sonar.sonar_beam.connect_to feature.sonar_input
    
    #ori.pose_samples.connect_to pos.echosounder_samples
    #ori.pose_samples.connect_to feature.orientation_sample
    #feature.new_feature.connect_to pos.laser_samples  ## Old single features
    feature.features_out.connect_to pos.obstacle_samples ##New multiple features
    #actuators.joint_commands.connect_to pos.thruster_samples
    #actuators.joint_commands.connect_to motion.thruster_samples
    hough.position.connect_to pos.pose_update
    
    ori.pose_samples.connect_to pos.orientation_samples do |sample|
      
      joint = pos.thruster_samples.writer.new_sample
      joint.time = sample.time
      #print joint.time
      #print joint.methods
      #joint.elements.resize(6)
      pos.thruster_samples.writer.write(joint)
      sample
    end
    
    #ori.pose_samples.connect_to motion.orientation_samples
    
    x_pos = []
    y_pos = []
    count = 0
    last_x = 0
    last_y = 0
    max_step_x = 0
    max_step_y = 0
    sum_steps_x = 0
    sum_steps_y = 0
  
=begin    pos.pose_samples.connect_to do |sample|
      
      count += 1
          
      #Ignore first 1000 stamples
      if count > 500
      
        x_pos.push(sample.position[0])
        y_pos.push(sample.position[1])                
        
        sum_x = 0
        sum_y = 0
        
        x_pos.each {|x| sum_x += x}
        y_pos.each {|y| sum_y += y}
        avg_x = sum_x / x_pos.size()
        avg_y = sum_y / y_pos.size()
        
        sum_diff_x = 0.0
        sum_diff_y = 0.0
        x_pos.each{|x| sum_diff_x += (x - avg_x ) **2 }
        y_pos.each{|y| sum_diff_y += (y - avg_y ) **2 }
        var_x = sum_diff_x / x_pos.size()
        var_y = sum_diff_y / y_pos.size()
        
        x_step = (last_x - sample.position[0]).abs
        y_step = (last_y - sample.position[1]).abs
        sum_steps_x += x_step
        sum_steps_y += y_step
        avg_steps_x = sum_steps_x / x_pos.size()
        avg_steps_y = sum_steps_y / y_pos.size()
        
        if x_step > max_step_x
          max_step_x = x_step
        end
        
        if y_step > max_step_y
          max_step_y = y_step
        end
         
       
        print "---------------------- \n"
        print "X: ", sample.position[0], "\n"
        print "Avg x: " , avg_x, "\n"
        print "Var x: " , var_x, "\n"
        print "Step x: ", x_step, "\n"
        print "Max step x: ", max_step_x, "\n"
        print "Avg step x: ", avg_steps_x, "\n"
        print "Y: ", sample.position[1], "\n"
        print "Avg y: " , avg_y, "\n"
        print "Var y: " , var_y, "\n"
        print "Step y: ", y_step, "\n"
        print "Max step y: ", max_step_y, "\n"
        print "Avg step y: ", avg_steps_y, "\n"
        
      end

      last_x = sample.position[0]
      last_y = sample.position[1]
      
    end
=end    
    
    puts "Conection start"
    
    ori.pose_samples.connect_to oriCor.orientation_input
    hough.position_quality.connect_to oriCor.orientation_offset
    
    puts "Connection end"
    
    feature.configure
    feature.start
    pos.configure
    pos.start
    hough.configure
    hough.start
    sonar_detector.start
    #motion.configure
    #motion.start
    #oriCor.configure
    #oriCor.start
    puts "started"
    
    
    #Vizkit.display feature
    #Vizkit.display pos.particles
    Vizkit.display pos
    Vizkit.display sonar_detector
    #Vizkit.display hough
    #Vizkit.display motion
    #Vizkit.display hough.lines
    #Vizkit.display oriCor
    Vizkit.control log
    

    Vizkit.exec


end
