require 'vizkit'
include Orocos

Orocos.initialize

widget = Vizkit.load "simulator.ui"

require 'vizkit'
include Orocos

Orocos.initialize

widget = Vizkit.load "simulator.ui"

#Orocos.run "AvalonSimulation" ,:wait => 10000, :valgrind => false, :valgrind_options => ['--undef-value-errors=no'] do 
Orocos.run "AvalonSimulation", "uw_particle_localization_test", "sonar_wall_hough", "orientation_correction_test" ,"sonar_feature_estimator::Task"=> "sonar_feature_estimator",:wait => 100, :valgrind => false, :valgrind_options => ['--undef-value-errors=no'] do 
    simulation = TaskContext.get 'avalon_simulation'
    
      white_light = TaskContext.get 'white_light'
#     white_light.interval_mode = 1
#     white_light.constantInterval = 3000
      white_light.interval_mode = 2
      white_light.randomInterval_min = 1;
      white_light.randomInterval_max = 5000;
      #white_light.start
      
#simulation.scenefile = "#{ENV['AUTOPROJ_CURRENT_ROOT']}/simulation/orogen/avalon_simulation/configuration/testhalle.scn"
    #simulation.initial_scene = "#{ENV['AUTOPROJ_CURRENT_ROOT']}/simulation/orogen/avalon_simulation/configuration/testhalle.scn"
    simulation.apply_conf_file("/home/fabio/avalon/bundles/avalon/config/orogen/simulation::Mars.yml")
    simulation.configure
    simulation.start

# Konfiguration der Aktuatoren für alle Fahrzeuge:    
require 'actuators'
values = ActuatorsConfig.new()
    
    actuators = TaskContext.get 'avalon_actuators'
    actuators.node_name = "avalon"
    actuators.amount_of_actuators = values.avalon_amount_of_actuators
    actuators.maximum_thruster_force = values.avalon_maximum_thruster_force    
    actuators.thruster_position = values.avalon_thruster_position    
    actuators.thruster_direction = values.avalon_thruster_direction    
    actuators.linear_damp = [[8, 24, 24, 10, 10, 10]]
    actuators.square_damp = [[0, 0, 0, 0, 0, 0]]
    actuators.thruster_coefficients = [0.005, 0.005, 0.005, 0.005, 0.005, 0.005]
    actuators.buoyancy_force = 69.9
    actuators.cob = [[0, 0, 0.08]]
    actuators.voltage = 28
    actuators.configure
    actuators.start
    

# Camera configuration

    front_cam = TaskContext.get 'front_camera'
    front_cam.name = 'front_cam'
    #front_cam.configure
    #front_cam.start
    
    bottom_cam = TaskContext.get 'bottom_camera'
    bottom_cam.name = 'bottom_cam'
    #bottom_cam.configure
    #bottom_cam.start
    
    top_cam = TaskContext.get 'top_camera'
    top_cam.name = 'top_cam'
    #top_cam.configure
    #top_cam.start


    sonar = TaskContext.get 'sonar'
    sonar.node_name = "sonar_top_sensor"
    sonar.left_limit = Math::PI
    sonar.right_limit = -Math::PI
    sonar.resolution = 0.1
    sonar.maximum_distance = 100.0
    sonar.ping_pong_mode = false
    sonar.configure
    sonar.start
    
    sonar_rear = TaskContext.get 'sonar_rear'
    sonar_rear.node_name = "sonar_rear_sensor"
    sonar_rear.left_limit = 0.7*Math::PI
    sonar_rear.right_limit = 0.3*Math::PI
    sonar_rear.resolution = 0.1
    sonar_rear.maximum_distance = 50.0
    sonar_rear.ping_pong_mode = true
    sonar_rear.configure
    sonar_rear.start
        
    ground_distance = TaskContext.get 'ground_distance'
    ground_distance.node_name = "ground_distance_sensor"
    ground_distance.configure
    ground_distance.start

    imu = TaskContext.get 'imu'
    imu.name = "avalon"
    imu.configure
    imu.start

#Control Draft
    
###########################ACCELERATION_CONTROLLER
    acceleration_controller = TaskContext.get 'acceleration_controller'
    
    expected = acceleration_controller.expected_inputs
    expected.linear[0] = true
    expected.linear[1] = true
    expected.linear[2] = true
    expected.angular[0] = true
    expected.angular[1] = true
    expected.angular[2] = true
    acceleration_controller.expected_inputs = expected
    
    acceleration_controller.timeout_in = 0
    acceleration_controller.timeout_cascade = 0

    matrix = acceleration_controller.matrix
    
    matrix.resize(6,6)
    matrix[0, 0] = 0
    matrix[0, 1] = 0
    matrix[0, 2] = -1 #-0.6
    matrix[0, 3] = -1 #-0.65
    matrix[0, 4] = 0
    matrix[0, 5] = 0

    matrix[1, 0] = 0
    matrix[1, 1] = 0
    matrix[1, 2] = 0
    matrix[1, 3] = 0
    matrix[1, 4] = 0.0001 #0.2
    matrix[1, 5] = -0.8 #-0.55

    matrix[2, 0] = 0.02 #0.3
    matrix[2, 1] = 1.0 #-0.6
    matrix[2, 2] = 0
    matrix[2, 3] = 0
    matrix[2, 4] = 0
    matrix[2, 5] = 0

    matrix[3, 0] = 0
    matrix[3, 1] = 0
    matrix[3, 2] = 0
    matrix[3, 3] = 0
    matrix[3, 4] = 0
    matrix[3, 5] = 0

    matrix[4, 0] = 1.0 #0.45
    matrix[4, 1] = -0.1 #0.15
    matrix[4, 2] = 0
    matrix[4, 3] = 0
    matrix[4, 4] = 0
    matrix[4, 5] = 0

    matrix[5, 0] = 0
    matrix[5, 1] = 0
    matrix[5, 2] = 0
    matrix[5, 3] = 0
    matrix[5, 4] = -1.0 #-0.6
    matrix[5, 5] = -0.2 #-0.4
    acceleration_controller.matrix = matrix

    acceleration_controller.cmd_out.connect_to(actuators.command) 

    acceleration_controller.configure
    acceleration_controller.start

    
    writer_ac_in = acceleration_controller.cmd_in.writer
    writer_ac_cascade = acceleration_controller.cmd_cascade.writer

   
    	sample = writer_ac_cascade.new_sample
        sample.time = Time.now
        sample.linear[0] = NaN
        sample.linear[1] = NaN
        sample.linear[2] = NaN
        sample.angular[0] = 0
        sample.angular[1] = 0
        sample.angular[2] = NaN
        writer_ac_cascade.write(sample)

        sample = writer_ac_in.new_sample
        sample.time = Time.now
        sample.linear[0] = 0.0
        sample.linear[1] = 0
        sample.linear[2] = 0
        sample.angular[0] = NaN
        sample.angular[1] = NaN
        sample.angular[2] = 0.0
        writer_ac_in.write(sample)

    widget.joystick1.connect(SIGNAL('axisChanged(double,double)'))do |x,y|
        sample.time = Time.now
        sample.linear[0] = x
        sample.angular[2] = y
        writer_ac_in.write(sample)
    end

    widget.joystick2.connect(SIGNAL('axisChanged(double,double)'))do |x,y|
        sample.linear[2] = - 0.6 + x
        sample.linear[1] = y
	writer_ac_in.write sample
    end
    
    imu = TaskContext.get "imu"
    imu.name = "avalon"
    #imu.configure
    #imu.start

 
    pos = TaskContext.get 'uw_particle_localization'
    feature = Orocos::TaskContext.get 'sonar_feature_estimator'
    feature.derivative_history_length = 1
    feature.plain_threshold = 0.5
    feature.signal_threshold = 0.7
    feature.enable_debug_output = true
    
    pos.debug = true
    pos.init_position = [0.0, 0.0, 0.0]
    pos.init_variance = [24.0, 20.0, 1.0]

    pos.pure_random_motion = false
    pos.particle_number = 500
    pos.minimum_depth = 0.0
    pos.minimum_perceptions = 3
    pos.effective_sample_size_threshold = 0.8
    pos.hough_interspersal_ratio = 0.05
    pos.sonar_maximum_distance = 25.0
    pos.sonar_covariance = 0.5
    pos.yaml_map = File.join("#{ENV['AUTOPROJ_CURRENT_ROOT']}/avalon/orogen/uw_particle_localization/maps/testhalle.yml")
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
    pos.orientation_offset = 0.0
    pos.sonar_position = [[0.6, 0.0, 0.0]]
    pos.laser_samples_period = 0.01
    pos.echosounder_samples_period = 0.01
    
    pos.apply_conf_file("#{ENV['AUTOPROJ_CURRENT_ROOT']}/bundles/avalon/config/orogen/uw_particle_localization::Task.yml", ["default"])
    pos.yaml_map = File.join("#{ENV['AUTOPROJ_CURRENT_ROOT']}/auv_avalon/orogen/uw_particle_localization/maps/testhalle_sim.yml")
    pos.orientation_offset = 0.0#-2.47 - (Math::PI / 2.0) #0.47951
    pos.hough_interspersal_ratio = 0.05
    pos.hough_timeout = 20.0
    pos.hough_timeout_interspersal = 0.0 #0.2
    pos.param_TCM = [0.0,0.0,-1.0,-1.0,0.0,0.0,
                     0.0,0.0,0.0,0.0,0.1,-1.0,
                     1.0,-1.0,0.0,0.0,0.0,0.0,
                     0.0,0.0,0.0,0.0,0.0,0.0,
                     0.0,0.0,0.0,0.0,0.0,0.0,
                     0.0,0.0,0.0,0.0,0.0,0.0]
    pos.static_motion_covariance = [0.1,0.0,0.0, 0.0,0.1,0.0, 0.0,0.0,0.0]
    pos.sonar_covariance = 0.5
    pos.aggregator_max_latency = 0.5
    #pos.particle_number = 1
    pos.use_best_feature_only = true
    pos.max_velocity_drift = 1.0    
    
    
    hough = TaskContext.get 'sonar_wall_hough'
    hough.usePositionSamples =false
    hough.correctToFirstPosition=true
    hough.filterThreshold=70
    hough.withMinimumFilter=false
    hough.show_debug=true
    hough.basinHeight = 19 #20
    hough.basinWidth = 23.5 #24
    hough.angleDelta = 0.0
    hough.minDistance = 1.0
    hough.maxDistance=700
    #hough.anglesPerBin = 1#2
    #hough.distancesPerBin = 1
    hough.minLineVotesRatio = 0.01
    hough.gain = 5    
    hough.continous_write = false
    
    hough.ignoreOrientation = true
    hough.sensorAngularTolerance = 20 
  
  
  #hough.apply_conf_file("#{ENV['AUTOPROJ_CURRENT_ROOT']}/bundles/avalon/config/orogen/sonar_wall_hough::Task.yml", ["maritime_hall"])
  oriCor = TaskContext.get 'orientation_correction'
  oriCor.buffer_size = 20
  oriCor.min_buffer_size = 6
    
  sonar.sonar_beam.connect_to hough.sonar_samples, :type => :buffer, :size => 100
  imu.pose_samples.connect_to hough.orientation_samples
  pos.dead_reckoning_samples.connect_to hough.pose_samples
    
    sonar.sonar_beam.connect_to feature.sonar_input
    imu.pose_samples.connect_to pos.orientation_samples
    imu.pose_samples.connect_to pos.echosounder_samples
    #ori.pose_samples.connect_to feature.orientation_sample
    feature.features_out.connect_to pos.obstacle_samples
    actuators.status.connect_to pos.thruster_samples
    hough.position.connect_to pos.pose_update
    
    puts "Conection start"
    
    imu.pose_samples.connect_to oriCor.orientation_input
    hough.position_quality.connect_to oriCor.orientation_offset
    
    puts "Connection end"
    
    feature.configure
    feature.start
    pos.configure
    pos.start
    hough.configure
    hough.start
    oriCor.configure
    oriCor.start
    puts "started"    
    
   
    
    #widget.horizontalSlider_1.connect(SIGNAL('valueChanged(int)'))do |x|
    #    sample.target[1] = x/100.0
#	 writer.write sample
#    end


    widget.show 

    #Vizkit.display feature
    Vizkit.display pos
    Vizkit.display hough
    #Vizkit.display actuators
    
    Vizkit.exec
    

end



=begin
#Orocos.run "AvalonSimulation" ,:wait => 10000, :valgrind => false, :valgrind_options => ['--undef-value-errors=no'] do 
Orocos.run "AvalonSimulation" ,"uw_particle_localization_test", "sonar_wall_hough" ,"sonar_feature_estimator::Task"=> "sonar_feature_estimator", :wait => 15000, :valgrind => false, :valgrind_options => ['--undef-value-errors=no'] do 
    simulation = TaskContext.get 'avalon_simulation'    

      white_light = TaskContext.get 'white_light'
#     white_light.interval_mode = 1
#     white_light.constantInterval = 3000
      white_light.interval_mode = 2
      white_light.randomInterval_min = 1;
      white_light.randomInterval_max = 5000;
      white_light.start
      
simulation.scenefile = "#{ENV['AUTOPROJ_CURRENT_ROOT']}/simulation/orogen/avalon_simulation/configuration/testhalle.scn"

    simulation.debug_sonar = false 
    simulation.use_osg_ocean = false 
    simulation.enable_gui = true
    simulation.configure
    simulation.reaction_to_physics_error = "warn"
    simulation.start

# Konfiguration der Aktuatoren für alle Fahrzeuge:    
require 'actuators'
values = ActuatorsConfig.new()
# Avalon
    avalon = TaskContext.get 'avalon'
    avalon.node_name = "avalon"
    avalon.cob = [[0.0, 0.0, 0.08]]
    avalon.buoyancy_force = 4
    avalon.configure
    avalon.start
    
    actuators = TaskContext.get 'avalon_actuators'
    actuators.node_name = "avalon"
    actuators.amount_of_actuators = values.avalon_amount_of_actuators
    actuators.maximum_thruster_force = values.avalon_maximum_thruster_force    
    actuators.thruster_position = values.avalon_thruster_position    
    actuators.thruster_direction = values.avalon_thruster_direction    
    actuators.configure
    actuators.start
    writer = actuators.command.writer
    
# ASV
    asv_actuators = TaskContext.get 'asv_actuators'
    asv_actuators.node_name = "asv"
    asv_actuators.amount_of_actuators = values.asv_amount_of_actuators  
    asv_actuators.maximum_thruster_force = values.asv_maximum_thruster_force
    asv_actuators.thruster_position = values.asv_thruster_position
    asv_actuators.thruster_direction = values.asv_thruster_direction    
    asv_actuators.configure
    asv_actuators.start
    asv_writer = asv_actuators.command.writer
    

# Camera configuration

    front_cam = TaskContext.get 'front_camera'
    front_cam.name = 'front_cam'
    front_cam.configure
    front_cam.start
    
    bottom_cam = TaskContext.get 'bottom_camera'
    bottom_cam.name = 'bottom_cam'
    bottom_cam.configure
    bottom_cam.start
    
    top_cam = TaskContext.get 'top_camera'
    top_cam.name = 'top_cam'
    top_cam.configure
    top_cam.start


    sonar = TaskContext.get 'sonar'
    sonar.node_name = "sonar_top_sensor"
    sonar.left_limit = Math::PI
    sonar.right_limit = -Math::PI
    sonar.resolution = 0.1
    sonar.maximum_distance = 100.0
    sonar.ping_pong_mode = false
    sonar.configure
    sonar.start
    
    sonar_rear = TaskContext.get 'sonar_rear'
    sonar_rear.node_name = "sonar_rear_sensor"
    sonar_rear.left_limit = 0.7*Math::PI
    sonar_rear.right_limit = 0.3*Math::PI
    sonar_rear.resolution = 0.1
    sonar_rear.maximum_distance = 50.0
    sonar_rear.ping_pong_mode = true
    sonar_rear.configure
    sonar_rear.start
        
    ground_distance = TaskContext.get 'ground_distance'
    ground_distance.node_name = "ground_distance_sensor"
    ground_distance.configure
    ground_distance.start
    
#    state_estimator = TaskContext.get 'state_estimator'
#    state_estimator.node_name = 'avalon'
#    state_estimator.configure
#    state_estimator.start
#    Vizkit.display sonar.sonar_beam, :widget => widget.sonar_top
#    Vizkit.display sonar_rear.sonar_beam, :widget => widget.sonar_rear
#    Vizkit.display state_estimator.pose_samples, :widget => widget.orientation


    sample = writer.new_sample
    sample.time = Time.now
    0.upto(5) do
        sample.mode << :DM_PWM
        sample.target << 0;
    end

    widget.joystick1.connect(SIGNAL('axisChanged(double,double)'))do |x,y|
        sample.target[2] = -x
        sample.target[3] = -x
        sample.target[4] = y
        sample.target[5] = y
	writer.write sample
    end

    widget.joystick2.connect(SIGNAL('axisChanged(double,double)'))do |x,y|
        sample.target[1] = x
        sample.target[5] = -y
	writer.write sample
    end

    widget.horizontalSlider_1.connect(SIGNAL('valueChanged(int)'))do |x|
        sample.target[1] = x/100.0
	writer.write sample
    end
    
    imu = TaskContext.get "imu"
    imu.name = "avalon"
    imu.configure
    imu.start
    
    hough = TaskContext.get 'sonar_wall_hough'
    hough.usePositionSamples =false
    hough.correctToFirstPosition=true
    hough.filterThreshold = 70
    hough.withMinimumFilter = false
    hough.show_debug = true
    hough.basinHeight = 20
    hough.basinWidth = 24
    hough.angleDelta = 0.0
    hough.maxDistance= 700
    hough.continous_write = false
    hough.show_debug = true
    
    sonar.sonar_beam.connect_to hough.sonar_samples, :type => :buffer, :size => 100
    imu.pose_samples.connect_to hough.orientation_samples
    
    hough.configure
    hough.start
    
    pos = TaskContext.get 'uw_particle_localization'
    feature = Orocos::TaskContext.get 'sonar_feature_estimator'
    feature.derivative_history_length = 1
    feature.plain_threshold = 0.5
    feature.signal_threshold = 0.7
    feature.enable_debug_output = true
    
    sonar.sonar_beam.connect_to feature.sonar_input
    feature.new_feature.connect_to pos.laser_samples
    imu.pose_samples.connect_to pos.orientation_samples
    actuators.status.connect_to pos.thruster_samples
    hough.position.connect_to pos.pose_update
        
    feature.configure
    feature.start
    
    pos.debug = true
    pos.init_position = [0.0, 0.0, 0.0]
    pos.init_variance = [3.0, 3.0, 0.0]

    pos.static_motion_covariance = [0.001,0.0,0.0, 0.0,0.001,0.0, 0.0,0.0,0.0]
    pos.pure_random_motion = false
    pos.particle_number = 50
    pos.minimum_depth = 0.0
    pos.minimum_perceptions = 3
    pos.effective_sample_size_threshold = 0.9
    pos.hough_interspersal_ratio = 0.1
    pos.sonar_maximum_distance = 24.0
    pos.sonar_covariance = 1.0
    pos.pipeline_covariance = 0.5
    pos.buoy_covariance = 1
    
    pos.sonar_position = Eigen::Vector3.new(0.6, 0 , 0.08)
    pos.pipeline_position = Eigen::Vector3.new(0.0, 0.0, 2)
    pos.buoy_cam_position = Eigen::Vector3.new(2.0, 0.0, 0.0)
    pos.buoy_cam_rotation = Eigen::Vector3.new(0.0, 0.0, 0.0)
    
    pos.yaml_map = File.join("/home/fabio/avalon/avalon/orogen/uw_particle_localization/maps/studiobad.yml")
    pos.advanced_motion_model = false
    pos.param_TCM =  	 [0.0, 0.0,-1.0,-1.0, 0.0, 0.0,  #x
			  0.0, 0.0, 0.0, 0.0, 0.0, 1.0,  #y
			  0.2, -1.0, 0.0, 0.0, 0.0, 0.0, #z
			  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  #roll
			  1.0, 0.0, 0.0, 0.0, 0.0, 0.0,  #pitch
                         #0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  #pitch
			  0.0, 0.0,-1.0, 1.0,-0.5, 0.0 ] #yaw
    pos.param_thrusterCoefficient = [0.005, 0.005,0.005,0.005,0.005,0.005,
				      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
    pos.param_thrusterVoltage = 22.225
    
    linDamp = pos.param_linDamp # Eigen::MatrixX
    sqDamp = pos.param_sqDamp
    linDamp.resize(6,6)
    sqDamp.resize(6,6)
    
    for i in (0..5) do
      for j in (0..5) do
	linDamp[i,j] = 0.0
	sqDamp[i,j] = 0.0
      end
    end
    
    linDamp[0,0] = 8.2
    linDamp[1,1] = 24.9
    linDamp[2,2] = 24
    linDamp[3,3] = 5
    linDamp[4,4] = 15
    linDamp[5,5] = 15
    
    print linDamp[1,0]
    pos.param_linDamp = linDamp
    pos.param_linDampNeg = linDamp

    pos.param_sqDamp = sqDamp
    pos.param_sqDampNeg = sqDamp
    
    pos.param_floating = false
    pos.param_centerOfGravity =  Eigen::Vector3.new(0.0,0.0,0.002)
    pos.param_centerOfBuoyancy = Eigen::Vector3.new(-0.0001, 0.0001, -0.002)  #(-0.03,0.08,-0.2)  #(0.12,-0.04,0.0)
    
    pos.param_length = 1.4
    pos.param_radius = 0.15
    pos.param_mass = 65
    
    pos.configure
    pos.start
    
    #Vizkit.display pos
#    Vizkit.display sonar
#    Vizkit.display feature
#    Vizkit.display simulation
#    Vizkit.display bottom_cam
#    Vizkit.display front_cam
#    Vizkit.display top_cam
#    Vizkit.display sonar
#    Vizkit.display sonar_rear
#    Vizkit.display ground_distance
#    Vizkit.display pinger_search
#    Vizkit.display actuators
#    Vizkit.display asv_actuators
#    Vizkit.display test_vehicle_actuators
    widget.show 
    Vizkit.exec


=end
