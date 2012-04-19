require 'orocos'
require 'vizkit'
require 'optparse'

include Orocos
Orocos.initialize

options = {}
components = ["sonar.0.log", "state_estimator.0.log"]
dir = "~/logs/final_run/"

files = components.map do |comp|
    File.join(dir, "#{comp}")
end

log = Orocos::Log::Replay.open(*files)

view3d = Vizkit.default_loader.create_widget 'vizkit::Vizkit3DWidget'
view3d.show
gt = view3d.createPlugin("RigidBodyStateVisualization")
ep = view3d.createPlugin("RigidBodyStateVisualization")
laserviz = view3d.createPlugin 'uw_localization_laserscan', 'LaserScanVisualization'
viz = view3d.createPlugin("uw_localization_particle", "ParticleVisualization")
map = view3d.createPlugin("uw_localization_mapfeature", "MapFeatureVisualization")
sonarbeamviz = view3d.createPlugin("sonarbeam","SonarBeamVisualization")

Orocos.run "AvalonSimulation", "uw_particle_localization_test", "sonar_feature_estimator", :wait => 999 do
    Orocos.log_all_ports

    sonar = log.task 'sonar'
    state = log.task "state_estimator"
    pos = TaskContext.get 'uw_particle_localization'
    feature = Orocos::TaskContext.get 'sonar_feature_estimator'

    sonar.BaseScan.connect_to feature.sonar_input
    state.orientation_samples.connect_to pos.orientation_samples
    state.orientation_samples.connect_to pos.speed_samples
    feature.new_feature.connect_to pos.laser_samples

    feature.derivative_history_length = 5
#    feature.plain_threshold = 0.5
    feature.signal_threshold = 0.7
    feature.enable_debug_output = true


    pos.init_position = [0.0, -4.0, 0.0]
    pos.init_covariance = [17.0, 0.0, 0.0,
                           0.0, 1.0, 0.0,
                           0.0, 0.0, 1.0]

    pos.static_motion_covariance = [2.0,0.0,0.0, 0.0,2.0,0.0, 0.0,0.0,0.0]

    pos.particle_number = 100
    pos.minimum_depth = -0.5
    pos.minimum_perceptions = 3
    pos.effective_sample_size_threshold = 80
    pos.particle_interspersal_ratio = 0.0
    pos.sonar_maximum_distance = 10.0
    pos.sonar_covariance = 2.0
    pos.yaw_offset = Math::PI / 2.0

    pos.perception_ratio = 0.7
    pos.noise_ratio = 0.0
    pos.max_distance_ratio = 0.3

    pos.yaml_map = File.join("..", "maps", "studiobad.yml")

    Vizkit.connect_port_to 'uw_particle_localization', 'particles', :pull => false, :update_frequency => 33 do |sample, _|
        viz.updateParticles(sample)
        sample
    end

    Vizkit.connect_port_to 'uw_particle_localization', 'map_landmarks', :pull => false, :update_frequency => 33 do |sample, _|
        map.updateLandmarks(sample)
        sample
    end

    Vizkit.connect_port_to 'uw_particle_localization', 'map_wall_lines', :pull => false, :update_frequency => 33 do |sample, _|
        map.updateLinemarks(sample)
        sample
    end

    Vizkit.connect_port_to 'sonar_feature_estimator', 'new_feature', :pull => false, :update_frequency => 33 do |sample, _|
        laserviz.updateLaserScan(sample)
        sample
    end

    Vizkit.connect_port_to 'pose_estimator', 'pose_samples', :pull => false, :update_frequency => 33 do |sample, _|
        laserviz.updatePose(sample)
#        sonarbeamviz.updateBodyState(sample)
        sample
    end

#    log.sonar.BaseScan  do |sample|
#        sonarbeamviz.updateSonarBeam(sample)
#        sample
#    end

    pos.configure
    feature.configure
    pos.start
    feature.start

    Vizkit.control log 
    Vizkit.exec
end

