require 'orocos'
require 'vizkit'
require 'optparse'

include Orocos
Orocos.initialize

options = {}
components = ["sonar.19.log", "pose_estimator.19.log"]
dir = "~/logs/slam/"

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
sonarbeamviz = view3d.createPlugin("sonarbeam", "SonarBeamVisualization")

Orocos.run "AvalonSimulation", "uw_particle_localization_test", "sonar_feature_estimator", :wait => 999 do
    Orocos.log_all_ports

    sonar = log.task 'sonar'
    state = log.task "pose_estimator"
    pos = TaskContext.get 'uw_particle_localization'
    feature = Orocos::TaskContext.get 'sonar_feature_estimator'

    sonar.SonarScan.connect_to feature.sonar_input
    state.pose_samples.connect_to pos.orientation_samples
    state.pose_samples.connect_to pos.speed_samples
    feature.new_feature.connect_to pos.laser_samples

    feature.derivative_history_length = 1

    pos.init_position =   [0.0,-4.0, 0.0]
    pos.init_covariance = [8.0, 0.0, 0.0,
                           0.0, 4.0, 0.0,
                           0.0, 0.0, 1.0]

    pos.static_motion_covariance = [2.0,0.0,0.0, 0.0,2.0,0.0, 0.0,0.0,0.0]

    pos.particle_number = 40
    pos.minimum_depth = 0.0
    pos.minimum_perceptions = 1
    pos.effective_sample_size_threshold = 35
    pos.particle_interspersal_ratio = 0.0
    pos.sonar_maximum_distance = 13.0
    pos.sonar_covariance = 2.0

    pos.perception_ratio = 0.7
    pos.noise_ratio = 0.1
    pos.max_distance_ratio = 0.2

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

    Vizkit.connect_port_to 'uw_particle_localization', 'pose_samples', :pull => false, :update_frequency => 33 do |sample, _|
        ep.updateRigidBodyState(sample)
        sample
    end

    log.pose_estimator.pose_samples do |sample|
        gt.updateRigidBodyState(sample)
        laserviz.updatePose(sample)
        sonarbeamviz.updateBodyState(sample)
        sample
    end

#    log.sonar.SonarScan  do |sample|
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

