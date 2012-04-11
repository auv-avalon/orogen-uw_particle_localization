require 'orocos'
require 'vizkit'
require 'optparse'

include Orocos
Orocos.initialize

options = {}
components = ["sonar.0.log", "state_estimator.0.log"]

args = OptionParser.new do |opts|
    opts.banner = "Usage: motion_replay.rb [options]"

    opts.on("-d", "--dir [FILE]", "sets the directory where logs are located") do |dir|
        options[:dir] = dir.to_s
    end
end

args.parse!

raise "missing directory argument" unless options[:dir]

files = components.map do |comp|
    File.join(options[:dir], "#{comp}")
end

log = Orocos::Log::Replay.open(*files)

view3d = Vizkit.default_loader.create_widget 'vizkit::Vizkit3DWidget'
gt = view3d.createPlugin("RigidBodyStateVisualization")
ep = view3d.createPlugin("RigidBodyStateVisualization")
laserviz = view3d.createPlugin 'uw_localization_laserscan', 'LaserScanVisualization'
viz = view3d.createPlugin("uw_localization_particle", "ParticleVisualization")
landmark = view3d.createPlugin("uw_localization_mixedmap", "MixedMapVisualization")
sonarbeamviz = view3d.createPlugin("sonarbeam","SonarBeamVisualization")
view3d.show

Orocos.run "AvalonSimulation", "uw_particle_localization_test", "sonar_feature_estimator", :wait => 999 do
    sonar = log.task 'sonar'
    state = log.task "pose_estimator"
    pos = TaskContext.get 'uw_particle_localization'
    feature = Orocos::TaskContext.get 'sonar_feature_estimator'

    sonar.SonarScan.connect_to feature.sonar_input
    state.pose_samples.connect_to pos.orientation_samples
    state.pose_samples.connect_to pos.speed_samples
    feature.new_feature.connect_to pos.laser_samples

    feature.derivative_history_length = 1
    feature.plain_threshold = 0.5
    feature.signal_threshold = 0.7
    feature.enable_debug_output = true


    pos.init_position = [0.0, 0.0, 0.0]
    pos.init_covariance = [17.0, 0.0, 0.0,
                           0.0, 5.0, 0.0,
                           0.0, 0.0, 1.0]

    pos.static_motion_covariance = [2.0,0.0,0.0, 0.0,2.0,0.0, 0.0,0.0,0.0]

    pos.particle_number = 100
    pos.minimum_perceptions = 5
    pos.effective_sample_size_threshold = 70
    pos.particle_interspersal_ratio = 0.0
    pos.sonar_maximum_distance = 20.0
    pos.sonar_covariance = 3.0

    pos.perception_ratio = 1.0
    pos.noise_ratio = 0.0
    pos.max_distance_ratio = 0.0

    pos.yaml_map = File.join("..", "maps", "studiobad.yml")



    Vizkit.connect_port_to 'uw_particle_localization', 'particles', :type => :buffer, :size => 100, :pull => false, :update_frequency => 33 do |sample, _|
        viz.updateParticles(sample)
        sample
    end

    Vizkit.connect_port_to 'uw_particle_localization', 'mixedmap', :type => :buffer, :size => 100, :pull => false, :update_frequency => 33 do |sample, _|
        landmark.updateMap(sample)
        sample
    end

    Vizkit.connect_port_to 'sonar_feature_estimator', 'new_feature', :type => :buffer, :size => 100, :pull => false, :update_frequency => 33 do |sample, _|
        laserviz.updateLaserScan(sample)
        sample
    end

    Vizkit.connect_port_to 'pose_estimator', 'pose_samples', :type => :buffer, :size => 100, :pull => false, :update_frequency => 33 do |sample, _|
        laserviz.updatePose(sample)
        gt.updateRigidBodyState(sample)
        sonarbeamviz.updateBodyState(sample)
        sample
    end

    log.sonar.SonarScan  do |sample|
        sonarbeamviz.updateSonarBeam(sample)
        sample
    end

    pos.configure
    feature.configure
    pos.start
    feature.start

    Vizkit.control log 
    Vizkit.exec
end

