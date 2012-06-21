require 'orocos'
require 'vizkit'
require 'optparse'
require File.join(File.dirname(__FILE__), 'params')

include Orocos
Orocos.initialize

options = {}
components = ["sonar.0.0.log", "orientation_estimator.0.log", "avalon_back_base_control.0.log"]
dir = "~/logs/sauc-e_first_day/20110704-1615/"

files = components.map do |comp|
    File.join(dir, "#{comp}")
end

mapfile =  File.join("..", "maps", "nurc.yml")

log = Orocos::Log::Replay.open(*files)

view3d = Vizkit.default_loader.create_widget 'vizkit::Vizkit3DWidget'
view3d.show_grid = false
view3d.show
ep = view3d.createPlugin("vizkit-base", "RigidBodyStateVisualization")
env = view3d.createPlugin("uw_localization_map", "MapVisualization")
pviz = view3d.createPlugin("uw_localization_particle", "ParticleVisualization")
sviz = view3d.createPlugin("uw_localization_sonarpoint", "SonarPointVisualization")
pc = view3d.createPlugin('sonarfeature', 'SonarFeatureVisualization')

mapyaml = YAML::load(File.open(mapfile))
puts mapyaml
pviz.min_z = 0.0
pviz.max_z = mapyaml["reference"][2]
sviz.min_z = 0.0
sviz.max_z = mapyaml["reference"][2]

env.resolution = 2.0

Orocos.run "uwv_dynamic_model", "uw_particle_localization_test", "sonar_feature_estimator", :wait => 999 do
    Orocos.log_all_ports

    sonar = log.task 'sonar'
    state = log.task 'orientation_estimator'
    motion = log.task 'motion_control'
    pos = TaskContext.get 'uw_particle_localization'
    feature = TaskContext.get 'sonar_feature_estimator'
    mm = TaskContext.get 'uwv_dynamic_model'

    sonar.BaseScan.connect_to feature.sonar_input
    motion.hbridge_commands.connect_to mm.thrusterinput
    state.orientation_samples.connect_to pos.orientation_samples
    state.orientation_samples.connect_to pos.speed_samples
    state.orientation_samples.connect_to feature.orientation_sample
    #mm.uwvstate.connect_to pos.speed_samples
    feature.new_feature.connect_to pos.laser_samples

    feature.derivative_history_length = 3

    params = mm.uwv_param
    #AvalonModelParameters::initialize_vehicle_parameters(params)
    mm.uwv_param = params

    pos.init_position = [0.0,-4.0, 0.0]
    pos.init_variance = [10.0, 10.0, 0.0]

    pos.static_motion_covariance = [4.0,0.0,0.0,  0.0,4.0,0.0,  0.0,0.0,0.0]
    pos.pure_random_motion = true

    pos.particle_number = 50
    pos.minimum_depth = -0.2
    pos.minimum_perceptions = 2
    pos.effective_sample_size_threshold = 0.8
    pos.sonar_maximum_distance = 100.0
    pos.sonar_minimum_distance = 2.0
    pos.sonar_covariance = 2.0

    pos.yaml_map = mapfile

    Vizkit.display pos
    Vizkit.display mm
    Vizkit.display feature

    Vizkit.connect_port_to 'sonar_feature_estimator', 'features', :pull => false, :update_frequency => 33 do |sample, _|
        pc.updatePointCloud(sample)
    end 


    Vizkit.connect_port_to 'uw_particle_localization', 'environment', :pull => false, :update_frequency => 33 do |sample, _|
        env.updateMap(sample)
        sample
    end

    Vizkit.connect_port_to 'uw_particle_localization', 'particles', :pull => false, :update_frequency => 33 do |sample, _|
        pviz.updateParticles(sample)
        sample
    end

    Vizkit.connect_port_to 'uw_particle_localization', 'debug_sonar_beam', :pull => false, :update_frequency => 33 do |sample, _|
        sviz.updatePointInfo(sample)
        sample
    end

    Vizkit.connect_port_to 'uw_particle_localization', 'pose_samples', :pull => false, :update_frequency => 33 do |sample, _|
        ep.updateRigidBodyState(sample)
        sample
    end

    mm.configure
    pos.configure
    feature.configure
    pos.start
    feature.start
    mm.start

    Vizkit.control log 
    Vizkit.exec
end

