require 'orocos'
require 'vizkit'
require 'optparse'
require File.join(File.dirname(__FILE__), 'params')

include Orocos
Orocos.initialize

options = {}
#components = ["sonar.6.0.log", "pose_estimator.6.0.log", "avalon_control.6.0.log"]
#dir = "~/logs/20101104_ekfslam/"

components = ["sonar.3.0.log", "pose_estimator.3.0.log", "avalon_control.3.0.log"]
dir = "~/logs/20101111_ekfslam/"

mapfile =  File.join("..", "maps", "studiobad.yml")

files = components.map do |comp|
    File.join(dir, "#{comp}")
end

log = Orocos::Log::Replay.open(*files)

view3d = Vizkit.default_loader.create_widget 'vizkit::Vizkit3DWidget'
view3d.show_grid = false
view3d.show
gt = view3d.createPlugin("vizkit-base", "RigidBodyStateVisualization")
ep = view3d.createPlugin("vizkit-base", "RigidBodyStateVisualization")
env = view3d.createPlugin("uw_localization_map", "MapVisualization")
pviz = view3d.createPlugin("uw_localization_particle", "ParticleVisualization")
sviz = view3d.createPlugin("uw_localization_sonarpoint", "SonarPointVisualization")

mapyaml = YAML::load(File.open(mapfile))
puts mapyaml
pviz.min_z = 0.0
pviz.max_z = mapyaml["reference"][2]
sviz.min_z = 0.0
sviz.max_z = mapyaml["reference"][2]

Orocos.run "uwv_dynamic_model", "uw_particle_localization_test", "sonar_feature_estimator", :wait => 999 do
    Orocos.log_all_ports

    sonar = log.task 'sonar'
    state = log.task 'pose_estimator'
    motion = log.task 'motion_control'
    pos = TaskContext.get 'uw_particle_localization'
    feature = TaskContext.get 'sonar_feature_estimator'
    mm = TaskContext.get 'uwv_dynamic_model'

    sonar.SonarScan.connect_to feature.sonar_input
    motion.hbridge_commands.connect_to mm.thrusterinput
    state.pose_samples.connect_to pos.orientation_samples
    state.pose_samples.connect_to pos.speed_samples
#    mm.uwvstate.connect_to pos.speed_samples
    feature.new_feature.connect_to pos.laser_samples

    feature.derivative_history_length = 1

    params = mm.uwv_param
    #AvalonModelParameters::initialize_vehicle_parameters(params)
    mm.uwv_param = params

#    pos.init_position = [0.0,-4.0, 0.0]
#    pos.init_variance = [4.0, 4.0, 0.0]

    pos.static_motion_covariance = [3.0,0.0,0.0, 0.0,3.0,0.0, 0.0,0.0,0.0]
    pos.pure_random_motion = true

    pos.particle_number = 50
    pos.minimum_depth = 0.0
    pos.minimum_perceptions = 2
    pos.effective_sample_size_threshold = 0.9
    pos.particle_interspersal_ratio = 0.0
    pos.sonar_maximum_distance = 12.0
    pos.sonar_covariance = 3.0

    pos.yaml_map = mapfile
    Vizkit.display pos
    Vizkit.display mm
    Vizkit.display feature
    
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

    log.pose_estimator.pose_samples do |sample|
        gt.updateRigidBodyState(sample)
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

