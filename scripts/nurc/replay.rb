require 'orocos'
require 'vizkit'
require 'optparse'
include Orocos
Orocos.initialize

options = {}

directory = ARGV[0]

if !File.directory?(directory)
    raise "#{dirname} is not a directory"
end

mapfile = File.join(ENV['AUTOPROJ_CURRENT_ROOT'], "supervision", "maps", "nurc.yml")
configpath = File.join(ENV['AUTOPROJ_CURRENT_ROOT'], "supervision", "config", "orogen")

log_files = Dir.glob(File.join(ARGV[0], "*.log"))
files = log_files.select do |sample|
    sample =~ /sonar\.\d+\.log|avalon_back_base_control\.\d+\.log|front_camera|bottom_camera/
end

log = Log::Replay.open(*files)

view3d = Vizkit.default_loader.create_widget "vizkit::Vizkit3DWidget" 
view3d.show_grid = false
view3d.show

position_viz = view3d.createPlugin("vizkit-base", "RigidBodyStateVisualization")
hough_viz = view3d.createPlugin("vizkit-base", "RigidBodyStateVisualization")
environment_viz = view3d.createPlugin("uw_localization_map", "MapVisualization")
particle_viz = view3d.createPlugin("uw_localization_particle", "ParticleVisualization")
sonarpoint_viz = view3d.createPlugin("uw_localization_sonarpoint", "SonarPointVisualization")

mapyaml = YAML::load(File.open(mapfile))

mapdepth = mapyaml["reference"][2]

particle_viz.min_z = 0.0
particle_viz.max_z = mapdepth
sonarpoint_viz.min_z = 0.0
sonarpoint_viz.max_z = mapdepth

environment_viz.map = mapfile
environment_viz.grid_resolution = 1.0

position_viz.setColor(Eigen::Vector3.new(1.0, 0.0, 0.0))
hough_viz.setColor(Eigen::Vector3.new(0.0, 1.0, 0.0))

Orocos.run "uw_particle_localization", "sonar_wall_hough", "sonar_feature_estimator", "pipeline_follower", :wait => 999 do
    sonar = log.task 'sonar'
    yaw_z = log.task 'depth_orientation_fusion'
    bottom_camera = log.task 'bottom_camera'
    front_camera = log.task 'front_camera'
    hbridge = log.task 'hbridge'
    localization = TaskContext.get 'uw_particle_localization'
    feature = TaskContext.get 'sonar_feature_estimator'
    hough = TaskContext.get 'sonar_wall_hough'
    pipeline = TaskContext.get 'pipeline_follower'

    sonar.sonar_beam.connect_to feature.sonar_input
    sonar.sonar_beam.connect_to hough.sonar_samples
    hbridge.status_motors.connect_to localization.thruster_samples
#    hough.position.connect_to localization.pose_update
    yaw_z.pose_samples.connect_to localization.orientation_samples
    yaw_z.pose_samples.connect_to hough.orientation_samples
    yaw_z.pose_samples.connect_to pipeline.orientation_sample
    yaw_z.pose_samples.connect_to pipeline.altitude_samples
    feature.new_feature.connect_to localization.laser_samples

    bottom_camera.frame.connect_to pipeline.frame 
    pipeline.pipeline.connect_to localization.pipeline_samples
    

    Orocos.apply_conf_file(feature, File.join(configpath, "sonar_feature_estimator::Task.yml"), ["default"])
    Orocos.apply_conf_file(localization, File.join(configpath, "uw_particle_localization::Task.yml"), ["default"])
    Orocos.apply_conf_file(hough, File.join(configpath, "sonar_wall_hough::Task.yml"), ["default"])
    Orocos.apply_conf_file(pipeline, File.join(configpath, "offshore_pipeline_detector::Task.yml"), ["default"])

    localization.yaml_map = mapfile

    Vizkit.display localization
    Vizkit.display feature
    Vizkit.display hough
    Vizkit.display pipeline
#    Vizkit.display front_camera.frame
    Vizkit.display bottom_camera.frame
 
    Vizkit.connect_port_to 'uw_particle_localization', 'particles', :pull => false, :update_frequency => 33 do |sample, _|
        particle_viz.updateParticles(sample)
        sample
    end

    Vizkit.connect_port_to 'uw_particle_localization', 'debug_sonar_beam', :pull => false, :update_frequency => 33 do |sample, _|
        sonarpoint_viz.updatePointInfo(sample)
        sample
    end

    Vizkit.connect_port_to 'uw_particle_localization', 'pose_samples', :pull => false, :update_frequency => 33 do |sample, _|
        position_viz.updateRigidBodyState(sample)
        sample
    end

    Vizkit.connect_port_to 'sonar_wall_hough', 'position', :pull => false, :update_frequency => 33 do |sample, _|
        hough_viz.updateRigidBodyState(sample)
        sample
    end
 
    localization.configure
    feature.configure
    hough.configure
    pipeline.configure
    localization.start
    feature.start
#    hough.start
    pipeline.start

    Vizkit.control log
    Vizkit.exec
end
