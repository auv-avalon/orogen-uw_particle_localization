require 'orocos'
require 'vizkit'

include Orocos
#Orocos.CORBA.name_service = cfg["nameserver"].to_s

#Orocos::CORBA.name_service.ip = "192.168.128.51"  #20"
Orocos.initialize

#view3d = Vizkit.default_loader.create_plugin 'vizkit3d::Vizkit3DWidget'
view3d = Vizkit.vizkit3d_widget
view3d.show

rbs = view3d.createPlugin("base", "RigidBodyStateVisualization")
env = view3d.createPlugin("uw_localization", "MapVisualization")
map = view3d.createPlugin("uw_localization", "SimpleGridVisualization")
features = view3d.createPlugin("sonar", "SonarDetectorVisualization")
#sonar_beam = view3d.createPlugin("sonar", "AvalonSonarBeamVisualization")
view3d.grid.setPluginEnabled(false)

Orocos.run do
    pos = TaskContext.get 'uw_particle_localization'
    #pos = TaskContext.get "localization"

    sonar_detector = TaskContext.get "sonar_feature_detector"

    first = true 
   
    pos.pose_samples.connect_to do |sample,_| 
        rbs.updateData(sample)
        sample
    end

    pos.environment.connect_to do |sample,_|
	#env.updateData(sample)
	sample
    end

    pos.grid_map.connect_to do |sample,_|
      map.updateSimpleGrid(sample)
      sample
    end
    
    sonar_detector.features.connect_to do |sample, _|
      features.updateSonarFeatures(sample)
      sample
    end
    

    Vizkit.exec
end

