require 'orocos'
require 'vizkit'

include Orocos
#Orocos.CORBA.name_service = cfg["nameserver"].to_s

#Orocos::CORBA.name_service.ip = "192.168.128.51"
Orocos.initialize

#view3d = Vizkit.default_loader.create_plugin 'vizkit3d::Vizkit3DWidget'
view3d = Vizkit.vizkit3d_widget
view3d.show

rbs = view3d.createPlugin("base", "RigidBodyStateVisualization")
#laser_scan  = view3d.createPlugin("base", "LaserScanVisualization")
ps = view3d.createPlugin("uw_localization", "ParticleSetVisualization")
env = view3d.createPlugin("uw_localization", "MapVisualization")
sonar = view3d.createPlugin("uw_localization", "SonarPointVisualization")
sonar_beam = view3d.createPlugin("sonar", "SonarBeamVisualization")
view3d.grid.setPluginEnabled(false)

Orocos.run do
    pos = TaskContext.get 'uw_particle_localization'
    sonar_f_task = TaskContext.get 'sonar_feature_estimator'
    sonar_task = TaskContext.get 'sonar'    
    first = true 
    Vizkit.connect_port_to 'uw_particle_localization', 'environment', :pull => false, :update_frequency => 33 do |sample, _|
        if first
            env.updateEnv(sample)
            first = false
        end
        sample
    end
   
    sonar_task.sonar_beam.connect_to do |sample,_|
	sonar_beam.updateSonarBeam(sample)
        sample
    end

#    sonar_f_task.new_feature.connect_to do |sample,_|
#	laser_scan.updateData(sample)
#        sample
#    end

    pos.pose_samples.connect_to do |sample,_| 
        rbs.updateData(sample)
        sample
    end

    #Vizkit.connect_port_to 'uw_particle_localization', 'particles', :pull => false, :update_frequency => 33 do |sample, _|
    pos.particles.connect_to do |sample, _|
        ps.updateParticleSet(sample)
        sample
    end
    
    #Vizkit.connect_port_to 'uw_particle_localization', 'debug_sonar_beam', :pull => false, :update_frequency => 33 do |sample, _|
    pos.debug_sonar_beam.connect_to do |sample, _|
	 sonar.updateInfo(sample)
	 sample
    end

    Vizkit.exec
end

