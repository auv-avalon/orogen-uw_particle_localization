require 'orocos'
require 'vizkit'

include Orocos
#Orocos.CORBA.name_service = cfg["nameserver"].to_s
#Orocos::CORBA.name_service.ip = localhost#"192.168.128.51"
Orocos.initialize

view3d = Vizkit.default_loader.create_plugin 'vizkit3d::Vizkit3DWidget'
view3d.show

ps = view3d.createPlugin("uw_localization", "ParticleSetVisualization")
env = view3d.createPlugin("uw_localization", "MapVisualization")
sonar = view3d.createPlugin("uw_localization", "SonarPointVisualization")

Orocos.run do
    pos = TaskContext.get 'uw_particle_localization'
    
    Vizkit.connect_port_to 'uw_particle_localization', 'environment', :pull => false, :update_frequency => 33 do |sample, _|
        env.updateEnv(sample)
        sample
    end

    Vizkit.connect_port_to 'uw_particle_localization', 'particles', :pull => false, :update_frequency => 33 do |sample, _|
        ps.updateParticleSet(sample)
        sample
    end
    
    Vizkit.connect_port_to 'uw_particle_localization', 'debug_sonar_beam', :pull => false, :update_frequency => 33 do |sample, _|
	 sonar.updateInfo(samples)
	 sample
    end

    Vizkit.exec
end

