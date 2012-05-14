require 'orocos'
require 'vizkit'

include Orocos
Orocos.initialize

view3d = Vizkit.default_loader.create_widget 'vizkit::Vizkit3DWidget'
view3d.show_grid = false
view3d.show
gt = view3d.createPlugin("vizkit-base", "RigidBodyStateVisualization")
ep = view3d.createPlugin("vizkit-base", "RigidBodyStateVisualization")
mon = view3d.createPlugin("uw_localization_monitor", "MonitorVisualization")

Orocos.run do
    pos = TaskContext.get 'uw_particle_localization'
    sonar = TaskContext.get 'sonar_feature_estimator'
    state = TaskContext.get 'state_estimator'
    uwv = TaskContext.get 'uwv_dynamic_model'

    Vizkit.display pos
    Vizkit.display sonar
    Vizkit.display state
    Vizkit.display uwv
    
    Vizkit.connect_port_to 'uw_particle_localization', 'environment', :pull => false, :update_frequency => 33 do |sample, _|
        mon.updateEnvironment(sample)
        sample
    end

    Vizkit.connect_port_to 'uw_particle_localization', 'particles', :pull => false, :update_frequency => 33 do |sample, _|
        mon.updateParticleSet(sample)
        sample
    end

    Vizkit.connect_port_to 'uw_particle_localization', 'debug_sonar', :pull => false, :update_frequency => 33 do |sample, _|
        mon.updateParticleInfo(sample)
        sample
    end

    Vizkit.connect_port_to 'uw_particle_localization', 'pose_samples', :pull => false, :update_frequency => 33 do |sample, _|
        ep.updateRigidBodyState(sample)
        sample
    end

    Vizkit.exec
end

