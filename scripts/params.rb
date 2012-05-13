class AvalonModelParameters
    # UWV dimensional parameters
    def self.vehicle_dimensional_parameters(task)
        # vector position from the origin of body-fixed frame to center of buoyancy in body-fixed frame
        task.distance_body2centerofbuoyancy[0] = 0;
        task.distance_body2centerofbuoyancy[1] = 0;
        task.distance_body2centerofbuoyancy[2] = 0;

        # vector position from the origin of body-fixed frame to center of gravity in body-fixed frame
        task.distance_body2centerofgravity[0] = 0;
        task.distance_body2centerofgravity[1] = 0;
        task.distance_body2centerofgravity[2] = 0;
    end

    # UWV physical parameters
    def self.vehicle_physical_parameters(task)
        task.uwv_mass = 63 		# total mass of the vehicle in kg
        task.uwv_volume = (2*(0.55*0.15*0.15*Math::PI)+0.30*Math::PI*0.15*0.15) # total volume of the vehicle - considered as two cylinders and center as cylinder
        task.uwv_float = true		# to assume that the vehicle floats . i.e gravity == buoyancy
    end

    # Environment parameters
    def self.environmental_parameters(task)
        task.waterDensity = 998.2	# density of pure water at 20°C in kg/m^3
        task.gravity = 9.81
    end

    # UWV interia + added mass Matrix
    # consider Avalon as one big cylinder
    def self.mass_matrix(task)
        task.mass_matrix = [50,0,0,0,0,0,  0,20,0,0,0,0,  0,0,30,0,0,0,  0,0,0,10,0,0,  0,0,0,0,10,0,  0,0,0,0,0,10]
    end

    # Linear damp coefficient
    def self.linDampCoeff(task)
        task.linDampCoeff = [40.88, 10, 10, 10, 10, 10 ]
    end

    # Quadratic damp coefficient
    def self.quadDampCoeff(task)
        task.quadDampCoeff = [155, 50, 50, 10, 10, 10]
    end
    # ThrusterCoefficient
    def self.thrusterCoefficient(task)
        task.thruster_coefficient.surge.positive = 0.0000045
        task.thruster_coefficient.surge.negative = 0.0000045
        task.thruster_coefficient.sway.positive = 0.0000045
        task.thruster_coefficient.sway.negative = 0.0000045
        task.thruster_coefficient.heave.positive = 0.0000045
        task.thruster_coefficient.heave.negative = 0.0000045
        task.thruster_coefficient.roll.positive = 0.0
        task.thruster_coefficient.roll.negative = 0.0
        task.thruster_coefficient.pitch.positive = 0.0000045
        task.thruster_coefficient.pitch.negative = 0.0000045
        task.thruster_coefficient.yaw.positive = 0.0000045
        task.thruster_coefficient.yaw.negative = 0.0000045
    end

    # Thruster mapping
    def self.thruster_mapping(task)
        array_thruster_value = task.thruster_value.to_a
            array_thruster_value= [10 , 10, 0, 0 ,0]
        task.thruster_value = array_thruster_value

        array_thruster_mapped_names = task.thruster_mapped_names.to_a
            #array_thruster_mapped_names= "SURGE,SURGE,SWAY,SWAY,HEAVE,HEAVE".split(/,/)
            array_thruster_mapped_names= ['SURGE','SURGE','SWAY','HEAVE','HEAVE']
        task.thruster_mapped_names = array_thruster_mapped_names

    end

    # Thruster control matrix
    def self.thruster_control_matrix(task)
        task.thruster_control_matrix = [1,0,0,0,0,-0.5, 1,0,0,0,0,0.5, 0,1,0,0,0,0, 0,0,1,0,-0.5,0, 0,0,1,0,0.5,0]
    end

    # Simulation parameters
    def self.simulation_data(task)
        task.sim_per_cycle = 5					# number of RK4 simulations per sampling interval
        task.plant_order = 12					# plant order - number of states in the model - generally 12 states -> 3 position, 3 orientation and 6 linear&angular velocity
        task.ctrl_order = 5;					# ctrl order - number of controllable inputs
        task.samplingtime = 0.1;           				# sampling time used in simulation
        task.initial_condition = [ 0,0,0,0,0,0, 0,0,0,0,0,0];	# Initial conditions used for simulation - currently it 12 states
        task.initial_time = 0.0;              			# Initial time	used for simulation
    end

    def self.initialize_vehicle_parameters(task)
        vehicle_dimensional_parameters(task)
        vehicle_physical_parameters(task)
        mass_matrix(task)
        linDampCoeff(task)
        quadDampCoeff(task)
        thrusterCoefficient(task)
        thruster_mapping(task.thrusters)
        thruster_control_matrix(task)
        simulation_data(task)
        task.thrusterVoltage = 30

    end
end

