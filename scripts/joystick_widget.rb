require 'vizkit'
include Orocos

Orocos.initialize

widget = Vizkit.load "simulator.ui"

actuactors = TaskContext.get 'actuators'

writer = actuactors.command.writer

widget.joystick1.connect(SIGNAL('axisChanged(double,double)'))do |x,y|
    sample = writer.new_sample
    sample.time = Time.now 
    0.upto(5) do
        sample.mode << :DM_PWM
        sample.target << 0;
    end
    sample.target[2] = -x
    sample.target[3] = -x
    sample.target[4] = y
    writer.write sample
end

widget.joystick2.connect(SIGNAL('axisChanged(double,double)'))do |x,y|
    sample = writer.new_sample
    sample.time = Time.now 
    0.upto(5) do
        sample.mode << :DM_PWM
        sample.target << 0;
    end
    sample.target[1] = -x
    sample.target[5] = y
    writer.write sample
end

widget.show 
Vizkit.exec


