require 'orocos'
include Orocos

Orocos.initialize

Orocos.run do
  
  oriCor = TaskContext.get "orientation_correction"
  
  if ARGV.length == 0
    oriCor.reset(0.0)
  else
    oriCor.reset( ARGV[0].to_f)
  end
  
  
end