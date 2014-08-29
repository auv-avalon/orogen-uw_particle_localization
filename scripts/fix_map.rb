require 'orocos'
include Orocos

Orocos.initialize

Orocos.run do
  
  detector = TaskContext.get "sonar_feature_detector"
  
  detector.fix_map
  
  puts "Fixed sonar feature map"
  
  
end