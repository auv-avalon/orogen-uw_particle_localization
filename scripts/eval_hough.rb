require 'orocos/log'
include Orocos

Result = Struct.new(:real, :estimate, :error, :time, :x, :y)
result = []
g = []
e = []


log = Log::Replay.open(ARGV)

log.pose_estimator.pose_samples { |sample| 
    g << sample 
    sample
}
log.sonar_wall_hough.position { |sample| 
    e << sample 
    sample
}

log.run

ptr = 0

error = 0.0
var = 0.0
std = 0.0
min = 1_000_000.0
max = 0.0

puts ""

start = nil
last_sample = nil

x = []
y = []

e.shift(5)

for pos in e
    data = Result.new
    start = pos.time unless start

    # next if last_sample and (pos.time - last_sample.time).abs < 2.0

    while ptr < g.size and (pos.time - g[ptr].time).abs > 0.01
        ptr += 1
    end

    if g[ptr] and (pos.time - g[ptr].time).abs <= 0.01
      data.real = g[ptr].position
      data.estimate = pos.position
      
      x_error = (data.real[0] - data.estimate[0]) * (data.real[0] - data.estimate[0])
      y_error = (data.real[1] - data.estimate[1]) * (data.real[1] - data.estimate[1])
      data.error = Math.sqrt(x_error + y_error)
      data.time = pos.time - start
      data.x = x_error
      data.y = y_error

      max = data.error if data.error > max
      min = data.error if data.error < min

      error += data.error

      result << data

      last_sample = pos
    end

    printf "\r Processing [#{ptr}/#{e.size}]"
end

avg = error / result.size

File.open("evaluation.data", "w") do |fd|
    for pos in result
        fd.printf "#{pos.real[0]} #{pos.real[1]} #{pos.real[2]} "
        fd.printf "#{pos.estimate[0]} #{pos.estimate[1]} #{pos.estimate[2]} "
        fd.printf "#{pos.error} "
        fd.printf "#{pos.time} "
        fd.printf "#{pos.x} "
        fd.printf "#{pos.y} "
        fd.printf "\n"

        var += (avg - pos.error) * (avg - pos.error)
    end
end

puts "\nAverage Error: #{error / result.size}"
puts "Max Error: #{max}"
puts "Min Error: #{min}"
puts "Variance: #{var / (result.size - 1)}"
puts "Std: #{Math::sqrt(var / (result.size - 1))}"
puts "Samples: #{result.size}"

system 'gnuplot ./templates/stats.plot'
