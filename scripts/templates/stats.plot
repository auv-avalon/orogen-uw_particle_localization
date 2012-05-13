set multiplot layout 1, 2
set tmargin 2

set title "Trajectory"
set xlabel "x position in metre"
set ylabel "y position in metre"
set xrange [-9:9]
set yrange [-6:6]

plot "evaluation.data" using 4:5 title "estimate position" with lines, \
     "evaluation.data" using 1:2 title "real position" with lines

set title "distance error over time"
set xlabel "time in s"
set ylabel "distance in metre"
set autoscale

plot "evaluation.data" using 8:7 title "distance error" with lines

unset multiplot
pause -1
