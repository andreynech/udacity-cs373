#!/usr/bin/gnuplot

set nokey
set term png
set termoption dashed
set size ratio -1
set grid

# Function to create the right call function
add_robot(x,y,phi) = sprintf(\
    'call "set_robot.gnu" "%f" "%f" "%f" "%f";',x,y,phi,0.23)

#call "set_robot.gnu" $0 $1 $2 0.23

set output 'particles000.png'
plot 'plan.dat' with lines lc 3, 'trajectory.dat' with lines lc 2, 'sonar_meas000.dat' lc 1, 'ideal_meas000.dat' with lines lc 1, 'iteration000.dat' with dots lc 0

set output 'particles001.png'
plot 'plan.dat' with lines lc 3, 'trajectory.dat' with lines lc 2, 'sonar_meas001.dat' lc 1, 'ideal_meas001.dat' with lines lc 1, 'iteration001.dat' with dots lc 0

set output 'particles002.png'
plot 'plan.dat' with lines lc 3, 'trajectory.dat' with lines lc 2, 'sonar_meas002.dat' lc 1, 'ideal_meas002.dat' with lines lc 1, 'iteration002.dat' with dots lc 0

set output 'particles003.png'
plot 'plan.dat' with lines lc 3, 'trajectory.dat' with lines lc 2, 'sonar_meas003.dat' lc 1, 'ideal_meas003.dat' with lines lc 1, 'iteration003.dat' with dots lc 0

set output 'particles004.png'
plot 'plan.dat' with lines lc 3, 'trajectory.dat' with lines lc 2, 'sonar_meas004.dat' lc 1, 'ideal_meas004.dat' with lines lc 1, 'iteration004.dat' with dots lc 0

set output 'particles005.png'
plot 'plan.dat' with lines lc 3, 'trajectory.dat' with lines lc 2, 'sonar_meas005.dat' lc 1, 'ideal_meas005.dat' with lines lc 1, 'iteration005.dat' with dots lc 0

set output 'particles006.png'
plot 'plan.dat' with lines lc 3, 'trajectory.dat' with lines lc 2, 'sonar_meas006.dat' lc 1, 'ideal_meas006.dat' with lines lc 1, 'iteration006.dat' with dots lc 0

set output 'particles007.png'
plot 'plan.dat' with lines lc 3, 'trajectory.dat' with lines lc 2, 'sonar_meas007.dat' lc 1, 'ideal_meas007.dat' with lines lc 1, 'iteration007.dat' with dots lc 0

set output 'particles008.png'
plot 'plan.dat' with lines lc 3, 'trajectory.dat' with lines lc 2, 'sonar_meas008.dat' lc 1, 'ideal_meas008.dat' with lines lc 1, 'iteration008.dat' with dots lc 0

set output 'particles009.png'
call "set_robot.gnu" 2.466 1.852 2.0614 0.10
plot 'plan.dat' with lines lc 3, 'trajectory.dat' with lines lc 2, 'sonar_meas009.dat' lc 1, 'ideal_meas009.dat' with lines lc 1, 'iteration009.dat' with dots lc 4

