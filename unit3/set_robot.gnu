#set_robot sets a robot as an gnuplot object
#   Usage: call 'set_robot.gnu' 'x' 'y' 'phi' 'size'
#
#   Input parameters:
#       x, y        - robot position
#       phi         - orientation of the robot
#       size        - size of the robot
#
#   set_robot sets a single gnuplot object in the
#   form of a robot at the given position and for the given orientation.
#   For every added robot the global variable object_number is counted one
#   up and is accessable in your gnuplot code.

# AUTHOR: Hagen Wierstorf

# Checking if we have enough input parameters
#if ($# != 4) print 'set_robot needs 4 input parameters'; exit

# Getting the input parameters
x = $0
y = $1
#p = $2+pi/2.0
p = $2
a = $3

# Initialize an object number
if (!exists("object_number")) object_number = 1;

# Set the robot at the given position and rotate it by a rotation matrix:
set object object_number polygon from \
-a*cos(p)+a/2*sin(p)+x+a/2,   -a*sin(p)-a/2*cos(p)+y+a/2    to \
-a*cos(p)-a/2*sin(p)+x+a/2,   -a*sin(p)+a/2*cos(p)+y+a/2    to \
-a/2*cos(p)-a/2*sin(p)+x+a/2, -a/2*sin(p)+a/2*cos(p)+y+a/2  to \
-a/2*cos(p)-a/6*sin(p)+x+a/2, -a/2*sin(p)+a/6*cos(p)+y+a/2  to \
0*cos(p)-a/2*sin(p)+x+a/2,    0*sin(p)+a/2*cos(p)+y+a/2     to \
0*cos(p)+a/2*sin(p)+x+a/2,    0*sin(p)-a/2*cos(p)+y+a/2     to \
-a/2*cos(p)+a/6*sin(p)+x+a/2, -a/2*sin(p)-a/6*cos(p)+y+a/2  to \
-a/2*cos(p)+a/2*sin(p)+x+a/2, -a/2*sin(p)-a/2*cos(p)+y+a/2  to \
-a*cos(p)+a/2*sin(p)+x+a/2,   -a*sin(p)-a/2*cos(p)+y+a/2

# Set the color etc.
set object object_number fc rgb "black" fillstyle solid 1 lw 1 front

# Count the object number
object_number = object_number+1