#!/usr/bin/env python
#
# The following code is provided as is without ANY warranty.  You can
# do whatever you want with it, i.e. modify, redistribute,
# etc. without any restrictions. However, I would appreciate if you
# will mention Udacity folks as original authors and me as the one who
# adapted it for the real robotics vehicle.
#   Andrey Nechypurenko (andreynech@gmail.com)

# Standard set for Ice
import sys, traceback, Ice
import time

# Sensors and actuators interface definitions
Ice.loadSlice('-I/usr/share/Ice/slice --all sensors.ice')
Ice.loadSlice('-I/usr/share/Ice/slice --all actuators.ice')
import sensors, actuators

from math import *
import random
import csv
import lineutils

# some top level parameters

eps = 1.0e-10

# Measurements could be inprecise

bearing_noise = 0.3 # Noise parameter: should be included in sense function.
steering_noise = 0.1 # Noise parameter: should be included in move function.
distance_noise = 0.05 # Noise parameter: should be included in move function.

# List of wall segments [((start_x,start_y),(end_x,end_y)), (...)]
room_plan = []

# Horizontal (x) and vertical (y) ranges of the room
world_x_range = [0.0, 0.0]
world_y_range = [0.0, 0.0]

# Robot dimensions in meters
robot_length = 0.320
robot_width = 0.195

def write_meas(robot_pos, mes, f):

    f.write(str(robot_pos[0])+'\t'+str(robot_pos[1])+'\n')
    f.write(str(robot_pos[0] + mes[0] * cos(robot_pos[2]))+'\t'+str(robot_pos[1] + mes[0] * sin(robot_pos[2]))+'\n\n')

    f.write(str(robot_pos[0])+'\t'+str(robot_pos[1])+'\n')
    f.write(str(robot_pos[0] - mes[2] * cos(robot_pos[2]))+'\t'+str(robot_pos[1] - mes[2] * sin(robot_pos[2]))+'\n\n')

    f.write(str(robot_pos[0])+'\t'+str(robot_pos[1])+'\n')
    f.write(str(robot_pos[0] + mes[1] * cos(robot_pos[2]+pi/2))+'\t'+str(robot_pos[1] + mes[1] * sin(robot_pos[2]+pi/2))+'\n\n')

    f.write(str(robot_pos[0])+'\t'+str(robot_pos[1])+'\n')
    f.write(str(robot_pos[0] + mes[3] * cos(robot_pos[2]-pi/2))+'\t'+str(robot_pos[1] + mes[3] * sin(robot_pos[2]-pi/2))+'\n')


def dump_measurements(plan_list, robot_pos, iter_count, Z):
    f = open('sonar_meas'+str(iter_count).zfill(3)+'.dat', 'w')
    print "Sonars:", Z
    write_meas(robot_pos, Z, f)
    f.close()
    mes = lineutils.measurements(plan_list, robot_pos)
    f = open('ideal_meas'+str(iter_count).zfill(3)+'.dat', 'w')
    write_meas(robot_pos, mes, f)
    f.close()


# Generates motions list based on the trajectory specified as a set of
# waypoints. We assume, that initial position of the robot has
# coordinates as specified in the first line of the trajectory file
# and bearing (angle) is 0, i.e. robot is parallel to X axis facing to
# the right.
def generate_motions(trajectory_file_name):
    motions = []
    f = open(trajectory_file_name, 'r')

    prev_angle = 0.0
    b = None
    for line in f:
        x, y = line.split('\t')
        if b is not None:
            a = b
            b = (float(x), float(y))
            (x1, y1) = a
            (x2, y2) = b

            if x1 == x2:
                angle = pi/2
            else:
                angle = atan((y2-y1)/(x2-x1))

            distance = sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1))

            motions.append((angle - prev_angle,distance,(x2,y2,angle)))
            prev_angle = angle
        else:
            b = (float(x), float(y))

    f.close()
    return motions


# Request left and right wheel to travel sl and sr distance with
# defined speed. Speed is measured in encoder pulses per second.
# There are 333 pulses for complete wheel turn. So default speed 33
# corresponds to the 1/10 wheel revolution per second.
def make_motion_step(chassis, sl, sr, speed = 33):
    #print "Motion step:", sl, sr
    # Create command for the first wheel
    actuator_cmd1 = actuators.ActuatorData()
    actuator_cmd1.id = 0
    if abs(sl) < eps:
        actuator_cmd1.speed = 0
        actuator_cmd1.distance = 0
    else:
        actuator_cmd1.speed = speed if sl >= 0 else -speed
        # distance should be provided as a fraction of wheel rotation.
        # Our current wheel has diameter 0.06m
        actuator_cmd1.distance = abs(sl) / (3.14159 * 0.06)
    # Create command for the second wheel
    actuator_cmd2 = actuators.ActuatorData()
    actuator_cmd2.id = 1
    if abs(sr) < eps:
        actuator_cmd2.speed = 0
        actuator_cmd2.distance = 0
    else:
        actuator_cmd2.speed = speed if sr >= 0 else -speed
        actuator_cmd2.distance = abs(sr) / (3.14159 * 0.06)
   # print "Actual motion l/r:", (actuator_cmd1.speed, actuator_cmd1.distance), (actuator_cmd2.speed, actuator_cmd2.distance)
    try:
        # Send motor commands to vehicle
        chassis.setActuatorsAndWait([actuator_cmd1, actuator_cmd2])
    except Ice.Exception, ex:
        print ex


def measurement_prob(plan_list, particle, measurements):
    # exclude particles outside the room
    if not lineutils.point_inside_polygon(plan_list, particle):
        return 0.0

    # calculate the correct measurement
    predicted_measurements = lineutils.measurements(room_plan, particle)

    # compute errors
    error = 1.0
    for i in xrange(1, len(measurements)):
        error_mes = abs(measurements[i] - predicted_measurements[i])
        # update Gaussian
        error *= (exp(- (error_mes ** 2) / (bearing_noise ** 2) / 2.0) / sqrt(2.0 * pi * (bearing_noise ** 2)))

    return error

# Here we are using equations for two-wheels differential
# steering system as presented here 
# http://rossum.sourceforge.net/papers/DiffSteer/DiffSteer.html :
#
# S = (Sr+Sl)/2
# Theta = (Sr-Sl)/2b+theta0
# X = s*cos(theta)+x0
# Y = s*sin(theta)+y0
# Where Sr and Sl is the distance travelled by each wheel and b is the
# distance between wheels (vehicle width)
#
def move(particle, motion):
    (x,y,theta0) = particle
    (delta_theta, s, _) = motion
    delta_theta = random.gauss(delta_theta, steering_noise)
    s = random.gauss(s, distance_noise)
    
    theta = theta0 + delta_theta;
    x += s * cos(theta)
    y += s * sin(theta)

    return (x,y,theta)


# extract position from a particle set
def get_position(p):
    x = 0.0
    y = 0.0
    orientation = 0.0
    (_,_,init_orientation) = p[0]
    for (px,py,theta) in p:
        x += px
        y += py
        # orientation is tricky because it is cyclic. By normalizing
        # around the first particle we are somewhat more robust to
        # the 0=2pi problem
        orientation += (((theta - init_orientation + pi) % (2.0 * pi)) 
                        + init_orientation - pi)
    return (x / len(p), y / len(p), orientation / len(p))


def particle_filter(chassis, sensor_receiver, motions, N=10000):

    # Make particles

    world_x_size = abs(world_x_range[1] - world_x_range[0])
    world_y_size = abs(world_y_range[1] - world_y_range[0])
    p = []
    while len(p) < N:
        # particle is a vector (x,y,bearing)
        particle = (random.random() * world_x_size + world_x_range[0],
                    random.random() * world_y_size + world_y_range[0],
                    random.random() * 2.0 * pi)
        # add only particles inside our room
        if lineutils.point_inside_polygon(room_plan, particle):
            p.append(particle)

    # Update particles
    iter_count = 0;
    for motion in motions:

        # Motion update (prediction)
        p = map(lambda particle: move(particle, motion), p)

        # Now move the real vehicle
        #
        # Calculating path need to be made by each wheel for the turn.
        # We assuming that Sl = -Sr, i.e. in-place turn by rotating wheels
        # in the opposite directions
        (delta_theta, s, _) = motion
        sr = (delta_theta) * robot_width
        sl = -sr
        # Build commands for the vehicle to rotate for delta_theta and
        # then drive request distance s
        make_motion_step(chassis, sl, sr) # Rotate on place
        make_motion_step(chassis, s, s) # Drive forward

        # Measurement update
        Z = sensor_receiver.sonars

        w = map(lambda particle: measurement_prob(room_plan, particle, Z), p)

        # Resampling
        p2 = []
        index = int(random.random() * N)
        beta = 0.0
        mw = max(w)
        for i in range(N):
            beta += random.random() * 2.0 * mw
            while beta > w[index]:
                beta -= w[index]
                index = (index + 1) % N
            p2.append(p[index])
        p = p2

        # Dump current particle set to file for plotting
        (_,_,robot_ideal_pos) = motion
        dump_measurements(room_plan, robot_ideal_pos, iter_count, Z)        

        # Dump current particle set to file for plotting
        f = open('iteration'+str(iter_count).zfill(3)+'.dat', 'w')
        map(lambda (x,y,_): f.write(str(x)+'\t'+str(y)+'\n'), p)
        f.close()

        iter_count += 1
    
    return get_position(p)


# Callback interface with function which will be invoked every time
# the new sensor data is available from the robot
class SensorDataReceiverI(sensors.SensorFrameReceiver):

    def __init__(self):
        self.sonars = [-1.0]*4

    # Callback function with new sonar data as frame parameter.
    # Sonars reporting distance in centimeters. Need convert them to
    # meters by dividing by 100
    def nextSensorFrame(self, frame, current=None):
        for measurement in frame:
            if len(measurement.shortdata) > 0:
                if measurement.sensorid == 0xE0: # Front sonar
                    self.sonars[0] = float(measurement.shortdata[0]) / 100
                elif measurement.sensorid == 0xE4: # Right sonar
                    self.sonars[3] = float(measurement.shortdata[0]) / 100
                elif measurement.sensorid == 0xE6: # Back sonar
                    self.sonars[2] = float(measurement.shortdata[0]) / 100
                elif measurement.sensorid == 0xE2: # Left sonar
                    self.sonars[1] = float(measurement.shortdata[0]) / 100


# Main application class with run() function as an entry point to the
# application
class Client(Ice.Application):

    # Main application entry point
    def run(self, args):

        # Read room plan and create list with wall coordinates
        with open('plan.dat', 'r') as planfile:
            planreader = csv.reader(planfile, delimiter='\t')
            b = None
            for (x, y) in planreader:
                # Calculate world boundaries
                if float(x) < world_x_range[0]:
                    world_x_range[0] = float(x)
                elif float(x) > world_x_range[1]:
                    world_x_range[1] = float(x)

                if float(y) < world_y_range[0]:
                    world_y_range[0] = float(y)
                elif float(y) > world_y_range[1]:
                    world_y_range[1] = float(y)

                # Construct wall segment and add to the room_plan
                if b is not None:
                    a = b
                    b = (float(x), float(y))
                    room_plan.append((a,b))
                else:
                    b = (float(x), float(y))

        # Create proxy interface to our robot's chassis (left and
        # right wheels) using parameters (host, port, etc.) specified
        # in the configuration file as Chassis.proxy property
        self.chassis = actuators.ActuatorGroupPrx.checkedCast(
            self.communicator().propertyToProxy('Chassis.proxy'))
        if not self.chassis:
            print self.appName() + ": invalid chassis proxy"
            return 1
        admin = self.chassis.getStateInterface()
        admin.start()

        # Create proxy interface to on-board sonars
        sonars = sensors.SensorGroupPrx.checkedCast(
            self.communicator().propertyToProxy('Sonars.proxy'))
        if not sonars:
            print self.appName() + ": invalid sonars proxy"
            return 1

        # Register and initialize sensor data callback receiver 
        adapter = self.communicator().createObjectAdapter("Callback.Client")
        self.receiver = SensorDataReceiverI()
        receiver_identity = self.communicator().stringToIdentity("callbackReceiver")
        receiverObj = adapter.add(self.receiver, receiver_identity)
        receiverPrx = sensors.SensorFrameReceiverPrx.uncheckedCast(receiverObj)
        adapter.activate()

        sonars.setSensorReceiver(receiverPrx)
        admin = sonars.getStateInterface()
        admin.start()

        # Wait untill we get the first sensor measurements
        tries = 0
        while self.receiver.sonars[0] < 0 and tries < 3:
            print 'waiting for reasonable sensor data...', '\tSonar: ', self.receiver.sonars
            tries += 1
            time.sleep(1)
        if self.receiver.sonars[0] < 0:
            print 'No reasonable sensor data received. Exiting.'
            return 2
        

        # Make some motions and estimate resulting position using our particle filter
        motions = generate_motions('trajectory.dat')
        #print "Motions:", motions
        estimated_position = particle_filter(self.chassis, 
                                             self.receiver, 
                                             motions)
        print "Estimated final position after motion: ", estimated_position


if __name__ == "__main__":
    app = Client()
    app.main(sys.argv, "client.config")
