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

# Write a function 'filter' that implements a multi-
# dimensional Kalman Filter for the example given

from math import *

class matrix:
    
    # implements basic operations of a matrix class
    
    def __init__(self, value):
        self.value = value
        self.dimx = len(value)
        self.dimy = len(value[0])
        if value == [[]]:
            self.dimx = 0
    
    def zero(self, dimx, dimy):
        # check if valid dimensions
        if dimx < 1 or dimy < 1:
            raise ValueError, "Invalid size of matrix"
        else:
            self.dimx = dimx
            self.dimy = dimy
            self.value = [[0 for row in range(dimy)] for col in range(dimx)]
    
    def identity(self, dim):
        # check if valid dimension
        if dim < 1:
            raise ValueError, "Invalid size of matrix"
        else:
            self.dimx = dim
            self.dimy = dim
            self.value = [[0 for row in range(dim)] for col in range(dim)]
            for i in range(dim):
                self.value[i][i] = 1
    
    def show(self):
        for i in range(self.dimx):
            print self.value[i]
        print ' '
    
    def __add__(self, other):
        # check if correct dimensions
        if self.dimx != other.dimx or self.dimy != other.dimy:
            raise ValueError, "Matrices must be of equal dimensions to add"
        else:
            # add if correct dimensions
            res = matrix([[]])
            res.zero(self.dimx, self.dimy)
            for i in range(self.dimx):
                for j in range(self.dimy):
                    res.value[i][j] = self.value[i][j] + other.value[i][j]
            return res
    
    def __sub__(self, other):
        # check if correct dimensions
        if self.dimx != other.dimx or self.dimy != other.dimy:
            raise ValueError, "Matrices must be of equal dimensions to subtract"
        else:
            # subtract if correct dimensions
            res = matrix([[]])
            res.zero(self.dimx, self.dimy)
            for i in range(self.dimx):
                for j in range(self.dimy):
                    res.value[i][j] = self.value[i][j] - other.value[i][j]
            return res
    
    def __mul__(self, other):
        # check if correct dimensions
        if self.dimy != other.dimx:
            raise ValueError, "Matrices must be m*n and n*p to multiply"
        else:
            # subtract if correct dimensions
            res = matrix([[]])
            res.zero(self.dimx, other.dimy)
            for i in range(self.dimx):
                for j in range(other.dimy):
                    for k in range(self.dimy):
                        res.value[i][j] += self.value[i][k] * other.value[k][j]
            return res
    
    def transpose(self):
        # compute transpose
        res = matrix([[]])
        res.zero(self.dimy, self.dimx)
        for i in range(self.dimx):
            for j in range(self.dimy):
                res.value[j][i] = self.value[i][j]
        return res
    
    # Thanks to Ernesto P. Adorio for use of Cholesky and CholeskyInverse functions
    
    def Cholesky(self, ztol=1.0e-5):
        # Computes the upper triangular Cholesky factorization of
        # a positive definite matrix.
        res = matrix([[]])
        res.zero(self.dimx, self.dimx)
        
        for i in range(self.dimx):
            S = sum([(res.value[k][i])**2 for k in range(i)])
            d = self.value[i][i] - S
            if abs(d) < ztol:
                res.value[i][i] = 0.0
            else:
                if d < 0.0:
                    raise ValueError, "Matrix not positive-definite"
                res.value[i][i] = sqrt(d)
            for j in range(i+1, self.dimx):
                S = sum([res.value[k][i] * res.value[k][j] for k in range(self.dimx)])
                if abs(S) < ztol:
                    S = 0.0
                res.value[i][j] = (self.value[i][j] - S)/res.value[i][i]
        return res
    
    def CholeskyInverse(self):
        # Computes inverse of matrix given its Cholesky upper Triangular
        # decomposition of matrix.
        res = matrix([[]])
        res.zero(self.dimx, self.dimx)
        
        # Backward step for inverse.
        for j in reversed(range(self.dimx)):
            tjj = self.value[j][j]
            S = sum([self.value[j][k]*res.value[j][k] for k in range(j+1, self.dimx)])
            res.value[j][j] = 1.0/tjj**2 - S/tjj
            for i in reversed(range(j)):
                res.value[j][i] = res.value[i][j] = -sum([self.value[i][k]*res.value[k][j] for k in range(i+1, self.dimx)])/self.value[i][i]
        return res
    
    def inverse(self):
        aux = self.Cholesky()
        res = aux.CholeskyInverse()
        return res
    
    def __repr__(self):
        return repr(self.value)



# Callback interface with function which will be invoked every time
# the new sensor data is available from the robot
class SensorDataReceiverI(sensors.SensorFrameReceiver):

    def __init__(self):
        self.sonar = -1

    # Callback function with new sensor data as frame parameter
    def nextSensorFrame(self, frame, current=None):
        for measurement in frame:
            if measurement.sensorid == 0xE6 and len(measurement.shortdata) > 0: # Sonar
                self.sonar = measurement.shortdata[0]


# Main application class with run() function as an entry point to the
# application
class Client(Ice.Application):

    # Drive forward 210mm = 0.21m
    def makeMotionStep(self):
        # Create command for the first wheel
        actuator_cmd1 = actuators.ActuatorData()
        actuator_cmd1.id = 0
        actuator_cmd1.speed = 33 # Rotations per second. We are not hurry..
        # distance should be provided as a fraction of wheel rotation.
        # Our current wheel has diameter 0.06m
        actuator_cmd1.distance = 0.21 / (3.14159 * 0.06)
        # Create the same command for the second wheel
        actuator_cmd2 = actuators.ActuatorData()
        actuator_cmd2.id = 1
        actuator_cmd2.speed = actuator_cmd1.speed
        actuator_cmd2.distance = actuator_cmd1.distance
        try:
            # Send motor commands to vehicle
            self.chassis.setActuatorsAndWait([actuator_cmd1, actuator_cmd2])
        except Ice.Exception, ex:
            print ex


    ########################################

    # Implement the filter function below

    def filter(self, x, P):
        I = matrix([[0., 0.], [0., 0.]])
        I.identity(I.dimx)
        for n in range(4):
            # measurement update
            y = self.receiver.sonar - (H * x).value[0][0]
            S = H * P * H.transpose() + R
            K = P * H.transpose() * S.inverse()
            x = x + matrix([[K.value[0][0]*y],[K.value[1][0]*y]])
            P = (I - K * H) * P
        
            # prediction
            x = F * x + u
            P = F * P * F.transpose()

            print 'x= '
            x.show()
            print 'P= '
            P.show()
            self.makeMotionStep()
            time.sleep(1)


    # Main application entry point
    def run(self, args):

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
        while self.receiver.sonar == -1 and tries < 3:
            print 'waiting for reasonable sensor data...', '\tSonar: ', self.receiver.sonar
            tries += 1
            time.sleep(1)
        if self.receiver.sonar == -1:
            print 'No reasonable sensor data received. Exiting.'
            return 2

        ########################################

        self.filter(x, P)

x = matrix([[0.], [0.]]) # initial state (location and velocity)
P = matrix([[1000., 0.], [0., 1000.]]) # initial uncertainty
u = matrix([[0.], [0.]]) # external motion
F = matrix([[1., 1.], [0, 1.]]) # next state function
H = matrix([[1., 0.]]) # measurement function
R = matrix([[1.]]) # measurement uncertainty
I = matrix([[1., 0.], [0., 1.]]) # identity matrix


if __name__ == "__main__":
    app = Client()
    app.main(sys.argv, "client.config")
