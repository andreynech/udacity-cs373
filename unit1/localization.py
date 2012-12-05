#!/usr/bin/env python
#
# Copyright (c) 2012 Andrey Nechypurenko (andreynech@gmail.com)
#
# The following code is provided as is without ANY warranty.  You can
# do whatever you want with it, i.e. modify, redistribute,
# etc. without any restrictions. However, I would appreciate if you
# will mention me as original author.

# Standard set for Ice
import sys, traceback, Ice
import time

# Sensors and actuators interface definitions
Ice.loadSlice('-I/usr/share/Ice/slice --all sensors.ice')
Ice.loadSlice('-I/usr/share/Ice/slice --all actuators.ice')
import sensors, actuators


# Our map
world = ['wall', 'wall', 'wall', 'door', 'wall', 'door', 'door', 'wall', 'wall', 'wall']
    
# Sensor and motion constants
pHit = 0.6
pMiss = 0.2
pExact = 0.8
pOvershoot = 0.1
pUndershoot = 0.1


# Callback interface with function which will be invoked every time
# the new sensor data is available from the robot
class SensorDataReceiverI(sensors.SensorFrameReceiver):

    def __init__(self):
        self.sonar = -1

    # Callback function with new sensor data as frame parameter
    def nextSensorFrame(self, frame, current=None):
        for measurement in frame:
            if measurement.sensorid == 0xE2 and len(measurement.shortdata) > 0: # Sonar
                self.sonar = measurement.shortdata[0]


# Main application class with run() function as an entry point to the
# application
class Client(Ice.Application):

    def __init__(self):
        # Position esimation array with probabilities
        self.estimatedPosition = [ 1.0/len(world) ] * len(world)


    # Sense and move functions are copy/pasted from the Unit1 of the
    # CS373 class by udacity.com without changes

    def sense(self, p, Z):
        q=[]
        for i in range(len(p)):
            hit = (Z == world[i])
            q.append(p[i] * (hit * pHit + (1-hit) * pMiss))
        s = sum(q)
        for i in range(len(q)):
            q[i] = q[i] / s
        return q

    
    def move(self, p, U):
        q = []
        for i in range(len(p)):
            s = pExact * p[(i-U) % len(p)]
            s = s + pOvershoot * p[(i-U-1) % len(p)]
            s = s + pUndershoot * p[(i-U+1) % len(p)]
            q.append(s)
        return q

    # Drive forward the A4-page width (210mm = 0.21m)
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

        # Ok, now we are ready to start sense/move cycle.
        # Let's make 5 steps and see if we can find our position.
        for i in range(9):
            # Just assume that if sonar is sensing something in front
            # of us, then it should be wall, otherwise we are facing
            # the door
            print 'Sonar: ', self.receiver.sonar
            if self.receiver.sonar < 30:
                Z = 'wall'
            else:
                Z = 'door'
            # Process received sensor data
            self.estimatedPosition = self.sense(self.estimatedPosition, Z)
            # Make physical movement
            self.makeMotionStep()
            # Update estimated position for the movement
            self.estimatedPosition = self.move(self.estimatedPosition, 1)
            print i, ', ', Z, self.estimatedPosition
            time.sleep(1)

        
if __name__ == "__main__":
    app = Client()
    app.main(sys.argv, "client.config")
