#!/usr/bin/env python
#
# Copyright (c) 2012 Andrey Nechypurenko (andreynech@gmail.com)
#
# The following code is provided as is without ANY warranty.  You can
# do whatever you want with it, i.e. modify, redistribute,
# etc. without any restrictions. However, I would appreciate if you
# will mention me as original author.

import sys, Ice
import time

# Load automatically generated interfaces to communicate with the
# robot
Ice.loadSlice("--all vehicle.ice")
import comtypes
import vehicleadmin
import vehicle


# Our map
world = ['wall', 'door', 'door', 'wall', 'wall', 'door', 'wall']
    
# Sensor and motion constants
pHit = 0.6
pMiss = 0.2
pExact = 0.8
pOvershoot = 0.1
pUndershoot = 0.1


# Callback interface with function which will be invoked every time
# the new sensor data is available from the robot
class SensorDataReceiverI(vehicle.SensorFrameReceiver):

    # Compas returns the value in the 0-255 range. The following
    # coefficient will be used to convert the measurement to degrees
    angleCoefficient = 360.0 / 255.0

    def __init__(self):
        self.sonar = -1
        self.compass = -1

    # Callback function with new sensor data as frame parameter
    def nextSensorFrame(self, frame, current=None):
        for measurement in frame:
            if measurement.sensorid == 2 and len(measurement.intdata) > 0: # Sonar
                self.sonar = measurement.intdata[0]
            elif measurement.sensorid == 3 and len(measurement.intdata) > 0: # Compass
                self.compass =  SensorDataReceiverI.angleCoefficient * measurement.intdata[0]


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

  
    # Stop motors
    def stop(self):
        # stop
        frame = []
        frame.append(vehicle.ActuatorData(0, 50))
        frame.append(vehicle.ActuatorData(1, 50))
        self.unit.setActuators(frame)
        time.sleep(0.2)


    # Since our robot has one sonar on the front, we need to turn
    # towards the wall to make measurements, then turn back and drive
    # forward. We assume, that we start at the position facing towards
    # the wall. So the motion step is: turn left, drive forward, turn
    # right back to the wall.
    def makeMotionStep(self):
        # turn 90 degrees left
        frame = []
        frame.append(vehicle.ActuatorData(0, 0))
        frame.append(vehicle.ActuatorData(1, 70))
        self.unit.setActuators(frame)
        time.sleep(1.787)

        self.stop()

        # Drive forward
        frame = []
        frame.append(vehicle.ActuatorData(0, 50))
        frame.append(vehicle.ActuatorData(1, 70))
        self.unit.setActuators(frame)
        time.sleep(2.0)

        self.stop()

        # turn 90 degrees right
        frame = []
        frame.append(vehicle.ActuatorData(0, 100))
        frame.append(vehicle.ActuatorData(1, 70))
        self.unit.setActuators(frame)
        time.sleep(1.43)

        self.stop()


    # Main application entry point
    def run(self, args):

        # Create proxy interface to our robot (Unit) using parameters
        # (host, port, etc.)  specified in the configuration file as
        # Unit.Proxy property
        self.unit = vehicle.RemoteVehiclePrx.checkedCast(
            self.communicator().propertyToProxy('Unit.Proxy'))
        if not self.unit:
            print self.appName() + ": invalid unit proxy"
            return 1

        # Register and initialize sensor data callback receiver 
        adapter = self.communicator().createObjectAdapter("Callback.Client")
        self.receiver = SensorDataReceiverI()
        receiver_identity = self.communicator().stringToIdentity("callbackReceiver")
        adapter.add(self.receiver, receiver_identity)
        adapter.activate()

        receiverPrx = vehicle.SensorFrameReceiverPrx.uncheckedCast(
            adapter.createProxy(receiver_identity))
        self.unit.setSensorReceiver(receiverPrx)

        # List and turn on (start) all sensors available on the robot
        print 'Unit sensors:'
        for sensor in self.unit.getSensorList():
            sd = sensor.getDescription()
            print("%i\t%s\t%s" % (sd.id, sd.description, sd.vendorid))
            if sd.id != 0:
                admin = sensor.getAdminInterface()
                admin.start()

        # List and turn on (start) all actuators (motors) available on
        # the robot
        print 'Unit actuators:'
        for actuator in self.unit.getActuatorList():
            ad = actuator.getDescription()
            print("%i\t%s\t%s" % (ad.id, ad.description, ad.type))
            admin = actuator.getAdminInterface()
            admin.start()

        # Stop all motors. Just for the case... :-)
        self.stop()

        # Wait untill we get the first sensor measurements
        tries = 0
        while self.receiver.sonar == -1 and tries < 3:
            print 'waiting for reasonable sensor data...', '\tSonar: ', self.receiver.sonar
            tries += 1
            time.sleep(1)
        if self.receiver.sonar == -1:
            print 'No reasonable sensor data received. Exiting.'
            return

        # Ok, now we are ready to start sense/move cycle.
        # Let's make 5 steps and see if we can find our position.
        for i in range(5):
            # Just assume that if sonar is sensing something in front
            # of us, then it should be wall, otherwise we are facing
            # the door
            print 'Sonar: ', self.receiver.sonar
            if self.receiver.sonar < 50:
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

        
if __name__ == "__main__":
    app = Client()
    app.main(sys.argv, "client.config")
