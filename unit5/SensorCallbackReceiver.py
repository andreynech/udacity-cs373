#!/usr/bin/env python
#
# Copyright (c) 2012 Andrey Nechypurenko
# See the file LICENSE for copying permission.

import Queue
import gst

# Standard set for Ice
import sys, traceback, Ice

# Sensors and actuators interface definitions
Ice.loadSlice('-I/usr/share/Ice/slice --all sensors.ice')
Ice.loadSlice('-I/usr/share/Ice/slice --all actuators.ice')
import sensors, actuators



# Callback interface implementation to receive video stream
class Callback(sensors.SensorFrameReceiver):

    def __init__(self, frame_queue):
        self.frame_queue = frame_queue


    def nextSensorFrame(self, frame, current=None):
        if len(frame) is 0:
            return
        if len(frame[0].bytedata) is 0:
            return
        # Enqueue received frame
        self.frame_queue.put(frame[0].bytedata)
 

# Main server class
class SensorCallbackReceiver(Ice.Application):

    def __init__(self, args):
        print("Ice runtime version %s" % Ice.stringVersion())
        try:
            if len(sys.argv) == 1:
                print 'using default configuration settings: --Ice.Config=driver-console.config'
                sys.argv.append('--Ice.Config=driver-console.config')
            self.communicator = Ice.initialize(sys.argv)

            # Create proxy interface to our robot's chassis (left and
            # right wheels) using parameters (host, port, etc.) specified
            # in the configuration file as Chassis.proxy property
            self.chassis = actuators.ActuatorGroupPrx.checkedCast(
                self.communicator.propertyToProxy('Chassis.proxy'))
            if not self.chassis:
                print self.appName() + ": invalid chassis proxy"
                return 1
            admin = self.chassis.getStateInterface()
            admin.start()

            # Connect to video sensor
            self.sensor = sensors.SensorGroupPrx.checkedCast(\
                self.communicator.propertyToProxy('Sensor.proxy'))
            if not self.sensor:
                print args[0] + ": invalid sensor proxy"
                return

            self.frame_queue = Queue.Queue()

            # Make callback instance remotely visible
            cb = Callback(self.frame_queue)
            adapter = self.communicator.createObjectAdapter("Video-callback")
            cbObj = adapter.add(cb, self.communicator.stringToIdentity("cockpit"))
            adapter.activate()
            print 'Sensor callback is online'

            # Instruct sensor to send data to our Callback instance
            cbPrx = sensors.SensorFrameReceiverPrx.uncheckedCast(cbObj)
            sensor_descr = self.sensor.getSensorDescription()
            print"Sensor " + str(sensor_descr[0].id) + ": " + sensor_descr[0].description + ". Model: " + sensor_descr[0].vendorid
            print 'Callback server initialization complete.'

            self.sensor.setSensorReceiver(cbPrx)
            self.sensor_state = self.sensor.getStateInterface()
            self.sensor_state.start()

        except:
            traceback.print_exc()
            return


    def get_chassis_actuators(self):
        return self.chassis


    def needdata(self, src, length):
        frame = self.frame_queue.get()
        if len(frame) == 0:
            src.emit('end-of-stream')
        else:
            src.emit('push-buffer', gst.Buffer(frame))
        self.frame_queue.task_done()


    def cleanup(self):
        # Shut down nicely
        print 'Sensor callback cleaning up...'
        self.frame_queue.put([]) # to provoke end-of-stream bus message
        self.sensor_state.stop()
        self.sensor.cleanSensorReceiver()
        self.communicator.destroy()
