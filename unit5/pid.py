import threading
import time

# Standard set for Ice
import sys, traceback, Ice

# Sensors and actuators interface definitions
Ice.loadSlice('-I/usr/share/Ice/slice --all actuators.ice')
import actuators


class Controller:

    def __init__(self, params=[0.0, 0.0, 0.0], min_val=-200.0, max_val=200.0):
        self.params = params
        self.max_ctl = max_val
        self.min_ctl = min_val
        self.buffer_x = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.buffer_y = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.buffer_r = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.cx = 0.0
        self.cy = 0.0
        self.r = 0.0
        self.lock = threading.Lock()


    def target_callback(self, center_x, center_y, radius):
        with self.lock:
            self.buffer_x = self.buffer_x[1:]
            self.buffer_x.append(center_x)
            self.buffer_y = self.buffer_y[1:]
            self.buffer_y.append(center_y)
            self.buffer_r = self.buffer_r[1:]
            self.buffer_r.append(radius)
        
            # Use triangular smoothing to reduce high-frequency noise
            self.cx = (self.buffer_x[0] + 
                       2 * self.buffer_x[1] + 
                       3 * self.buffer_x[2] + 
                       2 * self.buffer_x[3] + 
                       self.buffer_x[4]) / 9

            self.cy = (self.buffer_y[0] + 
                       2 * self.buffer_y[1] + 
                       3 * self.buffer_y[2] + 
                       2 * self.buffer_y[3] + 
                       self.buffer_y[4]) / 9

            self.r = (self.buffer_r[0] + 
                      2 * self.buffer_r[1] + 
                      3 * self.buffer_r[2] + 
                      2 * self.buffer_r[3] + 
                      self.buffer_r[4]) / 9


    def reset(self, params, min_val, max_val):
        with self.lock:
            self.params = params
            self.max_ctl = max_val
            self.min_ctl = min_val
            self.int_crosstrack_error = 0.0
            self.prev_crosstrack_error = self.cx - 320


    def current_measurements(self):
        with self.lock:
            ret = (self.cx, self.cy, self.r)

        return ret


    def target_lost(self):
        res = True
        for i in range(5):
            with self.lock:
                if self.r > 20:
                    res = False
                    break
        return res


    def get_control_value(self):
        with self.lock:
            # Calculate error. Only horizontal movement is considered for
            # this robot
            crosstrack_error = self.cx - 320

            # Calculate control output
            diff_crosstrack_error = crosstrack_error \
                - self.prev_crosstrack_error 
            self.prev_crosstrack_error = crosstrack_error
            self.int_crosstrack_error += crosstrack_error

            ctl = - self.params[0] * crosstrack_error  \
                - self.params[1] * diff_crosstrack_error \
                - self.int_crosstrack_error * self.params[2]

            # Clamp the control value between specified maximum and
            # minimum
            if ctl < self.min_ctl:
                ctl = self.min_ctl
            elif ctl > self.max_ctl:
                ctl = self.max_ctl

        return ctl



class Example(threading.Thread):


    def setup(self, pid_controller, actuators):
        self.pid_controller = pid_controller
        self.actuators = actuators
        self.should_stop = False
        self.run_cnt = 0
        self.last_direction = 1


    # Request left and right wheel to travel sl and sr distance with
    # defined speed. Speed is measured in encoder pulses per second.
    # There are 333 pulses for complete wheel turn.
    def make_motion_step(self, speedl, speedr, distance=2.0):
        eps = 0.5
        # print "Motion step:", speedl, speedr
        # Create command for the first wheel
        actuator_cmd1 = actuators.ActuatorData()
        actuator_cmd1.id = 0
        if abs(speedl) < eps or abs(distance) < eps/10:
            actuator_cmd1.speed = 0
            actuator_cmd1.distance = 0
        else:
            actuator_cmd1.speed = speedl
            # distance should be provided as a fraction of wheel rotation.
            # Our current wheel has diameter 0.06m
            actuator_cmd1.distance = abs(distance) / (3.14159 * 0.06)
        # Create command for the second wheel
        actuator_cmd2 = actuators.ActuatorData()
        actuator_cmd2.id = 1
        if abs(speedr) < eps or abs(distance) < eps/10:
            actuator_cmd2.speed = 0
            actuator_cmd2.distance = 0
        else:
            actuator_cmd2.speed = speedr
            actuator_cmd2.distance = abs(distance) / (3.14159 * 0.06)
        # print "Actual motion l/r:", (actuator_cmd1.speed, actuator_cmd1.distance), (actuator_cmd2.speed, actuator_cmd2.distance)
        try:
            # Send motor commands to vehicle
            self.actuators.setActuatorsNoWait([actuator_cmd1, actuator_cmd2])
        except Ice.Exception, ex:
            print ex
        time.sleep(0.05)


    # Rotate until we find the circle marker. If after 5 seconds we
    # still can not lock the marker, then something went wrong and we
    # return False. Otherwise the function returns True
    def rotate_until(self, until_find, max_wait_time=10):
        # print "Motion step:", speedl, speedr
        # Create command for the first wheel
        actuator_cmd1 = actuators.ActuatorData()
        actuator_cmd1.id = 0
        actuator_cmd1.speed = 5 * self.last_direction
        actuator_cmd1.distance = 4.0 / (3.14159 * 0.06)
        # Create command for the second wheel
        actuator_cmd2 = actuators.ActuatorData()
        actuator_cmd2.id = 1
        actuator_cmd2.speed = -actuator_cmd1.speed
        actuator_cmd2.distance = actuator_cmd1.distance
        # print "Actual motion l/r:", (actuator_cmd1.speed, actuator_cmd1.distance), (actuator_cmd2.speed, actuator_cmd2.distance)
        try:
            # Send motor commands to vehicle
            self.actuators.setActuatorsNoWait([actuator_cmd1, actuator_cmd2])
        except Ice.Exception, ex:
            print ex

        # Check periodically whether the marker was or was not found
        # (depending on the value of until_find parameter
        period = 0.2
        wait_time = 0.0
        while not self.should_stop and \
                (self.pid_controller.target_lost() if until_find else (not self.pid_controller.target_lost())):
            time.sleep(period)
            wait_time += period
            if wait_time > max_wait_time:
                self.make_motion_step(0, 0)
                return False;
        self.make_motion_step(0, 0)
        return True


    def cleanup(self):
        print "Example cleaning up"
        self.should_stop = True
        time.sleep(1)
        # Stop motors
        self.make_motion_step(0, 0)


    def test_run(self, params, iterations=50):
        print "Trying params:", params
        (cx, _, radius) = self.pid_controller.current_measurements()
        # Turn away from visible marker
        if radius >= 100:
            self.rotate_until(False)
        # Continue rotating until we find another marker
        self.rotate_until(True)

        # Set new PID parameters
        self.pid_controller.reset(params, -200, 200)

        N = iterations
        err = 0.0
        i = 0

        # Move N*2 steps towards the marker using new PID parameters
        (cx, _, radius) = self.pid_controller.current_measurements()
        while i < N * 2 and not self.should_stop:
            speedl = 5
            speedr = 5
            while radius < 100 and not self.should_stop and not self.pid_controller.target_lost():
                i += 1
                ctl = float(self.pid_controller.get_control_value()) / 100
                speedl -= ctl
                if speedl > 10:
                    speedl = 10
                elif speedl < 5:
                    speedl = 5
                speedr += ctl
                if speedr > 10:
                    speedr = 10
                elif speedr < 5:
                    speedr = 5
                self.last_direction = 1 if speedr > speedl else -1
                # Drive 0.05m with updated wheels speed
                # print "Ctl: ", ctl, "speed:", (speedl, speedr)
                self.make_motion_step(speedl, speedr, 2.0)
                (cx, _, radius) = self.pid_controller.current_measurements()
                crosstrack_error = cx - 320
                if i >= N:
                    err += (crosstrack_error ** 2)
            time.sleep(0.2)
            (_, _, radius) = self.pid_controller.current_measurements()
            if radius >= 100:
                # We are close enough to the wall with marker. Turn
                # away and continue test drive to the opposite marker
                # until i < N*2
                self.rotate_until(False)
                self.rotate_until(True)
            else:
                # We lost the marker. Try to find it again
                self.rotate_until(True)
        self.run_cnt += 1
        print self.run_cnt, "test run completed. Error:", err / float(N)
        return err / float(N)


    # Make this tolerance bigger if you are timing out!
    def twiddle(self, tol = 0.2): 

        params = [0.0, 0.0, 0.0]
        dp = [1.0]*len(params)
        lp = len(params)
        self.best_err = self.test_run(params)
        self.best_params = params
        while not self.should_stop and sum(dp) > tol:
            for i in range(lp):
                params[i] += dp[i]
                err = self.test_run(params)
                if err < self.best_err:
                    self.best_err = err
                    self.best_params = params
                    dp[i] *= 1.2
                else:
                    params[i] -= 2.0 * dp[i]
                    err = self.test_run(params)
                    if err < self.best_err:
                        self.best_err = err
                        self.best_params = params
                        dp[i] *= 1.2
                    else:
                        params[i] += dp[i]
                        dp[i] *= 0.8
                print "Best error:", self.best_err, "params:", self.best_params

        print 'Final params: ', params
        return self.test_run(params)


    def run(self):

        # Stop motors
        self.make_motion_step(0, 0)

        self.twiddle(0.01)
        err = self.test_run(self.best_params, 100)
        print "Last error:", err
