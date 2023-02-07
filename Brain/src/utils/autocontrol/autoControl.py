import os
import copy
from src.templates.workerprocess import WorkerProcess
import time
import numpy as np
import math
from threading import Thread
class RcBrainConfigParams:
    def __init__(self, maxSteerAngle, maxSpeed, steerAngleStep, speedStep, kpStep, kiStep, kdStep):
        """ The aim of the class is to group the configuration parameters for the rcBrain. 

        Parameters
        ----------
        maxSteerAngle : float
            Maximum value of steering angle
        maxSpeed : float
            Maximum value of speed
        steerAngleStep : float
            The step value of steering angle
        speedStep : [type]
            The step value of speed
        """
        self.maxSteerAngle = maxSteerAngle
        self.maxSpeed = maxSpeed
        self.steerAngleStep = steerAngleStep
        self.speedStep = speedStep
        self.kpStep = kpStep
        self.kiStep = kiStep
        self.kdStep = kdStep


class AutoControl(WorkerProcess):

    # ===================================== INIT =========================================
    def __init__(self,inPs,outPs):
        """It's an example to process the keyboard events and convert them to commands for the robot.
        """
        self.img_shape = (480, 640)
        self.speed = 0.0
        self.steerAngle = 0.0
        self.pida = False
        self.pids_kp = 0.115000
        self.pids_ki = 0.810000
        self.pids_kd = 0.000222
        self.pids_tf = 0.040000

        # ----------------- CONSTANT VALUES --------------------
        # this values do not change
        self.parameterIncrement = 0.1
        self.limit_configParam = RcBrainConfigParams(21.0, 30.0, 3.0, 4.0, 0.001, 0.001, 0.000001)

        self.startSpeed = 9.0
        self.startSteerAngle = 1.0

        # ----------------- DEFAULT VALUES ----------------------
        # when the RC is reset, this are the default values
        self.default_configParam = RcBrainConfigParams(20.5, 20.0, 1.5, 2.0, 0.001, 0.001, 0.000001)

        # ----------------- PARAMETERS -------------------------
        # this parameter can be modified via key events.
        self.configParam = copy.deepcopy(self.default_configParam)

        # ----------------- DIRECTION SIGNALS STATES -----------
        self.currentState = [False, False, False, False, False, False, False,
                             False]  # UP, DOWN , LEFT, RIGHT, BRAKE, PIDActive, PIDSvalues, SteerRelease
        self.angles_to_store = 3
        self.angle_weights = np.array([0.8, 0.15, 0.05])
        self.last_n_angles = np.zeros(self.angles_to_store)
        self.index = 0
        self.current_steer_angle = 0.0
        super(AutoControl, self).__init__(inPs, outPs)
    def run(self):
        command_speed = {"action": "1", "speed": 0.18}
        self.outPs[0].send(command_speed)
        super(AutoControl, self).run()
    def driver(self,t,speed=0.1):
        command_steer = {"action": "2", "steerAngle": self.steerAngle}
        # self.outPs[0].send(command_steer)
        for i in range(0,int(10*t)):
            self.outPs[0].send(command_steer)
    def _init_threads(self):
        """Initialize the read thread to transmite the received messages to other processes.
        """
        readTh = Thread(name='Control AUto', target=self._control_thread, args=(self.inPs,self.outPs,))
        self.threads.append(readTh)
    def chay_mu(self, inPs, outPs):
        command_speed = {"action": "1", "speed": 0.16}
        self.outPs[0].send(command_speed)

    def _control_thread(self, inPs, outPs):
        """Read the image from input stream, process it and send lane information

        Parameters
        ----------
        inPs : list(Pipe)
            0 - lanes
            1 - objects
        outPs : list(Pipe)
            0 - high-level instructions
        """
        # time.sleep(5)
        # self.bump()
        while True:
            try:
                # if self.start_moving == True and self.testing_objects == False:
                #     self.bump(speed=0.12)

                # information from the lane detector
                stamp,speed, steer_angle, = inPs[0].recv()
                print(steer_angle)
                self.steerAngle = steer_angle
                self.driver(0.1)
            except Exception as e:
                print("AutonomousController failed to obtain objects:", e, "\n")
                self.steerAngle = 0.0
                pass

    # ======================= LANE KEEPING =======================
    def routine_cruise(self, steering_angle):
        # steering_angle = self.calculate_steering_angle(left_lane_pts, right_lane_pts)
        steering_angle = np.clip(steering_angle, -21, 21)
        new_steer_angle = float(steering_angle)
        print(new_steer_angle)
        self.last_n_angles[self.index % self.angles_to_store] = new_steer_angle

        weighted_angle = 0.0

        for i in range(self.angles_to_store):
            weighted_angle += self.last_n_angles[(self.index + i + 1) % self.angles_to_store] * self.angle_weights[i]

        print('weighted angle', weighted_angle)

        sa_diff = self.current_steer_angle - new_steer_angle
        self.current_steer_angle = float(weighted_angle)  # new_steer_angle
        self.steerAngle = self.current_steer_angle
        self.driver(0.2,0.1)
        # if self.start_moving == True and self.testing_objects == False:
        #     self.bump()
        # # self.drive(t=0.25, speed=0.22)
        # # time.sleep(0.06) #without objects

        self.index += 1
        if self.index % self.angles_to_store == 0 and self.index >= 20:
            self.index = 0
            self.start_moving = True

        print(self.index)

    def calculate_steering_angle(self, left_lane_pts, right_lane_pts):
        # print("received lane array",lanes)
        # print("~~~calculating steering angle~~~")
        height, width = self.img_shape
        x_offset = 0.0

        left_x1, left_y1, left_x2, left_y2 = left_lane_pts
        right_x1, right_y1, right_x2, right_y2 = right_lane_pts

        left_found = False if (left_x1 == 0 and left_y1 == 0 and left_x2 == 0 and left_y2 == 0) else True
        # if left_found: print("found left lane")
        right_found = False if (right_x1 == 0 and right_y1 == 0 and right_x2 == 0 and right_y2 == 0) else True
        # if right_found: print("found right lane")

        if left_found and right_found:  # both lanes
            cam_mid_offset_percent = 0.02
            mid = int(width / 2 * (1 + cam_mid_offset_percent))
            x_offset = (left_x2 + right_x2) / 2 - mid
        elif left_found and not right_found:  # left lane only
            x_offset = left_x2 - left_x1
        elif not left_found and right_found:  # right lane ony
            x_offset = right_x2 - right_x1
        else:  # no lanes detected
            x_offset = 0

        # HACK: this was /2 before
        y_offset = int(height / 1.8)

        steering_angle = math.atan(x_offset / y_offset)  # in radians
        steering_angle = int(steering_angle * 180.0 / math.pi)
        return steering_angle
