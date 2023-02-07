import cv2
import numpy as np



import math
from threading import Thread
import src.predict.preProcessing as pp
import time
from controller3 import Controller
from detection import Detection
from helper_functions import *
from src.templates.workerprocess import WorkerProcess

# PARAMETERS
SAMPLE_TIME = 0.01      # [s]
WAIT_TIME_STOP = 3.0    # [s]
DESIRED_SPEED = 0.2    # [m/s]
OBSTACLE_DISTANCE = 0.20 # [m]

SLOWDOWN_DIST_VALUE = 0.2     # [m]
STOP_DIST_VALUE = 0.6         # [m]

SHOW_IMGS = True

# Pure pursuit controller parameters
k1 = 0.0 #4.0 gain error parallel to direction (speed)
k2 = 0.0 #2.0 perpedndicular error gain
k3 = 0.99 #1.5 yaw error gain
#dt_ahea  = 0.5 # [s] how far into the future the curvature is estimated, feedforwarded to yaw controller
ff_curvature = 0.0 # feedforward gain
class LaneDetector(WorkerProcess):
    # ======================= INIT =======================
    def __init__(self, inPs, outPs):
        """Accepts frames from the camera, processes the frame and detect lanes,
        and transmits information about left and right lanes.
        Parameters
        ------------
        inPs : list(Pipe)
            0 - receive image feed from the camera
        outPs : list(Pipe)
            0 - send lane information
        """

        self.img_shape = (480, 640)
        height = self.img_shape[0]
        width = self.img_shape[1]


#
#         region_top_left = (0.15 * width, 0.3 * height)
#         region_top_right = (0.85 * width, 0.3 * height)
#         region_bottom_left_A = (0.00 * width, 1.00 * height)
#         region_bottom_left_B = (0.00 * width, 0.8 * height)
#         region_bottom_right_A = (1.00 * width, 1.00 * height)
#         region_bottom_right_B = (1.00 * width, 0.8 * height)
#
#         self.mask_vertices = np.array([[region_bottom_left_A,
#                                         region_bottom_left_B,
#                                         region_top_left,
#                                         region_top_right,
#                                         region_bottom_right_B,
#                                         region_bottom_right_A]], dtype=np.int32)
#
        super(LaneDetector, self).__init__(inPs, outPs)
#
#     # ======================= RUN =======================
    def run(self):
        """Apply the initializing methods and start the threads.
        """
        super(LaneDetector, self).run()
#
#     # ======================= INIT THREADS =======================
    def _init_threads(self):
        """Initialize the read thread to receive the video.
        """
        if self._blocker.is_set():
            return

        thr = Thread(name='StreamSending', target=self._the_thread, args=(self.inPs, self.outPs,))
        thr.daemon = True
        self.threads.append(thr)
#
#     # ======================= METHODS =======================
    def laneDetection(self, frame):
        """
        Estimates:
        - the lateral error wrt the center of the lane (e2),
        - the angular error around the yaw axis wrt a fixed point ahead (e3),
        - the ditance from the next stop line (1/dist)
        """
        detect = Detection()
        controller = Controller(k1=k1, k2=k2, k3=k3, ff=ff_curvature, training=False)
        lane_info = detect.detect_lane(frame, show_ROI=SHOW_IMGS)
        e2, e3, point_ahead = lane_info
        curv = 0.0001
        speed_ref, angle_ref = controller.get_control(e2, e3, curv, DESIRED_SPEED)
        angle_ref = np.rad2deg(angle_ref)
        return speed_ref,angle_ref,frame

#
#         # TODO : check this threshold (it was determined using a very short test)
#         if len(horizontal_lines) >= 10:
#             intersection_y = self.check_for_intersection(horizontal_lines)
#
#         try:
#             left_lane_slope, left_intercept = pp.getLanesFormula(left_lane_lines)
#             smoothed_left_lane_coefficients = pp.determine_line_coefficients(left_lane_coefficients,
#                                                                              [left_lane_slope, left_intercept])
#         except Exception as e:
#             print("Using saved coefficients for left coefficients", e)
#             smoothed_left_lane_coefficients = pp.determine_line_coefficients(left_lane_coefficients, [0.0, 0.0])
#
#         try:
#             right_lane_slope, right_intercept = pp.getLanesFormula(right_lane_lines)
#             smoothed_right_lane_coefficients = pp.determine_line_coefficients(right_lane_coefficients,
#                                                                               [right_lane_slope, right_intercept])
#         except Exception as e:
#             print("Using saved coefficients for right coefficients", e)
#             smoothed_right_lane_coefficients = pp.determine_line_coefficients(right_lane_coefficients, [0.0, 0.0])
#
#         return np.array(
#             [smoothed_left_lane_coefficients, smoothed_right_lane_coefficients]), intersection_y, preprocessed_img
#
#     def check_for_intersection(self, lines):
#         # print("########### checking for intersection ###########")
#         # for l in lines:
#         #    print(l)
#         slope, intercept = pp.getLanesFormula(lines)
#         # print(slope)
#         # print(intercept)
#         # print("#################################################")
#         return intercept
#
#     def points_from_lane_coeffs(self, line_coefficients):
#         A = line_coefficients[0]
#         b = line_coefficients[1]
#
#         if A == 0.00 and b == 0.00:
#             return [0, 0, 0, 0]
#
#         height, width = self.img_shape
#
#         bottom_y = height - 1
#         top_y = self.mask_vertices[0][2][1]
#         # y = Ax + b, therefore x = (y - b) / A
#         bottom_x = (bottom_y - b) / A
#         # clipping the x values
#         bottom_x = min(bottom_x, 2 * width)
#         bottom_x = max(bottom_x, -1 * width)
#
#         top_x = (top_y - b) / A
#         # clipping the x values
#         top_x = min(top_x, 2 * width)
#         top_x = max(top_x, -1 * width)
#
#         return [int(bottom_x), int(bottom_y), int(top_x), int(top_y)]
#
    def _the_thread(self, inPs, outPs):
#         """Read the image from input stream, process it and send lane information
#
#         Parameters
#         ----------
#         inPs : list(Pipe)
#         0 - video frames
#         outPs : list(Pipe)
#         0 - array of left and right lane information
#         """
        last_steer = 0
        while True:
            stamps, image_in = inPs[0].recv()
            #                 # print("LANE DETECT LOG, GOTTEN IMAGE")
            #                 # proncess input frame and return array [left lane coeffs, right lane coeffs]
            speed_ref,angle_ref,frame = self.laneDetection(image_in)
            #                 # print("LANE DETECT LOG, GOTTEN COEFFS", lanes_coefficients)
            #
            stamp = time.time()
            outPs[0].send([[stamp], speed_ref,angle_ref])
            outPs[1].send([[stamp], frame])




