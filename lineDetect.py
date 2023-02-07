#!/usr/bin/env python
from gevent.pywsgi import WSGIServer
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import os
import serial
import struct
import time
import numpy as np
import signal
import sys
import pigpio
from PIL import Image

from threading import Thread
from enum import Enum
import base64
import tornado.web
import tornado.websocket
from tornado.ioloop import PeriodicCallback

try:
    import cStringIO as io
except ImportError:
    import io

from multiprocessing import Process, Value, Queue, Pipe
from Queue import Empty


ROOT = os.path.normpath(os.path.dirname(__file__))
#State = Enum(run=0, stop=1)

queue = Queue()
queuePic = Queue()

#output_p, input_p = Pipe()
N_SLICES = 5
MAX_SPEED = 250000


ser = serial.Serial('/dev/ttyAMA0', 115200)  # Establish the connection on a specific port

class State(Enum):
    run = 0
    stop = 1
    run_slow = 3

class PiVideoStream:
	def __init__(self, resolution=(640, 480), framerate=32):
		# initialize the camera and stream
		self.camera = PiCamera()
		self.camera.resolution = resolution
		self.camera.framerate = framerate
		self.rawCapture = PiRGBArray(self.camera, size=resolution)
		self.stream = self.camera.capture_continuous(self.rawCapture,
			format="bgr", use_video_port=True)

		# initialize the frame and the variable used to indicate
		# if the thread should be stopped
		self.frame = None
		self.stopped = False
	def start(self):
		# start the thread to read frames from the video stream
		Thread(target=self.update, args=()).start()
		return self

	def update(self):
		# keep looping infinitely until the thread is stopped
		for f in self.stream:
			# grab the frame from the stream and clear the stream in
			# preparation for the next frame
			self.frame = f.array
			self.rawCapture.truncate(0)

			# if the thread indicator variable is set, stop the thread
			# and resource camera resources
			if self.stopped:
				self.stream.close()
				self.rawCapture.close()
				self.camera.close()
				return
	def read(self):
		# return the frame most recently read
		return self.frame

	def stop(self):
		# indicate that the thread should be stopped
		self.stopped = True


class PID:
	"""
	Discrete PID control
	"""

	def __init__(self, P=2.0, I=0.0, D=1.0, Derivator=0, Integrator=0, Integrator_max=1000, Integrator_min=-1000):

		self.Kp=P
		self.Ki=I
		self.Kd=D
		self.Derivator=Derivator
		self.Integrator=Integrator
		self.Integrator_max=Integrator_max
		self.Integrator_min=Integrator_min

		self.set_point=0.0
		self.error=0.0

	def update(self,current_value):
		"""
		Calculate PID output value for given reference input and feedback
		"""

		self.error = self.set_point - current_value

		self.P_value = self.Kp * self.error
		self.D_value = self.Kd * ( self.error - self.Derivator)
		self.Derivator = self.error

		self.Integrator = self.Integrator + self.error

		if self.Integrator > self.Integrator_max:
			self.Integrator = self.Integrator_max
		elif self.Integrator < self.Integrator_min:
			self.Integrator = self.Integrator_min

		self.I_value = self.Integrator * self.Ki

		PID = self.P_value + self.I_value + self.D_value

		return PID

	def setPoint(self,set_point):
		"""
		Initilize the setpoint of PID
		"""
		self.set_point = set_point
		self.Integrator=0
		self.Derivator=0

	def setIntegrator(self, Integrator):
		self.Integrator = Integrator

	def setDerivator(self, Derivator):
		self.Derivator = Derivator

	def setKp(self,P):
		self.Kp=P

	def setKi(self,I):
		self.Ki=I

	def setKd(self,D):
		self.Kd=D

	def getPoint(self):
		return self.set_point

	def getError(self):
		return self.error

	def getIntegrator(self):
		return self.Integrator

	def getDerivator(self):
		return self.Derivator

def signal_handler(signal, frame):
    print('You pressed Ctrl+C!')
    s = struct.Struct('>ic')

    packedData = s.pack(127, chr(10))
    ser.write(packedData)  # Convert the decimal number to ASCII then send it to the Arduino
    #vs.stop()
    cam.release()
    pio.hardware_PWM(13, 800, 0)
    #left motor
    pio.hardware_PWM(12, 800, 0)
    sys.exit(0)

def findCenter(image):
    # frame = imutils.resize(frame, width=400)
    gray = cv2.bitwise_not(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY))
    #gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	# Gaussian blur
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    #Color thresholding
    ret, thresh = cv2.threshold(blur, 100, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
    #thresh = cv2.adaptiveThreshold(blur,255,cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 311,60)
    thresh = cv2.erode(thresh, None, iterations=2)
    thresh = cv2.dilate(thresh, None, iterations=2)
    # Find the contours of the frame
    contours, hierarchy = cv2.findContours(thresh.copy(), 1, cv2.CHAIN_APPROX_NONE)
 
    # Find the biggest contour (if detected)
    if len(contours) > 0:
        contour = max(contours, key=cv2.contourArea)
        M = cv2.moments(contour)

        if (M['m00'] > 0):
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
        else:
            cx = 0
            cy = 0


    else:
        print("I don't see the line")
        cx = 447
        cy = 447
        pio.hardware_PWM(13, 800, 0)
    	#left motor
    	pio.hardware_PWM(12, 800, 0)
        contour = []
    return (contour,cx,cy)



def calCurvature(cc):
    m1 = (cc[1,1] - cc[0,1])/(cc[1,0] - cc[0,0])
    m2 = (cc[2,1] - cc[1,1])/(cc[2,0] - cc[1,0])
    dxdy = (m1+m2)/2.
    mp1 = (cc[0,0] + cc[1,0])/2.
    mp2 = (cc[1,0] + cc[2,0])/2.
    dxdy2 = (m2 - m1)/(mp2 - mp1)
    KK = dxdy2/((1+dxdy**2)**1.5)*1000
    return KK

def getFrame(cam):
    images = []
    for i in range(N_SLICES):
		images.append(np.zeros((100,640,3), np.uint8))	
	
    while True:
    	success, image0 = cam.read()

        #image0 = vs.read()
		# grab the frame from the stream and resize it to have a maximum
        # width of 400 pixels
        images[0] = image0[0:99, 0:639]
        images[1] = image0[80:179, 0:639]
        images[2] = image0[180:279, 0:639]
        images[3] = image0[280:379, 0:639]
        images[4] = image0[380:479, 0:639]
        selectedPoint = 3
        contour, cx, cy = findCenter(images[selectedPoint])
        
        error = cx - 320
        pid = pidc.update(error)

        #print(error)
        #right motor
    	
    
        if(STATE.value == State.stop):
            pio.hardware_PWM(13, 800, 0)
            #left motor
            pio.hardware_PWM(12, 800, 0)
        elif(STATE.value == State.run):
			
			speed = 250000
			lefts = speed + pid
			#print(speed)
			if(lefts < 0):
				lefts = 0
			elif(lefts > 999999):
				lefts = 999999
			rights = speed - pid
			#print(rights)
			if(rights < 0):
				rights = 0
			elif(rights > 999999):
				rights = 999999
			#right motor
			pio.hardware_PWM(13, 800, rights)
            #left motor
			pio.hardware_PWM(12, 800, lefts)
    	elif(STATE.value == State.run_slow):
			speed = 150000
			lefts = speed + pid
			#print(speed)
			if(lefts < 0):
				lefts = 0
			elif(lefts > 999999):
				lefts = 999999
			rights = speed - pid
			#print(rights)
			if(rights < 0):
				rights = 0
			elif(rights > 999999):
				rights = 999999
			#right motor
			pio.hardware_PWM(13, 800, rights)
            #left motor
			pio.hardware_PWM(12, 800, lefts)
    	else:
			pio.hardware_PWM(13, 800, 0)
            #left motor
			pio.hardware_PWM(12, 800, 0)

        s = struct.Struct('>ic')

        packedData = s.pack(error, chr(10))
        ser.write(packedData)  # Convert the decimal number to ASCII then send it to the Arduino

        #queue.put(jpeg.tobytes())
        #input_p.send((images, image0, selectedPoint)) 
	queue.put((image0, selectedPoint))
	if queue.qsize() > 200:
	    while not queue.empty():
    		print('queue overflow')
    		queue.get()


def freeQ():
	while True:
		try:
			im = queue.get()
		except Empty:
			pass

def getCurve():
	
	
	boundaries = [
		([15, 10, 15], [50, 250, 50]),
		([40, 30, 5], [100, 80, 5])
	]

	color="#063458"

	images = []
	for i in range(N_SLICES):
	    images.append(np.zeros((100,640,3), np.uint8))	

	cx = np.zeros(N_SLICES,int)
	cy = np.zeros(N_SLICES,int)
	err = np.zeros(N_SLICES,int)
	while True:
		try:
			im = queue.get()
			#im = output_p.recv() 
        	#print 'size now %i'%(queue.qsiz
			image0, selectedPoint = im

		
			#gray = cv2.bitwise_not(cv2.cvtColor(image0, cv2.COLOR_BGR2GRAY))
			#edges = cv2.Canny(gray,100,200,apertureSize = 3)
			#minLineLength = 30
			#maxLineGap = 10
			#lines = cv2.HoughLinesP(edges,1,np.pi/180,15,minLineLength,maxLineGap)
			#for x in range(0, len(lines)):
			#	for x1,y1,x2,y2 in lines[x]:
			#		cv2.line(image0,(x1,y1),(x2,y2),(0,255,0),2)

			images[0] = image0[0:99, 0:639]
			images[1] = image0[80:179, 0:639]
			images[2] = image0[180:279, 0:639]
			images[3] = image0[280:379, 0:639]
			images[4] = image0[380:479, 0:639]
			color_idx = 0
			for (lower, upper) in boundaries:
				# create NumPy arrays from the boundaries
				lower = np.array(lower, dtype = "uint8")
				upper = np.array(upper, dtype = "uint8")
 
				# find the colors within the specified boundaries and apply
				# the mask
				mask = cv2.inRange(images[0], lower, upper)
				
				
				
				if(color_idx == 0):
					print("Green")
					print(np.sum(mask))
					if np.sum(mask) > 12000000:
						print("Green detected")
						STATE.value = State.run_slow
				elif (color_idx == 1):
					print("Blue")
					print(np.sum(mask))
					if np.sum(mask) > 6000:
						print("Blue detected")
						STATE.value = State.run
				color_idx = color_idx + 1

			for i in range(N_SLICES):
				contour, cx[i], cy[i] = findCenter(images[i])
				cy[i] = cy[i] + i*100
				err[i] = cx[i] - 320
				if i == selectedPoint:
					cv2.circle(image0, (cx[i], cy[i]), 5, (0,0,255), -1)
				else:
					cv2.circle(image0, (cx[i],cy[i]), 5, (0,255,0), -1)
				cv2.putText(image0, 'error = %i'%(cx[i]-320) ,(10,50*(i+1)), cv2.FONT_HERSHEY_SIMPLEX, 1,(255,255,255),2)            		
				if len(contour) != 0:
					cv2.drawContours(image0, contour, -1, (0, 255, 255), 1)
			
			cc = np.array([[cy[2], err[2]],[cy[3], err[3]],[cy[4], err[4]]])

			KK = calCurvature(cc)
			if KK != 0:
				SPEED.value = 150000
			else:
				SPEED.value = 250000
			
			

			cv2.putText(image0, 'curve = %.2f'%(KK) ,(10,50*(6)), cv2.FONT_HERSHEY_SIMPLEX, 1.2,(255,255,255),2)
			
			


			ret, jpeg = cv2.imencode('.jpg', image0)
			queuePic.put(jpeg)
			if queuePic.qsize() > 200:
				while not queuePic.empty():
					queuePic.get()

			
		except Empty:
			pass

class IndexHandler(tornado.web.RequestHandler):

    def get(self):
       self.render("index.html", port=5000)

class startHandler(tornado.web.RequestHandler):
    def get(self):
		STATE.value = State.run
		self.write('GET - Welcome to the CarHandler!')

class stopHandler(tornado.web.RequestHandler):
    def get(self):
		STATE.value = State.stop
		self.write('GET - Welcome to the CarHandler!')

class WebSocket(tornado.websocket.WebSocketHandler):

    def on_message(self, message):
        """Evaluates the function pointed to by json-rpc."""

        # Start an infinite loop when this is called
        if message == "read_camera":
            self.camera_loop = PeriodicCallback(self.loop, 10)
            self.camera_loop.start()

        # Extensibility for other methods
        else:
            print("Unsupported function: " + message)

    def loop(self):
        """Sends camera images in an infinite loop."""
    	sio = io.StringIO()

    	try:
			jpeg = queuePic.get()
			self.write_message(base64.b64encode(jpeg.tobytes()))
        except tornado.websocket.WebSocketClosedError:
			self.camera_loop.stop()




if __name__ == '__main__':
	pidc=PID(375.0,0.5,450.0)
	

	SPEED = Value('d', 250000)
	STATE = Value('i', State.stop)
	pidc.setPoint(0.0)
	pio = pigpio.pi()
	signal.signal(signal.SIGINT, signal_handler)
	cam = cv2.VideoCapture(0)
	p = Process(target=getFrame, args=(cam,))
	#vs = PiVideoStream().start()
	#time.sleep(2.0)

	#p = Process(target=getFrame, args=(vs,))
	p.start()

	p2 = Process(target=freeQ,args=())
	p2.start()
	p3 = Process(target=getCurve,args=())
	p3.start()

	handlers = [(r"/", IndexHandler), 
				(r"/websocket", WebSocket),
				(r"/start", startHandler),
				(r"/stop", stopHandler),
            	(r'/static/(.*)', tornado.web.StaticFileHandler, {'path': ROOT})]
	application = tornado.web.Application(handlers)
	application.listen(5000)



	tornado.ioloop.IOLoop.instance().start()
