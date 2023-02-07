from multiprocessing import Pipe,Event,Process
import cv2
from threading import Thread
import time
from sshkeyboard import listen_keyboard
statusR,statusS = Pipe(duplex = False)
def send_status(statusS):
    while True:
        statusS.send("bat")
def create_video(statusR):
    cap = cv2.VideoCapture(0)
    fourcc = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')
    out = cv2.VideoWriter('output.mp4', fourcc, 25, (640, 480), isColor=True)
    print("bat dau quay video")
    while True:
       status = statusR.recv()
       ret, frame = cap.read()
       frame = cv2.resize(frame, (640, 480))
       out.write(frame)
       print(status)
       if status == "tat":
          break
    print("Sao tat")
    cap.release()
    out.release()
    cv2.destroyAllWindows()
proc = Process(target=send_status, args=([statusS]))
proc.start()
proc1 = Process(target=create_video, args=([statusR]))
proc1.start() 
blocker = Event()  
try:
    blocker.wait()
except KeyboardInterrupt:
    proc.terminate()
    statusS.send("tat")
    proc1.terminate()
    print("tat")
