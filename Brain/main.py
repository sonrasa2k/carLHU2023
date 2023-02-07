# Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC orginazers
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#========================================================================
# SCRIPT USED FOR WIRING ALL COMPONENTS
#========================================================================
import sys
sys.path.append('.')
import time
import signal
from multiprocessing import Pipe, Process, Event 
import cv2
# hardware imports
from src.hardware.camera.cameraprocess                      import CameraProcess
from src.hardware.camera.CameraSpooferProcess               import CameraSpooferProcess
from src.hardware.serialhandler.SerialHandlerProcess        import SerialHandlerProcess
from src.utils.autocontrol.autoControl import AutoControl
from src.predict.laneDetect import LaneDetector
# utility imports
from src.utils.camerastreamer.CameraStreamerProcess         import CameraStreamerProcess
from src.utils.remotecontrol.RemoteControlReceiverProcess   import RemoteControlReceiverProcess

# =============================== CONFIG =================================================
enableStream        =  True
enableCameraSpoof   =  False
enableRc            =  True
enableAutoControl = True
laneDetect = True

# =============================== INITIALIZING PROCESSES =================================
allProcesses = list()

#auto
camR1, camS1 = Pipe(duplex = False)
laneR1,laneS1 = Pipe(duplex = False)
camStreamR,camStreamS = Pipe(duplex = False)

#test
acR, acS   = Pipe(duplex = False)
rcShR, rcShS   = Pipe(duplex = False)
# =============================== HARDWARE ===============================================
def _get_timestamp():
    stamp = time.gmtime()
    res = str(stamp[0])
    for data in stamp[1:6]:
        res += '_' + str(data)

    return res

if enableStream:
           # camera  ->  streamer
    if enableCameraSpoof:
        camSpoofer = CameraSpooferProcess([],[camS1],'vid')
        allProcesses.append(camSpoofer)

    else:
        # cap = cv2.VideoCapture(0)
        # _, src_img = cap.read()
        # camS1.send([[_get_timestamp()],src_img])
        camProc = CameraProcess([],[camS1])
        allProcesses.append(camProc)

    # streamProc = CameraStreamerProcess([camStreamR], [])
    # allProcesses.append(streamProc)

if laneDetect:
    laneProc = LaneDetector([camR1],[laneS1,camStreamS])
    allProcesses.append(laneProc)
    streamProc = CameraStreamerProcess([camStreamR], [])
    allProcesses.append(streamProc)
# =============================== DATA ===================================================
#LocSys client process
# LocStR, LocStS = Pipe(duplex = False)           # LocSys  ->  brain
# from data.localisationsystem.locsys import LocalisationSystemProcess
# LocSysProc = LocalisationSystemProcess([], [LocStS])
# allProcesses.append(LocSysProc)
#======================================== DETECT Lane ==============================


#=============================== AUTO CONTROL =============================
if enableAutoControl:
    shProc = SerialHandlerProcess([acR], [])
    allProcesses.append(shProc)

    autoProc = AutoControl([laneR1],[acS])
    allProcesses.append(autoProc)

# =============================== CONTROL ================================================
if enableRc:
              # rc      ->  serial handler

    # serial handler process
    shProc = SerialHandlerProcess([rcShR], [])
    allProcesses.append(shProc)

    rcProc = RemoteControlReceiverProcess([],[rcShS])
    allProcesses.append(rcProc)


# ===================================== START PROCESSES ==================================
print("Starting the processes!",allProcesses)
for proc in allProcesses:
    proc.daemon = True
    proc.start()


# ===================================== STAYING ALIVE ====================================
blocker = Event()  

try:
    blocker.wait()
except KeyboardInterrupt:
    print("\nCatching a KeyboardInterruption exception! Shutdown all processes.\n")
    acS.send({"action":"1","speed":0.0})
    for proc in allProcesses:
        if hasattr(proc,'stop') and callable(getattr(proc,'stop')):
            print("Process with stop",proc)
            proc.stop()
            proc.join()
        else:
            print("Process witouth stop",proc)
            proc.terminate()
            proc.join()
