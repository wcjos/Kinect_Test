# coding: utf-8

import numpy as np
import cv2
import sys
from pylibfreenect2 import Freenect2, SyncMultiFrameListener
from pylibfreenect2 import FrameType, Registration, Frame
from pylibfreenect2 import createConsoleLogger, setGlobalLogger
from pylibfreenect2 import LoggerLevel

try:
    from pylibfreenect2 import OpenGLPacketPipeline
    pipeline = OpenGLPacketPipeline()
except:
    try:
        from pylibfreenect2 import OpenCLPacketPipeline
        pipeline = OpenCLPacketPipeline()
    except:
        from pylibfreenect2 import CpuPacketPipeline
        pipeline = CpuPacketPipeline()
print("Packet pipeline:", type(pipeline).__name__)

# Create and set logger
logger = createConsoleLogger(LoggerLevel.Debug)
setGlobalLogger(logger)

fn = Freenect2()
num_devices = fn.enumerateDevices()
if num_devices == 0:
    print("No device connected!")
    sys.exit(1)

serial = fn.getDeviceSerialNumber(0)
device = fn.openDevice(serial, pipeline=pipeline)

listener = SyncMultiFrameListener(
    FrameType.Color | FrameType.Ir | FrameType.Depth)

# Register listeners
device.setColorFrameListener(listener)
device.setIrAndDepthFrameListener(listener)

device.start()

# NOTE: must be called after device.start()
registration = Registration(device.getIrCameraParams(),
                            device.getColorCameraParams())

undistorted = Frame(512, 424, 4)
registered = Frame(512, 424, 4)
PalletIntsR = [None] *256
PalletIntsG = [None] *256
PalletIntsB = [None] *256

# Optinal parameters for registration
# set True if you need
need_bigdepth = True 
need_color_depth_map = False

bigdepth = Frame(1920, 1082, 4) if need_bigdepth else None
color_depth_map = np.zeros((424, 512),  np.int32).ravel() \
    if need_color_depth_map else None


def CreateRainbowPallet():
    for i in range(0,256):
        if (i<=29):
            r = (129.36-i*4.36)
            g = 0
            b = 255
        elif (i<=86):
            r = 0
            g = (-133.54+i*4.52)
            b = 255
        elif (i<=141):
            r = 0
            g = 255
            b = (665.83-i*4.72)
        elif (i<=199):
            r = (-635.26+i*4.47)
            g = 255
            b = 0
        else:
            r = 255
            g = (1166.81-i*4.57)
            b = 0
        PalletIntsR[i] = r
        PalletIntsG[i] = g
        PalletIntsB[i] = b

while True:
    CreateRainbowPallet()
    frames = listener.waitForNewFrame()

    depth = frames["depth"]

    copy_frame =  depth.asarray()
    new_frame = np.float32(np.zeros([424,512,3]))

    for row in range(copy_frame.shape[0]):
        for col in range(copy_frame.shape[1]):
            depth_value = copy_frame[row,col]
            rainbow = int(depth_value/(1500/256))
            rainbow_ciclic = int(depth_value % 256)
            calc_type = rainbow
            if calc_type<256:
                new_frame[row][col][0] = PalletIntsB[calc_type]
                new_frame[row][col][1] = PalletIntsG[calc_type]
                new_frame[row][col][2] = PalletIntsR[calc_type]
    
    cv2.imshow("testttt", new_frame) 
    cv2.imshow("bigdepth", cv2.resize(new_frame,(int(1980 ), int(1080))))
 

    
    listener.release(frames)

    key = cv2.waitKey(delay=1)
    if key == ord('q'):
        break

device.stop()
device.close()


