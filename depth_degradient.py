# coding: utf-8

import colorsys
import numpy as np
import cv2
import sys
import random
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

undistorted = Frame(512, 424, 4)
registered = Frame(512, 424, 4)
PalletIntsR = [None] *256
PalletIntsG = [None] *256
PalletIntsB = [None] *256

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


def CreateRainbowPallet_new():
    for i in range(0,256):
        if (i<=29):
            r = (129.36-i*2.36)
            g = 0
            b = 255
        if (i<=69):
            r = (129.36-i*2.36)
            g = 0
            b = 0
        elif (i<=86):
            r = 0
            g = (-133.54+i*4.52)
            b = 255

        elif (i<=100):
            r = 255
            g = (-133.54+i*4.52)
            b = 0  
        elif (i<=141):
            r = 0
            g = 255
            b = (265.83-i*4.72)
        elif (i<=199):
            r = (-235.26+i*4.47)
            g = 255
            b = 0
        else:
            r = 255
            g = (566.81-i*4.57)
            b = 0

        PalletIntsR[i] = r
        PalletIntsG[i] = g
        PalletIntsB[i] = b

def generate_gradient_rgbs(num_buckets):
    rgb_codes = []
    step_size = 2000/ num_buckets
    for step in range(0,num_buckets):
        red = int(max(0, 255 - (step_size*step*1))) # step size is half of the step size since both this item goes down and the next one goes up
        blue = int(max(0, 255 - (step_size*1*(num_buckets-step-1))))
        green = (255 - red) if red else (255 - blue)
        rgb_codes.append((red, green, blue))
    return rgb_codes


while True:

    CreateRainbowPallet()
    frames = listener.waitForNewFrame()

    depth = frames["depth"]

    copy_frame =  depth.asarray()
    new_frame = np.float64(np.zeros([424,512,3]))
    black_frame= np.float32(np.zeros([424,512,3]))
    


    crop_frame = copy_frame[180:275,270:360]

    new_frame = np.float64(np.zeros([crop_frame.shape[0], crop_frame.shape[1], 3]))

    rgb  = generate_gradient_rgbs(crop_frame.shape[1])

    for row in range(crop_frame.shape[0]):
        for col in range(crop_frame.shape[1]):
            depth_value = crop_frame[row,col]
            if depth_value < 1200:
                rainbow = int(depth_value/(4333/256))
                rainbow_ciclic = int(depth_value % 256)
                print(depth_value)
                #calc_type = rainbow_ciclic + random.randint(0,140)
                calc_type = rainbow_ciclic 
 
                if calc_type<256:
                    new_frame[row][col] = rgb[col]
                    #new_frame[row][col][0] = PalletIntsB[calc_type]
                    #new_frame[row][col][1] = PalletIntsG[calc_type]
                    #new_frame[row][col][2] = PalletIntsR[calc_type]

    #for row in range(copy_frame.shape[0]):
    #    for col in range(copy_frame.shape[1]):
    #        depth_value = copy_frame[row,col]
    #        rainbow = int(depth_value/(1500/256))
    #        rainbow_ciclic = int(depth_value % 256)
    #        calc_type = rainbow
    #        if calc_type<256:
    #            new_frame[row][col][0] = PalletIntsB[calc_type]
    #            new_frame[row][col][1] = PalletIntsG[calc_type]
    #            new_frame[row][col][2] = PalletIntsR[calc_type]

    flip_image =cv2.flip(new_frame, 1)
    print(crop_frame.shape)
    #cv2.imshow("Crop_new", new_frame)
    cv2.imshow("bigdepth", cv2.resize(new_frame,(int(580 ), int(580))))
    cv2.imshow("bigdepth_2", cv2.resize(flip_image,(int(575 ), int(470))))
    #cv2.imshow("bigdepth_black", cv2.resize(black_frame,(int(1980 ), int(1080))))

    rgb = generate_gradient_rgbs(512)
    print(len(rgb))
    print(rgb[0])

    print(rgb[9])

    print(rgb[511])
    
    listener.release(frames)

    key = cv2.waitKey(delay=1)
    if key == ord('q'):
        break

device.stop()
device.close()


