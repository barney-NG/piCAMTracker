#!/usr/bin/env python
# vim: set et sw=4 sts=4 fileencoding=utf-8:
from time import sleep,time
#from argparse import ArgumentParser
import argparse
import numpy as np
#from picamtracker import MotionTracker,Configuration
import picamtracker.MotionTracker
import picamtracker.ConfigReader
import cv2

motion_dtype = np.dtype([
    ('x',   np.int8),
    ('y',   np.int8),
    ('sad', np.uint16),
    ])

class faked_frame:
    def __init__(self, index=0):
        self.index = index

class faked_camera:
    def __init__(self, resx=1280, resy=720):
        self.resolution = []
        self.resolution.append(resx)
        self.resolution.append(resy)
        self.frame = faked_frame()
        self.analog_gain = 77

    def request_key_frame(val=0):
        return

def show_input(img, motion):
    rr = motion[0]
    vv = motion[1]
    mm = motion[2]
    x0 = int(rr[0])
    y0 = int(rr[1])
    x1 = int(rr[0] + rr[2])
    y1 = int(rr[1] + rr[3])
    x0 *= 8
    y0 *= 8
    x1 *= 8
    y1 *= 8
    cv2.rectangle(img,(x0,y0),(x1,y1),(0,255,0),-4)
    cv2.line(img,(x0,y0),(x1,y1),(0,0,0),1)
    cv2.line(img,(x0,y1),(x1,y0),(0,0,0),1)

def cv_getNumber():
    str=''
    while True:
        ch = cv2.waitKey(0) & 0xFF
        if ch >= 48 and ch <= 57:
            str = str + chr(ch)
        else:
            break
    return int(str)

def main(fobj=None,width=1280,height=960):
    global config

    #width  = 1632
    #height = 896
    #width  = 1280
    #height = 960
    cols = ((width + 15) // 16) + 1
    rows = (height + 15) // 16
    chunk_size = cols*rows*4
    chunks_read = 0
    wtime = 0x00

    caption = 'piCAMTracker::cv2'
    camera = faked_camera(resx=width, resy=height)
    image = np.ones((height/2,width/2,3), np.uint8) * 220
    tracker = picamtracker.MotionTracker.Tracker(camera, greenLed=None, redLed=None, config=config)
    tracker.setup_sizes(height/16, width/16)
    x_disp = config.conf['previewX'] + config.conf['offsetX']
    y_disp = config.conf['previewY'] + config.conf['offsetY']
    display = picamtracker.Display(caption='piCAMTracker::debug',x=x_disp,y=y_disp,w=height/2,h=width/2)
    analyser = picamtracker.MotionAnalyser(camera, tracker, display, 0xff, config)
    analyser.rows = rows
    analyser.cols = cols

    while True:
        buf = fobj.read(chunk_size)
        if len(buf) == chunk_size:
            chunks_read += 1
            camera.frame.index = chunks_read
            camera.analog_gain = chunks_read
            a = np.frombuffer(buf, dtype=motion_dtype).reshape((rows,cols))
            analyser.analyse(a)
            delay,frame,motion = tracker.getStatus()
            #tracker.showTracks(chunks_read, image)
            if frame > 0:
                tracker.releaseLock()
                #show_input(image, motion)
            cv2.imshow(caption,np.flipud(np.rot90(np.flipud(image),k=1)))
            ch = cv2.waitKey(wtime) & 0xFF
            if ch == ord('s'):
                wtime ^= 1
            if ch == ord('g'):
                target_frame = cv_getNumber()
                print("goto frame: %d" % target_frame)
                try:
                    fobj.seek(0,0)
                    fobj.seek(target_frame*chunk_size,0)
                    camera.frame.index = chunks_read = target_frame
                except:
                    print("frame not reachable")

            if ch == 27:
                break
                
            image.fill(200)
            print(chunks_read)
        else:
            print("end")
            break
      


    fobj.close()
    tracker.stop()
    tracker.join()
    if display is not None:
        display.terminated = True

if __name__ == '__main__':
    global config
    config = picamtracker.ConfigReader.Configuration('config.json')
    config.conf['debug'] = False

    parser = argparse.ArgumentParser(prog='debugmotion.py')
    parser.add_argument(
        'input', 
        type=argparse.FileType("rb"),
        help   = 'input file to be debugged')
    parser.add_argument(
        '--width', 
        type=int,
        help   = 'input width',
        default=1280)
    parser.add_argument(
        '--height', 
        type=int,
        help   = 'input height',
        default=960)

    args = parser.parse_args()

    #curses.wrapper(main)
    main(args.input,args.width,args.height)
