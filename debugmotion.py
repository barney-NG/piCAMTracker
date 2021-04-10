#!/usr/bin/env python3
# vim: set et sw=4 sts=4 fileencoding=utf-8:
from time import sleep,time
#from argparse import ArgumentParser
import argparse
import numpy as np
#from picamtracker import MotionTracker,Configuration
import picamtracker.MotionTracker
import picamtracker.ConfigReader
import cv2
from struct import unpack_from,calcsize
import logging

# configure logging
LOGFILENAME = '/var/log/picamtracker/picamtracker.log'
logging.basicConfig(filename = LOGFILENAME,
                    format='%(asctime)s: %(module)s::%(funcName)s: %(message)s',
                    level=logging.DEBUG,
                    datefmt='%Y/%m/%d %H:%M:%S')
                    
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

    def capture(self, image=None,fmt=None, use_video_port=True, splitter_port=2):
        pass
    def request_key_frame(self, val=0):
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

def main(fobj=None,width=1280,height=960,video=False):
    global config
    writer = None

    print("args.video: ", video)
    k = 0
    config.conf['debug'] = False
    if config.conf['viewAngle'] == 90:
        k = -1
    if config.conf['viewAngle'] == 180:
        k = 2
    if config.conf['viewAngle'] == 270:
        k = 1

    
    #          0xa1a1a1a1WH
    headerfmt = 'Lll'
    headersz = calcsize(headerfmt)

    try:
        header = fobj.read(headersz)
        magic,w,h = unpack_from(headerfmt, header)
        if magic == 0xa1a1a1a1:
            print("width=%d, height=%d" % (w,h))
            height = h
            width = w
        else:
            fobj.seek(0)
            print("old format data file")
    except:
        raise

    #width  = 1632
    #height = 896
    #width  = 1280
    #height = 960
    cols = ((width + 15) // 16) + 1
    rows = (height + 15) // 16
    chunk_size = cols*rows*4
    chunks_read = 0
    wtime = 0x00

    xcross = config.conf['xCross'] * 16
    ycross = config.conf['yCross'] * 16
    
    if video:
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        writer = cv2.VideoWriter()
        writer.open('output.mp4v', 0, fourcc, 25, (int(height/2),int(width/2)))
        
        #fourcc = cv2.VideoWriter_fourcc(*'fmp4')
        #fourcc = cv2.VideoWriter_fourcc(*'mjpg')
        #writer = cv2.VideoWriter('output.mp4v', 0, fourcc, 25, (int(height/2),int(width/2)))
        
    caption = 'piCAMTracker::cv2'
    camera = faked_camera(resx=width, resy=height)
    image = np.ones((int(height/2),int(width/2),3), np.uint8) * 220
    rot_img = np.rot90(image,k=k)
    tracker = picamtracker.MotionTracker.Tracker(camera, greenLed=None, redLed=None, config=config)
    tracker.setup_sizes(height/16, width/16)
    x_disp = config.conf['previewX'] + config.conf['offsetX']
    y_disp = config.conf['previewY'] + config.conf['offsetY']
    show = 0x0004 | 0x0002
    analyser = picamtracker.MotionAnalyser(camera, tracker, display=None, show=show, config=config)
    
    analyser.rows = rows
    analyser.cols = cols
    warmup = True

    while True:
        buf = fobj.read(chunk_size)
        if len(buf) == chunk_size:
            chunks_read += 1
            camera.frame.index = chunks_read
            camera.analog_gain = chunks_read
            a = np.frombuffer(buf, dtype=motion_dtype).reshape((rows,cols))
            analyser.analyse(a)
            if warmup:
                warmup = False
                continue
                
            delay,updates,frame,motion = tracker.getStatus()
            #tracker.showTracks(chunks_read, image)
            if frame > 0:
                tracker.releaseLock()
                #show_input(image, motion)
            
            # get tracker image
            if ycross > 0:
                cv2.line(analyser.big,(0,int(ycross/2)),(int(width/2),int(ycross/2)),(0,0,0),1)
            if xcross > 0:
                cv2.line(analyser.big,(int(xcross/2),0),(int(xcross/2),int(height/2)),(0,0,0),1)

            # read image from analayser 
            rot_img = np.rot90(analyser.big,k=k)
                 
            if writer:
                writer.write(rot_img)

            # show track status
            for track in tracker.track_pool:
                track.printTrack(chunks_read)

            # show debug image
            cv2.imshow(caption,rot_img)
            analyser.big.fill(220)
                
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

            if ch == ord('q') or ch == 27:
                break
                
            print(chunks_read)
        else:
            print("end")
            break
      


    fobj.close()
    tracker.stop()
    tracker.join()
    if writer:
        writer.release()
    cv2.destroyAllWindows()
        

if __name__ == '__main__':
    global config
    parser = argparse.ArgumentParser(prog='debugmotion.py')
    parser.add_argument(
        'input', 
        type=argparse.FileType("rb"),
        help   = 'input file to be debugged')
    parser.add_argument(
        '--width', 
        type=int,
        help   = 'input width (default: 1632)',
        default=1632)
    parser.add_argument(
        '--height', 
        type=int,
        help   = 'input height (default: 896)',
        default=896)
    parser.add_argument(
        '--video',
        action='store_true',
        help   = 'create output.avi video file',
        default=False)
    parser.add_argument(
        '-c','--cfg', 
        type=str,
        help   = 'use an alternative config file',
        default='config.json')
            
    args = parser.parse_args()

    config = picamtracker.ConfigReader.Configuration(args.cfg)
    
    main(args.input,args.width,args.height,args.video)
