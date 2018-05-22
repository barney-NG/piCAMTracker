#!/usr/bin/env python
# vim: set et sw=4 sts=4 fileencoding=utf-8:
from time import sleep,time
from argparse import ArgumentParser
import numpy as np
#from picamtracker import MotionTracker,Configuration
import picamtracker.MotionTracker
import picamtracker.Configuration
import cv2

class faked_camera:
    def __init__(self, resx=1280, resy=720):
        self.resolution = []
        self.resolution.append(resx)
        self.resolution.append(resy)

    def request_key_frame(val=0):
        return

def show_input(img, pts):
    for rr,vv in pts:
        x0 = int(rr[0])
        y0 = int(rr[1])
        x1 = int(rr[0] + rr[2])
        y1 = int(rr[1] + rr[3])
        x0 *= 8
        y0 *= 8
        x1 *= 8
        y1 *= 8
        cv2.rectangle(img,(x0,y0),(x1,y1),(0,0,0),1)
        cv2.line(img,(x0,y0),(x1,y1),(0,0,0),1)
        cv2.line(img,(x0,y1),(x1,y0),(0,0,0),1)

def main(fobj=None):
    global config

    resx = 1280
    resy = 720
    caption = 'piCAMTracker'
    camera = faked_camera(resx=resx, resy=resy)
    image = np.ones((resy/2,resx/2,3), np.uint8) * 220
    tracker = MotionTracker.Tracker(camera, greenLed=None, redLed=None, config=config)
    tracker.setup_sizes(resy/16, resx/16)

    old_frame = 0
    wtime = 0x00
    new_points = []
    for line in fobj:
        line = line.rstrip()
        a = line.split(',')
        frame = int(a[0])
        if old_frame == 0:
            old_frame = frame

        if frame == old_frame:
            new_points.append([[int(a[1]),int(a[2]),int(a[3]),int(a[4])],[float(a[5]),float(a[6])]])
            print("<%s>" % line)
        else:
            tracker.deb_update_tracks(old_frame, new_points)
            tracker.showTracks(old_frame, image)
            show_input(image, new_points)
            cv2.imshow(caption,np.flipud(np.rot90(np.flipud(image),k=1)))
            ch = cv2.waitKey(wtime) & 0xFF
            if ch == ord('s'):
                wtime ^= 1
            if ch == 27:
                break
            image.fill(200)
            old_frame = frame
            del new_points[:]
            new_points.append([[int(a[1]),int(a[2]),int(a[3]),int(a[4])],[float(a[5]),float(a[6])]])
            print("<%s>" % line)

    fobj.close()
    tracker.stop()
    tracker.join()

if __name__ == '__main__':
    parser = ArgumentParser(prog='debugtracker.py')
    parser.add_argument('input', type=file,
                        help   = 'input file to be debugged')

    args = parser.parse_args()
    global config
    config = Configuration('config.json')
    config.conf['debug'] = False

    #curses.wrapper(main)
    main(args.input)
