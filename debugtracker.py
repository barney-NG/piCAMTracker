#!/usr/bin/env python
# vim: set et sw=4 sts=4 fileencoding=utf-8:
from time import sleep,time
from argparse import ArgumentParser
import numpy as np
from picamtracker import MotionTracker,Configuration
import cv2

class faked_camera:
    def __init__(self, resx=1280, resy=720):
        self.resolution = []
        self.resolution.append(resx)
        self.resolution.append(resy)

    def request_key_frame():
        return

def main(fobj=None):
    global config
    #win.nodelay(True)

    resx = 1280
    resy = 720
    caption = 'piCAMTracker'
    camera = faked_camera(resx=resx, resy=resy)
    image = np.ones((resy/2,resx/2,3), np.uint8) * 220
    tracker = MotionTracker.Tracker(camera, greenLed=None, redLed=None, config=config)

    #fobj = open(deb_file,"r")

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
            tracker.Supdate_tracks(old_frame, new_points)
            tracker.showTracks(old_frame, image)
            cv2.imshow(caption,image)
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
