#!/usr/bin/env python
# vim: set et sw=4 sts=4 fileencoding=utf-8:
from time import sleep,time
from argparse import ArgumentParser
import numpy as np
import picamtracker
#import curses

class faked_camera:
    def __init__(self, resx=1280, resy=720):
        self.resolution = []
        self.resolution.append(resx)
        self.resolution.append(resy)

    def request_key_frame():
        return

def main(show=False):
    global config
    #win.nodelay(True)

    resx = 1280
    resy = 720
    camera = faked_camera(resx=resx, resy=resy)
    image = np.ones((resy/2,resx/2,3), np.uint8) * 220
    display = picamtracker.Display(caption='piCAMTracker',x=config.conf['previewX'],y=config.conf['previewY'],w=resy/2,h=resx/2)
    greenLED = picamtracker.GPIOPort.gpioPort(config.conf['greenLEDPort'])
    redLED = picamtracker.GPIOPort.gpioPort(config.conf['redLEDPort'])
    tracker = picamtracker.Tracker(camera, greenLed=greenLED, redLed=redLED, config=config)
    
    fobj = open("debug_tracker.csv","r")

    old_frame = 0
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
            tracker.update_tracks(old_frame, new_points)
            tracker.showTracks(old_frame, image)
            display.imshow(image)
            try:
                input("")
            except SyntaxError:
                pass
            image.fill(200)
            old_frame = frame
            del new_points[:]
            new_points.append([[int(a[1]),int(a[2]),int(a[3]),int(a[4])],[float(a[5]),float(a[6])]])
            print("<%s>" % line)

    fobj.close()
    greenLED.terminated = True
    redLED.terminated = True
    tracker.stop()
    greenLED.join()
    redLED.join()
    tracker.join()

if __name__ == '__main__':
    parser = ArgumentParser(prog='piCAMTracker')
    parser.add_argument('-s', '--show', action='store_true',
                      help   = 'show graphical debug information (slow!)')
    args = parser.parse_args()
    global config
    config = picamtracker.Configuration('config.json')

    #curses.wrapper(main)
    main(args.show)
