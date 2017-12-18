# vim: set et sw=4 sts=4 fileencoding=utf-8:
import picamera
import picamera.array
import numpy as np
from time import sleep
from argparse import ArgumentParser
import MotionAnalyser
from MotionTracker import Tracker
from MotionDisplay import Display

def main(show=True):
    with picamera.PiCamera() as camera:
        stream = picamera.PiCameraCircularIO(camera, seconds=5)
        #resx = 1024
        #resy = 576
        #resx = 1280
        resx = 1296
        resy = 720
        cl = np.zeros((resy,resx,3), np.uint8)
        cl[resy/2,:,:] = 0xff  #horizantal line
        #cl[:,resx/2,:]  = 0xff  #vertical line
        camera.resolution = (resx,resy)
        #camera.annotate_text = "RaspberryPi3 Camera"
        if show:
            camera.framerate  = 30
            display = Display(caption='pyCAMTracker',x=52,y=2,w=resy,h=resx)
        else:
            display = None
            #camera.framerate  = 30  #- V1/V2 ~20ms/frame max
            camera.framerate  = 49  #- V1/V2 ~20ms/frame max
            #camera.framerate  = 68
            camera.sensor_mode  = 5 #V1
            #camera.sensor_mode  = 6 #V2
        print("warm-up 2 seconds...")
        sleep(2)
        print("...start")
        #- preview settings
        camera.start_preview()
        camera.preview.fullscreen = False
        #camera.preview.window = (50,100,resx,resy)
        camera.preview.alpha = 192
        camera.preview.window = (100,80,resy/2,resx/2)
        #camera.preview.window = (100,20,resy/2,resx/2)
        camera.preview.rotation = 90

        #- overlay settings
        overlay = camera.add_overlay(source=np.getbuffer(cl),
                                     size=(resx,resy),format='rgb')
        overlay.fullscreen = False
        #overlay.window = (50,100,resx,resy)
        overlay.alpha = 32
        overlay.layer = 3
        overlay.window = (100,80,resy/2,resx/2)
        #overlay.window = (100,20,resy/2,resx/2)
        overlay.rotation= 90

        #- disable auto (exposure + white balance)
        #camera.shutter_speed = camera.exposure_speed
        #camera.exposure_mode = 'off'
        #g = camera.awb_gains
        #camera.awb_mode  = 'off'
        #camera.awb_gains = g
        wait = 86400
        tracker = Tracker()
        with MotionAnalyser.MotionAnalyser(camera, tracker, display, show) as output:
            camera.start_recording(stream, 'h264', motion_output=output)
            while True:
                try:
                    camera.wait_recording(1) # continue recording for 20 seconds
                except KeyboardInterrupt:
                    break

            tracker.stop()
            tracker.join()
            if display is not None:
                display.terminated = True
                display.join()
            camera.stop_recording()
            camera.stop_preview()
            camera.remove_overlay(overlay)

if __name__ == '__main__':
    parser = ArgumentParser(prog='pyCAMTracker')
    parser.add_argument('-s', '--show', action='store_true',
                      help   = 'show graphical debug information (slow!)')
    args = parser.parse_args()

    main(args.show)
