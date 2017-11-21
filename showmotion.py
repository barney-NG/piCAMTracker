# vim: set et sw=4 sts=4 fileencoding=utf-8:
import picamera
import picamera.array
import numpy as np
from time import sleep
from argparse import ArgumentParser
from MotionAnalyser import MotionAnalyser as ma

def main(show=True):
    with picamera.PiCamera() as camera:
        resx = 1024
        resy = 576
        cl = np.zeros((resy,resx,3), np.uint8)
        #cl[resy/2,:,:] = 0xff  #horizantal line
        cl[:,resx/2,:]  = 0xff  #vertical line
        camera.resolution = (resx,resy)
        #camera.annotate_text = "RaspberryPi3 Camera"
        if show:
            camera.framerate  = 30
        else:
            camera.framerate  = 90
        print("warm-up 2 seconds...")
        sleep(2)
        print("...start")
        #- preview settings
        camera.start_preview()
        camera.preview.fullscreen = False
        camera.preview.window = (50,100,resx,resy)
        camera.preview.alpha = 192
        #camera.preview.window = (50,100,resy,resx)
        #camera.preview.rotation = 270

        #- overlay settings
        overlay = camera.add_overlay(source=np.getbuffer(cl),
                                     size=(resx,resy),format='rgb')
        overlay.fullscreen = False
        overlay.window = (50,100,resx,resy)
        overlay.alpha = 32
        overlay.layer = 3
        #overlay.window = (50,100,resy,resx)
        #overlay.rotation= 270

        #- disable auto (exposure + white balance)
        camera.shutter_speed = camera.exposure_speed
        camera.exposure_mode = 'off'
        g = camera.awb_gains
        camera.awb_mode  = 'off'
        camera.awb_gains = g
        wait = 86400
        with ma(camera, show) as output:
            try:
                if show is False:
                    wait = 10
                camera.start_recording('/dev/null', 'h264', motion_output=output)
                camera.wait_recording(wait) # continue recording for 20 seconds
            except NotImplementedError:
                pass

            camera.stop_recording()
            camera.stop_preview()
            camera.remove_overlay(overlay)

if __name__ == '__main__':
    parser = ArgumentParser(prog='pyCAMTracker')
    parser.add_argument('-s', '--show', action='store_true',
                      help   = 'show graphical debug information (slow!)')
    args = parser.parse_args()

    main(args.show)
