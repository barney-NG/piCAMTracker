#!/usr/bin/env python
# vim: set et sw=4 sts=4 fileencoding=utf-8:
import picamera
import picamera.array
import numpy as np
import datetime as dt
import os
import io
from time import sleep,time
from argparse import ArgumentParser
import picamtracker


def main(show=True):
    global config
    preview = True
    try:
        preview = config.conf['preview']
    except:
        raise


    #- open picamera device
    with picamera.PiCamera() as camera:
        #- determine camera module
        revision = camera._revision.upper()
        if revision == 'OV5647':
            # V1 module
            #resx = 1296 ### error mmal (change format during write)
            resx = 1280
            resy = 720
            fps  = 40 #49
            mode = 5
        elif revision == 'IMX219':
            # V2 module
            resx = 1280
            resy = 720
            fps  = 40  # 40 frames is maximum for the analyse function
                       #    there are no frames in the stream
                       # 68 would be  maximum for motion block frequency
            mode = 5
        else:
            raise ValueError('Unknown camera device')

        camera.resolution = (resx,resy)
        #camera.annotate_text = "RaspberryPi3 Camera"
        if show:
            preview = True
            camera.framerate  = 25
            display = picamtracker.Display(caption='piCAMTracker',x=config.conf['previewX'],y=config.conf['previewY'],w=resy/2,h=resx/2)
        else:
            display = None
            camera.framerate   = fps
            camera.sensor_mode = mode

        print("warm-up 2 seconds...")
        sleep(2.0)
        print("...start")

        if preview:
            cl = np.zeros((resy,resx,3), np.uint8)
            ycross = config.conf['yCross']
            if ycross > 0:
                ym = 16 * ycross
                cl[ym,:,:] = 0xff  #horizantal line
            xcross = config.conf['xCross']
            if xcross > 0:
                xm = 16 * xcross
                cl[:,xm,:]  = 0xff  #vertical line

            #- preview settings
            px = config.conf['previewX'] + config.conf['offsetX']
            py = config.conf['previewY'] + config.conf['offsetY']

            camera.start_preview()
            camera.preview.fullscreen = False
            #camera.preview.window = (50,100,resx,resy)
            if show:
                camera.preview.alpha = 192
            else:
                camera.preview.alpha = 255
            #camera.preview.window = (100,80,resy/2,resx/2)
            camera.preview.window = (px,py,resy/2,resx/2)
            camera.preview.rotation = 90

            #- overlay settings
            overlay = camera.add_overlay(source=np.getbuffer(cl),
                                         size=(resx,resy),format='rgb')
            overlay.fullscreen = False
            #overlay.window = (50,100,resx,resy)
            overlay.alpha = 32
            overlay.layer = 3
            #overlay.window = (100,80,resy/2,resx/2)
            overlay.window = (px,py,resy/2,resx/2)
            overlay.rotation= 90

        #- disable auto (exposure + white balance)
        #camera.shutter_speed = camera.exposure_speed
        #camera.exposure_mode = 'off'
        #g = camera.awb_gains
        #camera.awb_mode  = 'off'
        #camera.awb_gains = g

        vstream = picamera.PiCameraCircularIO(camera, seconds=config.conf['videoLength'])
        greenLED = picamtracker.GPIOPort.gpioPort(config.conf['greenLEDPort'])
        redLED = picamtracker.GPIOPort.gpioPort(config.conf['redLEDPort'])
        tracker = picamtracker.Tracker(camera, greenLed=greenLED, redLed=redLED, config=config)
        writer  = picamtracker.Writer(camera, stream=vstream, config=config)
        cmds    = picamtracker.CommandInterface(config=config)
        cmds.subscribe(tracker.set_maxDist, 'maxDist')
        cmds.subscribe(config.set_storeParams, 'storeParams')
        cmds.subscribe(greenLED.check, 'testBeep')

        with picamtracker.MotionAnalyser(camera, tracker, display, show, config) as output:
            loop = 0
            t_wait = 0.5
            old_frames = 0
            camera.annotate_text_size = 24
            #camera.annotate_frame_num = True
            camera.start_recording(vstream, 'h264', motion_output=output)
            cmds.subscribe(output.set_maxArea, 'maxArea')
            cmds.subscribe(output.set_minArea, 'minArea')
            cmds.subscribe(output.set_sadThreshold, 'sadThreshold')
            try:
                #writer.setupDecoder()
                while True:
                    loop += 1
                    # update statistics
                    if loop & 1:
                        add_text = ""
                        sep = ""
                        if tracker.noise > 0.4:
                            add_text += " NOISY"
                            sep = " +"
                        if camera.analog_gain > 7:
                            add_text = add_text + sep + " DARK"
                        if len(add_text):
                            add_text += "!"

                        frames = output.processed_frames
                        fs = (frames - old_frames)  / (2 * t_wait)
                        old_frames = frames
                        camera.annotate_text = "%s (%3.1f f/s) %s" % (dt.datetime.now().strftime('%Y-%m-%d %H:%M:%S'), fs, add_text)


                    frame,motion = tracker.getStatus()
                    if frame > 0:
                        t0 = time()
                        #camera.split_recording('after.h264')
                        #vstream.copy_to('before.h264',size=2147483648)
                        #vstream.copy_to('before.h264',size=1073741824)
                        #vstream.clear()
                        #camera.split_recording(vstream)
                        #name = "AAA-%d.jpg" % loop
                        #camera.capture(reader, format='rgb', use_video_port=True)

                        writer.takeSnapshot(frame, motion)
                        tracker.releaseLock()
                        #print("capture: %4.2fms" % (1000.0 * (time() - t0)))

                    camera.wait_recording(t_wait)

            except KeyboardInterrupt:
                pass
            finally:

                # stop camera and preview
                greenLED.terminated = True
                redLED.terminated = True
                camera.stop_recording()
                if preview:
                    camera.stop_preview()
                    camera.remove_overlay(overlay)
                # stop all threads
                if display is not None:
                    display.terminated = True
                cmds.stop()
                tracker.stop()
                writer.stop()
                # wait and join threads
                sleep(0.5)
                if display is not None:
                    display.join()
                greenLED.join()
                redLED.join()
                cmds.join()
                tracker.join()
                writer.join()
                #config.write()

if __name__ == '__main__':
    parser = ArgumentParser(prog='piCAMTracker')
    parser.add_argument('-s', '--show', action='store_true',
                      help   = 'show graphical debug information (slow!)')
    args = parser.parse_args()
    global config
    config = picamtracker.Configuration('config.json')
    os.system("[ ! -d /run/picamtracker ] && sudo mkdir -p /run/picamtracker && sudo chown pi:www-data /run/picamtracker && sudo chmod 775 /run/picamtracker")

    main(args.show)
