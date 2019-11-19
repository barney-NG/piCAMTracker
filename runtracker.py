#!/usr/bin/env python3
# vim: set et sw=4 sts=4 fileencoding=utf-8:
import picamera
import picamera.array
import numpy as np
import datetime as dt
import os
import io
import re
from time import sleep,time
import sys
import subprocess
from argparse import ArgumentParser
import picamtracker
import prctl

max_temp = 75.0
temp = 20.0

#-- start an arbitrary shell (Python-2.7 subprocess.Popen)
def shell(cmd, *argv):
    err = "Error for command: %s" % (cmd)

    command = [cmd]
    for item in argv:
      command.append(item)

    try:
        p = subprocess.Popen(command,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
        out,err = p.communicate()
        if(len(err)):
            print >> sys.stderr, err
    except:
        print >> sys.stderr, err
        out = ''

    return out.strip()

def get_raspi_revision():
    rev_file = '/sys/firmware/devicetree/base/model'
    info = { 'pi': '', 'model': '', 'rev': ''}
    try:
        fd = os.open(rev_file, os.O_RDONLY)
        line = os.read(fd,256)
        os.close(fd)
        #             Raspberry Pi 4 Model B Rev 1.1
        m = re.match('Raspberry Pi (\d+) Model (\w(?: Plus)?) Rev ([\d\.]+)', line)
        if m:
            info['pi'] = m.group(1)
            info['model'] = m.group(2)
            info['rev'] = m.group(3)
    except:
        pass

    return info

def get_temp():
    temp_file='/run/picamtracker/temp'
    temp = 20.0
    try:
        fd = os.open(temp_file, os.O_RDONLY)
        temp_string = os.read(fd,256)
        os.close(fd)
        temp = float(temp_string)
    except:
        pass

    return temp

def main(ashow=True, debug=False):
    global config
    preview = True
    show = 1 if ashow else 0
    try:
        preview = config.conf['preview']
    except:
        raise

    if debug:
        config.conf['debug'] = True

    print(get_raspi_revision())

    #- open picamera device
    with picamera.PiCamera() as camera:
        #- determine camera module
        revision = camera._revision.upper()
        print("camera chip: %s" % revision)
        if revision == 'OV5647':
            # V1 module
            # 1280x720 has a bug. (wrong center value)
            resx = 1280
            resy = 960
            fps  = 42
            mode = 4
        elif revision == 'IMX219':
            # V2 module
            resx = 1632
            resy = 896
            fps  = 40
            mode = 5
            #resx = 1280
            #resy = 720
            #fps  = 90
            #mode = 6 # this mode does not use the full FOV!            
        else:
            raise ValueError('Unknown camera device')

        #- check if the crossing line is in the center (this is not needed.)
        if config.conf['yCross'] > 0 and config.conf['yCross'] != (resy/32):
            print("WARNING: Y crossing %d expected but %d given!" % (resy/32, config.conf['yCross']))

        if config.conf['xCross'] > 0 and config.conf['xCross'] != (resx/32):
            print("WARNING: X crossing bar is not in the center of the screen!")

        #if 'fps' in config.conf and config.conf['fps'] > 0 and config.conf['fps'] < fps:
        #    fps = config.conf['fps']

        camera.resolution = (resx,resy)

        if show:
            preview = True
            camera.framerate  = 25
            x_disp = config.conf['offsetX']
            y_disp = config.conf['offsetY']
            display = picamtracker.Display(caption='piCAMTracker',x=x_disp,y=y_disp,w=resy/2,h=resx/2)
        else:
            display = None
            camera.sensor_mode = mode
            camera.framerate_range = (25, fps)

        act_fps = fps
        
        print("warm-up 2 seconds...")
        #serialPort = picamtracker.SerialIO.SerialCommunication(port=config.conf['serialPort'],options=config.conf['serialConf'])
        greenLED = picamtracker.GPIOPort.gpioPort(config.conf['greenLEDPort'],
            is_active_low=config.conf['ledActiveLow'],
            duration=config.conf['signalLength'],
            start_blinks=3)
        redLED = picamtracker.GPIOPort.gpioPort(config.conf['redLEDPort'],
            duration=config.conf['signalLength'],
            is_active_low=config.conf['ledActiveLow'])
        sleep(1.0)
        print("...start")
        picamtracker.GPIOPort.statusLED(config.conf['statusLEDPort'], on=True)

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
            px = config.conf['previewX']
            py = config.conf['previewY']

            camera.start_preview()
            camera.preview.fullscreen = False
            if show:
                camera.preview.alpha = 192
            else:
                camera.preview.alpha = 255

            rotation = config.conf['viewAngle']
            camera.preview.window = (px,py,int(resy/2),int(resx/2))
            camera.preview.rotation = rotation

            #- overlay settings
            overlay = camera.add_overlay(source=cl.tobytes(),size=(resx,resy),format='rgb')
            
            #overlay = camera.add_overlay(source=np.getbuffer(cl),
            #                             size=(resx,resy),format='rgb')
            overlay.fullscreen = False
            overlay.alpha = 32
            overlay.layer = 3
            overlay.window = (px,py,int(resy/2),int(resx/2))
            overlay.rotation= rotation

        #- set exposure mode
        camera.exposure_compensation = 5
        camera.exposure_mode = 'auto'
        #camera.exposure_mode = 'sports'
        #camera.exposure_mode = 'backlight
        #camera.contrast = 25
        
        vstream = picamera.PiCameraCircularIO(camera, seconds=config.conf['videoLength'])
        if 'IPUDPBEEP' in config.conf and re.match('.*\.255$', config.conf['IPUDPBEEP']):
            print("setup UDP Broadcast")
            udpThread = picamtracker.UDPBeep.udpBeep(config.conf['IPUDPBEEP'], 4445)
            udpThread.event.set ()
        else:
            udpThread = None

        writer = picamtracker.Writer(camera, stream=vstream, config=config)
        tracker = picamtracker.Tracker(camera, greenLed=greenLED, redLed=redLED, config=config, udpThread=udpThread, writer=writer)
        cmds = picamtracker.CommandInterface(config=config)
        cmds.subscribe(tracker.set_maxDist, 'maxDist')
        cmds.subscribe(tracker.set_trackMaturity, 'trackMaturity')
        cmds.subscribe(tracker.testCrossing, 'testBeep')
        cmds.subscribe(config.set_storeParams, 'storeParams')


        with picamtracker.MotionAnalyser(camera, tracker, display, show, config) as output:
            loop = 0
            t_wait = 0.5
            old_frames = 0
            #auto_mode = 10
            auto_mode = -1
            last_auto_mode = time()
            camera.annotate_text_size = 24
            camera.start_recording(output=vstream, format='h264', level='4.2', motion_output=output)
            cmds.subscribe(output.set_vMax, 'vMax')
            cmds.subscribe(output.set_vMin, 'vMin')
            cmds.subscribe(output.set_maxArea, 'maxArea')
            cmds.subscribe(output.set_minArea, 'minArea')
            cmds.subscribe(output.set_sadThreshold, 'sadThreshold')
            cmds.subscribe(output.set_debug, 'debug')
            cmds.subscribe(output.set_baseB, 'baseB')
            cmds.subscribe(output.set_exposure, 'exposure')
            
            if config.conf['debugInputPort']:
                picamtracker.GPIOPort.addCallback(config.conf['debugInputPort'], output.debug_button)
            prctl.set_name('python')

            try:
            
                while True:
                    global temp
                    loop += 1
                    # check temperature every minute
                    if loop % 120 == 0:
                        temp = get_temp()
                    # update statistics every second
                    if loop & 1:
                        add_text = ""
                        sep = ""
                        if tracker.noise > 0.8:
                            add_text += " NOISY"
                            sep = " +"
                        if camera.analog_gain > 7:
                            add_text = add_text + sep + " DARK"
                        if temp > max_temp:
                            add_text = add_text + sep + " HOT (%4.1f)" % temp
                        if len(add_text):
                            add_text += " !"

                        frames = output.processed_frames
                        fs = (frames - old_frames)  / (2 * t_wait)
                        old_frames = frames
                        camera.annotate_text = "%s (%3.1f f/s) %s" % (dt.datetime.now().strftime('%Y-%m-%d %H:%M:%S'), fs, add_text)
                        
                        # exposure mode
                        if auto_mode >= 0:
                            auto_mode -= 1
                            if auto_mode == 0:
                                print("auto_mode: off")
                                camera.exposure_mode = 'off'
                                camera.shutter_speed = camera.exposure_speed
                                g = camera.awb_gains
                                camera.awb_mode  = 'off'
                                camera.awb_gains = g


                    delay,frame,motion = tracker.getStatus()
                    if frame != 0:
                        #print("crossing for frame: %d" % frame)
                        t0 = time()
                        #camera.split_recording('after.h264')
                        #vstream.copy_to('before.h264',size=2147483648)
                        #vstream.copy_to('before.h264',size=1073741824)
                        #vstream.clear()
                        #camera.split_recording(vstream)
                        #name = "AAA-%d.jpg" % loop
                        #camera.capture(reader, format='rgb', use_video_port=True)
                        
                        writer.takeSnapshot(delay, frame, motion)
                        tracker.releaseLock()
                        
                        #print("capture: %4.2fms" % (1000.0 * (time() - t0)))
                        #auto_diff = time() - last_auto_mode
                        #if auto_diff > 180 and auto_mode < 0:
                        #    camera.exposure_mode = 'auto'
                        #    camera.exposure_compensation = 5
                        #    camera.awb_mode  = 'auto'
                        #    auto_mode = 10
                        #    last_auto_mode = time()
                        #    print("auto_mode: on")

                    # check for USB stick every 60 seconds

                    camera.wait_recording(t_wait)

            except KeyboardInterrupt:
                pass

            finally:
                # stop camera and preview
                #serialPort.terminated = True
                greenLED.terminated = True
                redLED.terminated = True
                if udpThread:
                    udpThread.terminated = True
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
                #serialPort.join()
                if udpThread:
                    udpThread.join()
                greenLED.join()
                redLED.join()
                cmds.join()
                tracker.join()
                writer.join()
                picamtracker.GPIOPort.statusLED(config.conf['statusLEDPort'], on=False)
                #config.write()

if __name__ == '__main__':
    parser = ArgumentParser(prog='piCAMTracker')
    parser.add_argument('-s', '--show', action='store_true',
                      help   = 'show internal graphical information (slow!)')
    parser.add_argument( '-d', '--debug', action='store_true',
                      help = 'write debug information for later investigation')
    args = parser.parse_args()
    global config
    config = picamtracker.Configuration('config.json')
    config.write()
    os.system("[ ! -d /run/picamtracker ] && sudo mkdir -p /run/picamtracker && sudo chown pi:www-data /run/picamtracker && sudo chmod 775 /run/picamtracker")
    os.system("/home/pi/piCAMTracker/etc/background_service.sh </dev/null&")
    out = shell('/usr/bin/vcgencmd', 'measure_temp')
    print("Actual core %s" % out)


    main(args.show, args.debug)
