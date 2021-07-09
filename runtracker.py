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
from websock import WebUtilities
import logging
import logging.handlers

logging.basicConfig(filename = '/dev/null')
#handler = logging.handlers.RotatingFileHandler(LOGFILENAME,maxBytes = 3*1024*1024, backupCount = 2)
#logging.basicConfig(filename = LOGFILENAME,
#                format='%(asctime)s: %(module)s::%(funcName)s: %(message)s',
#                handlers = [handler],
#                level=logging.DEBUG,
#                datefmt='%Y/%m/%d %H:%M:%S')
                
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
            logging.error(err)
    except:
        logging.error(err)
        out = ''

    return out.strip()

def get_screen_resolution():
    w = h = 0
    cmd = "/home/pi/piCAMTracker/etc/get_screen_resolution.sh"
    output = shell(cmd)
    m = re.match('(\d+) (\d+)', output.decode())
    if m:
        w = int(m.group(1))
        h = int(m.group(2))

    logging.debug("screen resolution: w: %d, h: %d" % (w,h))

    return (w,h)

def get_raspi_revision():
    rev_file = '/sys/firmware/devicetree/base/model'
    info = { 'pi': '', 'model': '', 'rev': ''}
    try:
        fd = os.open(rev_file, os.O_RDONLY)
        line = os.read(fd,256)
        os.close(fd)
        #             Raspberry Pi 4 Model B Rev 1.1
        #m = re.match('Raspberry Pi (\d+) Model (\w(?: Plus)?) Rev ([\d\.]+)', line)
        line = line.decode()
        m = re.match('Raspberry Pi (\d+) Model ([A-Z]+(: Plus)?) Rev (\d+(\.\d+)?)', line)
        if m:
            info['pi'] = m.group(1)
            info['model'] = m.group(2)
            info['rev'] = m.group(4)
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

def setFastMode(value):
        """
        callback to start/stop fastmode
        """
        global set_fastmode
        global rerun_main
        if value > 0:
            logging.info("setFastMode:On")
            if set_fastmode == False:
                set_fastmode = True
                rerun_main = True
        else:
            logging.info("setFastMode:Off")
            if set_fastmode == True:
                set_fastmode = False
                rerun_main = True

def evaluateLoggingLevel(level):
    if isinstance(level, str):
        levels = {"CRITICAL":logging.CRITICAL,"ERROR":logging.ERROR,"WARNING":logging.WARNING,"INFO":logging.INFO,"DEBUG":logging.DEBUG}
        level = level.upper()
        if level in levels:
            return levels[level]
        logging.error("wrong logging level: %s", level)
    elif isinstance(level, int):
        levels = [logging.CRITICAL,logging.ERROR,logging.WARNING,logging.INFO,logging.DEBUG]
        if level in levels:
            return level
        logging.error("wrong logging level: %d", level)
        
    return logging.WARNING

def setLoggingLevel(level):
    """
    call back to set logging level
    """
    log_level = evaluateLoggingLevel(level)
    log_root = logging.getLogger() 
    log_root.setLevel(log_level)

def main(ashow=True, debug=False, fastmode=False, wsserver=None, logfilename=None):
    global config
    global rerun_main
    rerun_main = False
    show = 1 if ashow else 0
    
    # here we go
    logging.info("<<<<<<<<<<<<<< starting tracker >>>>>>>>>>>>")

    if fastmode == False:
        fastmode = config.conf['fastMode']
        
    if debug:
        config.conf['debug'] = True

    # I am testing if camera,capture is fast enough for us
    capture = False
    
    # where are we
    logging.info(get_raspi_revision())
    # get screen resolution (0,0) if no monitor is connected
    screen_w,screen_h = get_screen_resolution()
    # preview
    #preview = False if(screen_w == 0 and screen_h == 0) else config.conf['preview']
    preview = config.conf['preview']
    
    # annotation
    an_height = 24
    an_black = picamera.Color('black')
    an_white = picamera.Color('white')
 
    #- open picamera device
    with picamera.PiCamera() as camera:
        #- determine camera module
        revision = camera._revision.upper()
        logging.info("camera chip: %s" % revision)
        if revision == 'OV5647':
            # V1 module
            # 1280x720 has a bug. (wrong center value)
            if fastmode:
                # Full FOV! Area = 1200
                resx = 640 # 40
                resy = 480 # 30
                fps  = 90
                mode = 7
                an_height = 16
            else:
                # Full FOV Area = 4800
                resx = 1280 # 80
                resy = 960 # 60
                fps  = 42
                mode = 4
        elif revision == 'IMX219':
            # V2 module
            if fastmode:
                # 50% FOV :-/ Area = 1200
                #resx = 640 # 40
                #resy = 480 # 30
                #fps  = 120
                #mode = 7 
                #an_height = 16
                # 80% FOV
                resx = 928 # 960 # 60 # 58 Area = 2552
                resy = 704 # 720 # 45 # 44
                fps  = 90
                mode = 6
            else:
                # full FOV Area = 102x56 = 5712
                resx = 1632 # 102
                resy = 896 # 56
                fps  = 40
                mode = 5
        else:
            raise ValueError('Unknown camera device')


        # evaluate crossing parameters
        ycross = config.conf['yCross']
        xcross = config.conf['xCross']

        # fastmode: if needed force x/ycross to middle of the screen
        if fastmode:
            if ycross > 0:
                # force ycross to the middle
                ycross = resy/32
                config.conf['yCross'] = int(ycross)
                logging.warning("fastmode: force yCross to %d" % ycross)
            if xcross > 0:
                # force xcross to the middle
                xcross = resx/32
                config.conf['xCross'] = int(xcross)
                logging.warning("fastmode: force xCross to %d" % xcross)

        #- check if the crossing line is in the center (this is not needed.
        if ycross > 0 and ycross != (resy/32):
            logging.warning("Y crossing %d expected but %d given!" % (resy/32, ycross))

        if xcross > 0 and xcross != (resx/32):
            logging.warning("X crossing %d expected but %d given!" % (resx/32, xcross))

        # setup camera resolution
        logging.info("camera resolution: %dx%d" % (resx,resy))
        camera.resolution = (resx,resy)

        # debugging mode
        if show:
            preview = True
            camera.framerate  = 25
            x_disp = config.conf['offsetX']
            y_disp = config.conf['offsetY']
            width = resy/2
            height = resx/2
            #if fastmode:
            #    width = resy
            #    height = resx
            display = picamtracker.Display(caption='piCAMTracker',x=x_disp,y=y_disp,w=width,h=height)
        else:
            display = None
            camera.sensor_mode = mode
            if fastmode:
                camera.framerate_range = (60, fps)
            else:
                camera.framerate_range = (25, fps)
  
        logging.info("warm-up 2 seconds...")

        # setup serial port
        #serialPort = picamtracker.SerialIO.SerialCommunication(port=config.conf['serialPort'],options=config.conf['serialConf'])

        # setup GPIO
        greenLED = picamtracker.GPIOPort.gpioPort(config.conf['greenLEDPort'],
            is_active_low=config.conf['ledActiveLow'],
            duration=config.conf['signalLength'],
            start_blinks=3)
        redLED = None
        #redLED = picamtracker.GPIOPort.gpioPort(config.conf['redLEDPort'],
        #    duration=config.conf['signalLength'],
        #    is_active_low=config.conf['ledActiveLow'])
        sleep(1.0)
        picamtracker.GPIOPort.statusLED(config.conf['statusLEDPort'], on=True)

        logging.info("starting camera ...")

        # setup preview
        if preview:
            cl = np.zeros((resy,resx,3), np.uint8)
            ycross = config.conf['yCross']
            if ycross > 0:
                if ycross >= int(resy/16):
                    ycross = int(resy/32)
                ym = 16 * ycross
                cl[ym,:,:] = 0xff  #horizantal line

            xcross = config.conf['xCross']
            if xcross > 0:
                if xcross >= int(resx/16):
                    xcross = int(resx/32)
                xm = 16 * xcross
                cl[:,xm,:]  = 0xff  #vertical line

            #- preview settings
            px = int(config.conf['previewX'])
            py = int(config.conf['previewY'])
            if resx < 800:
                pw = int(resx)
                ph = int(resy)
            else:
                pw = int(resx/2)
                ph = int(resy/2)

            rotation = int(config.conf['viewAngle'])
            if rotation == 90 or rotation == 270:
                hh = pw; pw = ph; ph = hh
            
            logging.info("preview w: %d, h: %d" % (pw,ph))

            camera.start_preview(fullscreen=False,window=(px,py,pw,ph),rotation=rotation)
            #camera.preview.fullscreen = False
            if show:
                camera.preview.alpha = 192
            else:
                camera.preview.alpha = 255

            #- overlay settings
            overlay = camera.add_overlay(source=cl.tobytes(),size=(resx,resy),format='rgb')            
            overlay.fullscreen = False
            overlay.alpha = 32
            overlay.layer = 3
            overlay.window = (px,py,pw,ph)
            overlay.rotation= rotation

        # set exposure mode
        #camera.exposure_mode = 'auto'
        camera.exposure_mode = 'sports'
        camera.exposure_compensation = config.conf["exposure"]
        # >>> debug
        # camera.annotate_frame_num = True
        
        # setup UDP broadcaster
        if 'IPUDPBEEP' in config.conf and re.match('.*\.255$', config.conf['IPUDPBEEP']):
            udpThread = picamtracker.UDPBeep.udpBeep(config.conf['IPUDPBEEP'], 4445)
            udpThread.event.set ()
        else:
            udpThread = None

        # setup used objects 
        vstream = picamera.PiCameraCircularIO(camera, seconds=config.conf['videoLength'], bitrate=50000000)
        writer = picamtracker.Writer(camera, stream=vstream, config=config, wsserver=wsserver)
        vwriter = picamtracker.vWriter(stream=vstream, config=config)        
        tracker = picamtracker.Tracker(camera, greenLed=greenLED, redLed=redLED, config=config, udpThread=udpThread, capture=capture)
       

        # assign external command interface
        cmds = picamtracker.CommandInterface(config=config)
        cmds.subscribe(tracker.set_maxDist, 'maxDist')
        cmds.subscribe(tracker.set_trackMaturity, 'trackMaturity')
        cmds.subscribe(tracker.testCrossing, 'testBeep')
        cmds.subscribe(config.set_storeParams, 'storeParams')
        cmds.subscribe(setFastMode, 'fastMode')
        cmds.subscribe(setLoggingLevel, 'loggingLevel')


        # enable overwritten camera's analyse callback 
        with picamtracker.MotionAnalyser(camera, tracker, display, show, config, vwriter) as output:
            prctl.set_name('python')
            # local variables
            loop = 0
            t_wait = 0.5
            old_frames = 0
            auto_mode = -1
            #if fastmode:
            #    auto_mode = 10
            last_auto_mode = time()
            fps = 25.0
            
            # start camera
            camera.annotate_text_size = an_height
            camera.annotate_foreground = an_white if camera.analog_gain > 5 else an_black
            camera.start_recording(output=vstream, format='h264', level='4.2', motion_output=output)

            # assign external commands to internal functions
            cmds.subscribe(output.set_vMax, 'vMax')
            cmds.subscribe(output.set_vMin, 'vMin')
            cmds.subscribe(output.set_maxArea, 'maxArea')
            cmds.subscribe(output.set_minArea, 'minArea')
            cmds.subscribe(output.set_sadThreshold, 'sadThreshold')
            cmds.subscribe(output.set_debug, 'debug')
            cmds.subscribe(output.set_baseB, 'baseB')
            cmds.subscribe(output.set_exposure, 'exposure')
            cmds.subscribe(output.set_extend, 'extension')

            # assign GPIO pin to enable debugging
            if config.conf['debugInputPort']:
                picamtracker.GPIOPort.addCallback(config.conf['debugInputPort'], output.debug_button)
            
            try:
                # go into endless camera recording loop and wake up every t_wait seconds
                while True:
                    
                    global temp
                    loop += 1
                    # check temperature/light every minute
                    if loop % 120 == 0:
                        temp = get_temp()
                        camera.annotate_foreground = an_white if camera.analog_gain > 5 else an_black
                        logging.debug("analog_gain: %3.1f exposure_speed: %d (%3.1f fps)" % (float(camera.analog_gain),camera.exposure_speed,fps))
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
                        fps = (frames - old_frames)  / (2 * t_wait)
                        old_frames = frames
                        camera.annotate_text = "%s (%3.1f fps) %s" % (dt.datetime.now().strftime('%Y-%m-%d %H:%M:%S'), fps, add_text)
                        # check for restart
                        if rerun_main:
                            break
                    
                    # crossing event happend?
                    delay,updates,frame,motion = tracker.getStatus()
                    if frame != 0:
                        # if crossing detected -> take a snapshot of the event
                        #t0 = time()
                        if capture:
                            writer.update_hits(delay, updates, frame, motion, tracker.image.copy())
                        else:
                            writer.takeSnapshot(delay, updates, frame, motion)
                        tracker.releaseLock()
                        #print("capture time: %4.2fms" % (1000.0 * (time() - t0)))

                    # check for USB stick every 60 seconds

                    camera.wait_recording(t_wait)

            except KeyboardInterrupt:
                logging.error("Got keyboard interrupt")
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
                vwriter.stop()
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
                vwriter.join()

                picamtracker.GPIOPort.statusLED(config.conf['statusLEDPort'], on=False)
                picamtracker.GPIOPort.cleanup()
                #config.write()

    logging.info("<<<<<<<<<<<< tracker ended >>>>>>>>>>>>")
    return rerun_main

if __name__ == '__main__':
    global config
    global set_fastmode

    # parse command line arguments
    parser = ArgumentParser(prog='piCAMTracker')
    parser.add_argument('-s', '--show', action='store_true',
                      help   = 'show internal graphical information (slow!)')
    parser.add_argument( '-d', '--debug', action='store_true',
                      help = 'write debug information for later investigation')
    parser.add_argument( '-f', '--fast', action='store_true',
                      help = 'run in 640x480 mode as fast as possible')
    parser.add_argument( '-c', '--config', type=str, default="config.json",
                      help = 'load specific config (default config.json)')
    args = parser.parse_args()

    # update config file (or create it if it does not exists)
    if args.fast:
        set_fastmode = True
        # set default config for fast mode
        if args.config == 'config.json':
            args.config = 'config.fast.json'
    else:
        set_fastmode = False

    config = picamtracker.Configuration(args.config)
    config.write()
    
    # prepare OS environment
    os.system("[ ! -d /run/picamtracker ] && sudo mkdir -p /run/picamtracker && sudo chown pi:www-data /run/picamtracker && sudo chmod 775 /run/picamtracker")

    # create directory under /var/log and adapt access rights
    logdir = '/var/log/picamtracker'
    logfile = 'picamtracker.log'
    mkdir_cmd = "[ ! -d %s ] && sudo mkdir -p %s && sudo chown -R pi:pi %s" % (logdir, logdir, logdir)
    mklog_cmd = "sudo touch %s/%s && sudo chown pi:pi %s/%s" % (logdir,logfile,logdir,logfile)
    os.system(mkdir_cmd)
    os.system(mklog_cmd)
    
    # configure logging
    logfilename = "%s/%s" % (logdir,logfile)
    log_root = logging.getLogger() 
    log_handler = logging.handlers.RotatingFileHandler(logfilename, maxBytes = 3*1024*1024, backupCount = 2)
    log_formatter = logging.Formatter('%(asctime)s: %(module)s::%(funcName)s: %(message)s',datefmt='%Y/%m/%d %H:%M:%S')
    log_handler.setFormatter(log_formatter)
    log_root.addHandler(log_handler)
    log_level = evaluateLoggingLevel(config.conf["loggingLevel"])
    log_root.setLevel(log_level)

    # start acces point if requested
    if 'accessPoint' in config.conf and config.conf['accessPoint'] == True and 'ssid' in config.conf and len(config.conf['ssid']) >= 3:
            cmd = "sudo /home/pi/piCAMTracker/etc/start-access-point.sh --ssid %s" % config.conf['ssid']
            os.system(cmd.encode())
    os.system("/home/pi/piCAMTracker/etc/background_service.sh </dev/null&")

    # create a config.fast.json
    os.system("[ -e config.json -a ! -e config.fast.json ] && cp config.json config.fast.json")
    os.system("[ -e %s ] && (cd www && ln -sf ../%s config.json)" % (args.config,args.config))
     
    # just a test ...
    # out = shell('/usr/bin/vcgencmd', 'measure_temp')
    # print("Actual core %s" % out)

    # start websocket thread outside of main loop. (protect it from switching fast/normal) 
    wserver = WebUtilities.TrackerWS(config=config, port=8084)

    # run main part until end requested
    while main(args.show, args.debug, args.fast, wsserver=wserver, logfilename=logfilename):
        reread_config = False
        if set_fastmode and not args.fast:
            # switching to fast mode requested by web interface...
            args.config = 'config.fast.json'
            reread_config = True
        if args.fast and not set_fastmode:
            # switching to 'normal' mode requested by web interface...
            args.config = 'config.json'
            reread_config = True
        if reread_config:
            logging.info("using <%s> as config", args.config)
            os.system("[ -e %s ] && (cd www && ln -sf ../%s config.json)" % (args.config,args.config))
            config = picamtracker.Configuration(args.config)
            config.write()
        args.fast = set_fastmode
        logging.info("restarting tracker...")
        logfilename = None

    # stop websocket thread
    wserver.stop()
    wserver.close_server()
