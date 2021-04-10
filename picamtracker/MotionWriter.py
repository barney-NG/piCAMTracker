# vim: set et sw=4 sts=4 fileencoding=utf-8:
#
# Python Motion Writer module of the piCAMTracker package
# Copyright (c) 2017-2018 Axel Barnitzke <barney@xkontor.org>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the copyright holder nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from __future__ import (
    unicode_literals,
    print_function,
    division,
    absolute_import,
    )

# Make Py2's str equivalent to Py3's
str = type('')
try:
    range = xrange
except NameError:
    pass

import sys
import threading
from time import sleep
from collections import deque
#from picamera.frames import PiVideoFrame, PiVideoFrameType, PiCameraMMALError
#from picamera import mmal,mmalobj as mo
import numpy as np
import logging
import re
from picamtracker import Display
if sys.version_info < (3,0):
    from picamtracker.python2 import libh264decoder as h264decoder
else:
    from picamtracker import libh264decoder as h264decoder

import cv2
import gc
import prctl

class Writer(threading.Thread):
    """
    take a snapshot from the circular buffer stream (h264)
    """
    #--------------------------------------------------------------------
    #-- constructor
    #--------------------------------------------------------------------
    def __init__(self, camera, stream=None, config=None, wsserver=None):
        super(Writer,self).__init__()
        self.name = 'Writer'
        self.config = config
        self.doStreaming = False
        self.stream = stream
        self.camera = camera
        self.wsserver = wsserver
        self.lastFrame = 0
        self.resx = camera.resolution[0]
        self.resy = camera.resolution[1]
        self.imgtemplate = '/run/picamtracker/mjpeg%03d.jpg'
        self.imgctrl_file = '/run/picamtracker/act_image.name'
        self.video_template = '/home/pi/piCAMTracker/media/videos/video%03d.h264'
        self.img_save_ctrl_file = '/home/pi/piCAMTracker/media/stills/act_image.name'

        #self.display = Display('Debug',10,10)
        self.image = np.ones((self.resx,self.resy,3), dtype=np.uint8) * 220
        self.imagesize = self.resx * self.resy * 3
        #self.capture = np.empty((self.resx*self.resy*3,),dtype=np.uint8)
        self.decoder = h264decoder.H264Decoder()
        self.isCut = False
        self.written = False
        self.maxDiff = 25 # empiric value :-/
        self.ycross = -1
        self.xcross = -1
        self.k = 0
        self.nimages = config.conf['maxSnapshots']
        self.nbimage = 0
        self.nvideos = 10
        self.nbvideo = 0
        gc.disable()
        if config.conf['viewAngle'] == 90:
            self.k = -1
        if config.conf['viewAngle'] == 180:
            self.k = 2
        if config.conf['viewAngle'] == 270:
            self.k = 1
        prctl.set_name('ptrk.Writer')

        try:
            with open(self.img_save_ctrl_file, "r") as fd:
                img_name = fd.readline()
                match = re.match('.*/mjpeg(\d+).jpg', img_name)
                if match:
                    img_no = int(match.group(1))
                    logging.info("last image number: %d", img_no)
                    self.nbimage = img_no
        except:
            pass
            
        #- do things according configuration
        if config is not None:
            self.ycross = config.conf['yCross'] * 16
            self.xcross = config.conf['xCross'] * 16

        #- thread initialisation stuff
        self.lock = threading.Lock()
        self.event = threading.Event()
        self.event.clear()
        self.q = deque()
        self.terminated = False
        self.daemon = True
        self.start()

    #--------------------------------------------------------------------
    #-- stop sub threads
    #--------------------------------------------------------------------
    def __del__(self):
        return

    def check(self):
        ret_code = self.written
        logging.info("writer::check (%d)" % ret_code)
        with self.lock:
            if self.written:
                self.written = False
        return ret_code
        
    #--------------------------------------------------------------------
    #-- queue new points and feed worker
    #--------------------------------------------------------------------
    def update_hits(self, delay,updates,framenb, motion, image=None):
        self.q.append([delay,updates,framenb,motion,image])
        self.event.set()

    #--------------------------------------------------------------------
    #-- stop all threading stuff
    #--------------------------------------------------------------------
    def stop(self):
        self.terminated = True
        self.q.append([[0,0,0,0],[0,0],[0,0,0,0]])

    #--------------------------------------------------------------------
    #-- Thread run function called from takesnapshot
    #--------------------------------------------------------------------
    def run(self):
        while not self.terminated:
            try:
                delay,updates,framenb,motion,cam_image = self.q.popleft()

                if cam_image is not None:
                    image = cam_image
                else:
                    fdata = None
                    with self.lock:
                        fdata,nbytes = self.decoder.decode_frame(self.frame2decode)

                    
                    image = self.image
                    image.fill(220)
                    
                    if fdata:
                        (frame,w,h,ls) = fdata
                        if frame:
                            image = np.fromstring(frame, dtype=np.uint8)
                
                w = self.resx
                h = self.resy    
                image = image.reshape((h,w,3))
                re = motion[0]
                vv = motion[1] * 16
                mm = motion[2]

                x0 = re[0]
                y0 = re[1]
                w  = re[2]
                h  = re[3]
                xmin = mm[0] * 16
                ymin = mm[1] * 16
                xmax = mm[2] * 16
                ymax = mm[3] * 16
                x1 = x0 + w
                y1 = y0 + h
                x0 *= 16
                x1 *= 16
                y0 *= 16
                y1 *= 16

                # center line
                if self.ycross > 0:
                    cv2.line(image,(0,int(self.ycross)),(self.resx,int(self.ycross)),(0,0,0),1)

                if self.xcross > 0:
                    cv2.line(image,(int(self.xcross),0),(int(self.xcross),self.resy),(0,0,0),1)

                if framenb < 0:
                    framenb = -framenb
                    #cv2.rectangle(image,(x0,y0),(x1,y1),(20,20,220),1)
                    cv2.rectangle(image,(xmin,ymin),(xmax,ymax),(20,20,220),1)
                else:
                    cv2.rectangle(image,(xmin,ymin),(xmax,ymax),(200,200,200),1)
                    # >> debug
                    #if xmax - xmin > 32:
                    #    for x in range(xmin+16,xmax,16):
                    #        cv2.line(image,(x,ymin),(x,ymax),(200,200,200),1,cv2.LINE_4)
                    #if ymax - ymin > 32:
                    #    for y in range(ymin+16,ymax,16):
                    #        cv2.line(image,(xmin,y),(xmax,y),(200,200,200),1)
                    # << debug        
                    cv2.rectangle(image,(x0,y0),(x1,y1),(20,220,20),1)
                    
                #txt = "%4.1fms" % (delay*1000.0)
                txt = "%d/%4.1f" % (updates,np.linalg.norm(motion[1]))
                cv2.putText(image,txt,(int(x0), int(y0)),cv2.FONT_HERSHEY_SIMPLEX,0.3,(20,220,20),1)

                #txt = "%d" % (framenb/2)
                #xpos = int(self.resx/2) - len(txt) * 8
                #cv2.putText(image,txt,(xpos, 120),cv2.FONT_HERSHEY_SIMPLEX,0.7,(220,220,220),2)
                
                xm = int((x1+x0) / 2)
                ym = int((y1+y0) / 2)
                xe = int(xm - vv[0])
                ye = int(ym - vv[1])
                cv2.arrowedLine(image,(xm,ym),(xe,ye),(20,220,20),1)
                #image = cv2.resize(image,None,fx=0.5,fy=0.5,interpolation=cv2.INTER_LINEAR)
                image = np.rot90(image,self.k)
                imagepath = self.imgtemplate % self.nbimage
                if self.nbimage > self.nimages:
                    self.nbimage = 0
                else:
                    self.nbimage += 1

                try:
                    cv2.imwrite(imagepath, image, [cv2.IMWRITE_JPEG_QUALITY, 90])
                    logging.info("image: %s" % imagepath)
                except:
                    logging.error("cannot write %s" % imagepath)
                    pass

                try:
                    fs = open(self.imgctrl_file, "w")
                    fs.write(imagepath)
                    fs.close()
                except:
                    logging.error("cannot write %s" % self.imgctrl_file)
                    pass
                    
                if self.wsserver:
                    self.wsserver.broadcast(imagepath)

                #- do garbage collection here!
                c0,c1,c2 = gc.get_count()
                t0,t1,t2 = gc.get_threshold()
                if c0 > t0 or c1 > t1 or c2 > t2:
                    gc.collect()
                self.written = True

            except IndexError:
                self.event.clear()
                self.event.wait(1)

    #--------------------------------------------------------------------
    #-- lock stream and find frame to take as snapshot
    #--------------------------------------------------------------------
    #def takeSnapshot(self, framenb):
    def takeSnapshot(self, delay, updates, framenb, motion):
        n = 0
        fmin = 9999999
        fmax = 0
        record = False
        i_size = 0
        minus_max_diff = -2 * self.maxDiff
        isCut = False
        
        if framenb < 0:
            framenb = -framenb
            self.isCut = True
        
        sleep(0.1) # wait for keyframe is written in circular buffer
        # lock stream by reading it
        with self.stream.lock:
            for frame in reversed(self.stream.frames):
                n += 1
                index = frame.index
                ftype = frame.frame_type
                fmin = min(fmin,index)
                fmax = max(fmax,index)
                
                if not record:
                    diff = 99999
                    # sps frame
                    if ftype == 1:
                        i_size = frame.frame_size
                    # i-frame
                    if ftype == 2:
                        diff = index - framenb
                        #logging.debug("index: %d type 2 diff: %d" % (index,diff))

                    # negative diff -> frame was created before event
                    # positive diff -> frame was created after event
                    # we allow i-frames created very short before the crossing event
                    if diff > -10 and diff <= self.maxDiff:
                        logging.debug("found sps frame @ %d (delta:%d)",index,diff)
                        record = True
                    # stop loop if difference is too big
                    if diff < minus_max_diff:
                        break
                        
                # decode the next sps + i-frame
                if record:
                    save_pos = self.stream.tell()
                    pos = frame.position
                    szs = frame.frame_size + i_size + 32
                    self.stream.seek(pos)
                    with self.lock:
                        self.frame2decode = self.stream.read(szs)
                        if self.isCut:
                            framenb = -framenb
                        self.update_hits(delay, updates, framenb, motion)
                    self.stream.seek(save_pos)
                    break

        if record == False:
            logging.error("%d frames searched (%d < _%d_ < %d) no i-frame found!" % (n,fmin,framenb,fmax))
        
        return record
