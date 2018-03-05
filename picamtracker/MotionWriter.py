# vim: set et sw=4 sts=4 fileencoding=utf-8:
#
# Python Motion Writer module of the pyCAMTracker package
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
from picamera.frames import PiVideoFrame, PiVideoFrameType, PiCameraMMALError
from picamera import mmal,mmalobj as mo
import numpy as np
from picamtracker import Display
import pygame as pg
import pygame.image
import cv2
from picamtracker import libh264decoder

class Writer(threading.Thread):
    """
    take a snapshot from the circular buffer stream (h264)
    """
    #--------------------------------------------------------------------
    #-- constructor
    #--------------------------------------------------------------------
    def __init__(self, camera, stream=None, config=None):
        super(Writer,self).__init__()
        self.config = config
        self.doStreaming = False
        self.stream = stream
        self.camera = camera
        self.lastFrame = 0
        self.resx = camera.resolution[0]
        self.resy = camera.resolution[1]
        self.imgpath = '/run/picamtracker/mjpeg.jpg'
        #self.display = Display('Debug',10,10)
        self.image = np.empty((self.resx,self.resy,3), dtype=np.uint8)
        self.decoder = libh264decoder.H264Decoder()
        self.frame2decode = None
        self.maxDiff = 15
        self.ycross = None

        #pygame.init()
        #self.screen = pygame.display.set_mode((1280,720))


        #- do things according configuration 
        if config is not None:
            self.doStreaming = config.conf['streamServer']
            self.ycross = config.conf['yCross'] * 16

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

    #--------------------------------------------------------------------
    #-- queue new points and feed worker
    #--------------------------------------------------------------------
    def update_hits(self, framenb, motion):
        self.q.append([framenb,motion])
        self.event.set()

    #--------------------------------------------------------------------
    #-- stop all threading stuff
    #--------------------------------------------------------------------
    def stop(self):
        self.terminated = True
        self.q.append([[0,0,0,0],[0,0]])

    #--------------------------------------------------------------------
    #-- Thread run function called from takesnapshot
    #--------------------------------------------------------------------
    def run(self):
        while not self.terminated:
            try:
                framenb,motion = self.q.popleft()
                fdata = None
                with self.lock:
                    fdata,nbytes = self.decoder.decode_frame(self.frame2decode)

                if fdata:
                    (frame,w,h,ls) = fdata
                    #print("=== w:%d h:%d ls:%d" % (w,h,ls))
                    if frame:
                        image = np.fromstring(frame, dtype=np.uint8)
                    else:
                        image = self.image
                        w = self.resx
                        h = self.resy

                    image = image.reshape((h,w,3))
                    re = motion[0]
                    vv = motion[1]
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
                    if self.ycross:
                        cv2.line(image,(0,int(self.ycross)),(self.resx,int(self.ycross)),(0,0,0),1)
                    else:
                        cv2.line(image,(0,int(self.resy/2)),(self.resx,int(self.resy/2)),(0,0,0),1)
                    cv2.rectangle(image,(xmin,ymin),(xmax,ymax),(200,200,200),1)
                    cv2.rectangle(image,(x0,y0),(x1,y1),(20,220,20),1)
                    txt = "%d" % (framenb/2)
                    cv2.putText(image,txt,(int(x0), int(y0)),cv2.FONT_HERSHEY_SIMPLEX,0.5,(20,220,20),1)
                    xm = int((x1+x0) / 2)
                    ym = int((y1+y0) / 2)
                    xe = int(xm - 4*vv[0])
                    ye = int(ym - 4*vv[1])
                    cv2.arrowedLine(image,(xm,ym),(xe,ye),(20,220,20),1)
                    #image = cv2.resize(image,None,fx=0.5,fy=0.5,interpolation=cv2.INTER_LINEAR)
                    image = np.rot90(image,k=-1)
                    cv2.imwrite(self.imgpath, image, [cv2.IMWRITE_JPEG_QUALITY, 90])
                    #pg.image.save(surface, self.imgpath)

            except IndexError:
                self.event.clear()
                #- TODO: enable (do) garbage collection here!
                self.event.wait(1)

    #--------------------------------------------------------------------
    #-- lock stream and find frame to take as snapshot
    #--------------------------------------------------------------------
    def takeSnapshot(self, framenb, motion):
        n = 0
        sleep(0.05) # wait for keyframe is written in circular buffer
        record = False
        i_size = 0
        # lock stream by reading it
        for frame in reversed(self.stream.frames):
            index = frame.index
            ftype = frame.frame_type
            if not record:
                diff = 99999
                if ftype == 1:
                    i_size = frame.frame_size
                if ftype == 2:
                    diff = framenb - index

                if diff <= 0 and diff > -self.maxDiff:
                    #print("found key/sps frame @ %d (type:%d delta:%d)" % (index,ftype,diff))
                    record = True
            
            # decode the next sps + i-frame
            if record:
                save_pos = self.stream.tell()
                pos = frame.position
                szs = frame.frame_size + i_size + 32
                self.stream.seek(pos)
                with self.lock:
                    self.frame2decode = self.stream.read(szs)
                    self.update_hits(framenb, motion)
                self.stream.seek(save_pos)
                break

            n += 1

        print("%d frames searched (%d found @ %d)" % (n,record,framenb))

