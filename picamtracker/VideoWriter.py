# vim: set et sw=4 sts=4 fileencoding=utf-8:
#
# Python Video Writer module of the piCAMTracker package
# Copyright (c) 2019-2020 Axel Barnitzke <barney@xkontor.org>
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
import cv2
import gc
import prctl

class vWriter(threading.Thread):
    """
    write a short sequence from the circular buffer stream (h264) to file
    """
    #--------------------------------------------------------------------
    #-- constructor
    #--------------------------------------------------------------------
    def __init__(self, stream=None, config=None):
        super(vWriter,self).__init__()
        self.name = 'vWriter'
        self.config = config
        self.stream = stream
        self.seconds = config.conf["videoLength"]
        self.filename = None
        self.video_template = '/home/pi/piCAMTracker/media/videos/deb_video%03d.h264'
        #self.video_template = '/run/picamtracker/debug_motion_%03d.h264'

        self.nvideos = 10
        self.nbvideo = 0
        prctl.set_name('ptrk.vWriter')

        #- thread initialisation stuff
        #self.lock = threading.Lock()
        self.event = threading.Event()
        self.event.clear()
        self.terminated = False
        self.daemon = True
        self.start()

    #--------------------------------------------------------------------
    #-- stop all threading stuff
    #--------------------------------------------------------------------
    def stop(self):
        self.terminated = True

    #--------------------------------------------------------------------
    #-- Thread run function called from takesnapshot
    #--------------------------------------------------------------------
    def run(self):
        while not self.terminated:
            if self.event.wait(1):
                if self.filename:
                    self.stream.copy_to(self.filename, seconds=self.seconds)
                    self.filename = None
                self.event.clear()
            
    #--------------------------------------------------------------------
    #-- run
    #--------------------------------------------------------------------
    def write(self, filenb=0):
        self.filename = self.video_template % filenb
        self.event.set()
        
        
