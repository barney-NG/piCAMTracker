# vim: set et sw=4 sts=4 fileencoding=utf-8:
#
# Python Motion Analysis module of the pyCAMTracker package
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

import picamera
import picamera.array
import numpy as np
import threading
import cv2
from time import sleep,time
from math import degrees,atan2,pi
from struct import pack,calcsize
import logging

class MotionAnalyser(picamera.array.PiMotionAnalysis):
    """
    Real time analysis of the picamera *motion_output* parameter
    Reduce the motion_block array by couple of characteristics:

    """
    def __init__(self,camera, tracker, display, show=0, config=None, vwriter=None):
        super(MotionAnalyser, self).__init__(camera)
        self.camera = camera
        self.tracker = tracker
        self.display = display
        self.vwriter = vwriter
        self.t0  = time()
        self.config = config
        self.minArea = config.conf['minArea']
        self.maxArea = config.conf['maxArea']
        self.extension = config.conf['extension']
        # 32768 is absolute maximum ; 8192 is maximum
        self.sadThreshold = config.conf['sadThreshold']
        self.big = None
        self.show = show
        self.started = False
        self.xcross = config.conf['xCross']
        self.ycross = config.conf['yCross']
        self.vmin = config.conf['vMin']
        self.vmax = config.conf['vMax']
        self.frame = 0
        self.processed_frames = 0
        self.updated = False
        self.maxMovements = 100
        self.debug = False
        self.fobj = None
        self.emptyPoints = [[[],[]]]
        self.max_debugged_frames = 1200 # 30 secs at 40f/s
        self.max_debugged_files = 10
        self.debugged_frames = 0
        self.filenb = 0
        self.name_template = '/run/picamtracker/debug_motion_%03d.data'
        
        self.checkY = 0
        self.checkX = 0
        if self.ycross > 0 and config.conf["baseB"] == "left":
            self.checkY = -1
        if self.ycross > 0 and config.conf["baseB"] == "right":
            self.checkY = 1
        if self.xcross > 0 and config.conf["baseB"] == "left":
            self.checkX = -1
        if self.xcross > 0 and config.conf["baseB"] == "right":
            self.checkX = 1

    def debug_out(self, array):
        """
        write out the the macro blocks for later investigation
        """
        if self.fobj is None:
            if self.filenb > self.max_debugged_files:
                self.filenb = 0
            else:
                self.filenb += 1
            deb_filename = self.name_template % self.filenb
            self.debugged_frames = 0
            try:
                self.fobj = open(deb_filename, "wb")
                header = pack('Lll', 
                    0xa1a1a1a1, 
                    self.camera.resolution[0],
                    self.camera.resolution[1])
                self.fobj.write(header)
            except:
                raise

        self.fobj.write(array)

        self.debugged_frames += 1
        if self.debugged_frames > self.max_debugged_frames:
            logging.info("MotionAnalyser:debug off")
            self.debug = False
            self.fobj.close()
            self.fobj = None
            if self.vwriter:
                self.vwriter.write(self.filenb)

    def intersects(self,rects,xn,yn,wn,hn):
        """
        find rects which intersect a new one
        """
        i = 0
        extend = self.extension
        append = True
        #logging.debug("=====")
        #logging.debug("new: x1/y1: %2d/%2d, x2/y2: %2d/%2d  w/h %d/%d" % (xn,yn,xn+wn,yn+hn,wn,hn))
        #- Loop through all existing rects
        for xo,yo,wo,ho in rects:
            #logging.debug("  old: x1/y1: %2d/%2d, x2/y2: %2d/%2d w/h: %d/%d" % (xo,yo,xo+wo,yo+ho,wo,ho))
            # full intersection (new isin old)
            if xn >= xo and xn+wn <= xo+wo and yn >= yo and yn+hn <= yo+ho:
                #logging.debug("new in old")
                return rects
            # full intersection (old isin new)
            if xo > xn and xo+wo <= xn+wn and yo > yn and yo+ho <= yn+hn:
                #logging.debug("  old in new delete old")
                rects.pop(i)
                i += 1
                #append = False
                continue

            # partly intersection (always join new to old)
            # extend the new rect by two in each direction
            x1nn  = max(xn-extend,0)
            y1nn  = max(yn-extend,0)
            x2nn  = min(xn+wn+extend,self.cols)
            y2nn  = min(yn+hn+extend,self.rows)

            if xo > x1nn and xo+wo <= x2nn and yo > y1nn and yo+ho <= y2nn:
                #logging.debug("  old in extended new")
                xint = yint = True
            else:
                # find x range
                xmin = min(xo,x1nn)
                xmax = max(xo+wo,x2nn)
                # does x intersect?
                xint = (xmax - xmin) <= (wo + wn + extend)
                # find y range
                ymin = min(yo,y1nn)
                ymax = max(yo+ho,y2nn)
                # does y intersect?
                yint = (ymax - ymin) <= (ho + hn + extend)

            if (xint and yint):
                # intersection if x and y intersect
                # make union of the 'original' boxes
                xmin = min(xo,xn)
                xmax = max(xo+wo,xn+wn)
                ymin = min(yo,yn)
                ymax = max(yo+ho,yn+hn)
                rects[i] = [xmin,ymin,xmax-xmin,ymax-ymin]
                #logging.debug("  joi: x1/y1: %2d/%2d, x2/y2: %2d/%2d w/h: %d/%d" % (xmin,ymin,xmax,ymax,(xmax-xmin),(ymax-ymin)))
                append = False

            #- continue searching for intersections
            i += 1

        # no intersection found -> add
        if append:
            #logging.debug("app: x1/y1: %2d/%2d, x2/y2: %2d/%2d" % (xn,yn,xn+wn,yn+hn))
            rects.append([xn,yn,wn,hn])

        return rects

    def removeIntersections(self,contours):
        """
        collect nearby rectangles into bigger ones
        """
        def bySize(c):
            x,y,w,h = cv2.boundingRect(c)
            return w*h

        rects = []
        #for cnt in sorted(contours, key=bySize, reverse=True):
        for cnt in sorted(contours, key=bySize):
            x,y,w,h = cv2.boundingRect(cnt)

            if len(rects) > 0:
                rects = self.intersects(rects,x,y,w,h)
            else:
                rects.append([x,y,w,h])
        
        if len(rects) > 1:
            rects1 = []
            for r in rects:
                if len(rects1) > 0:
                    rects = self.intersects(rects1,r[0],r[1],r[2],r[3])
                else:
                    rects1.append(r)                
            return rects1
        else:
            return rects

    def debug_button(self, source):
        self.set_debug(15)

    def set_debug(self, value):
        """
        callback to start/stop debugging
        """
        if value > 0:
            # only start debugging when old session has stopped
            if self.fobj == None:
                logging.info("MotionAnalyser:debug on (%d)" % value)
                self.max_debugged_frames = 40 * value
                self.debug = True
        else:
            self.debug = False
            self.max_debugged_frames = 1200
            if self.fobj:
                self.fobj.close()
                self.fobj = None

    def set_extend(self, value):
        """
        callback setting collection extension
        """
        if value >= 1 and value <= 20:
            logging.info("MotionAnalyser::object extension: %d" % value)
            self.extension = int(value)
            self.config.conf['extension'] = int(value)

    def set_exposure(self, value):
        """
        callback setting camera exposure compensation
        """
        if value >= -20 and value <= 20:
            logging.info("MotionAnalyser::exposure_compensation: %d" % value)
            self.camera.exposure_compensation = int(value)
            self.config.conf['exposure'] = int(value)
            
    def set_baseB(self,mode):
        """
        callback setting baseB mode
        """
        logging.info("MotionAnalyser::baseB: %d" % mode)
            
        if mode == 0:
            self.config.conf['baseB'] = 'none'
            self.checkX = 0
            self.checkY = 0
        if mode == 1:
            self.config.conf['baseB'] = 'left'
            if self.ycross > 0:
                self.checkY = -1
            if self.xcross > 0:
                self.checkX = -1
        if mode == 2:
            self.config.conf['baseB'] = 'right'
            if self.ycross > 0:
                self.checkY = 1
            if self.xcross > 0:
                self.checkX = 1
                
    def set_vMax(self,value):
        """
        callback setting vMax
        """
        if value > self.vmin:
            logging.info("MotionAnalyser::vMax: %d" % value)
            self.vmax = value
            if self.config:
                self.config.conf['vMax'] = value

    def set_vMin(self,value):
        """
        callback setting vMin
        """
        if value < 1:
            value = 1
        logging.info("MotionAnalyser::vMin: %d" % value)
        self.vmin = value
        if self.config:
            self.config.conf['vMin'] = value

    def set_maxArea(self,value):
        """
        callback setting max area
        """
        if value > self.minArea:
            logging.info("MotionAnalyser::maxArea: %d" % value)
            self.maxArea = value
            if self.config:
                self.config.conf['maxArea'] = value

    def set_minArea(self,value):
        """
        callback setting min area
        """
        if value < 1:
            value = 1
        logging.info("MotionAnalyser::minArea: %d" % value)
        self.minArea = value
        if self.config:
            self.config.conf['minArea'] = value

    def set_sadThreshold(self,value):
        """
        callback setting SAD threshold
        """
        if value >=0 and value < 16384:
            logging.info("MotionAnalyser::sadThreshold: %d" % value)
            self.sadThreshold = value
            if self.config:
                self.config.conf['sadThreshold'] = value

    def analyse(self, a=None):
        """
        motion analyse method
        """
        t1 = time()
        dt = t1 - self.t0
        self.t0 = t1
        self.frame = self.camera.frame.index
        self.processed_frames += 1

        
        # initialize values not known at class initialization
        if not self.started:
            self.tracker.setup_sizes(self.rows, self.cols-1)
            self.maxMovements = int(self.rows * self.cols * 0.8)
            self.started = True
            return

        # save the motion blocks
        if self.debug:
            self.debug_out(a)
        
        #logging.debug("---%5.0fms ---" % (dt*1000.0))
        #return

        #---------------------------------------------------------------
        #-- IDENTIFY MOVEMENT
        #---------------------------------------------------------------
        mag = np.abs(a['x']) + np.abs(a['y'])
        #has_movement = np.logical_and(mag >= self.vmin, mag < self.vmax, a['sad'] > self.sadThreshold)
        has_movement = np.logical_and(mag >= self.vmin, mag < self.vmax)

        #- reject if more than 80% of the macro blocks are moving
        moving_elements = np.count_nonzero(has_movement)

        #- STOP HERE IF THERE IS NO MOVEMENT! (NEW)
        if moving_elements == 0:
            self.tracker.update_tracks(t1,self.frame,self.emptyPoints)
            return

        # >> debug    
        ##if moving_elements > 0: logging.debug("%6d %3d moving elements" % (self.processed_frames, moving_elements))
        
        if moving_elements > self.maxMovements:
            logging.warning("MAXMOVEMENT! (%d)" % moving_elements)
            return

        #- mask out movement
        mask = has_movement.astype(np.uint8) * 255

        # prepare background image 
        if self.show:
            if self.big is None:
                #self.big = np.ones((8*(self.cols-1),8*self.rows,3), np.uint8) * 220
                self.big = np.ones((8*self.rows,8*(self.cols-1),3), np.uint8) * 220
            else:
                self.big.fill(200)

        # show movement vectors and sad values
        if self.show & 0x0002:
            #- thats's slow!
            coords =  np.transpose(np.nonzero(mask))
            for y,x in coords:
                u  = a[y,x]['x']
                v  = a[y,x]['y']
                m =  min(512,a[y,x]['sad'])
                # old: sad high -> color dark
                #c =  255 - int(255.0/512.0 * m)
                # new: sad low -> color dark
                c =  int(255.0/512.0 * m)
                cv2.rectangle(self.big,(x*8,y*8),((x+1)*8,(y+1)*8),(0,c,c),-1)
                #-- nice arrows
                if self.show & 0x0008:
                    xm = x
                    ym = y
                    xm *= 8
                    ym *= 8
                    xe  = xm - 3 * u
                    ye  = ym - 3 * v
                    cv2.arrowedLine(self.big,(xm,ym),(xe,ye),(c,0,c),1)

        #---------------------------------------------------------------
        #-- MARK MOVING REGIONS
        #---------------------------------------------------------------
        #_, mask = cv2.threshold(mask, 1, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        contours,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) 
        #_,contours,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) # opencv-3.X
        
        rects = self.removeIntersections(contours)

        #---------------------------------------------------------------
        #-- START SCANNING
        #---------------------------------------------------------------
        new_points = []
        noise   = False
        rejects = 0
    
        num_rects = len(rects)
        #- walk through all rects
        for x0,y0,w,h in rects:
            #-- reject areas which are too big
            area = w*h
            if area > self.maxArea:
                logging.warning( "MAXAEREA! %d > %d (%d/%d)" % (area,self.maxArea,w,h))
                rejects += 1
                continue
            #-- reject areas which are too small
            if area < self.minArea:
                rejects += 1
                continue

            #-- translate rectangle to array coordinates
            x1  = x0 + w
            y1  = y0 + h

            #-- reduce vectors
            if w < 2 and h < 2:
                #-- examine single moving vectors
                vx = a[y0,x0]['x'].astype(np.float64)
                vy = a[y0,x0]['y'].astype(np.float64)
            else:
                vx = np.mean(a[y0:y1,x0:x1]['x'])
                vy = np.mean(a[y0:y1,x0:x1]['y'])

                # SLOW! ( ~+2ms on RPi4)
                #-- we are searching for regions which don't differ much (sad is small)
                #sad_var = a[y0:y1,x0:x1]['sad'].var()
                #sad_weights = a[y0:y1,x0:x1]['sad'].flatten()
                #sad_weights = 512000.0 - 100.0 * sad_weights
                
                #-- develope composite vector from weightened vectors in region
                #try:
                #    vx = np.average(a[y0:y1,x0:x1]['x'].flatten(),weights=sad_weights)
                #    vy = np.average(a[y0:y1,x0:x1]['y'].flatten(),weights=sad_weights)
                #except ZeroDivisionError:
                #    vx = np.mean(a[y0:y1,x0:x1]['x'])
                #    vy = np.mean(a[y0:y1,x0:x1]['y'])
                

	        #-- check baseB option (allow movement from one direction only)
            append = True
            if self.checkX:
                append = False
                if self.checkX > 0 and vx >= 0: append = True
                if self.checkX < 0 and vx <= 0: append = True
            if self.checkY:
                append = False
                if self.checkY > 0 and vy >= 0: append = True
                if self.checkY < 0 and vy <= 0: append = True                
            #-- add points to list
            if append:
                new_points.append([[x0,y0,w,h],[vx,vy]])

        # insert/update new movements
        self.tracker.update_tracks(t1,self.frame,new_points)

        # crate image with objects
        if self.show:
            self.tracker.showTracks(self.frame, self.big)
            # create header
            #xm = 8*self.xcross
            ym = 8*self.ycross
            xe = int(8*(self.cols))
            #ye = 8*(self.rows)
            cv2.line(self.big,(0,ym),(xe,ym),(0,0,0),1)
            str_frate = "%4.0fms (%d) (%4.2f) (%0d)" % (dt*1000.0, self.camera.analog_gain, self.tracker.noise, self.tracker.active_tracks)
            cv2.putText(self.big, str_frate, (3, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (20,150,20), 1)
            if self.show & 0x0004:
                for cnt in contours:
                    x,y,w,h = cv2.boundingRect(cnt)
                    x0 = 8*x; y0 = 8*y; x1 = 8*(x+w); y1 = 8*(y+h)
                    cv2.rectangle(self.big,(x0,y0),(x1,y1),(255,255,255),2)
                    rect_txt = "%d,%d,%d,%d (%d)" % (x,y,x+w,y+h,w*h)
                    cv2.putText(self.big, rect_txt, (x0,y0), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 1)
                for x,y,w,h in rects:
                    x0 = 8*x; y0 = 8*y; x1 = 8*(x+w); y1 = 8*(y+h)
                    cv2.rectangle(self.big,(x0,y0),(x1,y1),(0,0,0),1)
                    cv2.line(self.big,(x0,y0),(x1,y1),(0,0,0),1)
                    cv2.line(self.big,(x0,y1),(x1,y0),(0,0,0),1)

            # Show the image in the window
            # without imshow we are at 5ms (sometimes 12ms)
            if self.display:
              self.display.imshow( self.big )

        #if self.processed_frames % 10 == 0:
        #    print("--- %4.2fms (%d)" % (1000.0 * (time() - self.t0), num_rects))
