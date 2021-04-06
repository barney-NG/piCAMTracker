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



    def __init__(self,camera, tracker, display=None, show=0, config=None, vwriter=None):
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
        if self.sadThreshold < 150:
            self.sadThreshold = 150
        self.rects = None
        self.num_rects = 20
        self.rect_index = 0
        self.rect_average = 0.0
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
        self.emptyPoints = [[[],[],[]]]
        self.max_debugged_frames = 1200 # 30 secs at 40f/s
        self.max_debugged_files = config.conf['debugFiles']
        self.debugged_frames = 0
        self.first_debugged_frame = 0
        self.filenb = 0
        self.name_template = '/run/picamtracker/debug_motion_%03d.data'
        self.number_safe = '/home/pi/piCAMTracker/media/stills/debug_last.txt'
        self.kernel = np.ones((3,3),np.uint8)

        try:
            with open(self.number_safe, "r") as fd:
                last_str = fd.readline()
                last = int(last_str)
                logging.info("last debug file number: %d", last)
                self.filenb = last
        except:
            pass
        
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

    def rect_avg(self, value):
        if self.rects is None:
            self.rects = np.zeros(self.num_rects)
        self.rects[self.rect_index] = value
        self.rect_index += 1
        if self.rect_index >= self.num_rects:
            self.rect_index = 0
        return np.mean(self.rects)
        
    def debug_out(self, array, framenb):
        """
        write out the the macro blocks for later investigation
        """
        if self.fobj is None:
            self.first_debugged_frame = framenb
            self.camera.request_key_frame()
            if self.filenb >= self.max_debugged_files:
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
        if self.debugged_frames >= self.max_debugged_frames:
            logging.info("MotionAnalyser:debug off")
            self.debug = False
            self.fobj.close()
            self.fobj = None
            if self.vwriter:
                self.vwriter.write(self.filenb, self.first_debugged_frame, self.debugged_frames )
            with open(self.number_safe, "w") as fd:
                fd.write("%03d\n" % self.filenb)

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
        self.set_debug(30)

    def set_debug(self, value):
        """
        callback to start/stop debugging
        """
        if value > 0:
            # only start debugging when old session has stopped
            if self.fobj == None:
                self.max_debugged_frames = self.camera.framerate_range[1] * value
                logging.info("MotionAnalyser:debug on (%ds / %d frames)" % (value,self.max_debugged_frames))
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
            self.maxMovements = int(self.rows * self.cols * 0.6)
            # prepare background image 
            if self.show:
                if self.big is None:
                    self.big = np.ones((8*self.rows,8*(self.cols-1),3), np.uint8) * 220
                else:
                    self.big.fill(200)
            self.started = True
            return

        # save the motion blocks
        if self.debug:
            self.debug_out(a, self.frame)
        
        #logging.debug("---%5.0fms ---" % (dt*1000.0))
        #return

        #---------------------------------------------------------------
        #-- IDENTIFY MOVEMENT
        #---------------------------------------------------------------
        threshold = self.sadThreshold + 4 * int(self.rect_average)
        mag = np.abs(a['x']) + np.abs(a['y'])
        has_movement = np.logical_and(mag >= self.vmin, mag < self.vmax)
        has_movement = np.logical_or(has_movement, a['sad'] >= threshold)

        #- reject if more than 60% of the macro blocks are moving
        moving_elements = np.count_nonzero(has_movement)

        #- STOP HERE IF THERE IS NO MOVEMENT! (NEW)
        if moving_elements == 0:
            #logging.debug("NO MOVEMENT!")
            self.tracker.update_tracks(t1,self.frame,self.emptyPoints)
            return

        # >> debug    
        ##if moving_elements > 0: logging.debug("%6d %3d moving elements" % (self.processed_frames, moving_elements))
        
        if moving_elements > self.maxMovements:
            logging.warning("MAXMOVEMENT! (%d)" % moving_elements)
            return

        #- mask out movement
        mask = has_movement.astype(np.uint8) * 255
        sad = np.array(a['sad'], np.uint16)

        # show movement vectors and sad values
        if self.show & 0x0002:
            #- thats's slow!
            coords =  np.transpose(np.nonzero(mask))
            for y,x in coords:
                u  = a[y,x]['x']
                v  = a[y,x]['y']
                c = int((0x0fff - sad[y,x]) & 0x00ff)
                cv2.rectangle(self.big,(x*8,y*8),((x+1)*8,(y+1)*8),(0,c,c),-1)
                #-- nice arrows
                if self.show & 0x0004:
                    xm = x
                    ym = y
                    xm *= 8
                    ym *= 8
                    xe  = xm - 4 * u
                    ye  = ym - 4 * v
                    cv2.arrowedLine(self.big,(xm,ym),(xe,ye),(c,0,c),1)

        #---------------------------------------------------------------
        #-- MARK MOVING REGIONS
        #---------------------------------------------------------------
        mask = cv2.morphologyEx(mask,cv2.MORPH_CLOSE,self.kernel,iterations=2)

        """ >>> debug
        mask1 = np.rot90(mask,k=-1)
        mask1 = cv2.resize(mask1,None,fx=8,fy=8,interpolation=cv2.INTER_AREA)
        cv2.imshow("v-vector",mask1)
        _,sadm = cv2.threshold(sad, threshold,255,cv2.THRESH_BINARY)
        sadm = sadm.astype(np.uint8)
        sadm1 = np.rot90(sadm,k=-1)
        sadm1 = cv2.resize(sadm1,None,fx=8,fy=8,interpolation=cv2.INTER_AREA)
        cv2.imshow("sad",sadm1)
        #mask2 = np.bitwise_or(mask,sadm)
        <<< debug """

        contours,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        self.rect_average = self.rect_avg(len(contours))
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
                #logging.debug( "MAXAEREA! %d > %d (%d/%d)" % (area,self.maxArea,w,h))
                rejects += 1
                continue

            #-- reject areas which are too small
            if area < self.minArea:
                rejects += 1
                continue

            #-- translate rectangle to array coordinates
            x1  = x0 + w
            y1  = y0 + h

            #-- evaluate average moving vector
            vstats = [0,0,0,0]
            if w < 2 and h < 2:
                #-- simply take the moving vector
                vx = a[y0,x0]['x'].astype(np.float64)
                vy = a[y0,x0]['y'].astype(np.float64)
                if vx >= 0.:
                    vstats[0] = 1
                else:
                    vstats[2] = 1
                if vy >= 0.:
                    vstats[1] = 1
                else:
                    vstats[3] = 1
            else:
                #-- give blocks which don't differ much (sad is small) higher priority
                arx = np.array(a[y0:y1,x0:x1]['x'])
                ary = np.array(a[y0:y1,x0:x1]['y'])
                similarity = 0x1000 - sad[y0:y1,x0:x1].flatten()
                vx = np.average(arx.flatten(),weights=similarity)
                vy = np.average(ary.flatten(),weights=similarity)
                #-- count element movings
                vstats[0] = np.count_nonzero(arx > 0)
                vstats[1] = np.count_nonzero(ary > 0)
                vstats[2] = np.count_nonzero(arx < 0)
                vstats[3] = np.count_nonzero(ary < 0)
                
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
                new_points.append([[x0,y0,w,h],[vx,vy],vstats])

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
            if self.show & 0x0008:
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

            if self.show & 0x0004:
                for rn,vn,vs in new_points:
                    x0=8*rn[0];y0=8*rn[1];x1=8*(rn[0]+rn[2]);y1=8*(rn[1]+rn[3])
                    cv2.rectangle(self.big,(x0-1,y0-1),(x1+1,y1+1),(0,0,220),1)
                    rect_txt = "[%03d %03d %03d %03d]" % (vs[0],vs[1],vs[2],vs[3])
                    cv2.putText(self.big, rect_txt, (x1,y1), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,20), 1)
                    
            # Show the image in the window
            # without imshow we are at 5ms (sometimes 12ms)
            if self.display:
              self.display.imshow( self.big )

        #if self.processed_frames % 50 == 0:
        #    logging.debug("--- %4.2fms (%d)" % (1000.0 * (time() - self.t0), num_rects))
