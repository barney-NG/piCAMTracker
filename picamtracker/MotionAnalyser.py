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

class MotionAnalyser(picamera.array.PiMotionAnalysis):
    """
    Real time analysis of the picamera *motion_output* parameter

    Reduce the motion_block array by couple of characteristics:

    """
    def __init__(self,camera, tracker, display, show=False, config=None):
        super(MotionAnalyser, self).__init__(camera)
        self.camera  = camera
        self.tracker  = tracker
        self.display  = display
        self.t0      = time()
        self.config  = config
        self.minArea = 1
        self.maxArea = config.conf['maxArea']
        # 32768 is absolute maximum ; 8192 is maximum
        self.sadThreshold = config.conf['sadThreshold']
        self.big     = None
        self.show    = show
        self.started = False
        self.xcross  = config.conf['xCross']
        self.ycross  = config.conf['yCross']
        self.vmin    = config.conf['vMin']
        self.vmax    = config.conf['vMax']
        self.frame   = 0
        self.processed_frames = 0
        self.updated = False
        self.maxMovements = 100


    def rot90(self, x, y):
        return ((self.cols-1-y,x))

    def intersects(self,rects,xn,yn,wn,hn):
        i = 0
        for xo,yo,wo,ho in rects:
            # full intersection (new isin old)
            if xn >= xo and wn <= wo and yn >= yo and hn <= ho:
                return rects
            # full intersection (old isin new)
            if xo > xn and wo < wn and yo > yn and ho < hn:
                rects[i] = [xn,yn,wn,hn]
                return rects
            # partly intersection (always join new to old)
            # extend the new rect by one in each direction
            xnn  = max(xn-1,0)
            ynn  = max(yn-1,0)
            wnn  = min(wn+1,self.cols)
            hnn  = min(hn+1,self.rows)

            xmin = min(xo,xnn)
            xmax = max(xo+wo,xnn+wnn)
            xint = (xmax - xmin) <= (wo + wnn)
            ymin = min(yo,ynn)
            ymax = max(yo+ho,ynn+hnn)
            yint = (ymax - ymin) <= (ho + hnn)
            if (xint and yint):
                xmin = min(xo,xn)
                xmax = max(xo+wo,xn+wn)
                ymin = min(yo,yn)
                ymax = max(yo+ho,yn+hn)
                rects[i] = [xmin,ymin,xmax-xmin,ymax-ymin]
                return rects
            i += 1

        # no intersection -> add
        rects.append([xn,yn,wn,hn])
        return rects

    def set_maxArea(self,value):
        if value > self.minArea:
            self.maxArea = value

    def set_minArea(self,value):
        if value < 1: 
            value = 1
        self.minArea = value

    def set_sadThreshold(self,value):
        if value >=0 and value < 16384:
            self.sadThreshold = value

    def removeIntersections(self,contours):
        rects = []
        for cnt in contours:
            x,y,w,h = cv2.boundingRect(cnt)
 
            if w*h > self.maxArea:
                continue

            if len(rects) > 0:
                rects = self.intersects(rects,x,y,w,h)
            else:
                rects.append([x,y,w,h])

        return rects

    def analyse(self, a=None):
        t1 = time()
        dt = t1 - self.t0
        self.t0 = t1
        self.frame = self.camera.frame.index
        self.processed_frames += 1
        #print("---%5.0fms ---" % (dt*1000.0))
        #return

        # initialize values not known at class initialization
        if not self.started:
            self.tracker.setup_sizes(self.rows, self.cols-1)
            self.maxMovements = self.rows * self.cols / 8
            self.started = True
            return

        #---------------------------------------------------------------
        #-- IDENTIFY MOVEMENT
        #---------------------------------------------------------------
        #- identify movement in actual frame
        #if self.dir[0] < 0:
        mag = np.abs(a['x']) + np.abs(a['y'])
        has_movement = np.logical_and(mag > self.vmin, mag < self.vmax, a['sad'] > self.sadThreshold)
        #rejects = np.count_nonzero(mag >= self.vmax)
        
        #- we can reduce half of the area and movement in just one direction
        #has_movement = np.logical_and(has_movement, a['y'] < 0 )

        #- reject if more than 25% of the macro blocks are moving
        moving_elements =  np.count_nonzero(has_movement)
        if moving_elements > self.maxMovements:
            return

        #- mask out movement
        mask = has_movement.astype(np.uint8) * 255

        if self.show:# and self.frame % 5:
            if self.big is None:
                #self.big = np.ones((8*(self.cols-1),8*self.rows,3), np.uint8) * 220
                self.big = np.ones((8*self.rows,8*(self.cols-1),3), np.uint8) * 220
            else:
                self.big.fill(200)
        
        #if False:
        if self.show:# and self.frame % 5:
            #- thats's slow!
            coords =  np.transpose(np.nonzero(mask))
            for y,x in coords:
                xm = x
                ym = y
                u  = a[y,x]['x']
                v  = a[y,x]['y']
                m =  min(512,a[y,x]['sad'])
                c =  255 - int(255.0/512.0 * m)
                #c =  255-int(mask[y,x])
                #c = 220
                x *= 8
                y *= 8
                xm *= 8
                ym *= 8
                xe  = xm - 3 * u
                ye  = ym - 3 * v
                cv2.rectangle(self.big,(x,y),(x+8,y+8),(0,c,c),-1)
                #cv2.arrowedLine(self.big,(xm,ym),(xe,ye),(c,0,c),1)




        #---------------------------------------------------------------
        #-- MARK MOVING REGIONS
        #---------------------------------------------------------------
        #_, mask = cv2.threshold(mask, 1, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        _,contours,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        rects = self.removeIntersections(contours)

        #---------------------------------------------------------------
        #-- START SCANNING
        #---------------------------------------------------------------
        new_points = []
        noise   = False
        rejects = 0
        num_small = num_large = 0
        #- walk through all contours
        #for cnt in contours:
        for x0,y0,w,h in rects:
            #x0,y0,w,h = cv2.boundingRect(cnt)

            #-- reject areas which are too big
            if w*h > self.maxArea:
                print( "MAXAEREA!")
                rejects += 1
                continue

            #-- perimeter blocks have limited vector direction (Bill Wilson)
            """
            if x0 == 0:
                x0 = 1; w -=1
            if x0 + w == self.cols -1:
                 w -= 1
            if w < 1:
                continue

            if y0 == 0:
                y0 = 1; h -=1
            if y0 + h == self.rows:
                 h -= 1
            if h < 1:
                continue
            """

            #-- translate rectangle to array coordinates
            x1  = x0 + w
            y1  = y0 + h

            #-- reduce vectors
            sad_var = 0.0
            if w < 2 and h < 2:
                #-- examine single moving vectors
                vx = a[y0,x0]['x'].astype(np.float64)
                vy = a[y0,x0]['y'].astype(np.float64)
                sad_var = a[y0,x0]['sad']
                #-- is this block a good foreground block?
                if sad_var < self.sadThreshold:
                    #print "sparkel: sad: %3d" % ( a[y0,x0]['sad'])
                    rejects += 1
                    #continue
                num_small += 1
                #print "vx/vy %2d,%2d (%d)" % (vx,vy,sad_var)
                #-- try to close gaps TODO: check borders
                #if vy < 0.0 and a[y0+1,x0]['sad'] > self.sadThreshold:
                #    print("y+")
                #    h += 1
                #if vy > 0.0 and a[y0-1,x0]['sad']  > self.sadThreshold:
                #    print("y-")
                #    y0 -= 1
                #    h += 1
                #if vx < 0.0 and a[y0,x0+1]['sad'] > self.sadThreshold:
                #    print("x+")
                #    w += 1
                #if vx > 0.0 and a[y0,x0-1]['sad']  > self.sadThreshold:
                #    print("y-")
                #    x0 -= 1
                #    w += 1
            else:
                #-- we are searching for regions which differ a lot from the previous frame
                #-- ignore small changes in foregroung (weaving fields)
                sad_var = a[y0:y1,x0:x1]['sad'].var()
                if sad_var < 2*self.sadThreshold:
                    rejects += 1
                    continue

                sad_weights = a[y0:y1,x0:x1]['sad'].flatten()
                sad_weights *= sad_weights

                #-- develope composite vector from weightened vectors in region
                try:
                    vx = np.average(a[y0:y1,x0:x1]['x'].flatten(),weights=sad_weights)
                    vy = np.average(a[y0:y1,x0:x1]['y'].flatten(),weights=sad_weights)
                except ZeroDivisionError:
                    vx = np.mean(a[y0:y1,x0:x1]['x'])
                    vy = np.mean(a[y0:y1,x0:x1]['y'])
                #vx = np.mean(a[y0:y1,x0:x1]['x'])
                #vy = np.mean(a[y0:y1,x0:x1]['y'])
                num_large += 1

            #-- add points to list
            new_points.append([[x0,y0,w,h],[vx,vy]])


            #if False:
            if self.show:# and self.frame % 5:
                x0 *= 8
                y0 *= 8
                w *= 8
                h *= 8
                #cv2.rectangle(self.big,(x0,y0),(x0+w,y0+h),(0,0,0),1)
                xm = int(x0+w/2)
                ym = int(y0+h/2)
                if rejects > 0:
                    c = (240,240,240)
                else:
                    c = (50,50,50)
                #cv2.putText(self.big,txt,(xm, ym),cv2.FONT_HERSHEY_SIMPLEX,0.5,c,2)
                #xe = int(xm-4*vx)
                #ye = int(ym-4*vy)
                #cv2.arrowedLine(self.big,(xm,ym),(xe,ye),c,2)
                cv2.rectangle(self.big,(x0,y0),(x0+w,y0+h),(20,20,20),1)

        # insert/update new movements
        #print("---%5.0fms --- (%d) (%d)" % (dt*1000.0,rejects,moving_elements))
        self.tracker.update_tracks(self.frame,new_points)

        #if not self.show:
        #if self.updated:
        #    self.tracker.printTracks()

        #self.tracker.printAll()
        if self.show:# and self.frame % 5:
            self.tracker.showTracks(self.frame, self.big)
            # create header
            #xm = 8*self.xcross
            ym = 8*self.ycross
            xe = int(8*(self.cols))
            #ye = 8*(self.rows)
            cv2.line(self.big,(0,ym),(xe,ym),(0,0,0),1)
            str_frate = "%4.0fms (%d) (%d) (%4.2f)" % (dt*1000.0, self.camera.analog_gain, self.sadThreshold, self.tracker.noise)
            cv2.putText(self.big, str_frate, (3, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (20,150,20), 1)

            # Show the image in the window
            # without imshow we are at 5ms (sometimes 12ms)
            key = self.display.imshow( self.big )
            if key == 27:
                raise NotImplementedError

        #print("proc_time: %4.2f" % (1000.0 * (time() - self.t0)))
