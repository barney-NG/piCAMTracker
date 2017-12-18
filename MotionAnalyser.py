# vim: set et sw=4 sts=4 fileencoding=utf-8:
import picamera
import picamera.array
import numpy as np
import threading
import cv2
from time import sleep,clock
from math import degrees,atan2,pi
import MotionTracker
import GPIOPort


class MotionAnalyser(picamera.array.PiMotionAnalysis):
    def __init__(self,camera, tracker, display, show=False, direction=(-1,0)):
        super(MotionAnalyser, self).__init__(camera)
        self.camera  = camera
        self.tracker  = tracker
        self.display  = display
        self.t0      = clock()
        self.dir = direction
        self.maxArea = 512
        self.sadThreshold = 192
        self.big     = None
        self.show    = show
        self.started = False
        self.xcross  = None
        self.ycross  = None
        self.frame   = 0
        self.updated = False
        self.maxMovements = 100

        self.greenLEDThread = GPIOPort.gpioPort(19)
        #self.redLEDThread   = GPIOPort.gpioPort(13)
        if self.greenLEDThread:
            MotionTracker.track.crossed_handler = self.greenLEDThread

    def __del__(self):
        if self.greenLEDThread:
            self.greenLEDThread.terminated = True
            self.greenLEDThread.join()

    def rot90(self, x, y):
        return ((self.cols-1-y,x))

    def analyse(self, a=None):
        t1 = clock()
        dt = t1 - self.t0
        self.t0 = t1
        self.frame = self.camera.frame.index

        # initialize values not known at class initialization
        if not self.started:
            self.tracker.setup_sizes(self.rows, self.cols-1)
            MotionTracker.track.yCross = (self.rows) / 2
            self.xcross   = self.cols / 2
            self.ycross   = self.rows / 2
            self.maxMovements = self.rows * self.cols / 8
            self.started = True
            return

        #---------------------------------------------------------------
        #-- IDENTIFY MOVEMENT
        #---------------------------------------------------------------
        #- identify movement in actual frame
        #if self.dir[0] < 0:
        mag = np.abs(a['x']) + np.abs(a['y'])
        has_movement = np.logical_and(mag > 1, mag < 100, a['sad'] > self.sadThreshold)
        #has_movement = np.logical_and(has_movement, a['y'] > 0 )

        #- reject if more than 25% of the macro blocks are moving
        if np.count_nonzero(has_movement) > self.maxMovements:
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
                #u  = a[y,x]['x']
                #v  = a[y,x]['y']
                c =  max(0, 255 - int(a[y,x]['sad']))
                #c =  255-int(mask[y,x])
                #c = 220
                x *= 8
                y *= 8
                xm *= 8
                ym *= 8
                #xe  = xm - u
                #ye  = ym - v
                cv2.rectangle(self.big,(x,y),(x+8,y+8),(0,c,c),-1)
                #cv2.arrowedLine(self.big,(xm,ym),(xe,ye),(c,0,c),1)




        #---------------------------------------------------------------
        #-- MARK MOVING REGIONS
        #---------------------------------------------------------------
        #_, mask = cv2.threshold(mask, 1, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        _,contours,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        #---------------------------------------------------------------
        #-- START SCANNING
        #---------------------------------------------------------------
        new_points = []
        noise   = False
        rejects = 0
        #for x0,y0,w,h in rects:
        for cnt in contours:
            x0,y0,w,h = cv2.boundingRect(cnt)

            #-- reject areas which are too big
            if w*h > self.maxArea:
                print "MAXAEREA!"
                continue

            #-- perimeter blocks have limited vector direction (Bill Wilson)
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

            #-- translate rectangle to array coordinates
            x1  = x0 + w
            y1  = y0 + h

            #-- reduce vectors
            sad_var = 0.0
            if w < 2 and h < 2:
                #-- examine single moving vectors
                vx   = a[y0,x0]['x'].astype(np.float64)
                vy   = a[y0,x0]['y'].astype(np.float64)
                sad_var = a[y0,x0]['sad']
                #-- is this block a good foreground block?
                if sad_var < self.sadThreshold:
                    #print "sparkel: sad: %3d" % ( a[y0,x0]['sad'])
                    rejects += 1
                    continue
                #print "vx/vy %2d,%2d (%d)" % (vx,vy,sad_var)
            else:
                #-- we are searching for regions which differ a lot from the previous frame
                #-- ignore small changes in foregroung (weaving fields)
                #sad    = a[y0:y1,x0:x1]['sad'].mean()
                sad_var = a[y0:y1,x0:x1]['sad'].var()
                if sad_var < 2*self.sadThreshold:
                    rejects += 1
                    continue

                #sad_weights = a[y0:y1,x0:x1]['sad'].flatten()
                #sad_weights *= sad_weights

                #-- develope composite vector from weightened vectors in region
                #try:
                #    vx   = np.average(a[y0:y1,x0:x1]['x'].flatten(),weights=sad_weights)
                #    vy   = np.average(a[y0:y1,x0:x1]['y'].flatten(),weights=sad_weights)
                #except ZeroDivisionError:
                #    vx   = np.mean(a[y0:y1,x0:x1]['x'])
                #    vy   = np.mean(a[y0:y1,x0:x1]['y'])
                vx   = np.mean(a[y0:y1,x0:x1]['x'])
                vy   = np.mean(a[y0:y1,x0:x1]['y'])

            #-- add points to list
            new_points.append([[x0,y0,w,h],[vx,vy]])


            if self.show:# and self.frame % 5:
                x0 *= 8
                y0 *= 8
                w *= 8
                h *= 8
                #cv2.rectangle(self.big,(x0,y0),(x0+w,y0+h),(0,0,0),1)
                xm = x0+w/2
                ym = y0+h/2
                if rejects > 0:
                    c = (240,240,240)
                else:
                    c = (150,150,150)
                #cv2.putText(self.big,txt,(xm, ym),cv2.FONT_HERSHEY_SIMPLEX,0.5,c,2)
                xe = int(xm-4*vx)
                ye = int(ym-4*vy)
                #cv2.arrowedLine(self.big,(xm,ym),(xe,ye),c,2)
                cv2.rectangle(self.big,(x0,y0),(x0+w,y0+h),(20,20,20),1)

        # insert/update new movements
        print "---%5.0fms --- (%d)" % (dt*1000.0,rejects)
        self.tracker.update_tracks(self.frame,new_points)

        #if not self.show:
        #if self.updated:
        #    self.tracker.printTracks()

        #self.tracker.printAll()
        if self.show:# and self.frame % 5:
            self.tracker.showTracks(self.frame, self.big)
            # create header
            #xm = 8*self.xcross
            ym = 8*(self.ycross+1)
            xe = 8*(self.cols)
            #ye = 8*(self.rows)
            cv2.line(self.big,(0,ym),(xe,ym),(0,0,0),1)
            str_frate = "%4.0fms (%d)" % (dt*1000.0, rejects)
            cv2.putText(self.big, str_frate, (3, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (20,150,20), 1)

            # Show the image in the window
            # without imshow we are at 5ms (sometimes 12ms)
            key = self.display.imshow( self.big )
            if key == 27:
                if self.greenLEDThread:
                    self.greenLEDThread.terminated = True
                    self.greenLEDThread.join()
                raise NotImplementedError
