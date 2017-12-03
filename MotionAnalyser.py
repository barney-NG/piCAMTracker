# vim: set et sw=4 sts=4 fileencoding=utf-8:
import picamera
import picamera.array
import numpy as np
import cv2
from time import sleep,clock
from math import degrees,atan2,pi
import MotionTracker
import RPi.GPIO as GPIO
p1=13
p2=19

# sort tracks by number of updates
def by_updates(t):
    return t.updates

class MotionAnalyser(picamera.array.PiMotionAnalysis):
    def __init__(self,camera, show=False, direction=(-1,0)):
        super(MotionAnalyser, self).__init__(camera)
        self.camera  = camera
        self.t0      = clock()
        self.dir = direction
        self.maxArea = 256
        self.sadThreshold = 128
        self.big     = None
        self.show    = show
        self.mmx     = None
        self.mmy     = None
        self.xcross  = None
        self.loop = 0
        self.maxDelta = 90.0
        self.notPlaced = True
        self.port_green = 19
        self.port_red   = 13
        self.maxMovements = 100
        self.all_tracks = []
        self.setupGPIO()
        self.greenLightsOn = 0
        self.redLightsOn = 0
        self.lightsOnTime = 100 # ms
        for i in range(0,32):
            self.all_tracks.append(MotionTracker.track())

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

    def setupGPIO(self):
      GPIO.setmode(GPIO.BCM)
      GPIO.setup(self.port_green,GPIO.OUT)
      GPIO.setup(self.port_red,GPIO.OUT)
      GPIO.output(self.port_green,GPIO.LOW)
      GPIO.output(self.port_red,GPIO.LOW)

    def setGreen(self, set=True):
        if set:
            GPIO.output(self.port_green,GPIO.HIGH)
            self.greenLightsOn = clock()
        else:
            GPIO.output(self.port_green,GPIO.LOW)
            self.greenLightsOn = 0

    def setRed(self, set=True):
        if set:
            GPIO.output(self.port_red,GPIO.LOW)
            self.redLightsOn = clock()
        else:
            GPIO.output(self.port_red,GPIO.HIGH)
            self.redLightsOn = 0

    def printTracks(self):
        for track in sorted(self.all_tracks, key=by_updates, reverse=True):
            track.printTrack()

    def showTracks(self, vis):
        for track in self.all_tracks:
            track.showTrack(vis)

    def update_tracks(self, motion):
        # walk through all changes
        for rn,vn in motion:
            # search a track for this coordinate
            tracked = 0x00000000
            #for track in self.all_tracks:
            for track in sorted(self.all_tracks, key=by_updates, reverse=True):
                tracked |= track.update_track(self.loop,rn,vn)
                if tracked <> 0:
                    if track.crossedX:
                        self.setGreen(True)
                    break
            # not yet tracked -> find a free slot
            if not tracked:
                for track in self.all_tracks:
                    if track.updates == 0:
                        track.new_track(self.loop,rn,vn)
                        break
        # remove aged tracks
        if self.loop % 30:
            for track in self.all_tracks:
                track.clean(self.loop)

            if self.greenLightsOn > 0:
                difftime = (clock() - self.greenLightsOn) * 1000
                if difftime > self.lightsOnTime:
                    self.setGreen(False)


        #if self.show:
        #    esti = (MotionTracker.track.estimates > 0).astype(np.uint8) * 255
        #    esti_big = cv2.resize(esti,(self.big.shape[1],self.big.shape[0]))
        #    self.big[:,:,0] = 255 - esti_big

        #MotionTracker.track.estimates.fill(0)


    def analyse(self, a=None):
        t1 = clock()
        dt = t1 - self.t0
        self.t0 = t1

        # initialize values not known at class initialization
        if self.mmx is None:
            self.mmx      = np.zeros(a.shape, np.float32)
            self.mmy      = np.zeros(a.shape, np.float32)
            MotionTracker.track.maxX = self.cols - 1
            MotionTracker.track.maxY = self.rows 
            MotionTracker.track.xCross = (self.cols-1) / 2
            self.xcross   = self.cols / 2
            self.maxMovements = self.rows * self.cols / 8
            return

        #---------------------------------------------------------------
        #-- IDENTIFY MOVEMENT
        #---------------------------------------------------------------
        #- identify movement in actual frame
        #if self.dir[0] < 0:
        #has_movement = np.abs(a['x']) + np.abs(a['y']) > 0
        mag = np.abs(a['x']) + np.abs(a['y'])
        has_movement = np.logical_and(mag > 2, mag < 200)

        #- reject if more than 25% of the macro blocks are moving
        if np.count_nonzero(has_movement) > self.maxMovements:
            return

        # loop increment
        self.loop += 1

        #- mask out movement
        mask = has_movement.astype(np.uint8) * 255

        if self.show and self.loop % 5:
            if self.big is None:
                self.big = np.ones((16*(mask.shape[0]-1),16*mask.shape[1],3), np.uint8) * 220
            else:
                self.big.fill(200)
                #sad_big = cv2.resize(a['sad'],(self.big.shape[1],self.big.shape[0]))
                #self.big[]
        
        #if self.show and self.loop % 5:
        if False:
            #- thats's slow!
            coords =  np.transpose(np.nonzero(mask))
            for y,x in coords:
                xm = x
                ym = y
                u  = 2 * a[y,x]['x']
                v  = 2 * a[y,x]['y']
                c =  max(0, 255 - int(a[y,x]['sad']))
                #c = 220
                xm *= 16
                ym *= 16
                xe  = xm - u
                ye  = ym - v
                #cv2.rectangle(self.big,(x,y),(x+16,y+16),(0,c,c),-1)
                cv2.arrowedLine(self.big,(xm,ym),(xe,ye),(c,0,c),1)




        #---------------------------------------------------------------
        #-- MARK MOVING REGIONS
        #---------------------------------------------------------------
        #_, mask = cv2.threshold(mask, 1, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        _,contours,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # develope rectangles
        #rects = self.removeIntersections(contours)

        #---------------------------------------------------------------
        #-- START SCANNING
        #---------------------------------------------------------------
        new_points = []
        #for x0,y0,w,h in rects:
        for cnt in contours:
            x0,y0,w,h = cv2.boundingRect(cnt)

            if w*h > self.maxArea:
                print "MAXAEREA!"
                continue

            # new movement attributes
            x1  = x0 + w
            y1  = y0 + h
            cov = np.zeros((2,2))
            sad_var = 0.0
            noise = False

            if w < 2 and h < 2:
                vx   = a[y0,x0]['x'].astype(np.float64)
                vy   = a[y0,x0]['y'].astype(np.float64)
                sad_var = a[y0,x0]['sad']
                if sad_var < self.sadThreshold:
                    #print "sparkel: sad: %3d" % ( a[y0,x0]['sad'])
                    noise = True
                    continue
            else:
                #sad        = a[y0:y1,x0:x1]['sad'].mean()
                sad_var     = a[y0:y1,x0:x1]['sad'].var()
                sad_weights = a[y0:y1,x0:x1]['sad'].flatten()

                #v_vec = np.concatenate((a[y0:y1,x0:x1]['x'],a[y0:y1,x0:x1]['y']),axis=0).reshape(2,-1)
                #print "cov"
                #cov  = np.cov(v_vec, aweights=sad_weights)
                #print v_vec
                #print cov
                vx   = np.average(a[y0:y1,x0:x1]['x'].flatten(),weights=sad_weights)
                vy   = np.average(a[y0:y1,x0:x1]['y'].flatten(),weights=sad_weights)

                # w: eigenvalue  := scaling
                # v: eigenvector := rotation
                #w,v = np.linalg.eig(cov)

                if sad_var < 255:
                    noise = True
                    continue

                #if not (cov[0,0] < vx*vx or  cov[1,1] < vy*vy or abs(cov[0,1]) < abs(vx*vy)):
                #    print "covariance too big !!!!"
                #    noise = True
                #    #continue

            #print "vx: %4.0f %3d varsad: %4.2f" % (vx, w, sad_var)
            #print "vy: %4.0f %3d varsad: %4.2f" % (vy, h, sad_var)
            #print "%4.0f %4.0f" % (vx, vy)
            #print "%4.0f %4.0f" % (cov[0,0], cov[0,1])
            #print "%4.0f %4.0f" % (cov[1,0], cov[1,1])

            new_points.append([[x0,y0,w,h],[vx,vy]])


            if self.show:# and self.loop % 5:
                x0 *= 16
                y0 *= 16
                w *= 16
                h *= 16
                #cv2.rectangle(self.big,(x0,y0),(x0+w,y0+h),(0,0,0),1)
                xm = x0+w/2
                ym = y0+h/2
                if noise:
                    c = (20,20,220)
                else:
                    c = (220,20,20)
                #cv2.putText(self.big,txt,(xm, ym),cv2.FONT_HERSHEY_SIMPLEX,0.5,c,2)
                xe = int(xm+4*-vx)
                ye = int(ym+4*-vy)
                cv2.arrowedLine(self.big,(xm,ym),(xe,ye),c,2)

        # insert/update new movements
        self.update_tracks(new_points)

        #if not self.show:
        #    print "---%5.0fms ---" % (dt*1000.0)
        #self.printTracks()

        #self.tracker.printAll()
        if self.show:# and self.loop % 5:
            self.showTracks(self.big)
            # create header
            xm = 16*self.xcross
            ye = 16*self.rows
            cv2.line(self.big,(xm,0),(xm,ye),(0,0,0),1)
            str_frate = "%4.0fms (%d)" % (dt*1000.0, self.camera.framerate)
            cv2.putText(self.big, str_frate, (3, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (20,150,20), 2)
            #self.tracker.showAll(self.big)

            # Show the image in the window
            # without imshow we are at 5ms (sometimes 12ms)
            cv2.imshow( 'PiMotionAnalysis', self.big )
            if self.notPlaced:
                cv2.moveWindow( 'PiMotionAnalysis',0,0 )
                self.notPlaced = False

            key = cv2.waitKey(1) & 0xff
            if key == 27:
                raise NotImplementedError
