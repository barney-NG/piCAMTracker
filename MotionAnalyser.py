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

def by_distance1(t,p):
    if t.updates == 0:
        return 99999

    px = p[0] + p[2] / 2
    py = p[1] + p[3] / 2
    return abs(t.cx - px) + abs(t.cy - py)
    #return np.hypot(t.cx - px, t.cy - py)

# sort tracks distance to point
def by_distance(t,m):
    p  = m[0]
    px = p[0] + p[2] / 2
    py = p[1] + p[3] / 2
    return np.hypot(t.cx - px, t.cy - py)

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
        self.frame   = 0
        self.maxDelta = 90.0
        self.notPlaced = True
        self.updated = False
        self.maxMovements = 100
        self.all_tracks = []
        for i in range(0,16):
            self.all_tracks.append(MotionTracker.track())

        self.greenLEDThread = GPIOPort.gpioPort(19)
        #self.redLEDThread   = GPIOPort.gpioPort(13)
        if self.greenLEDThread:
            MotionTracker.track.crossed_handler = self.greenLEDThread

    def __del__(self):
        if self.greenLEDThread:
            self.greenLEDThread.terminated = True
            self.greenLEDThread.join()

    def printTracks(self):
        for track in sorted(self.all_tracks, key=by_updates, reverse=True):
            track.printTrack(self.frame)

    def showTracks(self, vis):
        for track in self.all_tracks:
            track.showTrack(vis,frame=self.frame)

    def update_tracks(self, motion):
        # walk through all changes
        self.updated = False
        has_been_tracked = 0x00000000

        for rn,vn in motion:
            # search a track for this coordinate
            tracked = 0x00000000
            #for track in sorted(self.all_tracks, key=by_updates, reverse=True):
            cx = rn[0] + rn[2] / 2
            cy = rn[1] + rn[3] / 2
            print "try: ", cx,cy
            for track in sorted(self.all_tracks, key=lambda t: by_distance1(t,rn)):
                if track.updates == 0 or has_been_tracked & track.id:
                    continue

                print "   [%s]: (%2d) %3d" % (track.name, track.updates,  abs(track.cx - cx) + abs(track.cy - cy))

                tracked = track.update(self.frame,rn,vn)
                if tracked:
                    has_been_tracked |= tracked
                    print "[%s] updated" % track.name, cx,cy
                    self.updated = True
                    break
            # not yet tracked -> find a free slot
            if not tracked:
                for track in self.all_tracks:
                    if track.updates == 0:
                        print "[%s] new" % track.name, cx,cy
                        track.new_track(self.frame,rn,vn)
                        self.updated = True
                        break

        #ogfgf
        for track in self.all_tracks:
            if track.updates > 3:
                if not track.id & has_been_tracked:
                    track.predict(self.frame)

        # remove aged tracks
        if self.frame % 5:
            for track in self.all_tracks:
                track.clean(self.frame)


    def analyse(self, a=None):
        t1 = clock()
        dt = t1 - self.t0
        self.t0 = t1
        self.frame = self.camera.frame.index
        #print "fnum: %4d %4d (%d)" % (fnum,self.frame,self.camera.frame.frame_type)

        # initialize values not known at class initialization
        if self.mmx is None:
            self.mmx      = np.zeros(a.shape, np.float32)
            self.mmy      = np.zeros(a.shape, np.float32)
            MotionTracker.track.maxX = self.cols - 1
            MotionTracker.track.maxY = self.rows 
            #MotionTracker.track.estimates = np.zeros(a.shape, np.uint)
            MotionTracker.track.xCross = (self.cols-1) / 2
            self.xcross   = self.cols / 2
            self.maxMovements = self.rows * self.cols / 8
            return

        #---------------------------------------------------------------
        #-- IDENTIFY MOVEMENT
        #---------------------------------------------------------------
        #- identify movement in actual frame
        #if self.dir[0] < 0:
        mag = np.abs(a['x']) + np.abs(a['y'])
        has_movement = np.logical_and(mag > 2, mag < 200)

        #- reject if more than 25% of the macro blocks are moving
        if np.count_nonzero(has_movement) > self.maxMovements:
            return

        #- mask out movement
        mask = has_movement.astype(np.uint8) * 255

        if self.show:# and self.frame % 5:
            if self.big is None:
                self.big = np.ones((16*(mask.shape[0]-1),16*mask.shape[1],3), np.uint8) * 220
            else:
                self.big.fill(200)
                #sad_big = cv2.resize(a['sad'],(self.big.shape[1],self.big.shape[0]))
                #self.big[]
        
        #if self.show:# and self.frame % 5:
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
            else:
                #-- we are searching for regions which differ a lot from the previous frame
                #-- ignore small changes in foregroung (weaving fields)
                #sad    = a[y0:y1,x0:x1]['sad'].mean()
                sad_var = a[y0:y1,x0:x1]['sad'].var()
                if sad_var < 255:
                    rejects += 1
                    continue

                sad_weights = a[y0:y1,x0:x1]['sad'].flatten()
                sad_weights *= sad_weights

                #-- develope composite vector from weightened vectors in region
                vx   = np.average(a[y0:y1,x0:x1]['x'].flatten(),weights=sad_weights)
                vy   = np.average(a[y0:y1,x0:x1]['y'].flatten(),weights=sad_weights)

            #-- add points to list
            new_points.append([[x0,y0,w,h],[vx,vy]])


            if self.show:# and self.frame % 5:
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
        print "---%5.0fms ---" % (dt*1000.0)
        self.update_tracks(new_points)

        #if not self.show:
        #if self.updated:
        #    self.printTracks()

        #self.tracker.printAll()
        if self.show:# and self.frame % 5:
            self.showTracks(self.big)
            # create header
            xm = 16*self.xcross
            ye = 16*self.rows
            cv2.line(self.big,(xm,0),(xm,ye),(0,0,0),1)
            str_frate = "%4.0fms (%d)" % (dt*1000.0, self.camera.framerate)
            cv2.putText(self.big, str_frate, (3, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (20,150,20), 2)

            # Show the image in the window
            # without imshow we are at 5ms (sometimes 12ms)
            cv2.imshow( 'PiMotionAnalysis', self.big )
            if self.notPlaced:
                cv2.moveWindow( 'PiMotionAnalysis',0,0 )
                self.notPlaced = False

            key = cv2.waitKey(1) & 0xff
            if key == 27:
                if self.greenLEDThread:
                    self.greenLEDThread.terminated = True
                    self.greenLEDThread.join()
                raise NotImplementedError
