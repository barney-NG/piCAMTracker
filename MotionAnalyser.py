# vim: set et sw=4 sts=4 fileencoding=utf-8:
import picamera
import picamera.array
import numpy as np
import cv2
from time import sleep,clock
from math import degrees,atan2,pi
import MotionTracker

# sort tracks by number of updates
def by_updates(t):
    return t.updates

class MotionAnalyser(picamera.array.PiMotionAnalysis):
    def __init__(self,camera, show=False):
        super(MotionAnalyser, self).__init__(camera)
        self.camera  = camera
        self.t0      = clock()
        self.maxArea = 100
        self.big     = None
        self.show    = show
        self.mmx     = None
        self.mmy     = None
        self.hitList =  None
        self.xcross  = None
        self.loop = 0
        self.maxDelta = 90.0
        self.notPlaced = True
        self.maxMovements = 100
        self.all_tracks = []
        for i in range(0,32):
            self.all_tracks.append(MotionTracker.track())

    def intersects(self,rects,xn,yn,wn,hn):
        i = 0
        for xo,yo,wo,ho in rects:
            # full intersection (new isin old)
            if xn > xo and wn < wo and yn > yo and hn < ho:
                return rects
            # full intersection (old isin new)
            if xo > xn and wo < wn and yo > yn and ho < hn:
                rects[i] = [xn,yn,wn,hn]
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

    def printTracks(self):
        for track in sorted(self.all_tracks, key=by_updates, reverse=True):
            track.printTrack()

    def showTracks(self, vis):
        for track in self.all_tracks:
            track.showTrack(vis)

    def update_tracks(self, motion):
        # walk through all changes
        for x0,y0,w0,h0,vx,vy in motion:
            # search a track for this coordinate
            tracked = False
            #for track in self.all_tracks:
            for track in sorted(self.all_tracks, key=by_updates, reverse=True):
                tracked = track.update_track(x0,y0,w0,h0,vx,vy)
                if tracked:
                    break
            # not yet tracked -> find a free slot
            if not tracked:
                for track in self.all_tracks:
                    if track.updates == 0:
                        track.new_track(x0,y0,w0,h0,vx,vy)
                        break

        if self.show:
            esti = (MotionTracker.track.estimates > 0).astype(np.uint8) * 255
            esti_big = cv2.resize(esti,(self.big.shape[1],self.big.shape[0]))
            self.big[:,:,0] = 255 - esti_big

        MotionTracker.track.estimates.fill(0)


    def analyse(self, a=None):
        t1 = clock()
        dt = t1 - self.t0
        self.t0 = t1
        self.loop += 1

        # initialize values not known at class initialization
        if self.hitList is None:
            self.hitList  = np.zeros(a.shape, np.uint8)
            self.mmx      = np.zeros(a.shape, np.float32)
            self.mmy      = np.zeros(a.shape, np.float32)
            MotionTracker.track.estimates = np.zeros(a.shape, np.uint32)
            self.xcross   = self.cols / 2
            self.maxMovements = self.rows * self.cols / 4
            return

        #- identify movement in actual frame
        has_movement = np.abs(a['x']) + np.abs(a['y']) > 0

        #- reject if more than 25% of the macro blocks are moving
        if np.count_nonzero(has_movement) > self.maxMovements:
            return

        #- mask out movement
        mask = has_movement.astype(np.uint8) * 255

        if self.show:
            if self.big is None:
                self.big = np.ones((16*(mask.shape[0]-1),16*mask.shape[1],3), np.uint8) * 220
            else:
                self.big.fill(200)
        
        # erode movement (opencv does not shift on uint8)
        self.hitList = cv2.subtract(self.hitList,1)


        #if self.show:
        if False:
            #- thats's slow!
            coords =  np.transpose(np.nonzero(mask))
            for y,x in coords:
                xm = x
                ym = y
                u  = a[y,x]['x']
                v  = a[y,x]['y']
                c =  max(0, 255 - int(a[y,x]['sad']))
                #c = 220
                xm *= 16
                ym *= 16
                xe  = xm - u
                ye  = ym - v
                #cv2.rectangle(self.big,(x,y),(x+16,y+16),(0,c,c),-1)
                cv2.arrowedLine(self.big,(xm,ym),(xe,ye),(c,0,c),1)




        # fill gaps
        _, mask = cv2.threshold(mask, 1, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        # mark contours
        _,contours,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # remove intersections
        #rects  = self.removeIntersections(contours)

        new_points = []
        #for x0,y0,w,h in rects:
        for cnt in contours:
            x0,y0,w,h = cv2.boundingRect(cnt)
            if w*h > self.maxArea:
                continue

            # new movement attributes
            x1 = x0 + w
            y1 = y0 + h
            vx   = a[y0:y1,x0:x1]['x'].mean()
            vy   = a[y0:y1,x0:x1]['y'].mean()

            new_points.append([x0,y0,w,h,vx,vy])


            if self.show:
                x0 *= 16
                y0 *= 16
                w *= 16
                h *= 16
                cv2.rectangle(self.big,(x0,y0),(x0+w,y0+h),(0,0,0),1)
                xm = x0+w/2
                ym = y0+h/2
                xe = int(xm+4*-vx)
                ye = int(ym+4*-vy)
                cv2.arrowedLine(self.big,(xm,ym),(xe,ye),(220,20,0),3)

        ##
        self.update_tracks(new_points)
        #if not self.show:
        print "%4.0fms" % (dt*1000.0)
        self.printTracks()

        #self.tracker.printAll()
        if self.show:
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
