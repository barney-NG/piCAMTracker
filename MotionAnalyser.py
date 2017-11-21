# vim: set et sw=4 sts=4 fileencoding=utf-8:
import picamera
import picamera.array
import numpy as np
import cv2
from time import sleep,clock
from math import degrees,atan2,pi
import MotionTracker


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
        self.tmask   = None
        self.hitList =  None
        self.xcross  = None
        self.loop = 0
        self.maxDelta = 90.0
        self.notPlaced = True
        self.maxMovements = 100
        self.tracker = MotionTracker.Tracker()

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
            self.sad      = np.zeros(a.shape, np.float32)
            self.tmask    = np.zeros(a.shape, np.uint32)
            self.xcross   = self.cols / 2
            self.maxMovements = self.rows * self.cols / 4
            return

        #- remove blocks of too much pixel difference
        is_similar = a['sad'] < 255
        #- identify movement in actual frame
        has_movement = np.abs(a['x']) + np.abs(a['y']) > 0

        #- reject if more than 25% of the macro blocks are moving
        if np.count_nonzero(has_movement) > self.maxMovements:
            return

        #- mask out movement
        bmask = np.logical_and(is_similar, has_movement)
        mask  = bmask.astype(np.uint8) # * 255

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
            coords =  np.transpose(np.nonzero(bmask))
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



        if not self.show:
            print "%4.0fms" % (dt*1000.0)

        # mark contours
        _,contours,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # remove intersections
        rects  = self.removeIntersections(contours)

        for x0,y0,w,h in rects:
            # helper variables
            x1    = x0 + w
            y1    = y0 + h
            # new movement attributes
            value = self.hitList[y0:y1,x0:x1].mean()
            vx1   = a[y0:y1,x0:x1]['x'].mean()
            vy1   = a[y0:y1,x0:x1]['y'].mean()
            #sad1  = a[y0:y1,x0:x1]['sad'].mean()
            dir1  = degrees(atan2(vy1,vx1))
            trmask   = 0x00000000
            #for i in range(y0,y1):
            #    for j in range(x0,x1):
            #        trmask |= self.tmask[i,j]
            #print("read: %08x" % trmask) 
            #self.tmask[y0:y1,x0:x1] = 0

            # old movement attributes
            vx0  = self.mmx[y0:y1,x0:x1].mean()
            vy0  = self.mmy[y0:y1,x0:x1].mean()
            #sad0 = self.sad[y0:y1,x0:x1].mean()
            dir0 = degrees(atan2(vy0,vx0))

            avg_speed = np.array([vx0+vx1, vx0+vx1]) / 2

            # assessment
            delta_dir = (dir0 - dir1 + 360.0) % 360
            #delta_sad = abs(sad0 - sad1)
            #accel     = abs(vx0 - vx1) + abs(vy0 - vy1)

            # movement ok?
            value += np.linalg.norm(avg_speed)

            # direction ok?
            if delta_dir > self.maxDelta:
                value = 0
                #trmask = self.tracker.reset(trmask)
                #print("reset:%08x" % trmask) 
            else:
                #trmask = self.tracker.update(trmask,x1,y1,w,h,vx1,vy1)
                #print("new:  %08x" % trmask) 
                value += self.maxDelta / (delta_dir + 1)

            if value < 0:
                value = 0

            #@@print "x:%3d y:%3d vx:%3.0f vy:%3.0f wx:%3.0f wy:%3.0f ds:%4.2f value:%4.0f" % \
            #@@      (x0,y0,vx1,vy1,speed_x, speed_y,sad1,value)
            #value = 10 * (0.3 * dir + 0.3 * pos + 0.3 * acc)
            value_txt = "%4.2f" % (value)
            #---------------------------------------------------
            #-- HIT ESTIMATION
            #-- 
            #-- vx > 0 and x   >= tx - delta
            #-- vx < 0 and x+w <= tx + delta
            #---------------------------------------------------
            beep = 1
            if vx1 > 0.0:
                dx = abs(x0 - self.xcross)
                if dx < 3 and value > 10.0:
                    print "x:%d y:%d vx:%6.2f value:%6.2f %4.2fms" % (x0,y0,vx1,value,(1000.0*dt))
                    beep = -1
            if vx1 < 0.0:
                dx = abs(x1 - self.xcross)
                if dx < 3 and value > 10.0:
                    print "x:%d y:%d vx:%6.2f value:%6.2f %d" % (x0,y0,vx1,value,self.loop)
                    beep = -1

            #- estimate the next frame
            dx = int(vx1 / 4)  # TODO: should be vx * dt !!!!????
            dy = int(vy1 / 4)  # TODO: should be vy * dt !!!!????

            xl1 = max(0,x1+dx)
            xr1 = min(self.cols,x1+dx+w)
            yl1 = max(0,y1+dy)
            yr1 = min(self.rows,y1+dy+h)
            
            # update Hitlist
            self.hitList[yl1:yr1,xl1:xr1] += int(value)
            #print("write:%08x" % trmask) 
            #self.tmask[yl:yr,xl:xr] = trmask

            #- update vectors
            self.mmx[yl1:yr1,xl1:xr1] = vx1
            self.mmy[yl1:yr1,xl1:xr1] = vy1
            #self.sad[yl1:yr1,xl1:xr1] = sad1

            if self.show and value > 0:
                x0 *= 16
                y0 *= 16
                w *= 16
                h *= 16
                cv2.rectangle(self.big,(x0,y0),(x0+w,y0+h),(0,0,0),beep)
                xm = x0+w/2
                ym = y0+h/2
                xe = int(xm+4*-vx1)
                ye = int(ym+4*-vy1)
                cv2.arrowedLine(self.big,(xm,ym),(xe,ye),(220,20,0),3)
                cv2.putText(self.big, value_txt, (x0+w, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (20,150,20), 2)

        #self.tracker.printAll()
        if self.show:
            # create header
            xm = 16*self.xcross
            ye = 16*self.rows
            cv2.line(self.big,(xm,0),(xm,ye),(0,0,0),1)
            str_frate = "%4.0fms (%d)" % (dt*1000.0, self.camera.framerate)
            cv2.putText(self.big, str_frate, (3, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (20,150,20), 2)
            self.tracker.showAll(self.big)

            # Show the image in the window
            # without imshow we are at 5ms (sometimes 12ms)
            cv2.imshow( 'PiMotionAnalysis', self.big )
            if self.notPlaced:
                cv2.moveWindow( 'PiMotionAnalysis',0,0 )
                self.notPlaced = False

            key = cv2.waitKey(1) & 0xff
            if key == 27:
                raise NotImplementedError
