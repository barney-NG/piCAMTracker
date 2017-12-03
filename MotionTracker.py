# vim: set et sw=4 sts=4 fileencoding=utf-8:
import numpy as np
import sys
import cv2
from math import atan2,hypot,degrees,acos,pi,sqrt
from time import clock

class blinker:
    def __init__(self):
        self.port_green = 19
        self.port_red   = 13
        self.setupGPIO()
        self.greenLightsOn = 0
        self.redLightsOn = 0
        self.lightsOnTime = 100 # ms

    def __del__(self):
        GPIO.output(self.port_green,GPIO.LOW)
        GPIO.output(self.port_red,GPIO.LOW)

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
            GPIO.output(self.port_red,GPIO.HIGH)
            self.redLightsOn = clock()
        else:
            GPIO.output(self.port_red,GPIO.LOW)
            self.redLightsOn = 0

class track:
    track_names = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789"
    numtracks   = 0
    #minCosDelta = 0.707 #cos(45.0)
    minCosDelta = 0.5 #cos(45.0)
    maxDist     = 5.0
    maxLifeTime = 20
    estimates   = None
    maxX        = 99999
    maxY        = 99999
    xCross      = 99999
    yCross      = 99999

    def __init__(self):
        # track identification
        index     = track.numtracks % 32
        track.numtracks += 1
        self.id   = 1 << index
        self.name = track.track_names[index]
        # reset track data
        #self.reset()
        self.tr   = []
        self.re   = [0,0,0,0]
        self.vv   = np.array([0.0,0.0])
        self.cx   = 0
        self.cy   = 0
        self.ang  = 0.0
        self.maxx = 0
        self.maxy = 0
        self.minx = 99999
        self.miny = 99999
        self.progressx  = 0
        self.progressy  = 0
        self.updates    = 0
        self.lastFrame = 0
        self.cleanCrossings()

    # set all data back to start
    def reset(self):
        if self.updates < 1:
            return
        #print "[%s](%d) reset" % (self.name,self.updates)

        self.tr   = []
        self.re   = [0,0,0,0]
        self.vv   = np.array([0.0,0.0])
        self.cx   = 0
        self.cy   = 0
        self.ang  = 0.0
        self.maxx = 0
        self.maxy = 0
        self.minx = 99999
        self.miny = 99999
        self.progressx  = 0
        self.progressy  = 0
        self.updates    = 0
        self.lastFrame = 0
        self.cleanCrossings()

    def cleanCrossings(self):
        self.turnedX   = False
        self.turnedY   = False
        self.crossedX  = False
        self.crossedY  = False

    def clean(self,frame):
        if self.updates > 0:
            # TODO: keep this status alive for a couple of frames
            if self.updates > 10 and self.progressx == 0 and self.progressy == 0:
                print "[%s](%d) no motion!" % (self.name, self.updates)
                self.reset()
                return

            if frame - self.lastFrame > track.maxLifeTime:
                self.reset()

    def new_track(self,frame,rn,vn):

        self.reset()

        cxn  = rn[0]+rn[2]/2 #xn+wn/2
        cyn  = rn[1]+rn[3]/2 #yn+hn/2

        self.re  = rn
        self.vv  = np.array(vn)
        self.cx  = cxn
        self.cy  = cyn
        self.ang = 0.0
        #self.ang = degrees(atan2(cyn,cxn))
        self.ang = degrees(atan2(vn[1],vn[0]))
        self.tr.append([cxn,cyn])
        self.updates = 1
        self.lastFrame = frame

        return self.id
        
    def update_track(self,frame,rn,vn):

        # this is not the place for new tracks
        if self.updates < 1:
            return 0x00000000

        # remove expired tracks
        if frame - self.lastFrame > track.maxLifeTime:
            self.reset()
            return 0x00000000

        self.lastFrame = frame

        # PiMotionAnalysis.analyse may be called more than once per frame -> double hit?
        cxn  = rn[0]+rn[2]/2
        cyn  = rn[1]+rn[3]/2
        if self.cx == cxn and self.cy == cyn and self.vv[0] == vn[0] and self.vv[1] == vn[1]:
            print "[%s] double hit" % self.name
            return self.id

        # try to append new coordinates to track
        dx    = cxn - self.cx
        dy    = cyn - self.cy
        dist  = hypot(dx,dy)
        found = 0x00000000

        # 1. is the new point in range?
        # large objects may move really fast
        max_dist = max((rn[2]+rn[3]),track.maxDist)
        if dist >= 0.0 and dist < max_dist:
            # 2. is the new point in the right direction?
            # wait track to become mature and then check for angle
            if self.updates > 3:
                vl = np.linalg.norm(self.vv) * np.linalg.norm(vn)
                # accept direction if one movement vector is zero
                if vl == 0:
                    cos_delta = 1.0
                else:
                    cos_delta = np.dot(self.vv, vn) / vl
            else:
                cos_delta = 1.0

            # reject all tracks out of direction
            if self.updates < 3 or dist < 2.0 or abs(cos_delta) > track.minCosDelta:
                #print "delta+: %4.2f" % delta
                found   = self.id
                self.re = rn
                self.vv = np.array(vn)
                self.cx = cxn
                self.cy = cyn
                #self.ang = ang_n
                self.updates += 1

                # does the track expand into any direction?
                maxx = max(self.maxx, rn[0]+rn[2])
                maxy = max(self.maxy, rn[1]+rn[3])
                minx = min(self.minx, rn[0])
                miny = min(self.miny, rn[1])

                # update progress indicators
                if maxx > self.maxx or minx < self.minx:
                    self.progressx = dx
                else:
                    self.turnedX   = True
                    self.progressx = 0

                if maxy > self.maxy or miny < self.miny:
                    self.progressy = dy
                else:
                    self.turnedY   = True
                    self.progressy = 0


                # track leaves area in x direction
                if rn[0]+rn[2]+dx > track.maxX or rn[0]+dx < 0:
                    print "[%s](%d) X out! left:%03d right:%03d" %  \
                    (self.name, self.updates,rn[0]+dx,rn[0]+rn[2]+dx)
                    self.reset()
                    return self.id

                # track leaves area in y direction
                if rn[1]+rn[3]+dy > track.maxY or rn[1]+dy < 0:
                    print "[%s](%d) Y out! top:%03d bottom:%03d" %  \
                    (self.name, self.updates,rn[1]+dy,rn[1]+rn[3]+dy)
                    self.reset()
                    return self.id

                self.maxx = maxx
                self.maxy = maxy
                self.minx = minx
                self.miny = miny

                self.tr.append([cxn,cyn])
                if(len(self.tr) > 64):
                    del self.tr[0]

                # track is crossing target line
                if self.updates > 5 and self.turnedX == False and self.crossedX == False:
                   crossedXPositve  =  vn[0] < 0 and rn[0]+rn[2]  >= track.xCross
                   crossedXNegative =  vn[0] > 0 and rn[0]        <= track.xCross
                   #if crossedXPositve or crossedXNegative:
                   if crossedXNegative:
                       print "[%s](%d) !!!!!!!!! CROSSED !!!!!!!" % (self.name,self.updates)
                       self.crossedX = True

            else:
                print "[%s] delta-: %4.2f (%4.2f)" % (self.name,cos_delta, degrees(acos(cos_delta)))
                print "x:%3d->%3d, y:%3d->%3d dist: %4.2f" % (self.vv[0],vn[0], self.vv[1],vn[1],dist)
                self.reset()
                ii = 0
        else:
            #if self.updates < 2:
            print "[%s] dist-: %4.2f" % (self.name,dist)
            ii = 0

        return found 

    def showTrack(self, vis, color=(220,0,0)):
        if self.updates > 3:
            ci = ord(self.name) % 3
            if ci == 0:
                color = (220,20,20)
            if ci == 1:
                color = (20,220,20)
            if ci == 2:
                color = (20,20,220)
            r = self.re
            x = 16 * r[0]
            y = 16 * r[1]
            w = 16 * r[2]
            h = 16 * r[3]
            if self.progressx == 0 and self.progressy == 0:
                color = (220,220,0)
            pts=np.int32(self.tr[-25:]) * 16
            #pts=np.roll(pts,1,axis=1)
            cv2.polylines(vis, [pts], False, color)
            txt = "[%s] %d" % ( self.name, self.updates)
            cv2.putText(vis,txt,(x,y),cv2.FONT_HERSHEY_SIMPLEX,0.5,color,2)
            #cv2.putText(vis,self.name,(y,x),cv2.FONT_HERSHEY_SIMPLEX,0.5,color,1)
            if self.crossedX:
                cv2.rectangle(vis,(x,y),(x+w,y+h),color,2)
            else:
                cv2.rectangle(vis,(x,y),(x+w,y+h),color,1)
            ###
            xm = x+w/2
            ym = y+h/2
            dx = -8 * self.vv[0]
            dy = -8 * self.vv[1]
            xe = int(xm+dx)
            ye = int(ym+dy)
            cv2.arrowedLine(vis,(xm,ym),(xe,ye),color,2)

    def printTrack(self):
        if self.progressx <> 0 or self.progressy <> 0:
            sys.stdout.write("[%s]:" %(self.name))
            for x,y in self.tr[-4:]:
                sys.stdout.write("  %02d,%02d -> " %(x,y))
            print "(#%3d vx:%02d vy:%02d)" % (self.updates,int(self.vv[0]),int(self.vv[1]))

if __name__ == '__main__':

    t = track()
    t1 = track()
    print "mask: %08x" % t.id
    t.new_track([8,8,14,4],[0.0,-4.0])
    t1.new_track([1,1,4,4],[3.0,4.0])
    t.update_track([0,0,5,5],[3.0,4.0])
    #t.update(0x50, 3,3,5,5,3.0,4.0)
    #t.update(0xAA, 0,0,5,5,3.0,4.0)
    #t.update(0xAA, 1,1,5,4,2.0,4.0)
    #t.printAll()
    #t.reset(0x3f)
    #t.printAll()
