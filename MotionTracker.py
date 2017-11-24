# vim: set et sw=4 sts=4 fileencoding=utf-8:
import numpy as np
import sys
import cv2
from math import atan2,hypot,degrees
from time import clock

#class Tracker:
#    def __init__(self):
#        pass:

class track:
    track_names = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789"
    numtracks   = 0
    maxDelta    = 60.0
    maxDist     = 5.0
    maxLifeTime = 0.5
    estimates   = None

    def __init__(self):
        # track identification
        index     = track.numtracks % 32
        track.numtracks += 1
        self.id   = 1 << index
        self.name = track.track_names[index]
        # reset track data
        self.reset()

    # set all data back to start
    def reset(self):
        self.tr = []
        self.x  = 0
        self.y  = 0
        self.w  = 0
        self.h  = 0
        self.cx = 0
        self.cy = 0
        self.maxx = 0
        self.maxy = 0
        self.minx = 99999
        self.miny = 99999
        self.vx = 0.0
        self.vy = 0.0
        self.phi = 0.0
        self.updates = 0
        self.lastUpdate = 0
        print "[%s] reset" % self.name

    def update(self,mvector):
        for x0,y0,w0,h0,vx,vy in mvector:
            print "rect: ",x0,y0,w0,h0
            self.update_track(x0,y0,w0,h0,vx,vy)
            return True

        return False

    def update_with_estimate(self,mvector):
        count_myid = 0
        for x0,y0,w0,h0,vx,vy in mvector:
            x1 = x0 + w0
            y1 = y0 + h0
            count_myid = np.count_nonzero(track.estimates[y0:y1,x0:x1] & self.id)
            if count_myid > 0:
                print "rect: ",x0,y0,w0,h0
                self.update_track(x0,y0,w0,h0,vx,vy)
                return True

        return False

    def update_estimates(self,found=True):
        if found:
            #- estimate the next frame
            if self.updates < 3:
                dx = int(-self.vx)  # TODO: should be vx * dt !!!!????
                dy = int(-self.vy)  # TODO: should be vy * dt !!!!????
            else:
                dx = 2 * (self.tr[-1][0] - self.tr[-2][0])
                dy = 2 * (self.tr[-1][1] - self.tr[-2][1])

            if dx >= 0:
                xl = max(0,self.x)
                xr = min(track.estimates.shape[0],self.x+dx+self.w)
            else:
                xl = max(0,self.x+dx)
                xr = min(track.estimates.shape[0],self.x+self.w)

            if dy > 0:
                yl = max(0,self.y)
                yr = min(track.estimates.shape[1],self.y+dy+self.h)
            else:
                yl = max(0,self.y+dy)
                yr = min(track.estimates.shape[1],self.y+self.h)

            track.estimates[yl:yr,xl:xr] += self.id
        else:
            myid = track.estimates & self.id
            track.estimates -= myid

    def new_track(self,xn,yn,wn,hn,vxn,vyn):

        x1 = xn + wn
        y1 = yn + hn

        count_anyid = np.count_nonzero(track.estimates[yn:y1,xn:x1])

        if count_anyid > 0:
            print "new cell occupied"
            return False
        else:
            print "[%s] new" % self.name
  
            cxn  = xn+wn/2
            cyn  = yn+hn/2

            self.tr = []
            self.x  = xn
            self.y  = yn
            self.w  = wn
            self.h  = hn
            self.cx = cxn
            self.cy = cyn
            self.vx = vxn
            self.vy = vyn
            #self.ang = degrees(atan2(cyn,cxn))
            self.ang = degrees(atan2(vyn,vxn))
            self.tr.append([cxn,cyn])
            self.updates = 1
            self.lastUpdate = clock()
            # occupy maximal area for the first time
            dx = track.maxDist
            dy = track.maxDist
            xl = max(0,self.x-dx)
            xr = min(track.estimates.shape[0],self.x+dx+self.w)
            yl = max(0,self.y-dy)
            yr = min(track.estimates.shape[1],self.y+dy+self.h)
            track.estimates[yl:yr,xl:xr] += self.id

        return True
        
    def update_track(self,xn,yn,wn,hn,vxn,vyn):

        # this is not the place for new tracks
        if self.updates < 1:
            return False

        if clock() - self.lastUpdate > track.maxLifeTime:
            #self.update_estimates(False)
            self.reset()
            return False

        # double hit?
        cxn   = xn+wn/2
        cyn   = yn+hn/2
        if self.cx == cxn and self.cy == cyn and self.vx == vxn and self.vy == vyn:
            print "[%s] double hit" % self.name
            self.lastUpdate = clock()
            #self.vx = (self.vx + vxn) / 2
            #self.vy = (self.vx + vyn) / 2
            return True

        # try to append new coordinates to track
        dx    = cxn - self.cx
        dy    = cyn - self.cy
        dist  = hypot(dx,dy)
        found = False

        # 1. is the new point in range?
        if dist > 0.0 and dist < track.maxDist:
            # 2. is the new point in the right direction?
            ang_n = degrees(atan2(cyn,cxn))
            #ang_n = degrees(atan2(vyn,vxn))
            delta = (360.0 + abs(self.ang - ang_n)) % 360.0
            if delta < track.maxDelta:
                #print "delta+: %4.2f" % delta
                found   = True
                self.lastUpdate = clock()
                self.x  = xn
                self.y  = yn
                self.w  = wn
                self.h  = hn
                self.cx = cxn
                self.cy = cyn
                self.vx = vxn
                self.vy = vyn
                self.ang = ang_n
                self.updates += 1
                self.maxx = max(self.maxx, cxn)
                self.maxy = max(self.maxy, cyn)
                self.minx = min(self.minx, cxn)
                self.miny = min(self.miny, cyn)
                self.tr.append([cxn,cyn])
                if(len(self.tr) > 64):
                    del self.tr[0]
            else:
                if self.updates > 3:
                    print "delta-: %4.2f %4.2f -> %4.2f" % (delta, self.ang, ang_n)
                ii = 0
        else:
            #if self.updates < 2:
            #    print "dist-: %4.2f" % dist
            ii = 0

        self.update_estimates(found)
        return found 

    def showTrack(self, vis, color=(220,0,0)):
        if self.updates > 3:
            x = 16 * self.x
            y = 16 * self.y
            w = 16 * self.w
            h = 16 * self.h
            pts=np.int32(self.tr[-10:]) * 16
            #pts=np.roll(pts,1,axis=1)
            cv2.polylines(vis, [pts], False, color)
            txt = "[%s] %d" % ( self.name, self.updates)
            cv2.putText(vis,txt,(x,y),cv2.FONT_HERSHEY_SIMPLEX,0.5,color,1)
            #cv2.putText(vis,self.name,(y,x),cv2.FONT_HERSHEY_SIMPLEX,0.5,color,1)
            cv2.rectangle(vis,(x,y),(x+w,y+h),color,2)

    def printTrack(self):
        if self.updates > 2:
            sys.stdout.write("[%s]:" %(self.name))
            for x,y in self.tr[-4:]:
                sys.stdout.write("  %02d,%02d -> " %(x,y))
            print "(#%3d max:%02d,%02d min:%02d,%02d)" % (self.updates,self.maxx,self.maxy,self.minx,self.miny )

if __name__ == '__main__':

    t = track()
    print "mask: %08x" % t.id
    #t.update(0x55, 0,0,5,5,3.0,4.0)
    #t.update(0x50, 3,3,5,5,3.0,4.0)
    #t.update(0xAA, 0,0,5,5,3.0,4.0)
    #t.update(0xAA, 1,1,5,4,2.0,4.0)
    #t.printAll()
    #t.reset(0x3f)
    #t.printAll()
