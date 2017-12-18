# vim: set et sw=4 sts=4 fileencoding=utf-8:
import numpy as np
import threading
from time import sleep
import sys
import cv2
from math import atan2,hypot,degrees,acos,pi,sqrt
from collections import deque
from time import clock

MAX_TRACKS = 16

#- sorting functions
def distance(t,p):
    if t.updates == 0:
        return 99999

    px = p[0] + p[2] / 2
    py = p[1] + p[3] / 2
    return abs(t.cx - px) + abs(t.cy - py)
    #return np.hypot(t.cx - px, t.cy - py)

#- sort tracks by number of updates
def by_updates(t):
    return t.updates

class Tracker(threading.Thread):
    def __init__(self):
        super(Tracker,self).__init__()
        self.track_pool = []
        for i in range(0,MAX_TRACKS):
            self.track_pool.append(track())

        #- thread initialisation stuff
        self.event = threading.Event()
        self.event.clear()
        self.q = deque()
        #self.q.daemon = True
        self.terminated = False
        self.daemon = True
        self.start()

    def setup_sizes(self, rows, cols):
        track.maxX = cols
        track.maxY = rows

    def update_tracks(self, frame, motion):
        self.q.append([frame,motion])
        self.event.set()

    def stop(self):
        self.terminated = True
        self.q.append([0, [[[0,0,0,0],[0,0]]]])

    def run(self):
        while not self.terminated:
            try:
                frame,motion = self.q.popleft()
                self.update_track_pool(frame,motion)
            except IndexError:
                self.event.clear()
                self.event.wait(1)

    def update_track_pool(self, frame, motion):
        # walk through all changes
        self.updated = False
        has_been_tracked = 0x00000000
        min_dist = 0

        for rn,vn in motion:
            #-- search a track for this coordinate
            tracked = 0x00000000
            # >>> debug
            #cx = rn[0] + rn[2] / 2
            #cy = rn[1] + rn[3] / 2
            #print "try: ", cx,cy
            # <<< debug
            #-- sorting by distance really makes sence here
            for track in sorted(self.track_pool, key=lambda t: distance(t,rn)):
                #-- skip tracks already updated
                if has_been_tracked & track.id:
                    continue

                #-- the rest of the coordinates can be ignored
                dist = distance(track,rn)
                if dist > track.maxDist:
                    break

                # >>> debug
                #print "   [%s]: (%2d) %3d" % (track.name, track.updates,  dist)
                # <<< debug

                #-- check if track takes coordinates
                tracked = track.update(frame,rn,vn)
                if tracked:
                    has_been_tracked |= tracked
                    #print "[%s] updated" % track.name, cx,cy
                    self.updated = True
                    break

            #-- not yet tracked -> find a free slot
            if not tracked:
                for track in self.track_pool:
                    if track.updates == 0:
                        #print "[%s] new" % track.name, cx,cy
                        track.new_track(frame,rn,vn)
                        self.updated = True
                        break

        #ogfgf
        #for track in self.track_pool:
        #    if track.updates > 3:
        #        if not track.id & has_been_tracked:
        #            track.predict(frame)

        #-- remove aged tracks
        if frame % 5:
            for track in self.track_pool:
                track.clean(frame)

    def showTracks(self, frame, vis):
        for track in self.track_pool:
            track.showTrack(vis,frame)

'''
=========================================================================
'''

class track:
    track_names = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789"
    numtracks   = 0
    minCosDelta = 0.707 #cos(2*22.5)
    #minCosDelta = 0.5 #cos(2*30.0)
    maxDist     = 10.0
    maxLifeTime = 10
    estimates   = None
    maxX        = 99999
    maxY        = 99999
    xCross      = 99999
    yCross      = 99999
    crossed_handler = None
    image_handler   = None
    missed_handler  = None

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

    def crossed(self):
        if track.crossed_handler is not None:
            track.crossed_handler.event.set()

        if track.image_handler is not None:
            track.image_handler.event.set()

    def clean(self,frame):
        if self.updates > 0:
            # TODO: keep this status alive for a couple of frames
            #if self.updates > 20 and self.progressx == 0 and self.progressy == 0:
            #    print "[%s](%d) no motion!" % (self.name, self.updates)
            #    self.reset()
            #    return

            if frame - self.lastFrame > track.maxLifeTime:
                print "[%s] (%d) clean" % (self.name, self.updates)
                self.reset()

    def new_track(self,frame,rn,vn):

        #self.reset()
        if rn[0] - vn[0] < 0 or rn[1] - vn[1] < 0:
            print "[%s]: new track rejected (too low)" % self.name
            return 0
        if rn[0] + rn[2] - vn[0] > track.maxX or rn[1] + rn[3] - vn[1] > track.maxY:
            print "[%s]: new track rejected (too high)" % self.name
            return 0

        cxn  = rn[0]+rn[2]/2 #xn+wn/2
        cyn  = rn[1]+rn[3]/2 #yn+hn/2

        self.re  = rn
        self.vv  = np.array(vn)
        self.cx  = cxn
        self.cy  = cyn
        self.tr.append([cxn,cyn])
        self.updates = 1
        self.lastFrame = frame
        self.estimate()

        return self.id
        
    def estimate(self):
        if track.estimates is not None:
            vx = -int(self.vv[0])
            vy = -int(self.vv[1])
            dt = 4 #TODO: must delta frame or delta t

            if vx >= 0:
                xl = self.re[0]
                xr = min(track.maxX, self.re[0] + self.re[2] + vx*dt)
            else:
                xl = max(0, self.re[0] + vx*dt)
                xr = self.re[0]

            if vy >= 0:
                yl = self.re[1]
                yr = min(track.maxY, self.re[1] + self.re[3] + vy*dt)
            else:
                yl = max(0, self.re[1] + vy*dt)
                yr = self.re[1]

            track.estimates[yl:yr,xl:xr] += self.id

    def predict(self, frame):
        age = frame - self.lastFrame
        print "[%s] (%d) predict age: %d" % ( self.name, self.updates, age)

    def update(self,frame,rn,vn):

        # this is not the place for new tracks
        if self.updates < 1:
            return 0x00000000

        # remove expired tracks
        if frame - self.lastFrame > track.maxLifeTime:
            print "[%s]: (%d) expired" % (self.name,self.updates)
            self.reset()
            return 0x00000000

        self.lastFrame = frame

        # PiMotionAnalysis.analyse may be called more than once per frame -> double hit?
        cxn  = rn[0]+rn[2]/2
        cyn  = rn[1]+rn[3]/2
        if self.cx == cxn and self.cy == cyn and self.vv[0] == vn[0] and self.vv[1] == vn[1]:
            self.updates += 1
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
                do = np.array([self.cx - self.tr[-2][0], self.cy - self.tr[-2][1]])
                dn = np.array([dx ,dy])
                #vl = np.linalg.norm(self.vv) * np.linalg.norm(vn)
                vl = np.linalg.norm(do) * np.linalg.norm(dn)
                # accept direction if one movement vector is zero
                if vl == 0:
                    cos_delta = 1.0
                else:
                    #cos_delta = np.dot(self.vv, vn) / vl
                    cos_delta = np.dot(do, dn) / vl
            else:
                cos_delta = 1.0

            # reject all tracks out of direction
            if self.updates < 3 or dist <= 1.0 or abs(cos_delta) > track.minCosDelta:
                #print "delta+: %4.2f" % delta
                found   = self.id
                self.re = rn
                self.vv = np.array(vn)
                self.cx = cxn
                self.cy = cyn
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
                    # TURN-X if the area does not expand any more in x direction
                    self.turnedX   = True
                    self.progressx = 0

                if maxy > self.maxy or miny < self.miny:
                    self.progressy = dy
                else:
                    # TURN-Y if the area does not expand any more in y direction
                    self.turnedY   = True
                    self.progressy = 0


                # track leaves area in x direction
                if maxx+dx > track.maxX or minx+dx < 0:
                    #print "[%s](%d) <==> X out! %2d,%2d bott:%3d top:%3d" %  \
                    #(self.name, self.updates,self.cx,self.cy,minx+dx,maxx+dx)
                    self.reset()
                    return self.id

                # track leaves area in y direction
                if maxy+dy > track.maxY or miny+dy < 0:
                    #print "[%s](%d) <==> Y out! %2d,%2d left:%3d right:%3d" %  \
                    #(self.name, self.updates,self.cx,self.cy,miny+dy,maxy+dy)
                    self.reset()
                    return self.id

                self.maxx = maxx
                self.maxy = maxy
                self.minx = minx
                self.miny = miny

                self.tr.append([cxn,cyn])
                if(len(self.tr) > 16):
                    del self.tr[0]

                self.estimate()

                # track is crossing target line in Y direction
                if self.updates > 4 and self.turnedY == False and self.crossedY == False:
                   crossedYPositve  =  vn[1] > 0 and rn[1]+rn[3]  >= track.yCross
                   crossedYNegative =  vn[1] < 0 and rn[1]        <= track.yCross
                   #if crossedXPositve or crossedXNegative:
                   if crossedYNegative:
                       print "[%s](%d) %d/%d <<<<<<<<<<<<<< CROSSED <<<<<<<<<<<<<<<<<<" % (self.name,self.updates,self.cx,self.cy)
                       self.crossedY = True
                       self.crossed()

                # track is crossing target line in X direction
                #if self.updates > 4 and self.turnedX == False and self.crossedX == False:
                #   crossedXPositve  =  vn[0] < 0 and rn[0]+rn[2]  >= track.xCross
                #   crossedXNegative =  vn[0] > 0 and rn[0]        <= track.xCross
                #   #if crossedXPositve or crossedXNegative:
                #   if crossedXNegative:
                #       print "[%s](%d) %d/%d <<<<<<<<<<<<<< CROSSED <<<<<<<<<<<<<<<<<<" % (self.name,self.updates,self.cx,self.cy)
                #       self.crossedX = True
                #       self.crossed()

            else:
                print "[%s] delta-: %4.2f (%4.2f)" % (self.name,cos_delta, degrees(acos(cos_delta)))
                print "     x:%3d->%3d, y:%3d->%3d dist: %4.2f" % (self.vv[0],vn[0], self.vv[1],vn[1],dist)
                #self.reset()
                ii = 0
        else:
            #if self.updates < 2:
            #print "[%s] dist-: %4.2f" % (self.name,dist)
            ii = 0

        return found 

    def showTrack(self, vis, frame=0, color=(220,0,0)):
        if self.updates > 3:
            ci = ord(self.name) % 3
            if ci == 0:
                color = (220,20,20)
            if ci == 1:
                color = (20,220,20)
            if ci == 2:
                color = (20,20,220)
            r = self.re
            x = 8 * r[0]
            y = 8 * r[1]
            w = 8 * r[2]
            h = 8 * r[3]
            #if self.progressx == 0 and self.progressy == 0:
            #    color = (0,220, 220)
            pts=np.int32(self.tr[-25:]) * 8
            #pts=np.roll(pts,1,axis=1)
            cv2.polylines(vis, [pts], False, color)
            #age = self.lastFrame - frame
            #txt = "[%s] %d/%d %d" % ( self.name, self.cx,self.cy,age)
            #cv2.putText(vis,txt,(x,y),cv2.FONT_HERSHEY_SIMPLEX,0.3,color,1)
            #cv2.putText(vis,self.name,(y,x),cv2.FONT_HERSHEY_SIMPLEX,0.5,color,1)
            if self.crossedX:
                cv2.rectangle(vis,(x,y),(x+w,y+h),color,4)
            else:
                cv2.rectangle(vis,(x,y),(x+w,y+h),color,2)
            ###
            #xm = x+w/2
            #ym = y+h/2
            #dx = -4 * self.vv[0]
            #dy = -4 * self.vv[1]
            #xe = int(xm+dx)
            #ye = int(ym+dy)
            #cv2.arrowedLine(vis,(xm,ym),(xe,ye),color,1)

    def printTrack(self, frame=0):
        if self.progressx <> 0 or self.progressy <> 0:
            sys.stdout.write("[%s]:" %(self.name))
            for x,y in self.tr[-4:]:
                sys.stdout.write("  %02d,%02d -> " %(x,y))
            print "(#%2d vx:%02d vy:%02d) age:%d" % (self.updates,int(-self.vv[0]),int(-self.vv[1]),frame-self.lastFrame)

if __name__ == '__main__':

    t = track()
    t1 = track()
    print "mask: %08x" % t.id
    t.new_track(1,[8,8,14,4],[0.0,-4.0])
    t1.new_track(1,[1,1,4,4],[3.0,4.0])
    t.update(2,[0,0,5,5],[3.0,4.0])
    #t.update(0x50, 3,3,5,5,3.0,4.0)
    #t.update(0xAA, 0,0,5,5,3.0,4.0)
    #t.update(0xAA, 1,1,5,4,2.0,4.0)
    #t.printAll()
    #t.reset(0x3f)
    #t.printAll()
    tracker = Tracker()
    tracker.setup_sizes(10,10)
    aa = []
    tracker.update_tracks(1, [[[5,5,2,2],[1.,1.]]])
    tracker.update_tracks(2, [[[6,6,2,2],[1.,1.]]])
    tracker.update_tracks(3, [[[7,7,2,2],[1.,1.]]])
    sleep(1)
    tracker.stop()
    tracker.join()

