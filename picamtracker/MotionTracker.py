# vim: set et sw=4 sts=4 fileencoding=utf-8:
#
# Python Motion Tracker module of the pyCAMTracker package
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
try:
    range = xrange
except NameError:
    pass

import sys
import threading
from collections import deque
import numpy as np
import cv2
from math import atan2,hypot,degrees,acos,pi,sqrt
#from picamtracker import GPIOPort

#- globals
MAX_TRACKS     = 16

#- sorting functions
def distance(t,p):
    if t.updates == 0:
        return 99999
    # BUG: upper corner as reference point is more stable than center
    px = p[0] + p[2] / 2.0
    py = p[1] + p[3] / 2.0
    #px = p[0]
    #py = p[1]
    return abs(t.cx - px) + abs(t.cy - py)
    #return np.hypot(t.cx - px, t.cy - py)

# for same distance prefer the more mature track
# remember: we sort from smallest to biggest
def weighted_distance(t,p):
    dist = float(distance(t,p))
    val = 0.0
    divisor = 10.0
    if t.updates > 50:
        divisor = 100.0
    if dist > 0.0:
        val = 100.0 * (dist - t.updates / divisor)
    #print("   >%s(%d) %4.1f %4.1f"%(t.name,t.updates,dist,val))
    return(val)

#- sort tracks by number of updates
def by_updates(t):
    return t.updates

#- move angle into range [-pi, pi]
def normalize_angle(x):
    x %= 2 * np.pi
    if x > np.pi:
        x -= 2 * np.pi
    return(x)

class Tracker(threading.Thread):
    """
    Track manager: assigns macro blocks to tracks
    """
    #--------------------------------------------------------------------
    #-- constructor
    #--------------------------------------------------------------------
    def __init__(self, camera, greenLed=None, redLed=None, config=None):
        super(Tracker,self).__init__()
        self.lock = threading.Lock()
        self.config = config
        self.camera = camera
        self.resx = camera.resolution[0]
        self.resy = camera.resolution[1]
        self.greenLEDThread = greenLed
        self.redLEDThread   = redLed
        self.updates  = 0
        self.frame  = 0
        self.noise  = 0.0
        self.active_tracks = 0
        self.motion = None
        self.locked = False
        self.maxDist = 5
        self.trackLifeTime = 17
        self.trackMaturity = 10
        self.debug = False
        self.fobj = None
        self.cols = 0
        self.rows = 0
        self.direction = 0

        #- initialize a fixed number of threads (less garbarge collection)
        self.track_pool = []
        #for i in range(0,MAX_TRACKS):
        for i in range(0,config.conf['maxTracks']):
            self.track_pool.append(Track(self))

        #- do things according configuration
        if config is not None:
            Track.xCross = config.conf['xCross']
            Track.yCross = config.conf['yCross']
            Track.maxDist = self.maxDist = config.conf['maxDist']
            Track.minCosDelta = config.conf['minCosDelta']
            self.trackLifeTime = config.conf['trackLifeTime']
            self.trackMaturity = config.conf['trackMaturity']
            self.debug = config.conf['debug']

        #- thread initialisation stuff
        self.event = threading.Event()
        self.event.clear()
        self.q = deque()
        self.terminated = False
        self.daemon = True
        self.start()

    #--------------------------------------------------------------------
    #-- callback for 'trackMaturity' command
    #--------------------------------------------------------------------
    def set_trackMaturity(self, value):
        if value > 0 and value < 25:
            Track.trackMaturity = value
            if self.config:
                self.config.conf['trackMaturity'] = value

    #--------------------------------------------------------------------
    #-- callback for 'maxDist' command
    #--------------------------------------------------------------------
    def set_maxDist(self, value):
        if value > 0 and value < 25:
            Track.maxDist = value
            if self.config:
                self.config.conf['maxDist'] = value

    #--------------------------------------------------------------------
    #-- called by picamera after sizes are known
    #--------------------------------------------------------------------
    def setup_sizes(self, rows, cols):
        Track.maxX = self.cols = cols
        Track.maxY = self.rows = rows
        Track.maxArea = cols * rows

    def testCrossing(self, value):
        if value > 0:
            motion = [[10,10,10,10],[1,1],[9,9,11,11]]
            frame = self.camera.frame.index
            self.crossed(99,frame,motion)

    #--------------------------------------------------------------------
    #-- release lock
    #--------------------------------------------------------------------
    def releaseLock(self):
        with self.lock:
            self.locked = False
            frame  = self.frame
            motion = self.motion
        return (frame, motion)

    #--------------------------------------------------------------------
    #-- get crossing status
    #--------------------------------------------------------------------
    def getStatus(self):
        with self.lock:
            frame  = self.frame
            motion = self.motion
            self.frame = 0
            self.direction = 0
        return (frame, motion)

    #--------------------------------------------------------------------
    #-- callback for crossing event
    #--------------------------------------------------------------------
    def crossed(self, updates, frame, motion, positive_direction=False):
        if self.locked:
            print("blocked")
            return False

        with self.lock:
            self.locked = True
            self.camera.request_key_frame()
            if self.greenLEDThread:
                self.greenLEDThread.event.set()
            self.updates  = updates
            self.frame  = frame
            self.motion = motion
            self.direction = 1 if positive_direction else -1

        return True

    #--------------------------------------------------------------------
    #-- without thread (don't start the thread otherwise index error raised!)
    #--------------------------------------------------------------------
    def deb_update_tracks(self, frame, motion):
        self.update_track_pool(frame, motion)

    #--------------------------------------------------------------------
    #- create debug output
    #--------------------------------------------------------------------
    def debug_out(self, frame, motion):
        if self.fobj is None:
            try:
                self.fobj = open("debug_tracker.csv", "w")
            except:
                raise
        for rr,vv in motion:
            self.fobj.write("%4d,%0d,%0d,%0d,%0d,%4.2f,%4.2f\n" % (frame,rr[0],rr[1],rr[2],rr[3],vv[0],vv[1]))

    #--------------------------------------------------------------------
    #-- queue new points and feed worker
    #--------------------------------------------------------------------
    def update_tracks(self, frame, motion):
        self.q.append([frame,motion])
        self.event.set()

    #--------------------------------------------------------------------
    #-- stop all threading stuff
    #--------------------------------------------------------------------
    def stop(self):
        if self.fobj:
            self.fobj.close()
        self.terminated = True
        self.q.append([0, [[[0,0,0,0],[0,0]]]])

    #--------------------------------------------------------------------
    #-- Thread run function (calls update_track_pool)
    #-- (needs dqueue to be fats enough)
    #--------------------------------------------------------------------
    def run(self):
        while not self.terminated:
            try:
                frame,motion = self.q.popleft()
                self.update_track_pool(frame,motion)
            except IndexError:
                self.event.clear()
                self.event.wait(1)

    #--------------------------------------------------------------------
    #-- offer new points to existing tracks
    #--------------------------------------------------------------------
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
            #print( "frame:%d %d,%d ===============================" % ( frame,rn[0],rn[1] ))
            # <<< debug
            #-- sorting by distance really makes sence here
            #for track in sorted(self.track_pool, key=lambda t: distance(t,rn)):
            for track in sorted(self.track_pool, key=lambda t: weighted_distance(t,rn)):
                #-- skip tracks already updated
                if has_been_tracked & track.id:
                    continue

                #-- the rest of the coordinates can be ignored
                dist = distance(track,rn)
                maxDist = max(self.maxDist,2*(rn[2]+rn[3]))
                if dist > maxDist:
                    #print("   [%s] precheck: dist too big! %d > %d" % (track.name, dist, maxDist))
                    break

                # >>> debug
                #print("   [%s](%d): %d,%d dist:%d" % (track.name,track.updates,track.re[0],track.re[1],dist))
                # <<< debug

                #-- check if track takes coordinates
                tracked = track.update(frame,rn,vn)
                if tracked:
                    has_been_tracked |= tracked
                    #print( "   [%s] updated %d,%d" % (track.name, rn[0],rn[1]))
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

        #-- remove aged tracks
        noise = 0
        active = 0
        for track in self.track_pool:
            updates = track.updates
            if updates > 0:
                active += 1
                if updates < 3:
                    noise += 1
            if frame - track.lastFrame > 2:
                track.reset()
                continue
            if updates and frame - track.lastFrame > self.trackLifeTime:
                track.reset()

        self.noise = float(noise / len(self.track_pool))
        self.active_tracks = active

    def showTracks(self, frame, vis):
        """
        show graphical interpretation of all tracks
        """
        for track in self.track_pool:
            track.showTrack(vis,frame)


class Track:
    """
    data container for tracks
    """
    track_names = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789"
    numtracks   = 0
    #minCosDelta = 0.707 #cos(2*22.5)
    #minCosDelta = 0.9
    minCosDelta = 0.5 #cos(2*30.0)
    maxDist     = 20.0
    maxLifeTime = 17
    maxX        = 99999
    maxY        = 99999
    maxArea     = 1
    xCross      = -1
    yCross      = -1

    #--------------------------------------------------------------------
    #--
    #--------------------------------------------------------------------
    def __init__(self, parent=None):
        # track identification
        index = Track.numtracks % 32
        Track.numtracks += 1
        self.id = 1 << index
        self.name = Track.track_names[index]
        self.parent = parent
        # reset track data
        self.updates = 9999
        self.reset()

    def reset(self):
        """
        reset/reuse this object to to avoid too much garbage collection
        """
        if self.updates < 1:
            return

        #print("[%s](%d) reset" % (self.name,self.updates))

        self.updates = 0
        self.tr   = []
        self.re   = [0,0,0,0]
        self.vv   = np.array([0.0,0.0])
        self.cx   = 0
        self.cy   = 0
        self.old_dir  = None
        self.old_dist = None
        self.old_area = None
        self.maxx = 0
        self.maxy = 0
        self.minx = 99999
        self.miny = 99999
        self.maturity = self.parent.trackMaturity
        self.progressx = False
        self.progressy = False
        self.noprogressx  = 0
        self.noprogressy  = 0
        self.lastFrame = 0
        self.isGrowing = True
        self.cleanCrossings()

    #--------------------------------------------------------------------
    #--
    #--------------------------------------------------------------------
    def cleanCrossings(self):
        self.turnedX   = False
        self.turnedY   = False
        self.crossedX  = False
        self.crossedY  = False

    def clean(self,frame):
        """
        check if track can be resetted
        """
        if self.updates > 0:
            # TODO: keep this status alive for a couple of frames
            #if self.updates > 20 and self.progressx == 0 and self.progressy == 0:
            #    print "[%s](%d) no motion!" % (self.name, self.updates)
            #    self.reset()
            #    return

            if frame - self.lastFrame > Track.maxLifeTime:
                #print "[%s] (%d) clean" % (self.name, self.updates)
                self.reset()

    def new_track(self,frame,rn,vn):
        """
        start a new track
        """

        #if rn[0] + vn[0] < 0 or rn[1] + vn[1] < 0:
        #    #print("[%s]: %d/%d %3.1f/%3.1f new track rejected (too low)" % (self.name,rn[0],rn[1],vn[0],vn[1]))
        #    return 0
        #if rn[0] + rn[2] + vn[0] > Track.maxX or rn[1] + rn[3] + vn[1] > Track.maxY:
        #    #print("[%s]: %d/%d %3.1f/%3.1f new track rejected (too high)" % (self.name, rn[0]+rn[2],rn[1]+rn[3],vn[0],vn[1]))
        #    return 0

        cxn  = rn[0]+rn[2]/2 #xn+wn/2
        cyn  = rn[1]+rn[3]/2 #yn+hn/2

        #cxn  = rn[0]
        #cyn  = rn[1]

        self.re  = rn
        self.vv  = np.array(vn)
        self.cx  = cxn
        self.cy  = cyn
        self.tr.append([cxn,cyn])
        self.updates = 1
        self.lastFrame = frame

        return self.id

    #--------------------------------------------------------------------
    #-- get average direction of last n moves
    #--------------------------------------------------------------------
    def avgDir(self, moves=3):
        dx = dy = 0.0
        l = len(self.tr)
        if moves >= l:
           moves = l-1
        if moves <= 0:
            return np.zeros(2)

        for n in range(-moves,0):
            dx += self.tr[n-1][0] - self.tr[n][0]
            dy += self.tr[n-1][1] - self.tr[n][1]

        return np.array([dx/float(moves),dy/float(moves)])

    #--------------------------------------------------------------------
    #-- update growing status
    #--------------------------------------------------------------------
    def updateGrowingStatus(self, rn):
        # does the track expand into any direction?
        maxx = max(self.maxx, rn[0]+rn[2])
        maxy = max(self.maxy, rn[1]+rn[3])
        minx = min(self.minx, rn[0])
        miny = min(self.miny, rn[1])

        # update progress indicators in y direction
        if maxy > self.maxy or miny < self.miny:
            self.progressy = True
            self.noprogressy = 0
        else:
            # TURN-Y if the area does not expand any more in y direction
            if self.noprogressy > 2 and not self.turnedY and not self.crossedY:
                moving_area = (maxx - minx) * (maxy - miny)
                # track needs some maturity to have a turn detected
                if self.updates > self.maturity and moving_area > 2 * self.old_area:
                    if self.parent.redLEDThread:
                        self.parent.redLEDThread.event.set()
                    self.turnedY = True
                    print("[%s](%02d) %2d Y-TURN" % (self.name,self.updates,rn[1]))
            self.progressy = False
            self.noprogressy += 1

        # update progress indicators in x direction
        if maxx > self.maxx or minx < self.minx:
            self.progressx = True
            self.noprogressx = 0
        else:
            # TURN-X if the area does not expand any more in y direction
            if self.noprogressx > 2 and not self.turnedX and not self.crossedX:
                moving_area = (maxx - minx) * (maxy - miny)
                # track needs some maturity to have a turn detected
                if self.updates > self.maturity and moving_area > 2 * self.old_area:
                    if self.parent.redLEDThread:
                        self.parent.redLEDThread.event.set()
                    self.turnedX = True
                    print("[%s](%02d) %2d X-TURN" % (self.name,self.updates,rn[1]))
            self.progressx = False
            self.noprogressx += 1

        # update moving area
        self.maxx = maxx
        self.maxy = maxy
        self.minx = minx
        self.miny = miny

        self.isGrowing =  self.progressx or self.progressy

    #--------------------------------------------------------------------
    #-- raise crossing handler in parent class
    #--------------------------------------------------------------------
    def crossed(self, positive=False):
        if self.parent:
            self.parent.crossed(self.updates, self.lastFrame, [self.re, self.vv, [self.minx, self.miny, self.maxx, self.maxy]], positive)

    #--------------------------------------------------------------------
    #-- main target: is the object crossing the crossing line?
    #-- TODO: make the same for x direction
    #--------------------------------------------------------------------
    def detectCrossing(self, dx, dy, r):
        delta = 3

        if Track.yCross > 0:
            # track is crossing target line in Y direction
            #  x0,y0 +--------------+
            #        |              |
            #        +--------------+ x1,y1
            #
            #  v > 0 |          --->|
            #        |              |
            #  v < 0 |<---          |
            #if self.updates > 4 and self.progressy == True and self.maxy-self.miny > 2*r[3] and self.crossedY == False:
            if self.updates > self.maturity and self.progressy == True and self.crossedY == False:
                # develope indicators
                vx = -self.vv[0]; vy = -self.vv[1] # remember the velocity has wrong direction!
                x0 = r[0]; y0 = r[1]
                y1 = r[1] + r[3]

                # for low speeds take distance as indicator
                vy_ = abs(vy)
                if vy_ < 0.1:
                    vy = float(dy)
                if vy_ > 5:
                    delta = int(vy_/2) + 1

                # this model uses a band of width == delta to detect a crossing event
                #crossedYPositive =  vy >  0.0 and abs(y1-Track.yCross) < delta and self.miny < Track.yCross
                #crossedYNegative =  vy <= 0.0 and abs(y0-Track.yCross) < delta and self.maxy > Track.yCross

                # this model uses a simple >= limit to detect a crossing event
                crossedYPositive =  vy >  0.1 and y1 >= Track.yCross and (y1 - delta) < Track.yCross and self.miny < Track.yCross
                crossedYNegative =  vy < -0.1 and y0 <= Track.yCross and (y0 + delta) > Track.yCross and self.maxy > Track.yCross

                if crossedYPositive:
                    print("[%s](%02d) y1:%d/%d vy:%3.1f/%3.1f dy:%d/%d CROSSED++++++++++++++++++++"
                        % (self.name,self.updates,y1,x0,vy,vx,dy,dx))
                    self.crossedY = True
                    self.crossed(positive=True)

                if crossedYNegative:
                    print("[%s](%02d) y0:%d/%d vy:%3.1f/%3.1f dy:%d/%d CROSSED--------------------"
                        % (self.name,self.updates,y0,x0,vy,vx,dy,dx))
                    self.crossedY = True
                    self.crossed(positive=False)

        if Track.xCross > 0:
            # track is crossing target line in X direction
            #  x0,y0 +--------------+
            #        |              |
            #        +--------------+ x1,y1
            #
            #  v > 0 |          --->|
            #        |              |
            #  v < 0 |<---          |

            if self.updates > self.maturity and self.progressx == True and self.crossedX == False:
                # develope indicators
                vx = -self.vv[0]; vy = -self.vv[1] # remember the velocity has wrong direction!
                x0 = r[0]; y0 = r[1]
                x1 = r[0] + r[2]

                # for low speeds take distance as indicator
                vx_ = abs(vx)
                if vx_ < 0.1:
                    vx = float(dx)
                if vx_ > 5:
                    delta = int(vx_/2) + 1

                # this model uses a band of width == delta to detect a crossing event
                #crossedYPositive =  vy >  0.0 and abs(y1-Track.yCross) < delta and self.miny < Track.yCross
                #crossedYNegative =  vy <= 0.0 and abs(y0-Track.yCross) < delta and self.maxy > Track.yCross

                # this model uses a simple >= limit to detect a crossing event
                crossedXPositive =  vx >  0.1 and x1 >= Track.xCross and (x1 - delta) < Track.xCross and self.minx < Track.xCross
                crossedXNegative =  vx < -0.1 and x0 <= Track.xCross and (x0 + delta) > Track.xCross and self.maxx > Track.xCross

                if crossedXPositive:
                    print("[%s](%02d) x1:%d/%d vx:%3.1f/%3.1f dx:%d/%d CROSSED++++++++++++++++++++"
                        % (self.name,self.updates,x1,y0,vx,vy,dx,dy))
                    self.crossedX = True
                    self.crossed(positive=True)

                if crossedXNegative:
                    print("[%s](%02d) x0:%d/%d vx:%3.1f/%3.1f dx:%d/%d CROSSED--------------------"
                        % (self.name,self.updates,x0,y0,vx,vy,dx,dy))
                    self.crossedX = True
                    self.crossed(positive=False)

    #--------------------------------------------------------------------
    #-- does the track leave the tracking area?
    #--------------------------------------------------------------------
    def isLeaving(self, dx, dy):
        leavex = self.maxx+dx > Track.maxX or self.minx+dx < 0
        leavey = self.maxy+dy > Track.maxY or self.miny+dy < 0
        return leavex and leavey

    #--------------------------------------------------------------------
    #-- update track data
    #--------------------------------------------------------------------
    def update(self,frame,rn,vn):

        # this is not the place for new tracks
        if self.updates < 1:
            return 0x00000000

        # remove expired tracks
        if frame - self.lastFrame > Track.maxLifeTime:
            self.reset()
            return 0x00000000

        self.lastFrame = frame

        # PiMotionAnalysis.analyse may be called more than once per frame -> double hit?
        cxn  = rn[0]+rn[2]/2.0
        cyn  = rn[1]+rn[3]/2.0

        # estimating via the object center produces very much noise. (??? I didn't think much about that)
        # estimation is done via the upper left corner (until I have something better)

        #cxn  = rn[0]
        #cyn  = rn[1]

        # accept slow moving objects
        if self.cx == cxn and self.cy == cyn and self.re[2] == rn[2] and self.re[3] == rn[3]:
            self.updates += 1
            self.updateGrowingStatus(rn)
            #print "[%s] double hit" % self.name
            return self.id

        # try to append new coordinates to track
        # TODO: use vx/vy to improve accuracy
        dx    = cxn - self.cx
        dy    = cyn - self.cy
        vx    = -vn[0]
        vy    = -vn[1]
        dist  = hypot(dx,dy)
        area  = rn[2] * rn[3]

        if self.old_area is None:
            self.old_area = area

        # reject objects which change area too much from frame to frame
        if area > self.old_area:
            delta_area = area/self.old_area
        else:
            delta_area = self.old_area/area

        self.old_area = area
        found = 0x00000000
        #if delta_area >= 10.0:
        #    print("delta area: %4.2f" % (delta_area))

        # poor mans perspective
        # big nearby objects may move fast --- far away objects may move slow
        # max_dist = m*x + b
        max_dist = Track.maxDist
        fill_grade = area / Track.maxArea

        if(fill_grade > 0.2):
            m_factor = 1.0 / (1.0 - fill_grade)
            #- enhace distance for big objects (just an empiric estimation)
            max_dist *= int(2.0 * m_factor)
            #- reduce maturity for big objects
            self.maturity = max(3,int(self.parent.trackMaturity / m_factor))

        vlength = 0.0
        oodir = self.old_dir
        # >>> debug
        #print("%s[%d] xn/yn: %2d/%2d, vx/vy: %2d/%2d dist: %4.2f, deltaA: %4.2f" %
        #      (self.name,self.updates,rn[0],rn[1],vx,vy,dist,delta_area))
        # <<< debug

        # 1. is the new point in range?
        if dist >= 0.0 and dist < max_dist and delta_area <= 15.0:
            # 2. is the new point in the right direction?
            # wait track to become mature and then check for angle
            if self.updates >= 3:
                # save first old status
                if self.old_dir is None:
                    dxo = self.cx - self.tr[-1][0]
                    dyo = self.cy - self.tr[-1][1]
                    self.old_dist = hypot(dxo,dyo)
                    self.old_dir  = np.array([dyo,dxo])

                # multiply vector magnitudes
                vlength = self.old_dist * dist
                #new_dir = np.array([vy ,vx])
                new_dir = np.array([dy ,dx])
                # accept all directions if one movement vector is zero
                if vlength <= 2.0:
                    cos_delta = 1.0
                else:
                    cos_delta = np.dot(self.old_dir, new_dir) / vlength

                self.old_dir  = new_dir
                self.old_dist = dist
                self.old_area = area
            else:
                cos_delta = 1.0

            # >>> debug
            #print("%s,%5d,%2d,%2d,%2d,%2d,%2d,%4.2f,%4.2f" %
            #      (self.name,frame,self.updates,rn[0],rn[1],rn[2],rn[3],cos_delta,dist))
            # <<< debug
            # reject all tracks out of direction
            #if self.updates < 3 or dist <= 1.1 or abs(cos_delta) > Track.minCosDelta:
            if self.updates < 3 or abs(cos_delta) > Track.minCosDelta:
                #- update base data
                found = self.id
                self.re = rn
                self.vv = np.array(vn)
                self.cx = cxn
                self.cy = cyn
                self.updates += 1
                self.tr.append([cxn,cyn])
                if(len(self.tr) > 24):
                    del self.tr[0]

                #- is the coverered area still growing?
                self.updateGrowingStatus(rn)

                #- does the track leave the playground?
                if self.isLeaving(dx,dy):
                    self.reset()
                    return self.id

                # crossing status
                self.detectCrossing(dx,dy,rn)

                # >>> debug
                #print("%s[%d] append %2d/%2d" %
                #      (self.name,self.updates-1,rn[0],rn[1]))
                # <<< debug
            else:
                # >>> debug
                #if oodir is not None:
                #    dxo = oodir[0]
                #    dyo = oodir[1]
                #    print("[%s](%d) delta-: (%4.2f) dx/dy: %4.2f/%4.2f dxo/dyo %4.2f/%4.2f dist: %4.2f, vlength: %4.2f" %
                #        (self.name, self.updates,degrees(acos(cos_delta)),dx,dy,dxo,dyo,dist,vlength))
                #print("     vx:%3d->%3d, vy:%3d->%3d dist: %4.2f" % (self.vv[0],vn[0], self.vv[1],vn[1],dist))
                # <<< debug
                ii = 0
        else:
            #if delta_area < 10.0:
            #    print("[%s] dist: %4.2f > %4.2f delta_area: %4.2f" % (self.name,dist,max_dist,delta_area))
            ii = 0

        return found

    #--------------------------------------------------------------------
    #--
    #--------------------------------------------------------------------
    def showTrack(self, vis, frame=0, color=(220,0,0)):
        tsize = 2
        if self.updates > 3:
            ci = ord(self.name) % 3
            if ci == 0:
                color = (220,20,20)
            if ci == 1:
                color = (20,220,20)
            if ci == 2:
                color = (20,20,220)
        else:
            color = (80,80,80)
            tsize = 1

        r = self.re
        x = 8 * r[0]
        y = 8 * r[1]
        w = 8 * r[2]
        h = 8 * r[3]

        pts=np.int32(self.tr[-25:]) * 8
        cv2.polylines(vis, [pts], False, color,2)

        text = "%s(%d)" % (self.name, self.updates)
        if x > 3 and y > 3:
            cv2.putText(vis,text,(x-3,y-3),cv2.FONT_HERSHEY_SIMPLEX,0.5,color,tsize)
        else:
            cv2.putText(vis,text,(x+w+3,y+h+3),cv2.FONT_HERSHEY_SIMPLEX,0.5,color,tsize)
        hold = self.parent.greenLEDThread and self.parent.greenLEDThread.event.isSet()
        if self.crossedY and hold:
            cv2.rectangle(vis,(x,y),(x+w,y+h),color,-4)
        else:
            cv2.rectangle(vis,(x,y),(x+w,y+h),color,2)
        cv2.rectangle(vis,(8*self.minx,8*self.miny),(8*self.maxx,8*self.maxy),color,1)
        ###
        xm = int(x+w/2)
        ym = int(y+h/2)
        dx = -4 * self.vv[0]
        dy = -4 * self.vv[1]
        xe = int(xm+dx)
        ye = int(ym+dy)
        cv2.arrowedLine(vis,(xm,ym),(xe,ye),color,3)
        cv2.circle(vis,(xm,ym),3,color,1)

    #--------------------------------------------------------------------
    #--
    #--------------------------------------------------------------------
    def printTrack(self, frame=0):
        if self.progressx != 0 or self.progressy != 0:
            sys.stdout.write("[%s]:" %(self.name))
            for x,y in self.tr[-4:]:
                sys.stdout.write("  %02d,%02d -> " %(x,y))
            print("(#%2d vx:%02d vy:%02d) age:%d" % (self.updates,int(-self.vv[0]),int(-self.vv[1]),frame-self.lastFrame))

if __name__ == '__main__':

    from time import sleep
    #t = Track()
    #t1 = Track()
    #print("mask: %08x" % t.id)
    #t.new_track(1,[8,8,14,4],[0.0,-4.0])
    #t1.new_track(1,[1,1,4,4],[3.0,4.0])
    #t.update(2,[0,0,5,5],[3.0,4.0])
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
