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
from time import sleep,time
import prctl
import enum
import logging

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
    #logging.debug("   >%s(%d) %4.1f %4.1f"%(t.name,t.updates,dist,val))
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

class direction(enum.Enum):
    unknown = 0
    left2right = 1
    right2left = 2
    up2down = 3
    down2up = 4

class Tracker(threading.Thread):
    """
    Track manager: assigns macro blocks to tracks
    """
    #--------------------------------------------------------------------
    #-- constructor
    #--------------------------------------------------------------------
    def __init__(self, camera, greenLed=None, redLed=None, udpThread=None, config=None):
        super(Tracker,self).__init__()
        self.lock = threading.Lock()
        self.config = config
        self.camera = camera
        self.resx = camera.resolution[0]
        self.resy = camera.resolution[1]
        self.greenLEDThread = greenLed
        self.redLEDThread   = redLed
        self.udpThread = udpThread
        self.updates  = 0
        self.frame  = 0
        self.noise  = 0.0
        self.active_tracks = 0
        self.motion = None
        self.locked = False
        self.maxDist = 5
        self.trackLifeTime = 17
        self.trackMaturity = 7
        self.debug = False
        self.fobj = None
        self.cols = 0
        self.rows = 0
        self.direction = 0
        self.detectionDelay = -0.999
        prctl.set_name('ptrk.Tracker')

        #- initialize a fixed number of threads (less garbarge collection)
        self.track_pool = []

        #- do things according configuration
        if config is not None:
            Track.xCross = config.conf['xCross']
            Track.yCross = config.conf['yCross']
            Track.maxDist = self.maxDist = config.conf['maxDist']
            Track.minCosDelta = config.conf['minCosDelta']
            self.trackLifeTime = config.conf['trackLifeTime']
            self.trackMaturity = config.conf['trackMaturity']
            self.debug = config.conf['debug']

        #- create track instances
        for i in range(0,config.conf['maxTracks']):
            self.track_pool.append(Track(self))

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
        if value > 3 and value <= 10:
            self.trackMaturity = value
            logging.info("MotionTracker::trackMaturity: %d" % value)
            if self.config:
                self.config.conf['trackMaturity'] = value

    #--------------------------------------------------------------------
    #-- callback for 'maxDist' command
    #--------------------------------------------------------------------
    def set_maxDist(self, value):
        if value > 0 and value < 25:
            Track.maxDist = value
            logging.info("MotionTracker::maxDist: %d" % value)
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
            xm = int(self.cols / 2)
            ym = int(self.rows / 2)
            #         [target_rect    ]         [bounding_rect          ]
            motion = [[xm-5,ym-5,10,10],[10,10],[xm-10,ym-10,xm+10,ym+10]]

            frame = self.camera.frame.index
            self.crossed(99,time(),frame,motion)

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
            delay = self.detectionDelay
            self.frame = 0
            self.direction = 0
            self.detectionDelay = -0.999
        #logging.debug("Tracker::getStatus (%d)" % frame)
        return (delay, frame, motion)

    #--------------------------------------------------------------------
    #-- callback for crossing event
    #--------------------------------------------------------------------
    def crossed(self, updates, timestamp, frame, motion, positive_direction=False):
        if self.locked:
            logging.debug("blocked")
            return False

        with self.lock:
            self.locked = True
            if self.udpThread:
                self.udpThread.event.set()
            if self.greenLEDThread:
                self.greenLEDThread.event.set()
            self.detectionDelay = time() - timestamp
            self.camera.request_key_frame()
            self.updates  = updates
            self.frame  = frame
            self.motion = motion
            self.direction = 1 if positive_direction else -1
            sleep(0.5)

        return True

    #--------------------------------------------------------------------
    #-- callback for cut event
    #--------------------------------------------------------------------
    def turned(self, updates, timestamp, frame, motion, positive_direction=False):
        if self.locked:
            logging.debug("blocked")
            return False

        with self.lock:
            self.locked = True
            self.camera.request_key_frame()
            if self.redLEDThread:
                self.redLEDThread.event.set()
            self.updates  = updates
            self.frame  = -frame
            self.detectionDelay = time() - timestamp
            self.motion = motion
            self.direction = 0

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
    def update_tracks(self, timestamp, frame, motion):
        self.q.append([timestamp,frame,motion])
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
                timestamp,frame,motion = self.q.popleft()
                self.update_track_pool(timestamp,frame,motion)
            except IndexError:
                self.event.clear()
                self.event.wait(1)

    #--------------------------------------------------------------------
    #-- offer new points to existing tracks
    #--------------------------------------------------------------------
    def update_track_pool(self, timestamp, frame, motion):
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
            #logging.debug( "frame:%d %d,%d ===============================" % ( frame,rn[0],rn[1] ))
            # <<< debug
            #-- sorting by distance really makes sence here
            for track in sorted(self.track_pool, key=lambda t: weighted_distance(t,rn)):
                #-- skip tracks already updated
                if has_been_tracked & track.id:
                    continue

                #-- the rest of the coordinates can be ignored
                dist = distance(track,rn)
                maxDist = max(self.maxDist,2*(rn[2]+rn[3]))
                if dist > maxDist:
                    #logging.debug("   [%s] precheck: dist too big! %d > %d" % (track.name, dist, maxDist))
                    break

                # >>> debug
                #logging.debug("   check: [%s](%d): @%d,%d dist:%d" % (track.name,track.updates,track.re[0],track.re[1],dist))
                # <<< debug

                #-- check if track takes coordinates
                tracked = track.update(timestamp,frame,rn,vn)
                if tracked:
                    has_been_tracked |= tracked
                    #logging.debug( "   [%s](%d) updated with: %d,%d" % (track.name, track.updates, rn[0],rn[1]))
                    self.updated = True
                    break

            #-- not yet tracked -> find a free slot
            if not tracked:
                for track in self.track_pool:
                    if track.updates == 0:
                        #logging.debug("   [%s] new %d/%d" % (track.name, rn[0],rn[1]))
                        track.new_track(timestamp,frame,rn,vn)
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

        #self.noise = float(noise / len(self.track_pool))
        self.noise = float(active / len(self.track_pool))

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

        #logging.debug("[%s](%d) reset" % (self.name,self.updates))

        self.updates = 0
        self.tr   = []
        self.re   = [0,0,0,0]
        self.vv   = np.array([0.0,0.0])
        self.cx   = 0
        self.cy   = 0
        self.distance = [0,0]
        self.dirxOK = False
        self.diryOK = False
        self.direction = 0
        self.distxOK = False
        self.distyOK = False
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
        self.deltaX = 0
        self.deltaY = 0
        self.noprogressx  = 0
        self.noprogressy  = 0
        self.lastFrame = 0
        self.timestamp = 0
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
            #    logging.debug "[%s](%d) no motion!" % (self.name, self.updates)
            #    self.reset()
            #    return

            if frame - self.lastFrame > Track.maxLifeTime:
                #logging.debug "[%s] (%d) clean" % (self.name, self.updates)
                self.reset()

    def leadingEdge(self,rn):

        # right to left -> left edge is leading
        if self.direction == 1:
             return rn[0], rn[1] + rn[3]/2
        # left to right -> right edge is leading
        if self.direction == 2:
             return rn[0] + rn[2], rn[1] + rn[3]/2

        # up to down -> lower edge is leading
        if self.direction == 4:
             return rn[0] + rn[2]/2, rn[1] + rn[3]
        # down to up -> upper edge is leading
        if self.direction == 3:
             return rn[0] + rn[2]/2, rn[1]

        # unknown
        return rn[0] + rn[2]/2, rn[1] + rn[3]/2


    def new_track(self,timestamp,frame,rn,vn):
        """
        start a new track
        """

        #if rn[0] + vn[0] < 0 or rn[1] + vn[1] < 0:
        #    #logging.debug("[%s]: %d/%d %3.1f/%3.1f new track rejected (too low)" % (self.name,rn[0],rn[1],vn[0],vn[1]))
        #    return 0
        #if rn[0] + rn[2] + vn[0] > Track.maxX or rn[1] + rn[3] + vn[1] > Track.maxY:
        #    #logging.debug("[%s]: %d/%d %3.1f/%3.1f new track rejected (too high)" % (self.name, rn[0]+rn[2],rn[1]+rn[3],vn[0],vn[1]))
        #    return 0

        # determine followup type
        # xCross > 0 and x > xmax / 2 -> right_to_left
        # xCross > 0 and x < xmax / 2 -> left_to_right
        # yCross > 0 and y > ymax / 2 -> up_to_down
        # yCross > 0 and y < ymax / 2 -> down_to_up

        if Track.xCross > 0:
            if rn[0] > int(Track.maxX / 2):
                self.direction = 1
            else:
                self.direction = 2

        if Track.yCross > 0:
            if rn[1] > int(Track.maxY / 2):
                self.direction = 3
            else:
                self.direction = 4


        cxn,cyn = self.leadingEdge(rn)

        #cxn  = rn[0]+rn[2]/2 #xn+wn/2
        #cyn  = rn[1]+rn[3]/2 #yn+hn/2

        #cxn  = rn[0]
        #cyn  = rn[1]

        self.re  = rn
        self.vv  = np.array(vn)
        self.cx  = cxn
        self.cy  = cyn
        self.tr.append([cxn,cyn])
        self.updates = 1
        self.lastFrame = frame
        self.timestamp = timestamp

        return self.id

    def checkStartingConditions(self):
        """
        check conditions at point #3
        """

        if self.updates >= 3:
            delta = Track.maxDist
            if Track.xCross > 0:
                x0 = int(self.tr[0][0])
                x1 = int(self.tr[2][0])
                #xc = int(Track.maxX / 2)
                #- if the first occurence is right from center movement must be negative
                #- objects occuring too near to the center are rejected
                if x0 > (Track.xCross + delta) and x1 < x0:
                    return True
                #- if the first occurence is left from center movement must be positive
                if x0 < (Track.xCross - delta) and x1 >= x0:
                    return True

            if Track.yCross > 0:
                y0 = int(self.tr[0][1])
                y1 = int(self.tr[2][1])
                #yc = int(Track.maxY / 2)
                #- if the first occurence is above center movement must be negative
                #- objects occuring too near to the center are rejected
                if y0 > (Track.yCross + delta) and y1 < y0:
                    return True
                #- if the first occurence is below center movement must be positive
                if y0 < (Track.yCross - delta) and y1 > y0:
                    return True

                #logging.debug("[%s](%d) start failed: yc: %d dy: %d" % (self.name, self.updates, y0, (y1-y0)))

        return False


    #--------------------------------------------------------------------
    #-- update growing status
    #--------------------------------------------------------------------
    def updateGrowingStatus(self, rn):
        neg_count = 6
        # does the track expand into any direction?
        maxx = max(self.maxx, rn[0]+rn[2])
        maxy = max(self.maxy, rn[1]+rn[3])
        minx = min(self.minx, rn[0])
        miny = min(self.miny, rn[1])

        # update progress indicators in y direction
        if maxy > self.maxy or miny < self.miny:
            self.progressy = True
            self.deltaY = maxy - miny
            self.noprogressy = 0
        else:
            self.progressy = False

        # update progress indicators in x direction
        if maxx > self.maxx or minx < self.minx:
            self.progressx = True
            self.deltaX = maxx - minx
            self.noprogressx = 0
        else:
            self.progressx = False

        # update moving area
        self.maxx = maxx
        self.maxy = maxy
        self.minx = minx
        self.miny = miny

        self.isGrowing = self.progressx or self.progressy

        return self.isGrowing

    #--------------------------------------------------------------------
    #-- check if object is turnung before crossing plane
    #--------------------------------------------------------------------
    def detectTurn(self, dx, dy, rn):
        backward_maturiy = 5  #self.maturity
        dmin = 10
        checkDist = 5

        #- check for Y-Turn
        if Track.yCross > 0 and not (self.progressy or self.deltaY<dmin or self.turnedY or self.crossedY):
            # develope validity near by the turn
            if self.noprogressy == 1:
                # develope some criteria for a valid detection
                dy0 = self.distance[1]
                rel = -0.67 * (rn[3] /  self.deltaY) + 0.9
                self.diryOK = (dy0 < 0 and self.miny > Track.yCross and abs(self.miny-Track.yCross) < checkDist) or \
                              (dy0 > 0 and self.maxy < Track.yCross and abs(Track.yCross-self.maxy) < checkDist)
                self.distyOK = abs(dy0) > rel * self.deltaY
                #dist = (Track.yCross-self.maxy) if dy > 0 else (self.miny-Track.yCross)
                #logging.debug("[%s](%d) dy: %d, md: %d rel: %4.2f<%4.2f? dist: %d" % (self.name, self.updates, dy0, self.deltaY, rel, abs(dy0)/self.deltaY, dist))

            # track needs some maturity to have a turn detected
            self.noprogressy += 1
            if self.noprogressy > backward_maturiy and self.diryOK and self.distyOK:
                self.turned()
                self.turnedY = True
                logging.info("[%s](%02d) y:%d/%d Y-TURN"
                      % (self.name,self.updates,rn[1],rn[0]))

        #- check for X-Turn
        if Track.xCross > 0 and not (self.progressx or self.deltaX<dmin or self.turnedX or self.crossedX):
            # develope validity near by the turn
            if self.noprogressx == 1:
                # develope some criteria for a valid detection
                dx0 = self.distance[0]
                rel = -0.67 * (rn[2] /  self.deltaX) + 0.9
                self.dirxOK = (dx0 < 0 and self.minx > Track.xCross and abs(self.minx-Track.xCross) < checkDist) or \
                              (dx0 > 0 and self.maxx < Track.xCross and abs(Track.xCross-self.maxx) < checkDist)
                self.distxOK = abs(dx0) > rel * self.deltaX
                #logging.debug("[%s](%d) dy: %d, md: %d rel: %4.2f<%4.2f?" % (self.name, self.updates, dx0, self.deltaX, rel, abs(dx0)/self.deltaX))

            # track needs some maturity to have a turn detected
            self.noprogressx += 1
            if self.noprogressx > backward_maturiy and self.dirxOK and self.distxOK:
                self.turned()
                self.turnedX = True
                logging.info("[%s](%02d) x:%d/%d X-TURN"
                      % (self.name,self.updates,rn[0],rn[1]))

    #--------------------------------------------------------------------
    #-- raise crossing handler in parent class
    #--------------------------------------------------------------------
    def crossed(self, positive=False):
        if self.parent:
            self.parent.crossed(self.updates, self.timestamp, self.lastFrame, [self.re, self.vv, [self.minx, self.miny, self.maxx, self.maxy]], positive)

    def turned(self, positive=False):
        if self.parent:
            self.parent.turned(self.updates, self.timestamp, self.lastFrame, [self.re, self.vv, [self.minx, self.miny, self.maxx, self.maxy]], positive)


    #--------------------------------------------------------------------
    #-- main target: is the object crossing the crossing line?
    #-- TODO: make the same for x direction
    #--------------------------------------------------------------------
    def detectCrossing(self, dx, dy, r):
        delta = 3 # validate this!
        min_coverage = 0.4 # validate this!
        min_fill_grade = 0.25 # validate this!
        speed_limit = 5.0 # validate this!

        if Track.yCross > 0 and self.crossedY == False and self.progressy:
            # track is crossing target line in Y direction
            #  x0,y0 +--------------+
            #        |              |
            #        +--------------+ x1,y1
            #
            #  v > 0 |          --->|
            #        |              |
            #  v < 0 |<---          |
            
            # variables
            vx = -self.vv[0]; vy = -self.vv[1] # remember the velocity has wrong direction!
            x0 = r[0]; y0 = r[1]
            y1 = r[1] + r[3]
            # for low speeds take distance as indicator
            
            vy_ = abs(vy)
            if vy_ < 0.1:
                vy = float(dy)
            
            if vy_ > delta:
                delta = int(vy_) + 1

            #@@logging.debug("vy: %6.2f (%6.2f) dy: %3d delta: %3d" % (vy,-self.vv[1], dy,delta))
            # moving quality
            coverage = self.distance[1] / self.deltaY

            # for my memory:
            # deltaY = maxy - miny
            # distance[1] = sum of all dy
            # dy = actual step in y
            
            # mature check
            crossedYPositive = crossedYNegative = False
            fastText = ''
            if self.updates >= self.maturity:
                # this model uses a simple >= limit to detect a crossing event
                crossedYPositive =  vy >  0.1 and y1 >= Track.yCross and (y1 - delta) < Track.yCross and self.miny < Track.yCross and coverage > min_coverage
                crossedYNegative =  vy < -0.1 and y0 <= Track.yCross and (y0 + delta) > Track.yCross and self.maxy > Track.yCross and coverage < -min_coverage
            else:
                # fast crossing check for big and fast objects
                # a) object is faster than speed limit
                # b) object is longer than half of the distance side to turn
                # c) object front is over the turn line
                # d) object end is behind the turn line 
                fill_grade = r[3] / Track.maxY / 2
                if fill_grade > min_fill_grade and y0 <= Track.yCross and y1 >= Track.yCross:
                    crossedYPositive = vy > speed_limit
                    crossedYNegative = vy < -speed_limit
                    if crossedYPositive or crossedYNegative:
                        fastText = 'FAST-'
                        
            if crossedYPositive:
                delay = (time() - self.timestamp) * 1000.0
                
                logging.info("[%s](%02d/%4.1fms) y1:%2d/%2d vy:%+5.1f/%+5.1f dy:%2d/%2d deltaY:%2d dist:%3d cov:%4.1f %sY-CROSSED++++++++++++++++++++"
                    % (self.name,self.updates,delay,y1,x0,vy,vx,dy,dx,self.deltaY,self.distance[1],coverage,fastText))
                self.crossedY = True
                self.crossed(positive=True)

            if crossedYNegative:
                delay = (time() - self.timestamp) * 1000.0
                logging.info("[%s](%02d/%4.1fms) y0:%2d/%2d vy:%+5.1f/%+5.1f dy:%2d/%2d deltaY:%2d dist:%3d cov:%4.1f %sY-CROSSED--------------------"
                    % (self.name,self.updates,delay,y0,x0,vy,vx,dy,dx,self.deltaY,self.distance[1],coverage,fastText))
                self.crossedY = True
                self.crossed(positive=False)

        if Track.xCross > 0 and self.crossedX == False and self.progressx:
            # track is crossing target line in Y direction
            #  x0,y0 +--------------+
            #        |              |
            #        +--------------+ x1,y1
            #
            #  v > 0 |          --->|
            #        |              |
            #  v < 0 |<---          |
            
            # variables
            vx = -self.vv[0]; vy = -self.vv[1] # remember the velocity has wrong direction!
            x0 = r[0]; y0 = r[1]
            x1 = r[0] + r[2]
            # for low speeds take distance as indicator
            vx_ = abs(vx)
            if vx_ < 0.1:
                vx = float(dx)
            if vx_ > delta:
                delta = int(vx_) + 1
            # moving quality
            coverage = self.distance[0] / self.deltaX

            # mature check
            crossedXPositive = crossedXNegative = False
            fastText = ''
            if self.updates >= self.maturity:
                # this model uses a simple >= limit to detect a crossing event
                crossedXPositive =  vx >  0.1 and x1 >= Track.xCross and (x1 - delta) < Track.xCross and self.minx < Track.xCross and coverage > min_coverage
                crossedXNegative =  vx < -0.1 and x0 <= Track.xCross and (x0 + delta) > Track.xCross and self.maxx > Track.xCross and coverage < -min_coverage
            else:
                # fast crossing check for big and fast objects
                # a) object is faster than speed limit
                # b) object is longer than half of the distance side to turn
                # c) object front is over the turn line
                # d) object end is behind the turn line 
                fill_grade = r[2] / Track.maxX / 2
                if fill_grade > min_fill_grade and x0 <= Track.xCross and x1 >= Track.xCross:
                    crossedXPositive = vx > speed_limit
                    crossedXNegative = vx < -speed_limit
                    if crossedXPositive or crossedXNegative:
                        fastText = 'FAST-'
                        
            if crossedXPositive:
                delay = (time() - self.timestamp) * 1000.0
                
                logging.info("[%s](%02d/%4.1f) x1:%2d/%2d vx:%+5.1f/%+5.1f dx:%2d/%2d deltaX:%2d dist:%3d cov:%4.1f %sX-CROSSED++++++++++++++++++++"
                    % (self.name,self.updates,delay,x1,y0,vx,vy,dx,dy,self.deltaX,self.distance[0],coverage,fastText))
                self.crossedX = True
                self.crossed(positive=True)

            if crossedXNegative:
                delay = (time() - self.timestamp) * 1000.0
                logging.info("[%s](%02d/%4.1f) x0:%2d/%2d vx:%+5.1f/%+5.1f dx:%2d/%2d deltaX:%2d dist:%3d cov:%4.1f %sY-CROSSED--------------------"
                    % (self.name,self.updates,delay,x0,y0,vx,vy,dx,dy,self.deltaX,self.distance[0],coverage,fastText))
                self.crossedY = True
                self.crossed(positive=False)

        """
        if Track.xCross > 0:
            # track is crossing target line in X direction
            #  x0,y0 +--------------+
            #        |              |
            #        +--------------+ x1,y1
            #
            #  v > 0 |          --->|
            #        |              |
            #  v < 0 |<---          |

            if self.updates >= self.maturity and self.progressx == True and self.crossedX == False:
                # develope indicators
                vx = -self.vv[0]; vy = -self.vv[1] # remember the velocity has wrong direction!
                x0 = r[0]; y0 = r[1]
                x1 = r[0] + r[2]

                # for low speeds take distance as indicator
                vx_ = abs(vx)
                if vx_ < 0.1:
                    vx = float(dx)
                if vx_ > delta:
                    delta = int(vx_) + 1

                # moving quality
                coverage = self.distance[0] / self.deltaX

                # this model uses a simple >= limit to detect a crossing event
                crossedXPositive =  vx >  0.1 and x1 >= Track.xCross and (x1 - delta) < Track.xCross and self.minx < Track.xCross and coverage > 0.4
                crossedXNegative =  vx < -0.1 and x0 <= Track.xCross and (x0 + delta) > Track.xCross and self.maxx > Track.xCross and coverage < -0.4

                if crossedXPositive:
                    delay = (time() - self.timestamp) * 1000.0
                    logging.debug("[%s](%02d/%4.1f) x1:%d/%d vx:%3.1f/%3.1f dx:%d/%d X-CROSSED++++++++++++++++++++"
                        % (self.name,self.updates,delay,x1,y0,vx,vy,dx,dy))
                    self.crossedX = True
                    self.crossed(positive=True)

                if crossedXNegative:
                    delay = (time() - self.timestamp) * 1000.0
                    logging.debug("[%s](%02d/%4.1f) x0:%d/%d vx:%3.1f/%3.1f dx:%d/%d X-CROSSED--------------------"
                        % (self.name,self.updates,delay,x0,y0,vx,vy,dx,dy))
                    self.crossedX = True
                    self.crossed(positive=False)
        """
        
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
    def update(self,timestamp,frame,rn,vn):

        # this is not the place for new tracks
        if self.updates < 1:
            return 0x00000000

        # remove expired tracks
        if frame - self.lastFrame > Track.maxLifeTime:
            self.reset()
            return 0x00000000

        self.lastFrame = frame
        self.timestamp = timestamp

        # PiMotionAnalysis.analyse may be called more than once per frame -> double hit?
        #cxn  = rn[0]+rn[2]/2.0
        #cyn  = rn[1]+rn[3]/2.0

        cxn,cyn = self.leadingEdge(rn)

        #cxn  = rn[0]
        #cyn  = rn[1]

        # accept slow moving objects
        if self.cx == cxn and self.cy == cyn and self.re[2] == rn[2] and self.re[3] == rn[3]:
            self.updates += 1
            self.updateGrowingStatus(rn)
            #logging.debug "[%s] double hit" % self.name
            return self.id



        # reject objects which change area too much from frame to frame
        area  = rn[2] * rn[3]
        if self.old_area is None:
            self.old_area = area
        if area > self.old_area:
            delta_area = area/self.old_area
        else:
            delta_area = self.old_area/area
        if delta_area > 15.0 and self.updates > 2:
            return 0x00000000
        self.old_area = area
        found = 0x00000000

        # try to append new coordinates to track
        dx    = cxn - self.cx
        dy    = cyn - self.cy
        vx    = -vn[0]
        vy    = -vn[1]
        dist  = hypot(dx,dy)

        # poor mans perspective
        # big nearby objects may move fast --- far away objects may move slow
        # max_dist = m*x + b
        max_dist = Track.maxDist
        fill_grade = area / Track.maxArea

        # fast crossing check (fully empiric 8-)
        if(fill_grade > 0.3 and self.updates > 2):
            speed = hypot(vx,vy)
            m_factor = 10. if (fill_grade > 0.9) else 1.0 / (1.0 - fill_grade)
            #- enhace distance for big objects (just an empiric estimation)
            max_dist *= int(2.0 * m_factor)
            #- reduce maturity for big objects
            self.maturity = max(3,int(self.parent.trackMaturity / m_factor))

        vlength = 0.0
        oodir = self.old_dir
        # >>> debug
        #logging.debug("[%s](%d) xn/yn: %2d/%2d, vx/vy: %2d/%2d dist: %4.2f, delta_area: %4.2f" %
        #      (self.name,self.updates,rn[0],rn[1],vx,vy,dist,delta_area))
        # <<< debug

        # 1. is the new point in range?
        if dist >= 0.0 and dist < max_dist:
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

            # check direction. (the first three tracks may have a wron direction)
            if self.updates < 3 or abs(cos_delta) > Track.minCosDelta:
                #- update base data
                found = self.id
                self.distance[0] += dx
                self.distance[1] += dy
                self.re = rn
                self.vv = np.array(vn)
                self.cx = cxn
                self.cy = cyn
                self.updates += 1
                self.tr.append([cxn,cyn])
                if(len(self.tr) > 36):
                    del self.tr[0]

                #- stop investigation if object is moving in the wrong direction
                if self.updates == 3:
                    if not self.checkStartingConditions():
                        self.reset()
                        return 0x00000000

                #- is the coverered area still growing?
                isGrowing = self.updateGrowingStatus(rn)

                #@@ logging.debug("%s[%d] distance %3d/%3d Growing: %s",self.name,self.updates,self.distance[0],self.distance[1],"Yes" if isGrowing else "No")
                
                #- does the track leave the playground?
                if self.isLeaving(dx,dy):
                    self.reset()
                    return 0x00000000

                # crossing status
                if self.updates >= 2:
                    self.detectCrossing(dx,dy,rn)

                # turning status (disabled for now)
                # self.detectTurn(dx,dy,rn)

                # >>> debug
                #logging.debug("%s[%d] append %2d/%2d" %
                #      (self.name,self.updates-1,rn[0],rn[1]))
                # <<< debug
            else:
                # >>> debug
                #if oodir is not None:
                #    dxo = oodir[0]
                #    dyo = oodir[1]
                #    logging.info("[%s](%d) delta-: (%4.2f) dx/dy: %4.2f/%4.2f dxo/dyo %4.2f/%4.2f dist: %4.2f, vlength: %4.2f" %
                #        (self.name, self.updates,degrees(acos(cos_delta)),dx,dy,dxo,dyo,dist,vlength))
                #logging.debug("     vx:%3d->%3d, vy:%3d->%3d dist: %4.2f" % (self.vv[0],vn[0], self.vv[1],vn[1],dist))
                # <<< debug
                ii = 0
        else:
            #if delta_area < 10.0:
            #logging.info("[%s] dist: %4.2f > %4.2f delta_area: %4.2f" % (self.name,dist,max_dist,delta_area))
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
        #if self.crossedY and hold:
        #    cv2.rectangle(vis,(x,y),(x+w,y+h),color,-4)
        #else:
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
            logging.info("(#%2d vx:%02d vy:%02d) age:%d" % (self.updates,int(-self.vv[0]),int(-self.vv[1]),frame-self.lastFrame))

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
