# vim: set et sw=4 sts=4 fileencoding=utf-8:
import numpy as np
import sys
import cv2

class Tracker:
    def __init__(self):
        self.tracks = []
        for i in range(0,32):
            self.tracks.append(track(i))

    def update(self,mask,x,y,w,h,vx,vy):
        updated = False
        mask &= 0xffffffff
        for i in range(0,32):
            target = 1 << i
            if mask & target:
                updated = True
                self.tracks[i].update(x,y,w,h,vx,vy)

        if not updated:
            for i in range(0,32):
                if self.tracks[i].updates == 0:
                    target = 1 << i
                    mask |= target
                    self.tracks[i].update(x,y,w,h,vx,vy)
                    break
        return mask

    def reset(self,mask):
        for i in range(0,32):
            target = 1 << i
            if mask & target:
                mask &= ~target 
                mask &= 0xffffffff
                self.tracks[i].reset()
        return mask

    def showAll(self, vis):
        for i in range(0,32):
            if self.tracks[i].updates > 1:
                self.tracks[i].showTrack(vis)

    def printAll(self):
        for i in range(0,32):
            if self.tracks[i].updates > 1:
                self.tracks[i].printTrack()

class track:
    track_names = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789"

    def __init__(self,index=0):
        self.tr = []
        self.x  = 0
        self.y  = 0
        self.w  = 0
        self.h  = 0
        self.vx = 0.0
        self.vy = 0.0
        self.updates = 0
        #self.id = 1 << (index % 32)
        self.name = track.track_names[index%32]

    def reset(self):
        self.x  = 0
        self.y  = 0
        self.w  = 0
        self.h  = 0
        self.vx = 0.0
        self.vy = 0.0
        self.tr = []
        self.updates = 0

    def update(self,xn,yn,wn,hn,vxn,vyn):
        do_update = True
        if xn == self.x and yn == self.y:
            do_update = False
        self.x  = xn
        self.y  = yn
        if not do_update and (wn <> self.w or hn <> self.h):
            do_update = True
        self.w  = wn
        self.h  = hn
        self.vx = vxn
        self.vy = vyn
        if do_update:
            self.updates += 1
            self.tr.append([xn+wn/2,yn+hn/2])

    def showTrack(self, vis, color=(220,0,0)):
        pts=np.int32(self.tr) * 16
        cv2.polylines(vis, [pts], False, color)
        cv2.putText(vis, self.name, (self.x, self.y), cv2.FONT_HERSHEY_SIMPLEX, 0.5,color,1)

    def printTrack(self):
        sys.stdout.write("[%s]:" %(self.name))
        for x,y in self.tr[-3:]:
            sys.stdout.write("  %d,%d -> " %(x,y))
        print "(#%d vx: %4.2f vy: %4.2f)" % (self.updates, self.vx, self.vy)

if __name__ == '__main__':

    t = Tracker()
    mask = t.update(0x00, 0,0,5,5,3.0,4.0)
    print "mask: %08x" % mask
    #t.update(0x55, 0,0,5,5,3.0,4.0)
    #t.update(0x50, 3,3,5,5,3.0,4.0)
    #t.update(0xAA, 0,0,5,5,3.0,4.0)
    #t.update(0xAA, 1,1,5,4,2.0,4.0)
    #t.printAll()
    #t.reset(0x3f)
    #t.printAll()
