# vim: set et sw=4 sts=4 fileencoding=utf-8:
import picamera
import picamera.array
import numpy as np
import cv2
from time import sleep,clock
from math import degrees,atan2,pi

#t0 = clock()
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
        self.lastPoints = []
        self.hitList   = None
        self.xcross  = None
        self.loop = 0
        self.maxDelta = 90.0
        self.notPlaced = True

    def analyse(self, a=None):
        t1 = clock()
        dt = t1 - self.t0
        self.t0 = t1
        self.loop += 1

        if self.hitList is None:
            self.hitList  = np.zeros(a.shape, np.uint8)
            self.mmx  = np.zeros(a.shape, np.float32)
            self.mmy  = np.zeros(a.shape, np.float32)
            self.sad  = np.zeros(a.shape, np.float32)
            self.xcross = self.cols / 2
            return

        #a = np.delete(a,-1,axis=1)
        # Calculate the motion vector polar lengths
        #r = np.sqrt(
        #    np.square(a['x'].astype(np.float)) +
        #    np.square(a['y'].astype(np.float))
        #    ).clip(0, 255).astype(np.uint8)


        # Calculate the motion vector polar angles
        # arctan values are + or - pi (radians)
        # OpenCV hue values are 0-179 (degrees)
        #phi = ((np.arctan2(
        #      a['y'].astype(np.float),
        #      a['x'].astype(np.float)) +
        #      np.pi) * 180/(2*np.pi)
        #      ).clip(0, 179).astype(np.uint8)

        # Make an array for a fixed colour saturation
        #sat = np.ones_like(r) * 255
        #sat[:] = 255 # 100% saturation

        # Assemble the HSV image array
        #hsv = cv2.merge((phi,sat,r))

        # Change to the native OpenCV array (BGR)
        #bgr = cv2.cvtColor(hsv,cv2.COLOR_HSV2BGR)

        #identify movement in actual frame
        bmask = np.abs(a['x']) + np.abs(a['y']) > 0
        mask  = bmask.astype(np.uint8) # * 255

        #diff  = self.hitList - mask

        if self.show:
            if self.big is None:
                self.big = np.ones((16*mask.shape[0],16*mask.shape[1],3), np.uint8) * 220
            else:
                self.big.fill(200)
        
        # erode movement
        self.hitList = cv2.subtract(self.hitList,1)


        #if self.show:
        #    for x0,y0,x1,y1,vx,vy,ds in self.lastPoints:
        #        x0 *= 16
        #        x1 *= 16
        #        y0 *= 16
        #        y1 *= 16
        #        cv2.rectangle(self.big,(x0,y0),(x1,y1),(0,200,200),1)

        #self.lastPoints = []

        if self.show:
            #- thats's slow!
            #diff = self.hitList > 0
            #coords =  np.transpose(np.nonzero(diff))
            #for y,x in coords:
            #    c =  100 + self.hitList[y,x] * 10 
            #    x *= 16
            #    y *= 16
            #    cv2.rectangle(self.big,(x,y),(x+16,y+16),(0,c,c),-1)
            # doit fast
            bighits = cv2.resize(self.hitList.astype(np.uint8),(self.big.shape[1], self.big.shape[0]),interpolation=cv2.INTER_LINEAR)
            #self.big[...,0] = 10 * bighits + 100
            self.big = np.clip(30 * bighits + 100,0,255)



        #print "%4.0fms" % (dt*1000.0)
        #return

        #bgr = (r>2).astype(np.uint8) * 255

        # mark contours
        _,contours,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            x0,y0,w,h = cv2.boundingRect(cnt)
            if w*h > self.maxArea:
                continue
            #- evaluate movement of this area
            x1      = x0 + w
            y1      = y0 + h

            # new movement features
            value    = self.hitList[y0:y1,x0:x1].mean()
            vx_new   = a[y0:y1,x0:x1]['x'].mean()
            vy_new   = a[y0:y1,x0:x1]['y'].mean()
            sad_new  = a[y0:y1,x0:x1]['sad'].mean()
            dir_new  = degrees(atan2(vy_new,vx_new))

            # old movement features
            vx_old  = self.mmx[y0:y1,x0:x1].mean()
            vy_old  = self.mmy[y0:y1,x0:x1].mean()
            sad_old = self.sad[y0:y1,x0:x1].mean()
            dir_old = degrees(atan2(vy_old,vx_old))

            weight_x = (vx_old + vx_new) / 2.0
            weight_y = (vy_old + vy_new) / 2.0

            # assessment
            delta_dir = (dir_old - dir_new + 360.0) % 360
            delta_sad = abs(sad_old - sad_new)
            #accel     = abs(vx_old - vx_new) + abs(vy_old - vy_new)

            # movement ok?
            value += (abs(weight_x) + abs(weight_y))

            # direction ok?
            if delta_dir > self.maxDelta:
                value -= 5
            else:
                value += 1

            if value < 0:
                value = 0

            print "x:%3d y:%3d vx:%3.0f vy:%3.0f wx:%3.0f wy:%3.0f ds:%4.2f value:%4.0f" % \
                  (x0,y0,vx_new,vy_new,weight_x, weight_y,delta_sad,value)
            #value = 10 * (0.3 * dir + 0.3 * pos + 0.3 * acc)
            value_txt = "%4.2f,%4.2f" % (value,delta_sad)
            #---------------------------------------------------
            #-- HIT ESTIMATION
            #---------------------------------------------------
            beep = 1
            if vx_new > 0.0:
                dx = abs(x0 - self.xcross)
                if dx < 3 and value > 10.0:
                    print "x:%d y:%d vx:%6.2f value:%6.2f %4.2fms (%d)" % (x0,y0,vx_new,value,(1000.0*dt), self.camera.framerate)
                    beep = -1
            if vx_new < 0.0:
                dx = abs(x1 - self.xcross)
                if dx < 3 and value > 10.0:
                    print "x:%d y:%d vx:%6.2f value:%6.2f %d" % (x0,y0,vx_new,value,self.loop)
                    beep = -1

            #- estimate the next frame
            dx = int(vx_new / 4)  # TODO: should be vx * dt !!!!????
            dy = int(vy_new / 4)  # TODO: should be vy * dt !!!!????

            xl = max(0,x0+dx)
            xr = min(self.cols,x0+dx+w)
            yl = max(0,y0+dy)
            yr = min(self.rows,y0+dy+h)
            
            # update Hitlist
            self.hitList[yl:yr,xl:xr] += int(value)

            #- update vectors
            self.mmx[yl:yr,xl:xr] = vx_new
            self.mmy[yl:yr,xl:xr] = vy_new
            self.sad[yl:yr,xl:xr] = sad_new

            if self.show and value > 0:
                x0 *= 16
                y0 *= 16
                w *= 16
                h *= 16
                cnt *= 16
                cv2.rectangle(self.big,(x0,y0),(x0+w,y0+h),(0,0,0),beep)
                xm = x0+w/2
                ym = y0+h/2
                xe = int(xm+4*-vx_new)
                ye = int(ym+4*-vy_new)
                cv2.arrowedLine(self.big,(xm,ym),(xe,ye),(220,20,0),1)
                cv2.putText(self.big, value_txt, (x0+w, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (20,150,20), 2)

        if self.show:
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
                raise NotImplementedError


with picamera.PiCamera() as camera:
    resx = 1024
    resy = 576
    show = True
    cl = np.zeros((resy,resx,3), np.uint8)
    #cl[resy/2,:,:] = 0xff  #horizantal line
    cl[:,resx/2,:]  = 0xff  #vertical line
    camera.resolution = (resx,resy)
    #camera.annotate_text = "RaspberryPi3 Camera"
    if show:
        camera.framerate  = 30
    else:
        camera.framerate  = 90
    print("warm-up")
    sleep(2)
    #- preview settings
    camera.start_preview()
    camera.preview.fullscreen = False
    camera.preview.window = (50,100,resx,resy)
    camera.preview.alpha = 192
    #- overlay settings
    overlay = camera.add_overlay(source=np.getbuffer(cl),size=(resx,resy),format='rgb')
    
    overlay.fullscreen = False
    overlay.window = (50,100,resx,resy)
    overlay.alpha = 64
    overlay.layer = 3

    camera.shutter_speed = camera.exposure_speed
    camera.exposure_mode = 'off'
    g = camera.awb_gains
    camera.awb_mode  = 'off'
    camera.awb_gains = g
    wait = 86400
    with MotionAnalyser(camera, show) as output:
        #cv2.namedWindow( 'PiMotionAnalysis' )
        try:
            if show is False:
                wait = 10
            camera.start_recording('/dev/null', 'h264', motion_output=output)
            camera.wait_recording(wait) # continue recording for 20 seconds
        except NotImplementedError:
            pass

        camera.stop_recording()
        camera.stop_preview()
        camera.remove_overlay(overlay)
        #cv2.destroyWindow('PiMotionAnalysis')
