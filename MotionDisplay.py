# vim: set et sw=4 sts=4 fileencoding=utf-8:
import threading
from time import sleep
import numpy as np
import os
import pygame
import cv2

class Display(threading.Thread):
    def o__init__(self, caption="Display", x=0, y=0,w=640,h=480):
        super(Display, self).__init__()
        self.terminated = False
        self.event      = threading.Event()
        self.x = x
        self.y = y
        self.notPlaced = True
        self.key = 32
        self.vis = None
        self.caption = caption

        self.daemon = True
        self.event.clear()
        self.start()

    def __init__(self, caption="Display", x=0, y=0,w=640,h=480):
        super(Display, self).__init__()
        self.terminated = False
        self.event      = threading.Event()
        self.x = x
        self.y = y
        self.notPlaced = True
        self.key = 32
        os.environ['SDL_VIDEO_WINDOW_POS'] = "%d,%d" % (x,y)

        # Check which frame buffer drivers are available
        # Start with fbcon since directfb hangs with composite output
        drivers = ['fbcon', 'directfb', 'svgalib']
        found = False
        for driver in drivers:
            # Make sure that SDL_VIDEODRIVER is set
            if not os.getenv('SDL_VIDEODRIVER'):
                os.putenv('SDL_VIDEODRIVER', driver)
            try:
                pygame.display.init()
            except pygame.error:
                continue
            found = True
            break
    
        if not found:
            raise Exception('No suitable video driver found!')

        pygame.display.set_caption(caption)
        pygame.surfarray.use_arraytype('numpy')
        self.screen = None
        #size = (pygame.display.Info().current_w, pygame.display.Info().current_h)
        #size = (w,h)
        #self.screen = pygame.display.set_mode(size,pygame.NOFRAME)
        #self.screen.fill((220, 220, 220))        

        self.vis = None

        self.daemon = True
        self.event.clear()
        self.start()

    def imshow(self, vis):
        if self.event.is_set():
            return self.key

        self.vis = vis.copy()
        self.event.set()
        return self.key

    def run(self):
        while not self.terminated:
            # wait until somebody throws an event
            if self.event.wait(1):
                # show rotated image
                if self.vis is not None:
                    #image = np.rot90(self.vis,k=1)
                    #image = self.vis
                    image = np.flipud(self.vis)
                    if self.screen is None:
                        self.screen = pygame.display.set_mode(image.shape[:2],pygame.NOFRAME)
                    pygame.surfarray.blit_array(self.screen,image)
                    pygame.display.update()

                self.event.clear()

    def Orun(self):
        while not self.terminated:
            # wait until somebody throws an event
            if self.event.wait(1):
                # show rotated image
                if self.vis is not None:
                    cv2.imshow(self.caption, np.rot90(self.vis,k=-1))
                    if self.notPlaced:
                        cv2.moveWindow(self.caption,self.x,self.y)
                        self.notPlaced = False
                    self.key = cv2.waitKey(1) & 0xff
                    if self.key == 27:
                        self.terminated = True

                self.event.clear()


#if __name__ == '__main__':

