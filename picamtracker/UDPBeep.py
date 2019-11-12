#-udp fonctionnal test
#vim: set et sw=4 sts=4 fileencoding=utf-8:

import threading
import socket
import time
import logging

EVENTMSG = 'Event'

class udpBeep(threading.Thread):
    def __init__(self, udpip, udpport):
        super(udpBeep, self).__init__()
        self.udpip = udpip
        self.port = udpport
        logging.info("piCAMTracker: UDP IP: %s" % self.udpip)
        logging.info("piCAMTRacker: UDP port: %d" % self.port)
        self.terminated = False
        self.event = threading.Event()
        self.sock=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

        try:
            self.sock.sendto ('UDPBeep Thread Start'.encode(), (str(self.udpip).encode(), self.port))
        except:
            logging.error('piCAMTracker: No network available !')
            
        self.daemon = True
        self.event.clear()
        self.start()
 

    def check(self, value):
        self.event.set()

    def run(self):
        while not self.terminated:
            # wait until somebody throws an event
            if self.event.wait(0.5):
                try:
                    self.sock.sendto('Event'.encode(), (str(self.udpip).encode(), self.port)) 
                except socket.error as msg:
                    logging.error('udp error: %s' % msg)
                self.event.clear()


