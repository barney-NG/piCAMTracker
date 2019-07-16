#-udp fonctionnal test
#vim: set et sw=4 sts=4 fileencoding=utf-8:

import threading
import socket
import time
import logging

UDP_IP_UDPBEEP = "192.168.1.72"
UDP_IP_HOME = "192.168.0.23"
UDP_PORT = 4445
INITMSG = "Init"
EVENTMSG = "Event"

class udpBeep(threading.Thread):
    def __init__(self, udpip, udpport):
        super(udpBeep, self).__init__()
        self.udpip = udpip
        self.port = udpport
        logging.warning (self.udpip)
        logging.warning (self.port)
        self.terminated = False
        self.event      = threading.Event()
        self.sock=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        try:
            self.sock.sendto (bytes('UDPBeep Thread Start').encode('utf-8'), (bytes(self.udpip).encode('utf-8'), self.port))
        except:
            logging.warning('No network available !')
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
                    self.sock.sendto(bytes(EVENTMSG).encode('utf-8'), (self.udpip, self.port)) 
                except socket.error as msg:
                    logging.warning('udp error')
                self.event.clear()



if __name__ == '__main__':
        udpbeep = udpBeep ("255.255.255.255", UDP_PORT)
        #udpbeep = udpBeep ("192.168.255.255", UDP_PORT)
        udpbeep.event.set ()
        time.sleep (1)
        #udpbeep.event.set ()
        



