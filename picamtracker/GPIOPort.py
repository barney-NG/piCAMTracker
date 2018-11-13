# vim: set et sw=4 sts=4 fileencoding=utf-8:
import threading
from time import sleep
import RPi.GPIO as GPIO

class gpioPort(threading.Thread):
    def __init__(self, port, duration=200., is_active_low=False, start_blinks=0):
        super(gpioPort, self).__init__()
        self.terminated = False
        self.duration   = duration
        self.event      = threading.Event()
        self.port       = port
        self.activate   = GPIO.HIGH
        self.deactivate = GPIO.LOW

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.port,GPIO.OUT)

        if is_active_low:
            self.activate   = GPIO.LOW
            self.deactivate = GPIO.HIGH

        if start_blinks > 0:
            self.blink(start_blinks)

        GPIO.output(self.port,self.deactivate)

        self.daemon = True
        self.event.clear()
        self.start()

    def blink(self, numbers):
        for i in range(0,numbers):
            GPIO.output(self.port,self.activate)
            sleep(self.duration/1000.0)
            GPIO.output(self.port,self.deactivate)
            sleep(self.duration/1000.0)
      
    def check(self, value):
        self.event.set()

    def run(self):
        while not self.terminated:
            # wait until somebody throws an event
            if self.event.wait(1):
                # create rectect signal on GPIO port
                GPIO.output(self.port,self.activate)
                sleep(self.duration/1000.0)
                GPIO.output(self.port,self.deactivate)
                self.event.clear()

        GPIO.cleanup(self.port)

if __name__ == '__main__':
    p1=13
    p2=19

    port1 = gpioPort(13)
    port2 = gpioPort(19, duration=3000)
    port1.event.set()
    port2.event.set()
    sleep(2)
    port1.event.set()
    sleep(2)
    port1.terminated = True
    port2.terminated = True
    port1.join()
    port2.join()

