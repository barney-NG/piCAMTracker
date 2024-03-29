# vim: set et sw=4 sts=4 fileencoding=utf-8:
import threading
from time import sleep
import RPi.GPIO as GPIO
import prctl
import logging

def cleanup():
    GPIO.cleanup()
    
def statusLED(port, on=True):
    """
    enable the status led
    """
    GPIO.setmode(GPIO.BCM)
    logging.info("statusLED::port: %d" % port)
    GPIO.setup(port,GPIO.OUT)
    if on:
        GPIO.output(port,GPIO.HIGH)
    else:
        GPIO.output(port,GPIO.LOW)


def addCallback(port, fctn, falling=True):
    """
    add a callback function to a falling or raising edge of a port
    """
    # TODO: add exception handling
    GPIO.setmode(GPIO.BCM)
    #GPIO.setwarnings(False)
    if falling:
        GPIO.setup(port, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(port, GPIO.FALLING, callback=fctn, bouncetime=500)
    else:
        GPIO.setup(port, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.add_event_detect(port, GPIO.RISING, callback=fctn, bouncetime=500)



class gpioPort(threading.Thread):
    def __init__(self, port, duration=200., is_active_low=False, start_blinks=0):
        super(gpioPort, self).__init__()
        self.terminated = False
        self.duration = duration
        self.event = threading.Event()
        self.ports = []
        if isinstance(port, int):
            self.ports.append(port)
        else:
            self.ports = port
        self.activate   = GPIO.HIGH
        self.deactivate = GPIO.LOW
        prctl.set_name('ptrk.GPIO')

        GPIO.setmode(GPIO.BCM)
        logging.info("port: ", port)
        for p in self.ports:
            GPIO.setup(p,GPIO.OUT)

        if is_active_low:
            self.activate   = GPIO.LOW
            self.deactivate = GPIO.HIGH

        if start_blinks > 0:
            self.blink(start_blinks)

        for p in self.ports:
            GPIO.output(p,self.deactivate)

        self.daemon = True
        self.event.clear()
        self.start()

    def blink(self, numbers):
        for i in range(0,numbers):
            for port in self.ports:
                GPIO.output(port,self.activate)
            sleep(self.duration/1000.0)
            for port in self.ports:
                GPIO.output(port,self.deactivate)
            sleep(self.duration/1000.0)

    def check(self, value):
        self.event.set()

    def run(self):
        while not self.terminated:
            # wait until somebody throws an event
            if self.event.wait(1):
                # create rectangle signal on GPIO port
                for port in self.ports:
                    GPIO.output(port,self.activate)
                sleep(self.duration/1000.0)
                for port in self.ports:
                    GPIO.output(port,self.deactivate)
                self.event.clear()
        for port in self.ports:
            GPIO.cleanup(port)

if __name__ == '__main__':
    def pressed(value):
        print("pressed %d" % value)

    addCallback(2,pressed)
    #statusLED(23,on=True)

    p1=17
    p2=27

    port1 = gpioPort([17,23])
    port2 = gpioPort(p2, duration=3000)
    port1.event.set()
    port2.event.set()
    sleep(2)
    port1.event.set()
    sleep(2)
    port1.terminated = True
    port2.terminated = True
    port1.join()
    port2.join()

    statusLED(23,on=False)
    GPIO.cleanup()
