# vim: set et sw=4 sts=4 fileencoding=utf-8:
#
# Python Serial IO module of the pyCAMTracker package
# Copyright (c) 2018 Axel Barnitzke <barney@xkontor.org>
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

import threading
import serial
import sys
import re

class SerialCommunication(threading.Thread):
    def __init__(self, port='/dev/serial0', options='9600 8N1'):
        super(SerialCommunication,self).__init__()
        self.serial = None
        self.buff_size = 1024
        self.setup_serial(port, options)
        if self.serial:
            self.serial.open()
            self.serial.flushInput()
            #self.fd = self.serial.filno()
            self.terminated = False
            self.daemon = True
            self.start()


    def run(self):
        try:
            while not self.terminated:
                # read all that is there or wait for one byte
                data = self.serial.read(self.serial.in_waiting or 1)
                if data:
                    print("serial in: <%s>" % data)
                    
        except serial.SerialException:
            self.terminated = True
            raise       # re-raise???


    def stop(self):
        self.terminated = True
        if hasattr(self.serial, 'cancel_read'):
            self.serial.cancel_read()

    def setup_serial(self, port, options):
        m = re.match('^(\d+)\s+(\d)([NEOSM])(\d(\.5)?)$', options)
        if m:
            baudrate = int(m.group(1))
            bits     = int(m.group(2))
            parity   = m.group(3)
            stopbits = int(m.group(4)) # how to do 1.5? float or string ???
            try:
                self.serial = serial.serial_for_url(
                    port,
                    baudrate,
                    parity = parity,
                    stopbits = stopbits,
                    rtscts = 0,
                    xonxoff = 0,
                    timeout = 1,
                    do_not_open = True
                )
            except serial.SerialException as e:
                sys.stderr.write('could not open port {}: {}\n'.format(repr(port), e))
                raise
        else:
            sys.stderr.write("error: serial options '%s' are not supported\n" % options)
            raise NotImplementedError

if __name__ == '__main__':
    from time import sleep
    print("start")
    sc = SerialCommunication()
    sleep(3)
    sc.stop()
    print("stopped")
    sc.join()
    sc.serial.close()
    print("closed")
