# vim: set et sw=4 sts=4 fileencoding=utf-8:
#
# Python Motion Analysis module of the pyCAMTracker package
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


import threading
import os
import select
import json
import re
from time import sleep

class CommandInterface(threading.Thread):
    #--------------------------------------------------------------------
    #-- constructor
    #--------------------------------------------------------------------
    def __init__(self, config=None, buff_size=128):
        super(CommandInterface,self).__init__()
        self.config    = config
        self.fd        = None
        self.buff_size = buff_size
        self.tokenDict    = {}

        if config:
            pipe = config.conf['cmdFIFO']
            try:
                self.fd = os.open(pipe, os.O_RDONLY|os.O_NONBLOCK)
            except:
                raise


        # precompile some regular expressions
        self.keyval = re.compile('(\w+)\:(\w+);$')


        # start thread
        if self.fd:
            self.terminated = False
            self.daemon = True
            self.start()

    #--------------------------------------------------------------------
    #-- register a callback function for a specific keyword
    #--------------------------------------------------------------------
    def subscribe(self, function, keyword):
        self.tokenDict[keyword] = function

    #--------------------------------------------------------------------
    #-- interprete the command
    #--------------------------------------------------------------------
    def interprete(self, cmd):
         m = self.keyval.match(cmd)
         if m:
            key = m.group(1)
            val = int(m.group(2))
            #TODO: make this more pythonic!
            if key in self.tokenDict.keys():
                self.tokenDict[key](val)


    #--------------------------------------------------------------------
    #-- Thread run function
    #--------------------------------------------------------------------
    def run(self):
        while not self.terminated:
            rl,wl,xl = select.select([self.fd],[],[], 0.5)
            if rl:
                try:
                    cmd = os.read(self.fd, self.buff_size)
                except OSError as e:
                    if e.errno == 11:
                        continue
                    else:
                        raise e
                if cmd:
                    self.interprete(cmd)

        os.close(self.fd)


    #--------------------------------------------------------------------
    #-- stop all threading stuff
    #--------------------------------------------------------------------
    def stop(self):
        self.terminated = True
        sleep(0.7)

if __name__ == '__main__':
    from ConfigReader import configuration as cfg
    def ahandler(value):
        print("AHANDLER: ", value)
    def bhandler(value):
        print("BHANDLER: ", value)

    config = cfg('.config.json')
    cmd = CommandInterface(config)
    cmd.subscribe(ahandler, 'minArea')
    cmd.subscribe(bhandler, 'maxArea')
    sleep(5)
    cmd.stop()
    cmd.join()

