# vim: set et sw=4 sts=4 fileencoding=utf-8:
#
# Python JSON module of the pyCAMTracker package
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

import json

default_config = \
{
    "IPUDPBEEP": "255.255.255.255",
    "baseB": "none",
    "debug": False,
    "debugInputPort": 0,
    "greenLEDPort": 17,
    "redLEDPort": 27,
    "statusLEDPort": 23,
    "ledActiveLow": False,
    "exposure": 0,
    "extension": 7,
    "fastMode": False,
    "signalLength": 300.0,
    # "quitAppByGPIO": False,
    "minArea": 2,
    "maxArea": 5712,
    "maxSnapshots": 100,
    "sadThreshold": 0,
    "trackLifeTime": 17,
    "trackMaturity": 5,
    "xCross": -1,
    "yCross": 30,
    "vMin": 1,
    "vMax": 25,
    "minDist": 0,
    "maxDist": 8,
    "maxTracks" : 16,
    "minCosDelta" : 0.5,
    "videoLength" : 30,
    "viewAngle": 90,
    "preview": True,
    "previewX": 8,
    "previewY": 30,
    "previewW": 640,
    "previewH": 480,
    "previewR": 0,
    "offsetX": 0,
    "offsetY": 0,
    # "cmdFIFO": "/home/pi/piCAMTracker/www/FIFO",
    # "serialPort": "/dev/serial0",
    # "serialConf": "9600 8N1",
    "ssid": "PICAM",
    "accessPoint": False
    # "streamServer": True
}

class Configuration:
    def __init__(self, file_name=None):
        self.conf = default_config
        self.configFileName = file_name
        if file_name is not None:
            self.read(file_name)


    def read(self, config_file):
        try:
            self.conf = json.load(open(config_file,'r'))
            self.configFileName = config_file
        except IOError as err:
            print("I/O error: {0}".format(err))
        except:
            raise
        # insert new default keys
        for key in default_config:
            if not key in self.conf:
                self.conf[key] = default_config[key]
        # remove old/unused keys
        delkeys = []
        for key in self.conf:
            if key not in default_config:
                delkeys.append(key)
        for key in delkeys:
            del self.conf[key]

    def set_storeParams(self,value):
        self.write()

    def write(self, config_file=None):
        if config_file is None:
            fn = self.configFileName
        else:
            fn = config_file

        try:
            json.dump(self.conf, open(fn,'w'), indent=1, sort_keys=True)
            print("config written")
        except:
            raise

if __name__ == '__main__':
    cc = Configuration('does-not-exist.json')
    print(cc.conf)
    cc.conf['fps'] = 77
    cc.conf['AAAAA'] = 'text'
    cc.write('test.json')
    cc.read('test.json')
    print(cc.conf)
