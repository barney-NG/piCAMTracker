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
    "debug": False,
    "greenLEDPort": 19,
    "redLEDPort": 13,
    "ledActiveLow": False,
    "quitAppByGPIO": False,
    "minArea": 1,
    "maxArea": 512,
    "sadThreshold": 2048,
    "trackLifeTime": 17,
    "xCross": -1,
    "yCross": 23,
    "vMin": 1,
    "vMax": 100,
    "minDist": 0,
    "maxDist": 15,
    "maxTracks" : 16,
    "minCosDelta" : 0.5,
    "videoLength" : 5,
    "preview": True,
    "previewX": 0,
    "previewY": 0,
    "previewW": 648,
    "previewH": 360,
    "previewR": 0,
    "offsetX": 40,
    "offsetY": 40,
    "cmdFIFO": "/home/pi/piCAMTracker/www/FIFO",
    "streamServer": True
}

class Configuration:
    def __init__(self, file_name=None):
        self.conf = default_config
        self.configFileName = file_name
        if file_name is not None:
            self.read(file_name)


    def read(self, config_file):
        try:
            self.conf = json.load(open(config_file))
            self.configFileName = config_file
        except:
            pass

    def set_storeParams(self,value):
        self.write()

    def write(self, config_file=None):
        if config_file is None:
            fn = self.configFileName
        else:
            fn = config_file

        try:
            json.dump(self.conf, open(fn,'w'), indent=1, sort_keys=True)
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
