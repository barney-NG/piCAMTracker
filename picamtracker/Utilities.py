# vim: set et sw=4 sts=4 fileencoding=utf-8:
#
# Python Utilities module of the piCAMTracker package
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
try:
    range = xrange
except NameError:
    pass

import sys
import os
from time import time, strftime

class nameGenerator(object):
    """
    Generates file names dirname/basename-YYYY-MM-DD-hh-mm-ss-[000-NNN].extension
    """
    def __init__(self, dirname=".", basename="noname", extension='.txt', max_sequence=1):
        self.basename = basename
        self.dirname = dirname
        self.extension= extension
        self.max_sequence = max_sequence
        self.sequence = 0
        self.continous = False
        if max_sequence > 1:
            self.continous = True
            self.sequence = self._findLastSequenceIn(dirname) + 1
            if self.sequence > self.max_sequence:
                self.sequence = 0

    def _findLastSequenceIn(self, directory):
        """
        helper function to find the file with highest sequence
        """
        if not os.path.exists(directory):
            try:
                os.makedirs(directory)
            except OSError:
                raise
        maxsequence = 0
        for file in self._walkThroughFiles(directory):
            (basename,year,month,day,hour,minute,second,tmp) = file.split('-')
            (string,extension) = tmp.split('.')
            sequence = int(string)
            if sequence > maxsequence:
                maxsequence = sequence

        return maxsequence

    def _walkThroughFiles(self, path):
        """
        find all files with a specific extension
        """
        for (dirpath, dirnames, filenames) in os.walk(path):
            for filename in filenames:
                if filename.endswith(self.extension):
                    yield os.path.join(dirpath, filename)

    def generate(self):
        """
        generate a name basename-YYY-MM-DD-hh-mm-ss-sequence.extension from actual date
        """
        datepart = strftime('%Y-%m-%d-%H-%M-%S')
        sequence = self.sequence
        if self.continous and self.sequence < self.max_sequence:
            self.sequence += 1
        else:
            self.sequence = 0

        #- generate new file name
        newname = "%s/%s-%s-%03d%s" % (self.dirname,self.basename,datepart,sequence,self.extension)

        #- remove old file with actual sequence number
        oldname = "%s/%s-*-*-*-*-*-*-%03d%s" % (self.dirname,self.basename,sequence,self.extension)
        os.system('rm -f %s' % oldname.replace(' ', '\ '))

        return newname

if __name__ == '__main__':

    #generator = nameGenerator(dirname="/tmp/test", basename="aaa")
    #print("name: %s" % generator.generate())
    #print("name: %s" % generator.generate())
    generator1 = nameGenerator(dirname="/media/pi/HP v250w", basename="bbb", extension=".data", max_sequence=10)
    for n in range(1,12):
        fname = generator1.generate()
        print("name: %s" % fname )
        fobj = open(fname, "wb")
        fobj.close()
