# vim: set et sw=4 sts=4 fileencoding=utf-8:
#
# Python WebUtilities for piCAMTracker package
# Copyright (c) 2021-2022 Axel Barnitzke <barney@xkontor.org>
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

from .WebSocketServer import *

#from websock import WebsocketServer
import threading
import socket
from time import sleep


class TrackerWS(WebSocketServer):
    """
    generate a server to communicate with the clients
    """
    def __init__(self, config=None, port=8084, debug=False):
        # listen on all interfaces
        self.ip = '0.0.0.0'
        self.port = port
        self.debug = debug
        # initialize web socket server
        super(TrackerWS,self).__init__(
            ip = self.ip,
            port = self.port,
            on_data_receive = self.on_data_receive,
            on_connection_open = self.on_connection_open,
            on_connection_close = self.on_connection_close,
            on_server_destruct = self.on_server_destruct,
            on_error=self.on_error)

        self.server_thread = threading.Thread(target=self.serve_forever, args=(0.5,), daemon=True)
        self.server_thread.start()

    def stop(self):
        self.alive = False
        sleep(0.5)
        self.server_thread.join()
        
    def on_data_receive(self, client, data):
        pass
    def on_connection_open(self, client):
        pass
    def on_connection_close(self, client):
        pass
    def on_server_destruct(self):
        pass
    def on_error(self, exception):
        pass
        
if __name__ == '__main__':
    print("open ws")
    ws = TrackerWS()
    seconds = 5
    print("started ... waiting for %d seconds to stop" % seconds)
    sleep(seconds)
    print("ok try to stop it now")
    ws.stop()
    ws.close_server()
    print("stopped")
    
