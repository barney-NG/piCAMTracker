require=(function e(t,n,r){function s(o,u){if(!n[o]){if(!t[o]){var a=typeof require=="function"&&require;if(!u&&a)return a(o,!0);if(i)return i(o,!0);var f=new Error("Cannot find module '"+o+"'");throw f.code="MODULE_NOT_FOUND",f}var l=n[o]={exports:{}};t[o][0].call(l.exports,function(e){var n=t[o][1][e];return s(n?n:e)},l,l.exports,e,t,n,r)}return n[o].exports}var i=typeof require=="function"&&require;for(var o=0;o<r.length;o++)s(r[o]);return s})({1:[function(require,module,exports){
var AwesomeWebSocket, ReconnectingWebSocket, background;

ReconnectingWebSocket = require('./reconnecting-websocket.litcoffee');

background = require('./background-process.litcoffee');

AwesomeWebSocket = (function() {
  function AwesomeWebSocket(urls) {
    var openAtAll, sendloop, socket, url, _i, _len, _ref;
    this.urls = urls;
    openAtAll = false;
    this.lastSocket = void 0;
    this.sockets = [];
    this.messageQueue = [];
    if (typeof this.urls === "string") {
      this.urls = [this.urls];
    }
    _ref = this.urls;
    for (_i = 0, _len = _ref.length; _i < _len; _i++) {
      url = _ref[_i];
      socket = new ReconnectingWebSocket(url);
      this.sockets.push(socket);
      socket.onmessage = (function(_this) {
        return function(evt) {
          return _this.onmessage(evt);
        };
      })(this);
      socket.onerror = (function(_this) {
        return function(err) {
          return _this.onerror(err);
        };
      })(this);
      socket.onopen = (function(_this) {
        return function(evt) {
          if (!openAtAll) {
            openAtAll = true;
            return _this.onopen(evt);
          }
        };
      })(this);
    }
    this.forceclose = false;
    sendloop = (function(_this) {
      return function() {
        var data, trySocket;
        background(sendloop);
        if (_this.messageQueue.length) {
          if (_this.lastSocket) {
            trySocket = _this.lastSocket;
            _this.lastSocket = null;
          } else {
            trySocket = _this.sockets.pop();
            _this.sockets.unshift(trySocket);
          }
          if (trySocket.readyState === WebSocket.OPEN) {
            data = _this.messageQueue[_this.messageQueue.length - 1];
            trySocket.send(data);
            _this.lastSocket = trySocket;
            return _this.messageQueue.pop();
          }
        }
      };
    })(this);
    background(sendloop);
  }

  AwesomeWebSocket.prototype.send = function(data) {
    return this.messageQueue.unshift(data);
  };

  AwesomeWebSocket.prototype.keepAlive = function(timeoutMs, message) {
    var socket, _i, _len, _ref, _results;
    _ref = this.sockets;
    _results = [];
    for (_i = 0, _len = _ref.length; _i < _len; _i++) {
      socket = _ref[_i];
      _results.push(socket.keepAlive(timeoutMs, message));
    }
    return _results;
  };

  AwesomeWebSocket.prototype.close = function() {
    var socket, _i, _len, _ref;
    _ref = this.sockets;
    for (_i = 0, _len = _ref.length; _i < _len; _i++) {
      socket = _ref[_i];
      socket.close();
    }
    return this.onclose();
  };

  AwesomeWebSocket.prototype.onopen = function(event) {};

  AwesomeWebSocket.prototype.onclose = function(event) {};

  AwesomeWebSocket.prototype.onmessage = function(event) {};

  AwesomeWebSocket.prototype.onerror = function(event) {};

  return AwesomeWebSocket;

})();

module.exports = AwesomeWebSocket;


},{"./background-process.litcoffee":2,"./reconnecting-websocket.litcoffee":3}],2:[function(require,module,exports){
var background;

background = typeof window !== "undefined" && window !== null ? window.requestAnimationFrame : void 0;

if ((typeof chrome !== "undefined" && chrome !== null ? chrome.extension : void 0) || !(typeof window !== "undefined" && window !== null ? window.requestAnimationFrame : void 0)) {
  background = setTimeout;
}

module.exports = background;


},{}],3:[function(require,module,exports){
var ReconnectingWebSocket, background;

background = require('./background-process.litcoffee');

ReconnectingWebSocket = (function() {
  function ReconnectingWebSocket(url) {
    this.url = url;
    this.forceClose = false;
    this.wasConnected = false;
    this.reconnectAfter = 0;
    this.connectLoop();
  }

  ReconnectingWebSocket.prototype.connectLoop = function() {
    return background((function(_this) {
      return function() {
        if (_this.forceClose) {
          return;
        }
        if (_this.readyState !== WebSocket.OPEN && _this.readyState !== WebSocket.CONNECTING) {
          if (Date.now() > _this.reconnectAfter) {
            _this.reconnectAfter = Date.now() + 500;
            _this.connect();
          }
        }
        return _this.connectLoop();
      };
    })(this));
  };

  ReconnectingWebSocket.prototype.connect = function() {
    this.readyState = WebSocket.CONNECTING;
    this.ws = new WebSocket(this.url);
    this.ws.onmessage = (function(_this) {
      return function(event) {
        return _this.onmessage(event);
      };
    })(this);
    this.ws.onopen = (function(_this) {
      return function(event) {
        _this.readyState = WebSocket.OPEN;
        _this.wasConnected = true;
        return _this.onopen(event);
      };
    })(this);
    this.ws.onclose = (function(_this) {
      return function(event) {
        _this.readyState = WebSocket.CLOSED;
        if (_this.wasConnected) {
          _this.ondisconnect({
            forceClose: _this.forceClose
          });
        }
        if (_this.forceClose) {
          return _this.onclose(event);
        }
      };
    })(this);
    return this.ws.onerror = (function(_this) {
      return function(event) {
        _this.readyState = WebSocket.CLOSED;
        return _this.onerror(event);
      };
    })(this);
  };

  ReconnectingWebSocket.prototype.send = function(data) {
    var state;
    state = this.readyState;
    this.readyState = WebSocket.CLOSING;
    if (typeof data === "object") {
      this.ws.send(JSON.stringify(data));
    } else {
      this.ws.send(data);
    }
    return this.readyState = state;
  };

  ReconnectingWebSocket.prototype.close = function() {
    this.forceClose = true;
    return this.ws.close();
  };

  ReconnectingWebSocket.prototype.keepAlive = function(timeoutMs, message) {
    var sendMessage;
    sendMessage = (function(_this) {
      return function() {
        return _this.send(message);
      };
    })(this);
    return setInterval(sendMessage, timeoutMs);
  };

  ReconnectingWebSocket.prototype.onopen = function(event) {};

  ReconnectingWebSocket.prototype.onclose = function(event) {};

  ReconnectingWebSocket.prototype.onmessage = function(event) {};

  ReconnectingWebSocket.prototype.onerror = function(event) {};

  ReconnectingWebSocket.prototype.ondisconnect = function(event) {};

  return ReconnectingWebSocket;

})();

module.exports = ReconnectingWebSocket;


},{"./background-process.litcoffee":2}],"awesome-websocket":[function(require,module,exports){
var clients = {
  AwesomeWebSocket: require("./src/awesome-websocket.litcoffee")
  ,ReconnectingWebSocket: require("./src/reconnecting-websocket.litcoffee")
};

// avoid overwriting of something that may be there just trying to be nice...
window.ReconnectingWebSocket = window.ReconnectingWebSocket || clients.ReconnectingWebSocket;
window.AwesomeWebSocket = window.AwesomeWebSocket || clients.AwesomeWebSocket;

module.exports.AwesomeWebSocket = clients.AwesomeWebSocket
module.exports.ReconnectingWebSocket = clients.ReconnectingWebSocket

},{"./src/awesome-websocket.litcoffee":1,"./src/reconnecting-websocket.litcoffee":3}]},{},[]);
