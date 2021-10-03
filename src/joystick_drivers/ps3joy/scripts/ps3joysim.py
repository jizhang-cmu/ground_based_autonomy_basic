#!/usr/bin/env python
# Software License Agreement (BSD License)
#
#  Copyright (c) 2009, Willow Garage, Inc.
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the Willow Garage nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.

from __future__ import print_function
from bluetooth import *
import select
import fcntl
import os
import struct
import time
import sys
import traceback
import threading
import ps3joy
import socket
import signal


def mk_in_socket():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.bind(("127.0.0.1", 0))
    sock.listen(1)
    return sock, sock.getsockname()[1]


# Class to spawn the ps3joy.py infrastructure in its own thread
class driversim(threading.Thread):
    def __init__(self, intr, ctrl):
        threading.Thread.__init__(self)
        self.intr = intr
        self.ctrl = ctrl
        self.start()

    def run(self):
        self.cm = ps3joy.connection_manager(ps3joy.decoder())
        self.cm.listen(self.intr, self.ctrl)
        print("driversim exiting")

    def shutdown(self):
        self.cm.shutdown = True


class joysim(threading.Thread):
    def __init__(self, intr, ctrl):
        threading.Thread.__init__(self)
        print("Starting joystick simulator on ports", intr, "and", ctrl)
        self.intr = socket.socket()
        self.intr.connect(("127.0.0.1", intr))
        if self.intr == -1:
            raise "Error creating interrput socket."
        self.ctrl = socket.socket()
        self.ctrl.connect(("127.0.0.1", ctrl))
        if self.ctrl == -1:
            raise "Error creating control socket."
        self.active = False
        self.shutdown = False
        self.start()

    def run(self):
        while not self.active and not self.shutdown:
            (rd, wr, err) = select.select([self.ctrl], [], [], 1)
            if len(rd) == 1:
                cmd = self.ctrl.recv(128)
                if cmd == "\x53\xf4\x42\x03\x00\x00":
                    self.active = True
                    print("Got activate command")
                else:
                    print("Got unknown command (len=%i)" % len(cmd), end=' ')
                    time.sleep(1)
                    for c in cmd:
                        print("%x" % ord(c), end=' ')
                    print()
        print("joyactivate exiting")

    def publishstate(self, ax, butt):
        if self.active:
            ranges = [255] * 17 + [8191] * 20
            axval = [int((v + 1) * s / 2) for (v, s) in zip(ax, ranges)]
            buttout = []
            for i in range(0, 2):
                newval = 0
                for j in range(0, 8):
                    newval = (newval << 1)
                    if butt[i * 8 + j]:
                        newval = newval + 1
                buttout.append(newval)
            joy_coding = "!1B2x3B1x4B4x12B15x4H"
            self.intr.send(struct.pack(joy_coding, 161, *(buttout + [0] + axval)))
        else:
            print("Tried to publish while inactive")


if __name__ == "__main__":
    def stop_all_threads(a, b):
        exit(0)

    signal.signal(signal.SIGINT, stop_all_threads)

    # Create sockets for the driver side and pass them to the driver
    (intr_in, intr_port) = mk_in_socket()
    (ctrl_in, ctrl_port) = mk_in_socket()

    ds = driversim(intr_in, ctrl_in)

    # Give the simulator a chance to get going
    time.sleep(2)

    # Call up the simulator telling it which ports to connect to.
    js = joysim(intr_port, ctrl_port)
    buttons1 = [True] * 16
    axes1 = [1, 0, -1, .5] * 5
    buttons2 = [False] * 16
    axes2 = [-1] * 20
    buttons3 = [False] * 16
    axes3 = [0] * 20
    shutdown = False
    while not js.active and not shutdown:
        time.sleep(0.2)
    time.sleep(0.01)
    time.sleep(0.01)
    while not shutdown:
        js.publishstate(axes1, buttons2)
        time.sleep(0.01)
        axes1[0] = -axes1[0]
        js.publishstate(axes2, buttons2)
        time.sleep(0.01)

    print("main exiting")
