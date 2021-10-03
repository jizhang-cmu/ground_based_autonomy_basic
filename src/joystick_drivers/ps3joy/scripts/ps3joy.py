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
import time
import sys
import traceback
import subprocess

L2CAP_PSM_HIDP_CTRL = 17
L2CAP_PSM_HIDP_INTR = 19
inactivity_timout_string = "--inactivity-timeout"
no_disable_bluetoothd_string = "--no-disable-bluetoothd"
redirect_output_string = "--redirect-output"
continuous_motion_output_string = "--continuous-output"


class uinput:
    EV_KEY = 1
    EV_REL = 2
    EV_ABS = 3
    BUS_USB = 3
    ABS_MAX = 0x3f


class uinputjoy:
    def open_uinput(self):
        for name in ["/dev/input/uinput", "/dev/misc/uinput", "/dev/uinput"]:
            try:
                return os.open(name, os.O_WRONLY)
                break
            except Exception as e:
                pass
        return None

    def __init__(self, buttons, axes, axmin, axmax, axfuzz, axflat):
        self.file = self.open_uinput()
        if self.file is None:
            print("Trying to modprobe uinput.", file=sys.stderr)
            os.system("modprobe uinput > /dev/null 2>&1")
            time.sleep(1)  # uinput isn't ready to go right away.
            self.file = self.open_uinput()
            if self.file is None:
                print("Can't open uinput device. Is it accessible by this user? "
                      "Did you mean to run as root?", file=sys.stderr)
                raise IOError

        UI_SET_EVBIT = 0x40045564
        UI_SET_KEYBIT = 0x40045565
        UI_SET_RELBIT = 0x40045566
        UI_DEV_CREATE = 0x5501
        UI_SET_RELBIT = 0x40045566
        UI_SET_ABSBIT = 0x40045567
        uinput_user_dev = "80sHHHHi" + (uinput.ABS_MAX+1)*4*'i'

        if len(axes) != len(axmin) or len(axes) != len(axmax):
            raise Exception("uinputjoy.__init__: axes, axmin and axmax should have same length")

        absmin = [0] * (uinput.ABS_MAX+1)
        absmax = [0] * (uinput.ABS_MAX+1)
        absfuzz = [2] * (uinput.ABS_MAX+1)
        absflat = [4] * (uinput.ABS_MAX+1)
        for i in range(0, len(axes)):
            absmin[axes[i]] = axmin[i]
            absmax[axes[i]] = axmax[i]
            absfuzz[axes[i]] = axfuzz[i]
            absflat[axes[i]] = axflat[i]

        os.write(self.file,
                 struct.pack(uinput_user_dev, "Sony Playstation SixAxis/DS3",
                             uinput.BUS_USB, 0x054C, 0x0268, 0, 0, *(absmax + absmin + absfuzz + absflat)))

        fcntl.ioctl(self.file, UI_SET_EVBIT, uinput.EV_KEY)

        for b in buttons:
            fcntl.ioctl(self.file, UI_SET_KEYBIT, b)

        for a in axes:
            fcntl.ioctl(self.file, UI_SET_EVBIT, uinput.EV_ABS)
            fcntl.ioctl(self.file, UI_SET_ABSBIT, a)

        fcntl.ioctl(self.file, UI_DEV_CREATE)

        self.value = [None] * (len(buttons) + len(axes))
        self.type = [uinput.EV_KEY] * len(buttons) + [uinput.EV_ABS] * len(axes)
        self.code = buttons + axes

    def update(self, value):
        input_event = "LLHHi"
        t = time.time()
        th = int(t)
        tl = int((t - th) * 1000000)
        if len(value) != len(self.value):
            print("Unexpected length for value in update (%i instead of %i). "
                  "This is a bug." % (len(value), len(self.value)), file=sys.stderr)
        for i in range(0, len(value)):
            if value[i] != self.value[i]:
                os.write(self.file, struct.pack(input_event, th, tl, self.type[i], self.code[i], value[i]))
        self.value = list(value)


class BadJoystickException(Exception):
    def __init__(self):
        Exception.__init__(self, "Unsupported joystick.")


class decoder:
    def __init__(self, inactivity_timeout=float(1e3000), continuous_motion_output=False):
        # buttons=[uinput.BTN_SELECT, uinput.BTN_THUMBL, uinput.BTN_THUMBR, uinput.BTN_START,
        #         uinput.BTN_FORWARD, uinput.BTN_RIGHT, uinput.BTN_BACK, uinput.BTN_LEFT,
        #         uinput.BTN_TL, uinput.BTN_TR, uinput.BTN_TL2, uinput.BTN_TR2,
        #         uinput.BTN_X, uinput.BTN_A, uinput.BTN_B, uinput.BTN_Y,
        #         uinput.BTN_MODE]
        # axes=[uinput.ABS_X, uinput.ABS_Y, uinput.ABS_Z, uinput.ABS_RX,
        #       uinput.ABS_RX, uinput.ABS_RY, uinput.ABS_PRESSURE, uinput.ABS_DISTANCE,
        #       uinput.ABS_THROTTLE, uinput.ABS_RUDDER, uinput.ABS_WHEEL, uinput.ABS_GAS,
        #       uinput.ABS_HAT0Y, uinput.ABS_HAT1Y, uinput.ABS_HAT2Y, uinput.ABS_HAT3Y,
        #       uinput.ABS_TILT_X, uinput.ABS_TILT_Y, uinput.ABS_MISC, uinput.ABS_RZ,
        #       ]
        buttons = range(0x100, 0x111)
        axes = range(0, 20)
        axmin = [0] * 20
        axmax = [255] * 20
        axfuzz = [2] * 20
        axflat = [4] * 20
        for i in range(-4, 0):  # Gyros have more bits than other axes
            axmax[i] = 1023
            axfuzz[i] = 4
            axflat[i] = 4
            if continuous_motion_output:
                axfuzz[i] = 0
                axflat[i] = 0
        for i in range(4, len(axmin)-4):  # Buttons should be zero when not pressed
            axmin[i] = -axmax[i]
        self.joy = uinputjoy(buttons, axes, axmin, axmax, axfuzz, axflat)
        self.axmid = [sum(pair)/2 for pair in zip(axmin, axmax)]
        self.fullstop()  # Probably useless because of uinput startup bug
        self.outlen = len(buttons) + len(axes)
        self.inactivity_timeout = inactivity_timeout

    step_active = 1
    step_idle = 2
    step_error = 3

    def step(self, rawdata):  # Returns true if the packet was legal
        if len(rawdata) == 50:
            joy_coding = "!1B2x3B1x4B4x12B15x4H"
            data = list(struct.unpack(joy_coding, rawdata))
            prefix = data.pop(0)
            if prefix != 161:
                print("Unexpected prefix (%i). Is this a PS3 Dual Shock or Six Axis?" % prefix, file=sys.stderr)
                return self.step_error
            out = []
            for j in range(0, 2):  # Split out the buttons.
                curbyte = data.pop(0)
                for k in range(0, 8):
                    out.append(int((curbyte & (1 << k)) != 0))
            out = out + data
            self.joy.update(out)
            axis_motion = [
                abs(out[17:][i] - self.axmid[i]) > 20 for i in range(0, len(out)-17-4)
            ]  # 17 buttons, 4 inertial sensors
            if any(out[0:17]) or any(axis_motion):
                return self.step_active
            return self.step_idle
        elif len(rawdata) == 13:
            print("Your bluetooth adapter is not supported. "
                  "Does it support Bluetooth 2.0?", file=sys.stderr)
            raise BadJoystickException()
        else:
            print("Unexpected packet length (%i). "
                  "Is this a PS3 Dual Shock or Six Axis?" % len(rawdata), file=sys.stderr)
            return self.step_error

    def fullstop(self):
        self.joy.update([0] * 17 + self.axmid)

    def run(self, intr, ctrl):
        activated = False
        try:
            self.fullstop()
            lastactivitytime = lastvalidtime = time.time()
            while True:
                (rd, wr, err) = select.select([intr], [], [], 0.1)
                curtime = time.time()
                if len(rd) + len(wr) + len(err) == 0:  # Timeout
                    ctrl.send("\x53\xf4\x42\x03\x00\x00")  # Try activating the stream.
                else:  # Got a frame.
                    if not activated:
                        print("Connection activated")
                        activated = True
                    try:
                        rawdata = intr.recv(128)
                    except BluetoothError as s:
                        print("Got Bluetooth error %s. Disconnecting." % s)
                        return
                    if len(rawdata) == 0:  # Orderly shutdown of socket
                        print("Joystick shut down the connection, battery may be discharged.")
                        return
                    stepout = self.step(rawdata)
                    if stepout != self.step_error:
                        lastvalidtime = curtime
                    if stepout == self.step_active:
                        lastactivitytime = curtime
                if curtime - lastactivitytime > self.inactivity_timeout:
                    print("Joystick inactive for %.0f seconds. "
                          "Disconnecting to save battery." % self.inactivity_timeout)
                    return
                if curtime - lastvalidtime >= 0.1:
                    # Zero all outputs if we don't hear a valid frame for 0.1 to 0.2 seconds
                    self.fullstop()
                if curtime - lastvalidtime >= 5:
                    # Disconnect if we don't hear a valid frame for 5 seconds
                    print("No valid data for 5 seconds. Disconnecting. "
                          "This should not happen, please report it.")
                    return
                time.sleep(0.005)  # No need to blaze through the loop when there is an error
        finally:
            self.fullstop()


class Quit(Exception):
    def __init__(self, errorcode):
        Exception.__init__(self)
        self.errorcode = errorcode


def check_hci_status():
    # Check if hci0 is up and pscanning, take action as necessary.
    proc = subprocess.Popen(['hciconfig'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    (out, err) = proc.communicate()
    if out.find('UP') == -1:
        os.system("hciconfig hci0 up > /dev/null 2>&1")
    if out.find('PSCAN') == -1:
        os.system("hciconfig hci0 pscan > /dev/null 2>&1")


class connection_manager:
    def __init__(self, decoder):
        self.decoder = decoder
        self.shutdown = False

    def prepare_bluetooth_socket(self, port):
        sock = BluetoothSocket(L2CAP)
        return self.prepare_socket(sock, port)

    def prepare_net_socket(self, port):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        return self.prepare_socket(sock, port)

    def prepare_socket(self, sock, port):
        first_loop = True
        while True:
            try:
                sock.bind(("", port))
            except Exception as e:
                print(repr(e))
                if first_loop:
                    print("Error binding to socket, will retry every 5 seconds. "
                          "Do you have another ps3joy.py running? This error occurs on "
                          "some distributions (such as Ubuntu Karmic). "
                          "Please read http://www.ros.org/wiki/ps3joy/Troubleshooting for solutions.",
                          file=sys.stderr)
                first_loop = False
                time.sleep(0.5)
                continue
            sock.listen(1)
            return sock

    def listen_net(self, intr_port, ctrl_port):
        intr_sock = self.prepare_net_socket(intr_port)
        ctrl_sock = self.prepare_net_socket(ctrl_port)
        self.listen(intr_sock, ctrl_sock)

    def listen_bluetooth(self):
        intr_sock = self.prepare_bluetooth_socket(L2CAP_PSM_HIDP_INTR)
        ctrl_sock = self.prepare_bluetooth_socket(L2CAP_PSM_HIDP_CTRL)
        self.listen(intr_sock, ctrl_sock)

    def listen(self, intr_sock, ctrl_sock):
        self.n = 0
        while not self.shutdown:
            print("Waiting for connection. Disconnect your PS3 joystick from USB and press the pairing button.")
            try:
                intr_sock.settimeout(5)
                ctrl_sock.settimeout(1)
                while True:
                    try:
                        (intr, (idev, iport)) = intr_sock.accept()
                        break
                    except Exception as e:
                        if str(e) == 'timed out':
                            check_hci_status()
                        else:
                            raise

                try:
                    try:
                        (ctrl, (cdev, cport)) = ctrl_sock.accept()
                    except Exception as e:
                        print("Got interrupt connection without control connection. Giving up on it.", file=sys.stderr)
                        continue
                    try:
                        if idev == cdev:
                            self.decoder.run(intr, ctrl)
                            print("Connection terminated.")
                        else:
                            print("Simultaneous connection from two different devices. Ignoring both.", file=sys.stderr)
                    finally:
                        ctrl.close()
                finally:
                    intr.close()
            except BadJoystickException:
                pass
            except KeyboardInterrupt:
                print("CTRL+C detected. Exiting.")
                quit(0)
            except Exception as e:
                traceback.print_exc()
                print("Caught exception: %s" % str(e), file=sys.stderr)
                time.sleep(1)
            print()


def usage(errcode):
    print("usage: ps3joy.py [" + inactivity_timout_string + "=<n>] [" + no_disable_bluetoothd_string + "] "
          "[" + redirect_output_string + "] [" + continuous_motion_output_string + "]=<f>")
    print("<n>: inactivity timeout in seconds (saves battery life).")
    print("<f>: file name to redirect output to.")
    print("Unless "+no_disable_bluetoothd_string+" is specified, bluetoothd will be stopped.")
    raise Quit(errcode)


def is_arg_with_param(arg, prefix):
    if not arg.startswith(prefix):
        return False
    if not arg.startswith(prefix+"="):
        print("Expected '=' after "+prefix)
        print()
        usage(1)
    return True


if __name__ == "__main__":
    errorcode = 0
    try:
        # Get Root Privileges
        euid = os.geteuid()
        if euid != 0:
            args = ['sudo', sys.executable] + sys.argv + [os.environ]
            os.execlpe('sudo', *args)
        if euid != 0:
            raise SystemExit("Root Privlages Required.")

        inactivity_timeout = float(1e3000)
        disable_bluetoothd = True
        continuous_output = False

        for arg in sys.argv[1:]:  # Be very tolerant in case we are roslaunched.
            if arg == "--help":
                usage(0)
            elif is_arg_with_param(arg, inactivity_timout_string):
                str_value = arg[len(inactivity_timout_string)+1:]
                try:
                    inactivity_timeout = float(str_value)
                    if inactivity_timeout < 0:
                        print("Inactivity timeout must be positive.")
                        print()
                        usage(1)
                except ValueError:
                    print("Error parsing inactivity timeout: " + str_value)
                    print()
                    usage(1)
            elif arg == no_disable_bluetoothd_string:
                disable_bluetoothd = False
            elif arg == continuous_motion_output_string:
                continuous_output = True
            elif is_arg_with_param(arg, redirect_output_string):
                str_value = arg[len(redirect_output_string) + 1:]
                try:
                    print("Redirecting output to:", str_value)
                    sys.stdout = open(str_value, "a", 1)
                except IOError as e:
                    print("Error opening file to redirect output:", str_value)
                    raise Quit(1)
                sys.stderr = sys.stdout
            else:
                print("Ignoring parameter: '%s'" % arg)

        if disable_bluetoothd:
            os.system("/etc/init.d/bluetooth stop > /dev/null 2>&1")
            time.sleep(1)  # Give the socket time to be available.
        try:
            while os.system("hciconfig hci0 > /dev/null 2>&1") != 0:
                print("No bluetooth dongle found or bluez rosdep not installed. Will retry in 5 seconds.",
                      file=sys.stderr)
                time.sleep(5)
            if inactivity_timeout == float(1e3000):
                print("No inactivity timeout was set. (Run with --help for details.)")
            else:
                print("Inactivity timeout set to %.0f seconds." % inactivity_timeout)
            cm = connection_manager(decoder(inactivity_timeout=inactivity_timeout,
                                            continuous_motion_output=continuous_output))
            cm.listen_bluetooth()
        finally:
            if disable_bluetoothd:
                os.system("/etc/init.d/bluetooth start > /dev/null 2>&1")
    except Quit as e:
        errorcode = e.errorcode
    except KeyboardInterrupt:
        print("CTRL+C detected. Exiting.")
    exit(errorcode)
