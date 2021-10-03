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
import roslib
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from bluetooth import *
import select
import struct
import fcntl
import os
import time
import sys
import traceback
import subprocess
from array import array
import sensor_msgs.msg
import rosgraph.masterapi

roslib.load_manifest('ps3joy')

L2CAP_PSM_HIDP_CTRL = 17
L2CAP_PSM_HIDP_INTR = 19
inactivity_timout_string = "--inactivity-timeout"
no_disable_bluetoothd_string = "--no-disable-bluetoothd"
redirect_output_string = "--redirect-output"


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
                print("Can't open uinput device. Is it accessible by this user? Did you mean to run as root?",
                      file=sys.stderr)
                raise IOError

        UI_SET_EVBIT = 0x40045564
        UI_SET_KEYBIT = 0x40045565
        UI_SET_RELBIT = 0x40045566
        UI_DEV_CREATE = 0x5501
        UI_SET_RELBIT = 0x40045566
        UI_SET_ABSBIT = 0x40045567
        uinput_user_dev = "80sHHHHi" + (uinput.ABS_MAX + 1) * 4 * 'i'

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

        os.write(self.file, struct.pack(uinput_user_dev, "Sony Playstation SixAxis/DS3",
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
            print("Unexpected length for value in update (%i instead of %i). This is a bug."
                  % (len(value), len(self.value)), file=sys.stderr)
        for i in range(0, len(value)):
            if value[i] != self.value[i]:
                os.write(self.file, struct.pack(input_event, th, tl, self.type[i], self.code[i], value[i]))
        self.value = list(value)


class BadJoystickException(Exception):
    def __init__(self):
        Exception.__init__(self, "Unsupported joystick.")


class decoder:
    def __init__(self, deamon, inactivity_timeout=float(1e3000)):
        # buttons=[uinput.BTN_SELECT, uinput.BTN_THUMBL, uinput.BTN_THUMBR, uinput.BTN_START,
        #          uinput.BTN_FORWARD, uinput.BTN_RIGHT, uinput.BTN_BACK, uinput.BTN_LEFT,
        #          uinput.BTN_TL, uinput.BTN_TR, uinput.BTN_TL2, uinput.BTN_TR2,
        #          uinput.BTN_X, uinput.BTN_A, uinput.BTN_B, uinput.BTN_Y,
        #          uinput.BTN_MODE]
        # axes=[uinput.ABS_X, uinput.ABS_Y, uinput.ABS_Z, uinput.ABS_RX,
        #       uinput.ABS_RX, uinput.ABS_RY, uinput.ABS_PRESSURE, uinput.ABS_DISTANCE,
        #       uinput.ABS_THROTTLE, uinput.ABS_RUDDER, uinput.ABS_WHEEL, uinput.ABS_GAS,
        #       uinput.ABS_HAT0Y, uinput.ABS_HAT1Y, uinput.ABS_HAT2Y, uinput.ABS_HAT3Y,
        #       uinput.ABS_TILT_X, uinput.ABS_TILT_Y, uinput.ABS_MISC, uinput.ABS_RZ]
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
        for i in range(4, len(axmin) - 4):  # Buttons should be zero when not pressed
            axmin[i] = -axmax[i]
        self.joy = uinputjoy(buttons, axes, axmin, axmax, axfuzz, axflat)
        self.axmid = [sum(pair) / 2 for pair in zip(axmin, axmax)]
        self.fullstop()  # Probably useless because of uinput startup bug
        self.outlen = len(buttons) + len(axes)
        self.inactivity_timeout = inactivity_timeout
        self.deamon = deamon
        self.init_ros()
    step_active = 1
    step_idle = 2
    step_error = 3

    def init_ros(self):
        rospy.init_node('ps3joy', anonymous=True, disable_signals=True)
        rospy.Subscriber("joy/set_feedback", sensor_msgs.msg.JoyFeedbackArray, self.set_feedback)
        self.diagnostics = Diagnostics()
        self.led_values = [1, 0, 0, 0]
        self.rumble_cmd = [0, 255]
        self.led_cmd = 2
        self.core_down = False

    # ********************************************************************************
    # Raw Data Format
    # unsigned char ReportType;         //Report Type 01
    # unsigned char Reserved1;          // Unknown
    # unsigned int  ButtonState;        // Main buttons
    # unsigned char PSButtonState;      // PS button
    # unsigned char Reserved2;          // Unknown
    # unsigned char LeftStickX;         // left Joystick X axis 0 - 255, 128 is mid
    # unsigned char LeftStickY;         // left Joystick Y axis 0 - 255, 128 is mid
    # unsigned char RightStickX;        // right Joystick X axis 0 - 255, 128 is mid
    # unsigned char RightStickY;        // right Joystick Y axis 0 - 255, 128 is mid
    # unsigned char Reserved3[4];       // Unknown
    # unsigned char PressureUp;         // digital Pad Up button Pressure 0 - 255
    # unsigned char PressureRight;      // digital Pad Right button Pressure 0 - 255
    # unsigned char PressureDown;       // digital Pad Down button Pressure 0 - 255
    # unsigned char PressureLeft;       // digital Pad Left button Pressure 0 - 255
    # unsigned char PressureL2;         // digital Pad L2 button Pressure 0 - 255
    # unsigned char PressureR2;         // digital Pad R2 button Pressure 0 - 255
    # unsigned char PressureL1;         // digital Pad L1 button Pressure 0 - 255
    # unsigned char PressureR1;         // digital Pad R1 button Pressure 0 - 255
    # unsigned char PressureTriangle;   // digital Pad Triangle button Pressure 0 - 255
    # unsigned char PressureCircle;     // digital Pad Circle button Pressure 0 - 255
    # unsigned char PressureCross;      // digital Pad Cross button Pressure 0 - 255
    # unsigned char PressureSquare;     // digital Pad Square button Pressure 0 - 255
    # unsigned char Reserved4[3];       // Unknown
    # unsigned char Charge;             // charging status ? 02 = charge, 03 = normal
    # unsigned char Power;              // Battery status
    # unsigned char Connection;         // Connection Type
    # unsigned char Reserved5[9];       // Unknown
    # unsigned int AccelerometerX;      // X axis accelerometer Big Endian 0 - 1023
    # unsigned int Accelero             // Y axis accelerometer Big Endian 0 - 1023
    # unsigned int AccelerometerZ;      // Z axis accelerometer Big Endian 0 - 1023
    # unsigned int GyrometerX;          // Z axis Gyro Big Endian 0 - 1023
    # *********************************************************************************
    def step(self, rawdata):  # Returns true if the packet was legal
        if len(rawdata) == 50:
            joy_coding = "!1B2x3B1x4B4x12B3x1B1B1B9x4H"
            all_data = list(struct.unpack(joy_coding, rawdata))  # removing power data
            state_data = all_data[20:23]
            data = all_data[0:20]+all_data[23:]
            prefix = data.pop(0)
            self.diagnostics.publish(state_data)
            if prefix != 161:
                print("Unexpected prefix (%i). Is this a PS3 Dual Shock or Six Axis?" % prefix,
                      file=sys.stderr)
                return self.step_error
            out = []
            for j in range(0, 2):  # Split out the buttons.
                curbyte = data.pop(0)
                for k in range(0, 8):
                    out.append(int((curbyte & (1 << k)) != 0))
            out = out + data
            self.joy.update(out)
            axis_motion = [
                abs(out[17:][i] - self.axmid[i]) > 20 for i in range(0, len(out) - 17 - 4)
            ]  # 17 buttons, 4 inertial sensors

            if any(out[0:17]) or any(axis_motion):
                return self.step_active
            return self.step_idle
        elif len(rawdata) == 13:
            print("Your bluetooth adapter is not supported. Does it support Bluetooth 2.0?",
                  file=sys.stderr)
            raise BadJoystickException()
        else:
            print("Unexpected packet length (%i). Is this a PS3 Dual Shock or Six Axis?"
                  % len(rawdata), file=sys.stderr)
            return self.step_error

    def fullstop(self):
        self.joy.update([0] * 17 + self.axmid)

    def set_feedback(self, msg):
        for feedback in msg.array:
            if feedback.type == sensor_msgs.msg.JoyFeedback.TYPE_LED and feedback.id < 4:
                self.led_values[feedback.id] = int(round(feedback.intensity))
            elif feedback.type == sensor_msgs.msg.JoyFeedback.TYPE_RUMBLE and feedback.id < 2:
                self.rumble_cmd[feedback.id] = int(feedback.intensity*255)
            else:
                rospy.logwarn("Feedback %s of type %s does not exist for this joystick.", feedback.id, feedback.type)
        self.led_cmd = self.led_values[0] * pow(2, 1) + self.led_values[1] * pow(2, 2)
        self.led_cmd = self.led_cmd + self.led_values[2] * pow(2, 3) + self.led_values[3] * pow(2, 4)
        self.new_msg = True

    def send_cmd(self, ctrl):
        command = [0x52,
                   0x01,
                   0x00, 0xfe, self.rumble_cmd[1], 0xfe, self.rumble_cmd[0],        # rumble values
                   0x00, 0x00, 0x00, 0x00, self.led_cmd,
                   0xff, 0x27, 0x10, 0x00, 0x32,        # LED 4
                   0xff, 0x27, 0x10, 0x00, 0x32,        # LED 3
                   0xff, 0x27, 0x10, 0x00, 0x32,        # LED 2
                   0xff, 0x27, 0x10, 0x00, 0x32,        # LED 1
                   0x00, 0x00, 0x00, 0x00, 0x00
                   ]
        ctrl.send(array('B', command).tostring())
        self.new_msg = False

    def run(self, intr, ctrl):
        activated = False
        try:
            self.fullstop()
            lastactivitytime = lastvalidtime = time.time()
            while not rospy.is_shutdown():
                (rd, wr, err) = select.select([intr], [], [], 0.1)
                curtime = time.time()
                if len(rd) + len(wr) + len(err) == 0:  # Timeout
                    ctrl.send("\x53\xf4\x42\x03\x00\x00")  # Try activating the stream.
                else:  # Got a frame.
                    if not activated:
                        self.send_cmd(ctrl)
                        time.sleep(0.5)
                        self.rumble_cmd[1] = 0
                        self.send_cmd(ctrl)
                        print("Connection activated")
                        activated = True
                    try:
                        if(self.new_msg):
                            self.send_cmd(ctrl)
                        rawdata = intr.recv(128)
                    except BluetoothError as s:
                        print("Got Bluetooth error %s. Disconnecting." % s)
                        return
                    if len(rawdata) == 0:  # Orderly shutdown of socket
                        print("Joystick shut down the connection, battery may be discharged.")
                        return
                    if not rosgraph.masterapi.is_online():
                        print("The roscore or node shutdown, ps3joy shutting down.")
                        return

                    stepout = self.step(rawdata)
                    if stepout != self.step_error:
                        lastvalidtime = curtime
                    if stepout == self.step_active:
                        lastactivitytime = curtime
                if curtime - lastactivitytime > self.inactivity_timeout:
                    print("Joystick inactive for %.0f seconds. Disconnecting to save "
                          "battery." % self.inactivity_timeout)
                    return
                if curtime - lastvalidtime >= 0.1:
                    # Zero all outputs if we don't hear a valid frame for 0.1 to 0.2 seconds
                    self.fullstop()
                if curtime - lastvalidtime >= 5:
                    # Disconnect if we don't hear a valid frame for 5 seconds
                    print("No valid data for 5 seconds. Disconnecting. This should not happen, please report it.")
                    return
                time.sleep(0.005)  # No need to blaze through the loop when there is an error
        finally:
            self.fullstop()


class Diagnostics():
    def __init__(self):
        self.STATE_TEXTS_CHARGING = {
                                0: "Charging",
                                1: "Not Charging"}
        self.STATE_TEXTS_CONNECTION = {
                                18: "USB Connection",
                                20: "Rumbling",
                                22: "Bluetooth Connection"}
        self.STATE_TEXTS_BATTERY = {
                                0: "No Charge",
                                1: "20% Charge",
                                2: "40% Charge",
                                3: "60% Charge",
                                4: "80% Charge",
                                5: "100% Charge",
                                238: "Charging"}
        self.diag_pub = rospy.Publisher('/diagnostics', DiagnosticArray)
        self.last_diagnostics_time = rospy.get_rostime()

    def publish(self, state):
        STATE_INDEX_CHARGING = 0
        STATE_INDEX_BATTERY = 1
        STATE_INDEX_CONNECTION = 2

        # timed gate: limit to 1 Hz
        curr_time = rospy.get_rostime()
        if (curr_time - self.last_diagnostics_time).to_sec() < 1.0:
            return
        self.last_diagnostics_time = curr_time

        # compose diagnostics message
        diag = DiagnosticArray()
        diag.header.stamp = curr_time
        # battery info
        stat = DiagnosticStatus(name="Battery", level=DiagnosticStatus.OK, message="OK")
        try:
            battery_state_code = state[STATE_INDEX_BATTERY]
            stat.message = self.STATE_TEXTS_BATTERY[battery_state_code]
            if battery_state_code < 3:
                stat.level = DiagnosticStatus.WARN
                if battery_state_code < 1:
                    stat.level = DiagnosticStatus.ERROR
                stat.message = "Please Recharge Battery (%s)." % self.STATE_TEXTS_BATTERY[battery_state_code]
        except KeyError as ex:
            stat.message = "Invalid Battery State %s" % ex
            rospy.logwarn("Invalid Battery State %s" % ex)
            stat.level = DiagnosticStatus.ERROR
        diag.status.append(stat)
        # connection info
        stat = DiagnosticStatus(name='ps3joy'": Connection Type", level=DiagnosticStatus.OK, message="OK")
        try:
            stat.message = self.STATE_TEXTS_CONNECTION[state[STATE_INDEX_CONNECTION]]
        except KeyError as ex:
            stat.message = "Invalid Connection State %s" % ex
            rospy.logwarn("Invalid Connection State %s" % ex)
            stat.level = DiagnosticStatus.ERROR
        diag.status.append(stat)
        # charging info
        stat = DiagnosticStatus(name='ps3joy'": Charging State", level=DiagnosticStatus.OK, message="OK")
        try:
            stat.message = self.STATE_TEXTS_CHARGING[state[STATE_INDEX_CHARGING]]
        except KeyError as ex:
            stat.message = "Invalid Charging State %s" % ex
            rospy.logwarn("Invalid Charging State %s" % ex)
            stat.level = DiagnosticStatus.ERROR
        diag.status.append(stat)
        # publish message
        self.diag_pub.publish(diag)


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
                          "Do you have another ps3joy.py running? This error occurs "
                          "on some distributions. Please read "
                          "http://www.ros.org/wiki/ps3joy/Troubleshooting for solutions.", file=sys.stderr)
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
        while not rospy.is_shutdown():
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
                        print("Got interrupt connection without control connection. Giving up on it.",
                              file=sys.stderr)
                        continue
                    try:
                        if idev == cdev:
                            self.decoder.run(intr, ctrl)
                            print("Connection terminated.")
                            quit(0)
                        else:
                            print("Simultaneous connection from two different devices. Ignoring both.",
                                  file=sys.stderr)
                    finally:
                        ctrl.close()
                finally:
                    intr.close()
            except BadJoystickException:
                pass
            except KeyboardInterrupt:
                print("\nCTRL+C detected. Exiting.")
                rospy.signal_shutdown("\nCTRL+C detected. Exiting.")
                quit(0)
            except Exception as e:
                traceback.print_exc()
                print("Caught exception: %s" % str(e), file=sys.stderr)
                time.sleep(1)


def usage(errcode):
    print("usage: ps3joy.py [" + inactivity_timout_string + "=<n>] [" + no_disable_bluetoothd_string + "] "
          "[" + redirect_output_string + "]=<f>")
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
        inactivity_timeout = float(1e3000)
        disable_bluetoothd = True
        deamon = False
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
                    print("Error parsing inactivity timeout: "+str_value)
                    print()
                    usage(1)
            elif arg == no_disable_bluetoothd_string:
                disable_bluetoothd = False
            elif is_arg_with_param(arg, redirect_output_string):
                str_value = arg[len(redirect_output_string)+1:]
                try:
                    print("Redirecting output to:", str_value)
                    sys.stdout = open(str_value, "a", 1)
                except IOError as e:
                    print("Error opening file to redirect output:", str_value)
                    raise Quit(1)
                sys.stderr = sys.stdout
            else:
                print("Ignoring parameter: '%s'" % arg)

        # If the user does not have HW permissions indicate that ps3joy must be run as root
        if os.getuid() != 0:
            print("ps3joy.py must be run as root.", file=sys.stderr)
            quit(1)
        if disable_bluetoothd:
            os.system("/etc/init.d/bluetooth stop > /dev/null 2>&1")
            time.sleep(1)  # Give the socket time to be available.
        try:
            while os.system("hciconfig hci0 > /dev/null 2>&1") != 0:
                print("No bluetooth dongle found or bluez rosdep not installed. "
                      "Will retry in 5 seconds.", file=sys.stderr)
                time.sleep(5)
            if inactivity_timeout == float(1e3000):
                print("No inactivity timeout was set. (Run with --help for details.)")
            else:
                print("Inactivity timeout set to %.0f seconds." % inactivity_timeout)
            cm = connection_manager(decoder(deamon, inactivity_timeout=inactivity_timeout))
            cm.listen_bluetooth()
        finally:
            if disable_bluetoothd:
                os.system("/etc/init.d/bluetooth start > /dev/null 2>&1")
    except Quit as e:
        errorcode = e.errorcode
    except KeyboardInterrupt:
        print("\nCTRL+C detected. Exiting.")
        rospy.signal_shutdown("\nCTRL+C detected. Exiting.")
    exit(errorcode)
