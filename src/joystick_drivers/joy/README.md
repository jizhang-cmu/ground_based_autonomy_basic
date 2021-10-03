# ROS Driver for Generic Linux Joysticks

The joy package contains joy_node, a node that interfaces a generic Linux joystick to ROS. This node publishes a "Joy" message, which contains the current state of each one of the joystick's buttons and axes.

## Supported Hardware

This node should work with any joystick that is supported by Linux.

## Published Topics

* joy ([sensor_msgs/Joy](http://docs.ros.org/api/sensor_msgs/html/msg/Joy.html)): outputs the joystick state.

## Device Selection

There are two parameters controlling which device should be used:

* ~dev (string, default: "/dev/input/js0")
* ~dev_name (string, default: "" (empty = disabled))

If ~dev_name is empty, ~dev defines the Linux joystick device from which to read joystick events.

If ~dev_name is defined, the node enumerates all available joysticks, i.e. /dev/input/js*. The first joystick matching ~dev_name is opened. If no joystick matches the desired device name, the device specified by ~dev is used as a fallback.

To get a list of the names of all connected joysticks, an invalid ~dev_name can be specified. For example:

`rosrun joy joy_node _dev_name:="*"`

The output might look like this:

```
[ INFO]: Found joystick: ST LIS3LV02DL Accelerometer (/dev/input/js1).
[ INFO]: Found joystick: Microsoft X-Box 360 pad (/dev/input/js0).
[ERROR]: Couldn't find a joystick with name *. Falling back to default device.
```

Then the node can be started with the device name given in the list. For example:

`rosrun joy joy_node _dev_name:="Microsoft X-Box 360 pad"`

## Advanced Parameters

* ~deadzone (double, default: 0.05)
  * Amount by which the joystick has to move before it is considered to be off-center. This parameter is specified relative to an axis normalized between -1 and 1. Thus, 0.1 means that the joystick has to move 10% of the way to the edge of an axis's range before that axis will output a non-zero value. Linux does its own deadzone processing, so in many cases this value can be set to zero.

* ~autorepeat_rate (double, default: 0.0 (disabled))
  * Rate in Hz at which a joystick that has a non-changing state will resend the previously sent message.

* ~coalesce_interval (double, default: 0.001)
  * Axis events that are received within coalesce_interval (seconds) of each other are sent out in a single ROS message. Since the kernel sends each axis motion as a separate event, coalescing greatly reduces the rate at which messages are sent. This option can also be used to limit the rate of outgoing messages. Button events are always sent out immediately to avoid missing button presses.

## Further Information

For further information have a look at the [Wiki page](http://wiki.ros.org/joy).