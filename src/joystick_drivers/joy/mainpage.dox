/**

\mainpage
\htmlinclude manifest.html

\b joy ROS joystick drivers for Linux. This package works with Logitech joysticks and PS3 SIXAXIS/DUALSHOCK devices. This package will only build is linux/joystick.h

\section rosapi ROS API

This package contains the message "Joy", which carries data from the joystick's axes and buttons. It also has the joy_node, which reads from a given joystick port and publishes "Joy" messages.

List of nodes:
- \b joy_node

Deprecated nodes:
- \b txjoy 
- \b joy 
The deprecated nodes are the same as the joy_node, but they will warn users when used. They are actually just bash scripts that call "joy_node" and print a warning. 

<hr>

\subsection joy joy

\b joy ROS joystick driver for all linux joysticks. The driver will poll a given port until it can read from it, the publish Joy messages of the joystick state. If the port closes, or it reads an error, it will reopen the port. All axes are in the range [-1, 1], and all buttons are 0 (off) or 1 (on). 

Since the driver will poll for the joystick port, and automatically reopen the port if it's closed, the joy_node should be "on" whenever possible. It is typically part of the robot's launch file.

\subsubsection autorepeat Auto-Repeat/Signal Loss

The joy_node takes an "~autorepeat_rate" parameter. If the linux kernal receives no events during the autorepeat interval, it will automatically repeat the last value of the joystick. This is an important safety feature, and allows users to recover from a joystick that has timed out.

\subsubsection usage Usage
\verbatim
$ joy [standard ROS args]
\endverbatim

\subsubsection topic ROS topics

Subscribes to (name / type):
- None

Publishes to (name / type):
- \b "joy/Joy" : Joystick output. Axes are [-1, 1], buttons are 0 or 1 (depressed).

\subsubsection parameters ROS parameters
- \b "~dev" : Input device for joystick. Default: /dev/input/js0
- \b "~deadzone" : Output is zero for axis in deadzone. Range: [-0.9, 0.9]. Default 0.05
- \b "~autorepeat_rate" : If no events, repeats last known state at this rate. Defualt: 0 (disabled)
- \b "~coalesce_interval" : Waits for this interval (seconds) after receiving an event. If multiple events happen in this interval, only one message will be sent. Reduces number of messages. Default: 0.001.


<hr>



*/
