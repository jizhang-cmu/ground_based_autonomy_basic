 # Testing procedures for the ps3joy package # 

## Pairing the ps3 controller via bluetooth ##
If the controller is not paired to the bluetooth dongle connect 
the controller via usb 

Enter the following commands: 

``` 
sudo bash 
rosrun ps3joy sixpair
```
The current bluetooth master address and setting master address should the same.

``` 
Current Bluetooth master: 00:15:83:ed:3f:21
Setting master bd_addr to 00:15:83:ed:3f:21
``` 

## Running the ps3joy nodes ## 
The package should consists of the following nodes:  
* ps3joy.py 
* ps3joy_node.py 
 

Running ps3joy_node.py will **require** being root to run if the user does not have 
permissions to the hardware. 

Enter the following commands in the terminal:
 
```
sudo bash 
rosrun ps3joy ps3joy_node.py
``` 
The following message should be diplayed aftwards: 

```
Waiting Connection. Disconnect your PS3 joystick from USB and press the pairing button.
```
 
If your joystick does not pair, open a new terminal and restart bluez by 
entering the following command: 

```
sudo systemctl restart bluetooth 
``` 
The terminal where ps3joy_node.py was launched should display the following message: 
``` 
Connection is Activated. 
```
## ps3joy Diagnostics test ## 
Open a new terminal and enter the following following command: 
```
rostopic list
``` 
You should see the following: 
``` 
/diagnostics
/joy/set_feedback
/rosout
/rosout_agg
```
Enter the following command to diagnose the current state of the joystick
```  
rostopic echo /diagnostics
```
You will see the charging state, battery percentage and, connection type in your terminal:
``` 
header: 
  seq: 1
  stamp: 
    secs: 1498667204
    nsecs: 603754043
  frame_id: ''
status: 
  - 
    level: 0
    name: Battery
    message: 100% Charge
    hardware_id: ''
    values: []
  - 
    level: 0
    name: Connection
    message: Bluetooth Connection
    hardware_id: ''
    values: []
  - 
    level: 0
    name: Charging State
    message: Not Charging
    hardware_id: ''
    values: []
```

If you plug in the usb cable both the connection type and charging state will change: 
```
header: 
  seq: 8
  stamp: 
    secs: 1498667507
    nsecs: 798973083
  frame_id: ''
status: 
  - 
    level: 0
    name: Battery
    message: Charging
    hardware_id: ''
    values: []
  - 
    level: 0
    name: Connection
    message: USB Connection
    hardware_id: ''
    values: []
  - 
    level: 0
    name: Charging State
    message: Charging
    hardware_id: ''
    values: []
```

## Confirming the ps3 joystick input ## 
Check to see if your joystick is recgonized by your computer.   

``` 
ls /dev/input/ 
``` 
Tell the ps3joy node which device is the ps3 joystick 
``` 
rosparam set joy_node/dev "dev/input/jsX"
``` 
X would be the number your device was given.
you can now start the joy node: 

``` 
rosrun joy joy_node 
```
You should see something that looks like this: 
```
[ INFO] [1498673536.447541090]: Opened joystick: /dev/input/js0. deadzone_: 0.050000.
```
Open a new terminal and use rostopic to observe the data from the controller. 
```
rostopic echo joy 
``` 
You should see the input data dipslayed on your terminal. 

## Recharging the PS3 joystick 
1. Have an available USB port on a computer, and the computer must be on while the joystick is
   charging. 

2. Connect the PS3 joystick to a computer using an A to mini-B USB cable. 

3. The LEDs on the joystick should blink at about 1Hz to indicate the the joystick is charging.

## Shutting down the ps3 joystick
There are two ways to turn of the ps3 joystick   
1. Press and hold the pairing button on the joystick for approximately 10 seconds
 
2. You can also also shut the controller down by killing the process of ps3joy_node.py
   press Ctrl-c on your keyboard to kill the processes and the joystick will turn off 
   as well. 
