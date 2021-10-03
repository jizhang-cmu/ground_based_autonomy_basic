## Troubleshooting ##
-------------------------
#### Issues pairing the PS3 joystick ####
When pairing your joystick via bluetooth, you may recieve the following message on
your terminal:  
```
Current Bluetooth master: 00:15:83:ed:3f:21
Unable to retrieve local bd_addr from `hcitool dev`.
Please enable Bluetooth or specify an address manually.
``` 
This would indicate that your bluetooth is disabled. To enable your bluetooth, try the 
following: 

1. Check the status of your bluetooth by entering the following: 
``` 
sudo systemctl status bluetooth
``` 
You may see something like this: 

```
● bluetooth.service - Bluetooth service
   Loaded: loaded (/lib/systemd/system/bluetooth.service; disabled; vendor preset: enabled)
   Active: inactive (dead)
     Docs: man:bluetoothd(8)
```

If you do, that means your bluetooth service is disabled. Turn enable it enter 
```
sudo systemctl start bluetooth
sudo systemctl status bluetooth  
```
After running these commands your bluetooth service should be up and running: 

```
● bluetooth.service - Bluetooth service
   Loaded: loaded (/lib/systemd/system/bluetooth.service; disabled; vendor preset: enabled)
   Active: active (running) since Thu 2017-06-29 16:21:43 EDT; 16s ago
     Docs: man:bluetoothd(8)
 Main PID: 27362 (bluetoothd)
   Status: "Running"
   CGroup: /system.slice/bluetooth.service
           └─27362 /usr/local/libexec/bluetooth/bluetoothd
```
Retry the commands that were mentioned in step 2 for pairing the PS3 joystick. 

2. Run the following command: 
``` 
hciconfig hci0 reset 
``` 
followed by: 
```
sudo bash 
rosrun ps3joy sixpair
``` 
