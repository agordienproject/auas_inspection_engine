### Enabling ROS2

In a command line, use this :

```
source /opt/ros/humble/setup.bash
unset GTK_PATH
export QT_QPA_PLATFORM=xcb
```


### Gantry system

* The gantry system cannot load programs anymore after using it and break the connection
* To use properly the gantry system, you have to connect to it with the python program first (first here, first served), and after that start igusRobotControl (to see logs)
* To avoid looping programs, you have to change the execution mode to "single" in the Igus Software

### USB connection

* If you're using a VM, you have to start it before connecting the systems with USB. Also, you have to use USB 3.0 in the VM configuration (for the camera). When your VM is started, you can plug the systems. You will normally see in "devices" the camera and the Arduino.
* For the Arduino, if you click on it in the devices options, the box will be checked (so the system is connected to the VM). If you want to see that it's really connected, type this in a terminal : ```ls -la /dev/tty*```, you must see "/dev/ttyACM0", witch is the arduino port
* For the camera, do the same method to connect it. To see if the camera is connected to the VM, use this command : ```lsusb```. You must see "Interl Corp.....", depends on the model that you have. You could also use that command to see arduino connection.