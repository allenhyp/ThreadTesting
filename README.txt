README
#Change i2c baud rate from 100kHz to 1MHz:

$ sudo modprobe -r i2c_bcm2708 && sudo modprobe i2c_bcm2708 baudrate=1000000



#You can check if it's right in

$ sudo nano /sys/module/i2c_bcm2708/parameters/baudrate

#Or you can go even higher haha