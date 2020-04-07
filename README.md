# python_misc
various python tools for data analysis and data capture

## Brief

Most of these tools are really just a place for me to store things that have been useful at somepoint in the past. And, to prevent me from having to open excel for any reason. 

## VNLogAsync.py
Simple VN data logging tool. 

### Async Data Types

The `VNLogAsync.py` tool will allow a user to connect a VectorNav VN200 or VN100 to a laptop and use the serial port to communincate, configure, and log relatively "low rate" data from a pre-defined set (see `-a` flag below) of data fields. 

### Input port

Use the `-i` flag at the command line to specify where the sensor is connected to the computer (usually `/dev/tty.usbserial...` for mac or `/dev/ttyUSB0` for linux).

### Baud rate

Use the `-b` flag to specify the baudrate of the sensor. Default baud rate is 115200 bps. 

### Logged data (.txt) output file

Use the `-o` flag to specify where to save the data data that is logged.

### Sample rate

Use the `-f` flag to specify how quickly data is sent out of the VN. Note: This is entirely baud rate dependent. If you need high rate data, I would suggest using a binary receive tool (to be written in python).

### Nothing

Use the `-n` flag to still log data, but print nothing in the terminal. `l33t_h4ck3r5_0nly!!11!`

### Async data groups

Use the `-a` flag to specify the number of the async data type you want to use. 

```
0 	Asynchronous output turned off
1 	Yaw, Pitch, Roll
2 	Quaternion
8 	Quaternion, Magnetic, Acceleration and Angular Rates
9 	Directional Cosine Orientation Matrix
10 	Magnetic Measurements
11 	Acceleration Measurements
12 	Angular Rate Measurements
13 	Magnetic, Acceleration, and Angular Rate Measurements
14 	Yaw, Pitch, Roll, Magnetic, Acceleration, and Angular Rate Measurements
16 	Yaw, Pitch, Roll, Body True Acceleration, and Angular Rates
17 	Yaw, Pitch, Roll, Inertial True Acceleration, and Angular Rates
19 	IMU Measurements
20 	GPS LLA
21 	GPS ECEF
22 	INS LLA
23 	INS ECEF
28 	INS LLA 2
29 	INS ECEF 2
30 	Delta theta and delta velocity
```