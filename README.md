# AUTONOMOUS SAILBOAT

For our capstone we are building a semi-autonomous sailboat capable of navigating between a series of waypoints set by an operator. This will include sensor integration, path planning, wireless communication, and mechanical operation of the boat. For our Embedded class we split into 2 teams in order to handle the sensor integration and wireless communication.

This project runs on the STM32H7 Nucleo-144. This board was chosen for its dual-core, floating point unit, and high clock rate.

### Sensor Team (refs/heads/main-sensor)

Sensor team was in charge of sensor integration. We combined a servo, IMU, magnetometer, GPS, and wind vane. In the final boat, servos will be used as the control mechanisms, the magnetometer, IMU, and GPS provide position and heading for waypoint navigation, the wind vane is used to correctly position the sail for optimal lift when on the water.

- GPS (GT-u7 gps module): The GPS module is used to get Latitude and Longitude reading printouts to know current position of sailboat. Furthermore, it contains logic to set up destination lat and long, and printing out a message when destination is reached.
- Wind Vane (DFRobot Windvane): The wind vane uses RS485 protocol, and therefore connects to a RS485 to UART adapter in order to read wind direction. The wind direction is used to actuate the sail servo and maintain a 45 degree angle relative to the wind (to maximize lift). 
- IMU and Magnetometer (Adafruit 9-DOF Absolute Orientation- BNO055): This breakout board contains a Gyroscope, Accelerometer, and Magnetometer. It provides individual readings from each sensor unit, and a fused reading of all to visualize and quantize the orientation of the board. This is used on the sailboat to visualize the model, and to control emergency shut off in the case of the boat tipping over.
- Power Distribution: We are using a 4S battery connected up to a Power Distribution board that provides a stable 5V and 12 V to the electronics.

Useful wiring diagrams and datasheets are in the docs file.

* Corbin Barney (u1066089)
* Charbel Salloum (u1446840)
* Derek Tinoco (u1382366)
* Connor Stevens (u1463295)

### Antenna Team

* Harrison LeTourneau (u1460207)
* Adam Welsh (u1456456)
* Jared Pratt
* Aliou Tippett

## SCRIPTS

`./build.sh [-cm4|-cm7|-startup]`

The build script can be used to setup the repo when it is first cloned or compile the individual processor .elf files. 

* After first cloning the repo run the build script with the -startup flag, it will install tools, configure projects, and build both processors. 
* If you run with the -cm4 or -cm7 flags it will build that processors but will not perform the startup routine. You can declare both flags at the same time to build both .elf files.

`./flash.sh [-cm4|-cm7|-b]`

The flash script can be used to build and flash the STM32 for a specific processor.

* The -b flag will build the binary for the declared processors and then flash them. If -b is omitted then the script will just flash, no build.
* the -cm4 and -cm7 flags are used to flash to those specific processors. If both are declared the script will always flash the cm4 and then the cm7.

## DEBUG

This STM32 is equipped with a virtual serial port that lets you connect to its serial debug port directly over the USB you use to flash, no extra nonsense needed. To run it you need to use the following commands.

linux
```bash
screen $(ls /dev/ttyACM* | head -1) 115200 # To start the screen
# To stop the screen click "k".
```

## CALIBRATION

On initial startup the IMU will need to calibrate. The status of the calibration can be seen on the virtual serial port where the magnetometer, gyroscope, accelerometer, and system will have a value from 0-3/3. Only when all three sensors are calibrated will the system reach 3/3. To calibrate the gyroscope hold the IMU still for a couple seconds, for the magnetometer move it in a figure 8 to calibrate, and finally for the accelerometer hold it in 6 stable positions covering all three axis. 