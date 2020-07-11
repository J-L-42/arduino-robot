# Arduino Robot Controller

## Usage

This robot controller requires a robot chassis with:
- 2 driving wheels
- 2 wheel-encoders
- MPU9250 gyroscope chip
- Solenoid pen holder

Upload `robot-driver.ino` to an Arduino using the Arduino IDE and a programming cable. To send
commands to the robot, keep the programming cable connected to the Arduino and run
```bash
$ python robot_controller.py
```
A test program should now run. Alternative ways of programming the robot
include:
- Importing `robot_controller` into a Python script and writing your own commands using the provided
methods
- Sending commands directly to the Arduino using the USB Serial Monitor in the Arduino IDE

The robot can be made wireless using a Raspberry Pi to run the `robot_controller` script.

## Requirements

- [Arduino IDE](https://www.arduino.cc/en/Main/Software)
- [`pyserial`](https://pypi.org/project/pyserial/)
- [`FaBo9Axis MPU9250`](https://github.com/FaBoPlatform/FaBo9AXIS-MPU9250-Library)

## Commands

The following commands are currently supported:
Command | Python Function | Parameters
--------|-----------------|-------------------
`f`     | `forward`       | Distance (cm)
`b`     | `backward`      | Distance (cm)
`l`     | `turn_left`     | Rotation (degrees)
`r`     | `turn_right`    | Rotation (degrees)
`u`     | `pen_up`        |
`d`     | `pen_down`      |
