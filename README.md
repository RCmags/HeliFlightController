# Flight controller for an RC helicopter :helicopter:

This arduino sketch is a flight controller for a 4ch radio-controlled helicopter. It uses an [MPU6050 IMU](https://invensense.tdk.com/products/motion-tracking/6-axis/mpu-6050/) to stabilize the aircraft. 

## How it works

The pitch, roll and yaw axes are stabilized with a respective [PID controller](https://en.wikipedia.org/wiki/PID_controller) using the angular velocity as a proportional term. The yaw axis includes a counter-torque term that increases with throttle. 

In general, the program only improves stability in the _short-term_ and a pilot (or other control algorithm) is needed to stabilize the helicopter in the _long-term_. Despite this, the aircraft can maintain a steady hover without pilot input if the PIDs are tuned correctly (in the order of 5 seconds).  

## Receiver Inputs

The program is designed to receive __4 PWM inputs__ from an RC receiver and generates __3 PWM outputs__. Two signals go to the cyclic servos and the third goes to the tail-servo or speed controller. Since it assumes that only two servos are used for the cyclic, only a [90-degree swashplate](https://www.rchelicopterfun.com/ccpm.html) is supported. The code assumes the transmitter uses MODE 2, but MODE 1 can be used by swapping the arduino connections to the receiver.

## Schematic
The sketch was written for an Arduino Nano but it should be compatible with other boards.

<p align="center">
<img width="80%" src = "/images/diagram/schematic.png" width = "80%" height = "80%"> 
</p>

__Note__: replace the tail motor and speed controller with a servo if you're using a variable pitch rotor.

## Dependencies
These libraries are needed to compile the code:

- [basicMPU6050](https://github.com/RCmags/basicMPU6050)
- [imuFilter](https://github.com/RCmags/imuFilter)

## Example
This is the helicopter the code was initially writen for:

See: [Flight video](https://www.youtube.com/watch?v=nHYoC72DZKo)

<p align="center">
<img src = "/images/arduino_view.JPG" width = "30%" height = "30%"> <img src = "/images/side_view.JPG" width = "30%" height = "30%"> 
</p>

<p align="center">
<img src = "/images/pitch_stab.gif" width = "30%" height = "30%"> <img src = "/images/yaw_stab.gif" width = "30%" height = "30%">
</p>

