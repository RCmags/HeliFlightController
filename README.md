# Flight controller for an RC helicopter :helicopter:
This arduino sketch is a flight controller for a 4ch Radio controlled helicopter. It uses an [MPU6050 IMU](https://invensense.tdk.com/products/motion-tracking/6-axis/mpu-6050/) to stabilize the aircraft. 

## How it works
Stabilization about the pitch, roll and yaw axes is accomplished with three PID loops using the angular velocity as a proportional term. The yaw axis includes a counter-torque term that increases with throttle. 

In general, the program only improves stability in the _short-term_ and a pilot (or other control algorithm) is needed stabilize the helicopter in the _long-term_. Despite this, the aircraft can maintain a steady hover without pilot input if the PIDs are tuned correctly (in the order of 5 seconds or more).  

## Receiver Inputs
The program is designed to receive __4 PWM inputs__ from a RC receiver and generates __3 PWM outputs__. Two signals go to the cyclic servos and the third goes to the tail-servo or speed controller. Since it assumes that only two servos are used for the cyclic, only a [90-degree swashplate](https://www.rchelicopterfun.com/ccpm.html) is supported. The code assumes the transmitter uses MODE 2, but MODE 1 can be used by swapping the arduino connections to the receiver.

## Schematic
The sketch was written for an Arduino Nano but it should be compatible with other boards.

<p align="center">
<img width="80%" src = "heli_flight_control_schem.png" width = "80%" height = "80%"> 
</p>

__Note__: replace the tail motor and speed controller with a servo if you're using a variable pitch rotor.

## Example usage
Here's images of the helicopter this code was initially writen for:

<p align="center">
<img src = "/example_pictures/arduino_view_res.JPG" width = "30%" height = "30%"> <img src = "/example_pictures/side_rev_view_res.JPG" width = "30%" height = "30%"> 
</p>

<p align="center">
<img src = "/example_pictures/pitch_stab.gif" width = "30%" height = "30%"> <img src = "/example_pictures/yaw_stab.gif" width = "30%" height = "30%">
</p>

See: [Flight video](https://www.youtube.com/watch?v=nHYoC72DZKo)
