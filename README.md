# Flight controller for RC helicopter :helicopter:
This arduino sketch is a flight controller for a 4ch Radio controlled helicopter. It uses an [MPU6050 IMU](https://invensense.tdk.com/products/motion-tracking/6-axis/mpu-6050/) to stabilize the aircraft. 

## How it works
Stabilization about pitch and roll is accomplished via a PID loop using the Gyro as the proportional term. Stabilization about the yaw axis uses a PI loop along with a counter-torque term that increases as throttle is added. In general, the program only improves the short-term stability of the helicopter, and a human (or other algorithm) must stabilize the vehicle in the long-term.  

## Receiver Inputs
The program is designed to receive 4 PWM inputs from an RC receiver operating in MODE 2 and outputs 3 PWM signals. Two signals go to the cyclic servos and the third goes to a tail-servo or a speed controller. Since it is assumed only two servo motors are used for controlling the rotor, only a [90-degree swashplate](https://www.rchelicopterfun.com/ccpm.html) is supported. MODE 1 can be used with this program by swapping the RX connections at the arduino. 

The program was written for an Arduino Nano but it should be compatible with other boards. See the schematic for the tail-motor variant of the required circuit:

## Schematic
<p align="center">
<img width="80%" src = "heli_flight_control_schem.png" width = "80%" height = "80%"> 
</p>

## Example 
<!--
See these links for flight videos of a helicopter that uses this software:
  
- https://www.youtube.com/watch?v=qZ7qUPAXkvc
- https://www.youtube.com/watch?v=zrrgVdPAhFI
-->

Here are images of the helicopter this code was initially writen for:

<img src = "/example_pictures/front_view_res.JPG" width = "30%" height = "30%"> <img src = "/example_pictures/side_rev_view_res.JPG" width = "30%" height = "30%"> <img src = "/example_pictures/arduino_view_res.JPG" width = "30%" height = "30%"> 

<img src = "/example_pictures/stab_motion.gif" width = "30%" height = "30%"> <img src = "/example_pictures/pitch_stab.gif" width = "30%" height = "30%"> <img src = "/example_pictures/yaw_stab.gif" width = "30%" height = "30%">
