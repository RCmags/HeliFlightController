/* HELICOPER FLIGHT CONTROLLER */

/* Arduino NANO sketch to stabilize a 4ch RC helicopter controlled by cyclic or weight shift. 
 * It commands two cyclic servos and a tail servo/ESC to stabilize each axis.
 * Author: RCmags https://github.com/RCmags
 * 
 * NOTE: 
 * This sketch uses an MPU6050 IMU (gyroscope and accelerometer) to stabilize the vehicle. It assumes 
 * that only lift and gravity act on the aircraft, and that any external torques are negligible.
*/
//=============== Connections ================
// See included schematic
  // Inputs:
// Pin 8  -> Receiver CH1
// Pin 9  -> Receiver CH2
// Pin 10 -> Receiver CH4
// Pin 11 -> Receiver CH3
  // Outputs:
// Pin 3  -> Pitch servo
// Pin 4  -> Roll servo 
// Pin 5  -> Tail servo / ESC  

//=================== Code ===================

#include "parameters.h"
#include "receiver.h"
#include "imu.h"
#include "fusion.h"
#include "pid.h"
#include "servos.h"

//----- Main loop

void setup() { 
  digitalWrite(LED_BUILTIN, LOW); 
  delay(1000);       // Allow RX to startup
  
  setupISR();
  setupServos();
  
  imu.setup();
  imu.setBias();

  #ifdef USING_AUTO_LEVEL
    setupFusion();
  #endif

  digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
  // RX inputs
  float input[4]; filterInputs(input);            // get RX inputs

  // controller output
  imu.updateBias();
  
  #ifdef USING_AUTO_LEVEL
    autoLevel(input);
  #endif  
  
  float output[3]; PIDcontroller(input, output);
  
  #ifdef USING_TAIL_ROTOR
    output[2] += torqueBias( input[3] );          // anti-torque
  #endif

  // move servos
  servoPosition(output);   
}
