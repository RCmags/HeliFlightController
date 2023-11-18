//=========== Servos / Tail ESC ============
#include <Servo.h>
Servo servo[3];

//---------------------- Constants -------------------------

constexpr float PWM_MID_SERVO[] = { PWM_MID + TRIM_PITCH, 
                                    PWM_MID + TRIM_ROLL, 
                                    PWM_MID + TRIM_YAW };
                                    
constexpr int SERVO_PIN[] = {3,4,5};    // Output pins: roll, pitch, yaw

//---------------------------------------------------------

/* initialize and center servos */
void setupServos() {  
  for( int i=0; i < 3; i += 1 ) {
    pinMode( SERVO_PIN[i], OUTPUT);
    servo[i].attach( SERVO_PIN[i] );
    servo[i].writeMicroseconds( PWM_MID_SERVO[i] );  
  }
}

/* set servo positions */
void servoPosition( float* output ) {
  for( int i=0; i < 3; i += 1 ) {
    output[i] = constrain( output[i], -PWM_CHANGE, PWM_CHANGE);
    servo[i].writeMicroseconds( PWM_MID_SERVO[i] + output[i] );  
  }
}
