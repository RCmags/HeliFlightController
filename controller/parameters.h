/*      CONSTANT     |    VALUE   |  UNIT  |   DESCRIPTION */
/*=========================================================*/
//                  1. Control inputs
//----------------------------------------------------------
#define GAIN_PITCH       0.8                 // Pitch servo
#define GAIN_ROLL        0.8                 // Roll servo
#define GAIN_YAW         1.6                 // Yaw servo / Tail ESC

//----------------------------------------------------------
//                  2A. Heading stabilization
//---------------------------s-------------------------------
/* NOTE: For a given axis, the propotional, integral and 
         derivative terms must have the same sign */
  // I. Proportional:                         // Proportional gain. Adjusts damping response. Increase to retard rotation. 
#define GAIN_PROP_ROLL    140.0
#define GAIN_PROP_PITCH   350.0
#define GAIN_PROP_YAW     18.0
  
  // II. Integral:                            // Integral gain. Adjusts spring respose. Increase to have a stronger restoring force.
#define GAIN_INT_ROLL     700.0
#define GAIN_INT_PITCH    900.0
#define GAIN_INT_YAW      150.0

  // III. Derivative:                         // Derivative gain. Reduces oscillations of proportional term. Magnifies vibration noise.
#define GAIN_DERIV_ROLL   8.0
#define GAIN_DERIV_PITCH  10.0
#define GAIN_DERIV_YAW    1.0

  // IV. Phase angle
#define STAT_ANGLE        60.0     // deg      // Pitch-roll coupling angle. Adjust until control axes respond independently.
#define DYN_ANGLE         60.0     // deg

  // V. Other:      
#define NEGATE_ROLL                            // Uncomment to reverse roll deflection
//#define NEGATE_PITCH                           // Uncomment to reverse pitch deflection
//#define NEGATE_YAW                             // Uncomment to reverse yaw deflection

/* NOTE: must enable integral decay to use */
#define DECAY_ROTOR       0                   // Decay rate of pitch/roll integral term. 
#define DECAY_YAW         0                   // Decay rate of yaw integral term. Increase to have integral term reduce with time. 

//----------------------------------------------------------
//                  2B. Auto level
//----------------------------------------------------------
/* NOTE: must enable auto level to use */
#define TRIM_ANG_PITCH    0.0                 // trim to self-level aircraft in pitch
#define TRIM_ANG_ROLL   -20.0                   // trim to self-level aircraft in roll

//----------------------------------------------------------
//                  3. Servo trims
//----------------------------------------------------------
#define TRIM_PITCH        0       // us       // Pitch servp
#define TRIM_ROLL         0       // us       // Roll servo
#define TRIM_YAW          0       // us       // Yaw servo / Tail ESC

//----------------------------------------------------------
//                  4. Signal filters
//----------------------------------------------------------
#define NUMBER_MEAN       50                  // number of readings used for input calibration
#define INPUT_DEADBAND    24      // us       // deadband near center-stick to prevent integrator drift.
#define INPUT_CHANGE      4       // us       // Change in PWM signal needed to update receiver inputs
#define ALPHA             0.4                /* Gain of alpha-beta filter applied to sensor input. 
                                               A smaller value smoothens the derivative at the cost of slower response */
//----------------------------------------------------------
//                  5. Tail rotor counter-torque
//---------------------------------------------------------- 
#define TORQUE_POWER      0.5                 /* Exponent of tail rotor correction curve. Make smaller to have 
                                                 correction change faster at low throttle. */
#define TORQUE_GAIN       250                 // Magnitude of tail rotor correction. Increase to apply a larger counter-torque. 

//----------------------------------------------------------
//                  6. Output PWM signal
//----------------------------------------------------------
#define PWM_MID           1500    // us       // Center/middle servo position
#define PWM_CHANGE        550     // us       // Maximum change in servo position

//----------------------------------------------------------
//                  7. Settings
//---------------------------------------------------------- 
//#define USING_WEIGHT_SHIFT                    // Uncomment for weight-shift control 
//#define USING_TAIL_ROTOR                      // Uncomment for tail rotor correction
//#define USING_INTEGRAL_DECAY                  // Uncomment to enable integral decay
#define USING_AUTO_LEVEL                      // Uncomment to enable self-leveling correction
