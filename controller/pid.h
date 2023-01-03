//============= PID controller =============

/* Apply PID controller to each rotational axis */
void PIDcontroller( float* input, float* output ) {
  // 1. Time step:
  static uint32_t time_last = micros()-1;  // dither initial measurement (avoid zero division)
   
  uint32_t time_now = micros();
  float dt = float(time_now - time_last)*1e-6;
  time_last = time_now;

  // 0. Constants:
      // PID coefficients:
  constexpr float DECAY[]      = { DECAY_ROTOR    , DECAY_ROTOR     , DECAY_YAW      };
  constexpr float GAIN_PROP[]  = { GAIN_PROP_ROLL , GAIN_PROP_PITCH , GAIN_PROP_YAW  };
  constexpr float GAIN_INT[]   = { GAIN_INT_ROLL  , GAIN_INT_PITCH  , GAIN_INT_YAW   };
  constexpr float GAIN_DERIV[] = { GAIN_DERIV_ROLL, GAIN_DERIV_PITCH, GAIN_DERIV_YAW };
    
    // integral term:
  constexpr float SCALE = (DEG_TO_RAD * 150) / 500;   // control input rate. Convert 500us to 150 deg/s 
    
    // alpha-beta filter:
  constexpr float BETA = ALPHA * ALPHA * 0.25;

  // 1. Calculus
  static float angle[3] = {0};      // integral
  static float angacc[3] = {0};     // derivative
  static float angvel_s[3] = {0};   // smoothed proportional

  float angvel[] = { gyroX(), gyroY(), gyroZ() };
  
  for( int i = 0; i < 3; i += 1 ) {
    // alpha-beta filter:
    float dvel = angvel[i] - angvel_s[i];
    angvel_s[i] += angacc[i]*dt + ALPHA*dvel;   // smoothed proportional
    angacc[i] += BETA*dvel / dt;                // derivative

    // target rate
    float angvel_t = input[i] * SCALE;          // target angular velocity
    angvel[i] -= angvel_t; 
    
    // integrate
    angvel[i] += angacc[i] * 0.5 * dt;           // trapezoidal rule. Use derivative to improve area slice. 
    angle[i] += GAIN_INT[i] * angvel[i] * dt;

    #ifdef USING_INTEGRAL_DECAY
      angle[i] -= DECAY[i] * angle[i] * dt;       
    #endif
    
    angle[i] = constrain(angle[i], -PWM_CHANGE, PWM_CHANGE);  // prevent windup

    // sum terms
    output[i] = GAIN_PROP[i]*angvel[i] + angle[i] + GAIN_DERIV[i]*angacc[i];
  }

  // 2. parameters:
  
  #ifdef USING_WEIGHT_SHIFT
    float accel = accelZ();
    float gain = accel == 0 ? 0 : 1/accel;    // deflection must increase with lower lift
    output[0] *= gain;
    output[1] *= gain;
  #endif

  #ifdef NEGATE_ROLL
    output[0] = -output[0];
  #endif

  #ifdef NEGATE_PITCH
    output[1] = -output[1];
  #endif

  #ifdef NEGATE_YAW
    output[2] = -output[2];
  #endif
}

/* bias controls to self upright */
void autoLevel(float* output) {
  constexpr float ANG_ROLL = TRIM_ANG_ROLL * DEG_TO_RAD;
  constexpr float ANG_PITCH = TRIM_ANG_PITCH * DEG_TO_RAD;
    
    // horizontal angles
  updateFusion();
  float pitch = fusion.roll() - ANG_ROLL;   
  float roll = fusion.pitch() - ANG_PITCH; 

    // control rates
  output[1] -= float(GAIN_ANG_ROLL) * roll; 
  output[0] -= float(GAIN_ANG_PITCH) * pitch;   
}

/* counter main-rotor torque via yaw bias */
float torqueBias(float input) {
  constexpr float SCALE = 0.5 / PWM_CHANGE;
  input = input < 0 ? 0 : input;
  input *= SCALE;
  return pow(input, TORQUE_POWER) * TORQUE_GAIN;  
}
