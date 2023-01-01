//============= PID controller =============

/* Apply PID controller to each rotational axis */
void PIDcontroller( float* input, float* output ) {
  // 1. Time step:
  static uint32_t time_last = micros();  // initial measurement
   
  uint32_t time_now = micros();
  float dt = float(time_now - time_last)*1e-6;
  time_last = time_now;

  // 0. Constants:
      // PID coefficients:
  constexpr float GAIN_PROP[]  = { GAIN_PROP_ROLL , GAIN_PROP_PITCH , GAIN_PROP_YAW  };
  constexpr float GAIN_INT[]   = { GAIN_INT_ROLL  , GAIN_INT_PITCH  , GAIN_INT_YAW   };
  constexpr float GAIN_DERIV[] = { GAIN_DERIV_ROLL, GAIN_DERIV_PITCH, GAIN_DERIV_YAW };
    
    // integral term:
  constexpr float SCALE_RATE = (DEG_TO_RAD * 150) / 500;    // Convert 500us to 150 deg/s 
  constexpr float SCALE_ANGLE = (DEG_TO_RAD * 45) / 500;      // Convert 500us to 30 deg
  
  constexpr float ANG_ROLL = TRIM_ANG_ROLL * DEG_TO_RAD;
  constexpr float ANG_PITCH = TRIM_ANG_PITCH * DEG_TO_RAD;
    
    // alpha-beta filter:
  constexpr float BETA = ALPHA * ALPHA * 0.25;

  // 1. Calculus
  static vec3_t angacc = {0,0,0};     // derivative
  static vec3_t angvel_s = {0,0,0};   // smoothed proportional

    // state
  vec3_t angle = { fusion.roll(), fusion.pitch(), fusion.yaw() };
  vec3_t angvel = { gyroX(), gyroY(), gyroZ() };

    // alpha-beta filter
  vec3_t dvel = angvel - angvel_s;
  angvel_s += angacc*dt + ALPHA*dvel;
  angacc += BETA * dvel/dt;
  
  #ifdef USING_AUTO_LEVEL
    vec3_t accel = { accelX(), accelY(), accelZ() };    
      // only integrate yaw
    angvel.z -= input[2] * SCALE_RATE;   
    fusion.update( angvel, accel, GAIN, SD_ACCEL );  
      // bias angle
    angle.x -= ANG_ROLL + input[0]*SCALE_ANGLE;
    angle.y -= ANG_PITCH + input[1]*SCALE_ANGLE;
     
  #else // relative control
    angvel -= vec3_t(input)*SCALE_RATE;
    fusion.update( angvel );
  #endif

  output[0] = GAIN_PROP[0]*angvel.x + GAIN_INT[0]*angle.x + GAIN_DERIV[0]*angacc.x;
  output[1] = GAIN_PROP[1]*angvel.y + GAIN_INT[1]*angle.y + GAIN_DERIV[1]*angacc.y;
  output[2] = GAIN_PROP[2]*angvel.z + GAIN_INT[2]*angle.z + GAIN_DERIV[2]*angacc.z;
  
  // 2. parameters:
  
  #ifdef USING_WEIGHT_SHIFT
    float acc = accelZ();
    float gain = acc == 0 ? 0 : 1/acc;    // deflection must increase with lower lift
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

/* counter main-rotor torque via yaw bias */
float torqueBias(float input) {
  constexpr float SCALE = 0.5 / PWM_CHANGE;
  input = input < 0 ? 0 : input;
  input *= SCALE;
  return pow(input, TORQUE_POWER) * TORQUE_GAIN;  
}
