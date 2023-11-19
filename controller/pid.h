//============= PID controller =============

/* Linearized error function */
vec3_t crossAngle(vec3_t v1, vec3_t v2) {
  vec3_t cross = v1.cross(v2);  // sin
  float  dot   = v1.dot(v2);    // cos
  
  // triangle wave approximation
  float scale = 0.5 + (1.1 - 0.1*dot*dot)/(1 + 0.9*dot);
  return cross*scale;
}

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
  constexpr float SCALE = (DEG_TO_RAD * 150) / 500.0;   // control input rate. Convert 500us to 150 deg/s 
    
    // alpha-beta filter:
  constexpr float BETA = ALPHA * ALPHA * 0.25;

  // 1. Calculus
  static float angle[3] = {0};      // integral
  static float angacc[3] = {0};     // derivative
  static float angvel_s[3] = {0};   // smoothed proportional

  float angvel[] = { gyroX(), gyroY(), gyroZ() };
  float angvel_t[] = { input[0]*SCALE, 
                       input[1]*SCALE,
                       input[2]*SCALE };
                       
  #ifdef USING_AUTO_LEVEL    
    // offsets
    constexpr float ANG_ROLL = TRIM_ANG_ROLL * DEG_TO_RAD;
    constexpr float ANG_PITCH = TRIM_ANG_PITCH * DEG_TO_RAD;

    // yaw change
    float dyaw = -0.5 * angvel_t[2] * dt;
    fusion.rotateHeading( dyaw, SMALL_ANGLE );

    // heading error
    updateFusion();

      // target
    vec3_t vec_rot = vec3_t( angvel_t[0], angvel_t[1], 0 ) * 0.25;
    
    quat_t quat_rot; 
    quat_rot.setRotation( vec_rot, SMALL_ANGLE );
    quat_rot = quat_rot.norm();

    vec3_t vec_z = quat_rot.rotate( vec3_t(0,0,1) , LOCAL_FRAME );
    vec3_t vec_y = quat_rot.rotate( vec3_t(0,1,0) , LOCAL_FRAME );
    vec3_t vec_x = quat_rot.rotate( vec3_t(1,0,0) , LOCAL_FRAME );

      // measure
    vec3_t error; 
    error  = crossAngle( fusion.getZaxis(LOCAL_FRAME), vec_z );
    error += crossAngle( fusion.getYaxis(LOCAL_FRAME), vec_y );
    error += crossAngle( fusion.getXaxis(LOCAL_FRAME), vec_x );
    error *= 0.5;
    
      // covert to channel axes
    angle[0] = GAIN_INT[0] * error.x;    // roll
    angle[1] = GAIN_INT[1] * error.y;    // pitch
    angle[2] = GAIN_INT[2] * error.z;    // yaw
  #endif
  
  for( int i = 0; i < 3; i += 1 ) {
    // alpha-beta filter:
    float dvel = angvel[i] - angvel_s[i];
    angvel_s[i] += angacc[i]*dt + ALPHA*dvel;   // smoothed proportional
    angacc[i] += BETA*dvel / dt;                // derivative

    // target rate
    float dangvel = angvel[i] - angvel_t[i]; 
    
    // integrate
    angvel[i] += angacc[i] * 0.5 * dt;           // trapezoidal rule. Use derivative to improve area slice. 

    #ifndef USING_AUTO_LEVEL
      angle[i] += GAIN_INT[i] * dangvel * dt;    // Use quaternion? Need to prevent saturation.
      
      #ifdef USING_INTEGRAL_DECAY
        angle[i] -= DECAY[i] * angle[i] * dt;       
      #endif
   #endif
    
    angle[i] = constrain(angle[i], -PWM_CHANGE, PWM_CHANGE);  // prevent windup
  }

  // yaw axis
  output[2] = GAIN_PROP[2]*(angvel[2] - angvel_t[2]) + angle[2] + GAIN_DERIV[2]*angacc[2];

  // 3. Phase angle mixing

    // dynamic angle
  constexpr float DYN_ANG = DYN_ANGLE * DEG_TO_RAD;
  constexpr float SIN_DYN = sin(DYN_ANG);
  constexpr float COS_DYN = cos(DYN_ANG);
  
  float dyn_x = GAIN_PROP[0]*angvel[0] + GAIN_DERIV[0]*angacc[0];
  float dyn_y = GAIN_PROP[1]*angvel[1] + GAIN_DERIV[1]*angacc[1];

  output[0] = dyn_x*COS_DYN + dyn_y*SIN_DYN;
  output[1] = dyn_y*COS_DYN - dyn_x*SIN_DYN; 

    // static angle
  constexpr float ANGLE = DEG_TO_RAD * STAT_ANGLE;
  constexpr float SIN_ANG = sin(ANGLE);
  constexpr float COS_ANG = cos(ANGLE);

  float stat_x = -GAIN_PROP[0]*angvel_t[0] + angle[0];
  float stat_y = -GAIN_PROP[1]*angvel_t[1] + angle[1];
  
  float output_0 = output[0];
  output[0] += stat_x*COS_ANG + stat_y*SIN_ANG;
  output[1] += stat_y*COS_ANG - stat_x*SIN_ANG;
  
  // 3. parameters:
  
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

// counter main-rotor torque via yaw bias
float torqueBias(float input) {
  constexpr float SCALE = 0.5 / PWM_CHANGE;
  input = input < 0 ? 0 : input;
  input *= SCALE;
  return pow(input, TORQUE_POWER) * TORQUE_GAIN;  
}
