// Program description: 

// Flight controller for a four-channel RC helicopter with a tail rotor. A PID controller
// is used to apply cyclic corrections while a PI controller is used for the tail rotor.
// Both integral terms have leaky integration. That is to say, the accumulated output
// reaches a constant value in the presence of a contant error. 

//####### NOTE: This requires an MPU-6050 to get gyro measurements #######

//=========== Libraries ===========
#include<Wire.h>
#include<Servo.h>


//=========== Constants ===========

//Misc:
const int MPU6050_addr      = 0x68;
const int N_AXES            = 3;
const int LED_DELAY         = 200;


//Gyro Calibration:
const int N_ITERATE         = 5;
const float CALIB_COUNT     = 200;
const float MOTION_LIMIT    = 1000;

// PWM inputs:
const int N_INPUTS          = 4;
const int DEADBAND          = 4;
const float DECAY_INPUT     = 0.15;
const int INPUT_CALIB_TIME  = 200;
const int INPUT_COUNT       = 10;

// PWM outputs:
const int PWM_MID           = 1500;
const int PWM_CHANGE        = 700;
const int PIN_OUTPUTS[]     = { 3, 4, 5, 6 };
const int N_OUTPUTS = sizeof(PIN_OUTPUTS)/sizeof( PIN_OUTPUTS[0] ) ;

    //Gyro filters:
const float DECAY_RAW       = 0.3;
const float COEFF_DERIV[]   = {2, 1, 0, -1, -2};
const float DENOM_DERIV     = 8;
const int N_DERIV = sizeof( COEFF_DERIV )/sizeof( COEFF_DERIV[0] );

    //Gyro Misc:
const int GYRO_AXIS[]       = { 2, 1, 0 };
const float DEAD_GYRO       = 150;

//PID Gains:
    //Input gains:
const float GAIN_INPUT[N_AXES]      = { -0.30, 0.30, 0.2 };
const float DEAD_INPUT_INT          = 20;
const float GAIN_INPUT_INT[N_AXES]  = { -0.009, 0.009, 0.002 }; 
    //Gyro gains:
const float GAIN_PROP[N_AXES]       = { 0.0135 , 0.0135 , 0.006 };
const float GAIN_DERIV[N_AXES]      = { 0.22 , 0.22 , 0.145 };
const float GAIN_INT[N_AXES]        = { 0.00035 , 0.00035 , 0.00005 }; 
    
    // Servo offset:
const int SUBTRIM[N_AXES]           = { 60, 0, -250 };

    //Integral term:
const float INT_MAX         = 400;
const float INT_DECAY       = 0.0005;

    //Torque offset:
const int T_BIAS            = 400;
const float T_GAIN          = 280.0;
const float T_POWER         = 0.7;
const float PWM_RANGE       = 750;

    //Phase angle:
const float PHASE = ( PI/180.0 ) * -25.0;
const float SIN_F = sin(PHASE);
const float COS_F = cos(PHASE);


//=========== Variables ===========

    //Gyro:
int8_t Motion_Switch = 0;
float Gyro_Raw[N_AXES] = {0};
float Gyro_Init[N_AXES] = {0};
float Gyro_Val[N_AXES][N_DERIV] = {0};
float Gyro_Deriv[N_AXES] = {0};
float Gyro_Int[N_AXES] = {0};
    
    //PWM inputs:
volatile int16_t pwm_raw[N_INPUTS] = {0};
int16_t pwm_input[N_INPUTS] = {0};
int16_t pwm_Init[N_AXES] = {0};
Servo servo[N_OUTPUTS];

//=========== Functions ===========

  //-- Miscellaneous:
void blink_led( uint8_t n_times ) {  
  for( uint8_t index = 0; index < n_times; index += 1 ) {
    digitalWrite( LED_BUILTIN, HIGH );
    delay( LED_DELAY );
    digitalWrite( LED_BUILTIN, LOW );
    delay( LED_DELAY );
  }
}

float deadband( float input, float band ) {
  if( input <= -band ) {  
    return input + band;
  } else if ( input >= band ) {
    return input - band;
  } else {
    return 0;
  } 
}

float clamp( float input, float i_min, float i_max ) {
  if( input <= i_min ) {
    return i_min;
  } else if ( input >= i_max ) {
    return i_max;
  } else {
    return input;
  }
}


//=== Gyro reading functions 
  
void gyro_begin( void ) {
  Wire.begin();
  Wire.beginTransmission(MPU6050_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission();

    //- Low pass filter:
  Wire.beginTransmission(MPU6050_addr);
  Wire.write(0x1A);                    // write to address 26 of the register
  Wire.write(0x06);
  Wire.endTransmission();

    //- Gyro sensitivity:
  Wire.beginTransmission(MPU6050_addr);
  Wire.write(0x1B);                    // write to address 27 of the register
  Wire.write(0x00);
  Wire.endTransmission();
}

void gyro_get( void ) {
  
  Wire.beginTransmission(MPU6050_addr);
  Wire.requestFrom(MPU6050_addr,14);
  Wire.write(0x3B);
  Wire.endTransmission();

  for( int index = 0; index < 4; index +=1 ) {
    Wire.read()<<8|Wire.read();
  }

  for( int index = 0; index < N_AXES; index += 1 ) { // { roll, yaw, pitch )
    int8_t val = GYRO_AXIS[index];
    Gyro_Raw[val] = Wire.read()<<8|Wire.read();
    Gyro_Raw[index] -= Gyro_Init[index];
  }
}

void shift_array( void ) {

  for( int index = 0; index < N_AXES; index += 1 ) {
    for( int count = N_DERIV - 1; count > 0; count -= 1 ) {
      Gyro_Val[index][count] = Gyro_Val[index][count - 1];
    }
    Gyro_Val[index][0] = deadband( Gyro_Raw[index] , DEAD_GYRO )*DECAY_RAW + ( Gyro_Val[index][0] )*(1 - DECAY_RAW);
  }
}


//=== Calibration:

void gyro_calibrate( void ) {
  
  double sum[N_AXES][N_ITERATE] = {0};

  for( int pass = 0; pass < N_ITERATE ; pass += 1 ) {

    for( int count = 0; count < CALIB_COUNT ; count += 1 ) {
    
      gyro_get();
  
      for( int index = 0; index < N_AXES; index += 1 ) { 
        sum[index][pass] += Gyro_Raw[index];
      }
    }

    //-

    for( int index = 0; index < N_AXES; index += 1 ) {
      Gyro_Init[index] += sum[index][pass]/CALIB_COUNT;
    }
  }

  //--

  for( int count = 0; count < CALIB_COUNT ; count += 1 ) {
    
    gyro_get();
    
    for( int index = 0; index < N_AXES ; index += 1 ) {
    
      if( abs( Gyro_Raw[index] ) > MOTION_LIMIT ) {
        Motion_Switch = 1;
        break;
      }
    }

    if( Motion_Switch == 1 ) {
      break;
    }
  }
}

void input_calibrate( void ) {
  
  int sum[N_AXES][N_ITERATE] = {0};

  for( int pass = 0; pass < N_ITERATE ; pass += 1 ) {

    for( int index = 0; index < INPUT_COUNT ; index += 1 ) {
      for( int index = 0; index < N_AXES; index += 1 ) {
         sum[index][pass] += pwm_raw[index] - pwm_Init[index];
      }
      delay(20);
    }
   
    for( int index = 0; index < N_AXES; index += 1 ) {
      pwm_Init[index] += sum[index][pass]/INPUT_COUNT;
    }
  } 
}


//=== PWM inputs

ISR( PCINT0_vect ) {  

  static int32_t change_time[N_INPUTS] = {0};
  
  uint8_t mask = B00000001;
  
  for ( uint8_t index = 0 ; index < N_INPUTS ; index += 1 ) {      
   
    if( (change_time[index] == 0) && (PINB & mask) ) {  
      change_time[index] = micros();
    }
    
    else if( (change_time[index] != 0) && !(PINB & mask) ) {
      
      change_time[index] = ( micros() - change_time[index] ) - PWM_MID;

      //-
    
      if( ( change_time[index] - pwm_raw[index] ) > DEADBAND ) {
        pwm_raw[index] = change_time[index] - DEADBAND;
      } 

      if( ( change_time[index] - pwm_raw[index] ) < -DEADBAND ) {
        pwm_raw[index] = change_time[index] + DEADBAND;
      }       
      
      change_time[index] = 0;
    } 
    mask = mask << 1;
  }
}

void input_low_pass( void ) {
  
  int16_t input; 
  
  for( int index = 0; index < N_INPUTS ; index += 1 ) {
  
    if( index == 3 ) {
      input = pwm_raw[index];
    } else {
      input = pwm_raw[index] - pwm_Init[index];
    }
    
    pwm_input[index] += ( input - pwm_input[index] )*DECAY_INPUT ;
  }
}


//=== PID Stabilization and servo output
void fir_deriv( void ) {
  
  for( int index = 0; index < N_AXES; index += 1 ) {
    
    Gyro_Deriv[index] = ( Gyro_Val[index][0] * COEFF_DERIV[0] );
    
    for( int count = 1; count < N_DERIV ; count += 1 ) {
      Gyro_Deriv[index] += ( Gyro_Val[index][count] * COEFF_DERIV[count] );
    }
    
    Gyro_Deriv[index] = Gyro_Deriv[index]/DENOM_DERIV;
  }  
}

void trap_integ( void ) {
    
    for( int index = 0; index < N_AXES; index += 1 ) {
      
      float slice = ( Gyro_Val[index][0] + Gyro_Val[index][1] )/2.0; 
      
      Gyro_Int[index] += slice*GAIN_INT[index];
      
      Gyro_Int[index] += deadband( float ( pwm_input[index] ) , DEAD_INPUT_INT )*GAIN_INPUT_INT[index] ;
      
      Gyro_Int[index] -= Gyro_Int[index]*INT_DECAY; 
      
      Gyro_Int[index] = clamp( Gyro_Int[index] , -INT_MAX, INT_MAX );
    }
}

float torque_offset( void ) {
  float input = ( pwm_input[3] + T_BIAS )/PWM_RANGE;
  if( input >= 0 ) {
    return pow( input , T_POWER )*T_GAIN ;
  } else {
    return 0; 
  }
}

void set_servo_pid( void ) {

  float output[N_AXES];
  
  for( int index = 0 ; index < N_AXES ; index += 1 ) { 
    output[index] = Gyro_Val[index][0]*GAIN_PROP[index] + pwm_input[index]*GAIN_INPUT[index] + Gyro_Deriv[index]*GAIN_DERIV[index] + Gyro_Int[index]; 
  }

  float input;

  //-
  input = ( COS_F*output[0] - SIN_F*output[1] );
  input = clamp( input, -PWM_CHANGE, PWM_CHANGE );
  
  servo[0].writeMicroseconds( PWM_MID + SUBTRIM[0] + int ( input ) );

  //--
  input = ( COS_F*output[1] + SIN_F*output[0] );
  input = clamp( input, -PWM_CHANGE, PWM_CHANGE );
    
  servo[1].writeMicroseconds( PWM_MID + SUBTRIM[1] + int ( input ) );

  //--
  servo[2].writeMicroseconds( PWM_MID + SUBTRIM[2] + int( torque_offset() + output[2] ) );
}


//=========== Main functions ===========

void setup(){
  
  gyro_begin();

  blink_led( 5 );
  gyro_calibrate();
  
  //-

    // Enabling interrupt:
  PCICR |= (B00000001 << 0);

  // Setting PWM inputs:
  for ( uint8_t index = 0; index < N_INPUTS ; index += 1 ) {
    PCMSK0 = PCMSK0 | (B00000001 << index);
    pinMode( index + 8 , INPUT_PULLUP );
  }

  blink_led( 1 );
  input_calibrate();

  if( Motion_Switch == 0 ) { 
    blink_led( 2 );    
  } else {
    blink_led( 20 );
    while( true ) {
      //do nothing
    }
  }

  //-
  
  for( int index = 0 ; index < N_OUTPUTS ; index += 1 ) {
    pinMode( PIN_OUTPUTS[index] , OUTPUT );
    servo[index].attach( PIN_OUTPUTS[index] );

    if( index == 2 || index == 3 ) {
      servo[index].writeMicroseconds( PWM_MID - PWM_CHANGE );
    } else {
      servo[index].writeMicroseconds( PWM_MID );
    }
  }
}


void loop() {
    gyro_get();
    shift_array();

    fir_deriv();
    input_low_pass();
    trap_integ();
    set_servo_pid();
}
