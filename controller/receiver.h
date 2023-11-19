//========= Input receiver signals =========
volatile uint16_t pwm_input[4] = {0};

// PORTB = {8 .. 13} -> using pins {8 .. 11} = B00001111

/* port change interrupt to read PWM inputs from receiver */
ISR( PCINT0_vect ) {
  static uint32_t initial_time[4] = {0}; 
  static uint8_t port_last = PINB;  
  
  // port changes
  uint32_t current_time = micros(); 
  uint8_t port_rise = ~port_last & PINB;
  uint8_t port_fall = port_last & ~PINB;
  
  // find changing pins
  for( uint8_t index = 0; index < 4; index += 1) {
    uint8_t mask = B00000001 << index;  // Start at PCINT0   
    if( port_rise & mask ) {                
        initial_time[index] = current_time;
    } else if ( port_fall & mask ) {       
        pwm_input[index] = current_time - initial_time[index];
    }
  }
  port_last = PINB;    
}

void setupISR() {
  // enable PORTB interrupts  
  PCICR |= (1 << PCIE0);
  // enable PCINT for pins 8-11                                                   
  PCMSK0 |= (1 << PCINT0);                                              
  PCMSK0 |= (1 << PCINT1);                                              
  PCMSK0 |= (1 << PCINT2); 
  PCMSK0 |= (1 << PCINT3); 
  // set pins inputs
  pinMode(8,  INPUT_PULLUP);
  pinMode(9,  INPUT_PULLUP);
  pinMode(10, INPUT_PULLUP);
  pinMode(11, INPUT_PULLUP);
}

//----- Input filter

/* average of rx signals */
int* calibrateInputs() {  
  uint32_t sum[4] = {0};
  for( uint8_t count = 0; count < NUMBER_MEAN; count += 1 ) {
    for( uint8_t k = 0; k < 4; k += 1 ) {
      sum[k] += pwm_input[k];
    }
    delay(20);                  // period of 50hz pwm signal
  }
  
  static int output[4];         // store value to limit scope  
  for( uint8_t k = 0; k < 4; k += 1 ) {
    float mean = float( sum[k] ) / float( NUMBER_MEAN );
    output[k] = int( mean );
  }
  return output; 
}

/* make variable zero within bounds */
float deadband(float input, const float MIN, const float MAX) {
  return input > MAX ? input - MAX :
         input < MIN ? input - MIN : 0;  
}

/* remove noise from PWM inputs then scale and center */
void filterInputs(float* output) {
  static int* pwm_mean = calibrateInputs();
  static int filter[4] = {0};  
  
  for( uint8_t index = 0; index < 4; index += 1 ) {
    // remove noise
    int input = int( pwm_input[index] );
    int change = input - filter[index]; 
    filter[index] += deadband(change, -INPUT_CHANGE, INPUT_CHANGE);

    // center output
    output[index] = filter[index] - pwm_mean[index];
    output[index] = deadband( output[index], -INPUT_DEADBAND, INPUT_DEADBAND );
  }
  // scale output
  output[0] *= float(GAIN_ROLL);    
  output[1] *= float(GAIN_PITCH);   
  output[2] *= float(GAIN_YAW);
}
