/**************************************************************************************/
/***************                  Motor Pin order                  ********************/
/**************************************************************************************/
// since we are uing the PWM generation in a direct way, the pin order is just to inizialie the right pins 
// its not possible to change a PWM output pin just by changing the order
uint8_t PWM_PIN[8] = {9,10,5,6,4,A2,SW_PWM_P3,SW_PWM_P4};   //for a quad+: rear,right,left,front

/**************************************************************************************/
/************  Writes the Motors values to the PWM compare register  ******************/
/**************************************************************************************/
void writeMotors() { // [1000;2000] => [125;250]
  /******** Specific PWM Timers & Registers for the atmega32u4 (Promicro)   ************/
  // Timer 1 A & B [1000:2000] => [8000:16000]
  OCR1A = ((motor[0]<<4) - 16000) + 128;
  OCR1B = ((motor[1]<<4) - 16000) + 128;
  // Timer 4 A & D [1000:2000] => [1000:2000]
  TC4H = 2047-(((motor[2]-1000)<<1)+16)>>8; OCR4A = (2047-(((motor[2]-1000)<<1)+16)&0xFF); //  pin 5
  TC4H = (((motor[3]-1000)<<1)+16)>>8; OCR4D = ((((motor[3]-1000)<<1)+16)&0xFF); //  pin 6
}

/**************************************************************************************/
/************          Writes the mincommand to all Motors           ******************/
/**************************************************************************************/
void writeAllMotors(int16_t mc) {   // Sends commands to all motors
  for (uint8_t i=0;i<NUMBER_MOTOR;i++) {
    motor[i]=mc;
  }
  writeMotors();
}

/**************************************************************************************/
/************        Initialize the PWM Timers and Registers         ******************/
/**************************************************************************************/
void initOutput() {
  /****************            mark all PWM pins as Output             ******************/
  for(uint8_t i=0;i<NUMBER_MOTOR;i++) {
    pinMode(PWM_PIN[i],OUTPUT);
  }
    
  /******** Specific PWM Timers & Registers for the atmega32u4 (Promicro)   ************/
      TCCR1A |= (1<<WGM11); // phase correct mode & no prescaler
      TCCR1A &= ~(1<<WGM10);
      TCCR1B &= ~(1<<WGM12) &  ~(1<<CS11) & ~(1<<CS12);
      TCCR1B |= (1<<WGM13) | (1<<CS10); 
      ICR1   |= 0x3FFF; // TOP to 16383;     
      TCCR1A |= _BV(COM1A1); // connect pin 9 to timer 1 channel A

      TCCR1A |= _BV(COM1B1); // connect pin 10 to timer 1 channel B

      // timer 4A
      TCCR4E |= (1<<ENHC4); // enhanced pwm mode
      TCCR4B &= ~(1<<CS41); TCCR4B |= (1<<CS42)|(1<<CS40); // prescaler to 16
      TCCR4D |= (1<<WGM40); TC4H = 0x3; OCR4C = 0xFF; // phase and frequency correct mode & top to 1023 but with enhanced pwm mode we have 2047
      TCCR4A |= (1<<COM4A0)|(1<<PWM4A); // connect pin 5 to timer 4 channel A 

      TCCR4C |= (1<<COM4D1)|(1<<PWM4D); // connect pin 6 to timer 4 channel D

  writeAllMotors(MINCOMMAND);
  delay(300);
}

/**************************************************************************************/
/********** Mixes the Computed stabilize values to the Motors & Servos  ***************/
/**************************************************************************************/
void mixTable() {
  int16_t maxMotor;
  uint8_t i;

  #define PIDMIX(X,Y,Z) rcCommand[THROTTLE] + axisPID[ROLL]*X + axisPID[PITCH]*Y + YAW_DIRECTION * axisPID[YAW]*Z

  #if NUMBER_MOTOR > 3
    //prevent "yaw jump" during yaw correction
    axisPID[YAW] = constrain(axisPID[YAW],-100-abs(rcCommand[YAW]),+100+abs(rcCommand[YAW]));
  #endif
  /****************                   main Mix Table                ******************/
  #ifdef MY_PRIVATE_MIXING
    #include MY_PRIVATE_MIXING
  #else
    #ifdef QUADX
      motor[0] = PIDMIX(-1,+1,-1); //REAR_R
      motor[1] = PIDMIX(-1,-1,+1); //FRONT_R
      motor[2] = PIDMIX(+1,+1,+1); //REAR_L
      motor[3] = PIDMIX(+1,-1,-1); //FRONT_L
    #endif
  #endif //MY_PRIVATE_MIXING

  /****************                normalize the Motors values                ******************/
    maxMotor=motor[0];
    for(i=1; i< NUMBER_MOTOR; i++)
      if (motor[i]>maxMotor) maxMotor=motor[i];
    for(i=0; i< NUMBER_MOTOR; i++) {
      // this is a way to still have good gyro corrections
      // if at least one motor reaches its max.
      if (maxMotor > MAXTHROTTLE)
        motor[i] -= maxMotor - MAXTHROTTLE;
      motor[i] = constrain(motor[i], conf.minthrottle, MAXTHROTTLE);
      if (rcData[THROTTLE] < MINCHECK)
        motor[i] = MINCOMMAND;
      if (!f.ARMED)
        motor[i] = MINCOMMAND;
    }
}
