#ifndef _DEF_H
#define _DEF_H

/**************************************************************************************/
/***************             Proc specific definitions             ********************/
/**************************************************************************************/
// Proc auto detection
#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
  #error "not support"
  #define PROMINI
#endif
#if defined(__AVR_ATmega32U4__) || defined(TEENSY20)
  #define PROMICRO
#endif
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
  #error "not support"
  #define MEGA
#endif


/***************             motor and servo numbers               ********************/
#define NUMBER_MOTOR     4

/**************************  atmega32u4 (Promicro)  ***********************************/
  #define LEDPIN_PINMODE             //
  #define LEDPIN_TOGGLE              PIND |= 1<<5;     //switch LEDPIN state (Port D5)
  #define LEDPIN_OFF                 PORTD |= (1<<5);
  #define LEDPIN_ON                  PORTD &= ~(1<<5);  
  #define BUZZERPIN_PINMODE          DDRD |= (1<<3);
  #define BUZZERPIN_ON               PORTD |= 1<<3;
  #define BUZZERPIN_OFF              PORTD &= ~(1<<3); 
  #define POWERPIN_PINMODE           //
  #define POWERPIN_ON                //
  #define POWERPIN_OFF               //
  #define I2C_PULLUPS_ENABLE         PORTD |= 1<<0; PORTD |= 1<<1;   // PIN 2&3 (SDA&SCL)
  #define I2C_PULLUPS_DISABLE        PORTD &= ~(1<<0); PORTD &= ~(1<<1);
  #define PINMODE_LCD                DDRD |= (1<<2);
  #define LCDPIN_OFF                 PORTD &= ~1;
  #define LCDPIN_ON                  PORTD |= 1;
  #define STABLEPIN_PINMODE          ;
  #define STABLEPIN_ON               ;
  #define STABLEPIN_OFF              ;
  #define PPM_PIN_INTERRUPT          DDRE &= ~(1 << 6);PORTE |= (1 << 6);EIMSK |= (1 << INT6);EICRB |= (1 << ISC61)|(1 << ISC60);
  #define USB_CDC_TX                 3
  #define USB_CDC_RX                 2
  
  //soft PWM Pins
  #define SW_PWM_P3                  A1        
  #define SW_PWM_P4                  A0
  
  #define THROTTLEPIN                3
  #define ROLLPIN                    4
  #define PITCHPIN                   5
  #define YAWPIN                     2
  #define AUX1PIN                    6
  #define AUX2PIN                    7
  #define AUX3PIN                    1 // unused
  #define AUX4PIN                    0 // unused
  #define PCINT_PIN_COUNT            4
  #define PCINT_RX_BITS              (1<<1),(1<<2),(1<<3),(1<<4)
  #define PCINT_RX_PORT              PORTB
  #define PCINT_RX_MASK              PCMSK0
  #define PCIR_PORT_BIT              (1<<0)
  #define RX_PC_INTERRUPT            PCINT0_vect
  #define RX_PCINT_PIN_PORT          PINB

  #define V_BATPIN                   A3    // Analog PIN 3
  #define PSENSORPIN                 A2    // Analog PIN 2

/**************************************************************************************/
/***************      IMU Orientations and Sensor definitions      ********************/
/**************************************************************************************/
#if defined(HEX_NANO)
  #define MPU6050
  #define BMP085
  //#define HMC5883
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -X; accADC[PITCH]  = -Y; accADC[YAW]  =  Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  Y; gyroADC[PITCH] = -X; gyroADC[YAW] = -Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  X; magADC[PITCH]  =  -Y; magADC[YAW]  = -Z;}
  #define MPU6050_EN_I2C_BYPASS // MAG connected to the AUX I2C bus of MPU6050 
  #undef INTERNAL_I2C_PULLUPS
  //#define MINTHROTTLE 1050
  //#define MAXTHROTTLE 2000
  #define EXT_MOTOR_RANGE
#endif

/**************************************************************************************/
/***************              Sensor Type definitions              ********************/
/**************************************************************************************/

#if defined(ADXL345) || defined(BMA020) || defined(BMA180) || defined(NUNCHACK) || defined(MMA7455) || defined(ADCACC) || defined(LIS3LV02) || defined(LSM303DLx_ACC) || defined(MPU6050) || defined(MMA8451Q) || defined(NUNCHUCK)
  #define ACC 1
#else
  #define ACC 0
#endif

#if defined(HMC5883) || defined(HMC5843) || defined(AK8975) || defined(MAG3110)
  #define MAG 1
#else
  #define MAG 0
#endif

#if defined(ITG3200) || defined(L3G4200D) || defined(MPU6050) || defined(MPU3050) || defined(WMP)
  #define GYRO 1
#else
  #define GYRO 0
#endif

#if defined(BMP085) || defined(MS561101BA)
  #define BARO 1
#else
  #define BARO 0
#endif

#if defined(GPS_SERIAL)  || defined(I2C_GPS) || defined(GPS_FROM_OSD) || defined(TINY_GPS)
  #define GPS 1
#else
  #define GPS 0
#endif

#if defined(SRF02) || defined(SRF08) || defined(SRF10) || defined(SRC235) || defined(TINY_GPS_SONAR)
  #define SONAR 1
#else
  #define SONAR 0
#endif


/**************************************************************************************/
/***************      Multitype decleration for the GUI's          ********************/
/**************************************************************************************/
#if defined(TRI)
  #define MULTITYPE 1
#elif defined(QUADP)
  #define MULTITYPE 2
#elif defined(QUADX)
  #define MULTITYPE 3
#elif defined(BI)
  #define MULTITYPE 4
#elif defined(GIMBAL)
  #define MULTITYPE 5
#elif defined(Y6)
  #define MULTITYPE 6
#elif defined(HEX6)
  #define MULTITYPE 7
#elif defined(FLYING_WING)
  #define MULTITYPE 8
#elif defined(Y4)
  #define MULTITYPE 9
#elif defined(HEX6X)
  #define MULTITYPE 10
#elif defined(OCTOX8)
  #define MULTITYPE 11   //the JAVA GUI is the same for all 8 motor configs 
#elif defined(OCTOFLATP)
  #define MULTITYPE 12   //12  for MultiWinGui
#elif defined(OCTOFLATX)
  #define MULTITYPE 13   //13  for MultiWinGui 
#elif defined(AIRPLANE)|| defined(SINGLECOPTER)|| defined(DUALCOPTER)    
  #define MULTITYPE 14    
#elif defined (HELI_120_CCPM)   
  #define MULTITYPE 15      
#elif defined (HELI_90_DEG)   
  #define MULTITYPE 16      
#elif defined(VTAIL4)
 #define MULTITYPE 17
#elif defined(HEX6H)
 #define MULTITYPE 18
#endif

/**************************************************************************************/
/***************          Some unsorted "chain" defines            ********************/
/**************************************************************************************/

#if defined (AIRPLANE) || defined(HELICOPTER)|| defined(SINGLECOPTER)|| defined(DUALCOPTER) && defined(PROMINI) 
  #if defined(D12_POWER)
    #define SERVO_4_PINMODE            ;  // D12
    #define SERVO_4_PIN_HIGH           ;
    #define SERVO_4_PIN_LOW            ;
  #else
    #define POWERPIN_PINMODE           ;
    #define POWERPIN_ON                ;
    #define POWERPIN_OFF               ;
  #endif
#endif

#if defined(POWERMETER_HARD) || defined(POWERMETER_SOFT)
  #define POWERMETER
#endif

#if defined PILOTLAMP 
  #define    PL_CHANNEL OCR0B  //use B since A can be used by camstab
  #define    PL_ISR TIMER0_COMPB_vect
  #define    PL_INIT   TCCR0A=0;TIMSK0|=(1<<OCIE0B);PL_CHANNEL=PL_IDLE;PilotLamp(PL_GRN_OFF);PilotLamp(PL_BLU_OFF);PilotLamp(PL_RED_OFF);PilotLamp(PL_BZR_OFF);
  #define    BUZZERPIN_ON PilotLamp(PL_BZR_ON);
  #define    BUZZERPIN_OFF PilotLamp(PL_BZR_OFF);
  #define    PL_GRN_ON    25    // 100us
  #define    PL_GRN_OFF   50    // 200us
  #define    PL_BLU_ON    75    // 300us
  #define    PL_BLU_OFF   100    // 400us
  #define    PL_RED_ON    125    // 500us
  #define    PL_RED_OFF   150    // 600us
  #define    PL_BZR_ON    175    // 700us
  #define    PL_BZR_OFF   200    // 800us
  #define    PL_IDLE      125    // 100us
#endif

#if defined(PILOTLAMP) || defined(VBAT)
  #define BUZZER
#endif

//all new Special RX's must be added here
//this is to avoid confusion :)
#if !defined(SERIAL_SUM_PPM) && !defined(SBUS) && !defined(RCSERIAL)
  #define STANDARD_RX
#endif

#if defined(SBUS)
  #define RC_CHANS 18
#elif defined(SERIAL_SUM_PPM)
  #define RC_CHANS 12
#else
  #define RC_CHANS 8
#endif

#if !defined(ALT_HOLD_THROTTLE_NEUTRAL_ZONE)
  #define ALT_HOLD_THROTTLE_NEUTRAL_ZONE 40
#endif 

/**************************************************************************************/
/***************               override default pin assignments ?  ********************/
/**************************************************************************************/
#ifdef OVERRIDE_V_BATPIN
  #define V_BATPIN OVERRIDE_V_BATPIN
#endif
#ifdef OVERRIDE_LEDPIN_PINMODE
  #define LEDPIN_PINMODE OVERRIDE_LEDPIN_PINMODE
  #define LEDPIN_TOGGLE  OVERRIDE_LEDPIN_TOGGLE
  #define LEDPIN_OFF     OVERRIDE_LEDPIN_OFF
  #define LEDPIN_ON      OVERRIDE_LEDPIN_ON
#endif
#ifdef OVERRIDE_BUZZERPIN_PINMODE
  #define BUZZERPIN_PINMODE OVERRIDE_BUZZERPIN_PINMODE
  #define BUZZERPIN_ON      OVERRIDE_BUZZERPIN_ON
  #define BUZZERPIN_OFF     OVERRIDE_BUZZERPIN_OFF
#endif

/**************************************************************************************/
/********* enforce your sensors orientation - possibly overriding board defaults  *****/
/**************************************************************************************/
#ifdef FORCE_GYRO_ORIENTATION
  #define GYRO_ORIENTATION FORCE_GYRO_ORIENTATION
#endif
#ifdef FORCE_ACC_ORIENTATION
  #define ACC_ORIENTATION FORCE_ACC_ORIENTATION
#endif
#ifdef FORCE_MAG_ORIENTATION
  #define MAG_ORIENTATION FORCE_MAG_ORIENTATION
#endif

/**************************************************************************************/
/***************               Error Checking Section              ********************/
/**************************************************************************************/

#ifndef NUMBER_MOTOR
        #error "NUMBER_MOTOR is not set, most likely you have not defined any type of multicopter"
#endif

#if (defined(LCD_DUMMY) || defined(LCD_SERIAL3W) || defined(LCD_TEXTSTAR) || defined(LCD_VT100) || defined(LCD_TTY) || defined(LCD_ETPP) || defined(LCD_LCD03) || defined(OLED_I2C_128x64) )
  #define HAS_LCD
#endif

#if (defined(LCD_CONF) || defined(LCD_TELEMETRY)) && !(defined(HAS_LCD) )
  #error "LCD_CONF or LCD_TELEMETRY defined, and choice of LCD not defined.  Uncomment one of LCD_SERIAL3W, LCD_TEXTSTAR, LCD_VT100, LCD_TTY or LCD_ETPP, LCD_LCD03, OLED_I2C_128x64"
#endif

#if defined(POWERMETER) && !(defined(VBAT))
        #error "to use powermeter, you must also define and configure VBAT"
#endif

#if defined(LCD_TELEMETRY_AUTO) && !(defined(LCD_TELEMETRY))
        #error "to use automatic telemetry, you MUST also define and configure LCD_TELEMETRY"
#endif

#if defined(LCD_TELEMETRY_STEP) && !(defined(LCD_TELEMETRY))
        #error "to use single step telemetry, you MUST also define and configure LCD_TELEMETRY"
#endif
#endif	/* _DEF_H */
