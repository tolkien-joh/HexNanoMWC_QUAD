#include "extra.h"

volatile uint8_t serialHeadRX[UART_NUMBER],serialTailRX[UART_NUMBER];
uint8_t serialBufferRX[RX_BUFFER_SIZE][UART_NUMBER];
static volatile uint8_t serialHeadTX[UART_NUMBER],serialTailTX[UART_NUMBER];
static uint8_t serialBufferTX[TX_BUFFER_SIZE][UART_NUMBER];
static uint8_t inBuf[INBUF_SIZE][UART_NUMBER];

#define BIND_CAPABLE 0;  //Used for Spektrum today; can be used in the future for any RX type that needs a bind and has a MultiWii module. 
// Capability is bit flags; next defines should be 2, 4, 8...

const uint32_t PROGMEM capability = 0+BIND_CAPABLE;

#ifdef DEBUGMSG
  #define DEBUG_MSG_BUFFER_SIZE 128
  static char debug_buf[DEBUG_MSG_BUFFER_SIZE];
  static uint8_t head_debug;
  static uint8_t tail_debug;
#endif

// Multiwii Serial Protocol 0 
#define MSP_VERSION              0

//to multiwii developpers/committers : do not add new MSP messages without a proper argumentation/agreement on the forum
#define MSP_IDENT                100   //out message         multitype + multiwii version + protocol version + capability variable
#define MSP_STATUS               101   //out message         cycletime & errors_count & sensor present & box activation & current setting number
#define MSP_RAW_IMU              102   //out message         9 DOF
#define MSP_SERVO                103   //out message         8 servos
#define MSP_MOTOR                104   //out message         8 motors
#define MSP_RC                   105   //out message         8 rc chan and more
#define MSP_RAW_GPS              106   //out message         fix, numsat, lat, lon, alt, speed, ground course
#define MSP_COMP_GPS             107   //out message         distance home, direction home
#define MSP_ATTITUDE             108   //out message         2 angles 1 heading
#define MSP_ALTITUDE             109   //out message         altitude, variometer
#define MSP_ANALOG               110   //out message         vbat, powermetersum, rssi if available on RX
#define MSP_RC_TUNING            111   //out message         rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_PID                  112   //out message         P I D coeff (9 are used currently)
#define MSP_BOX                  113   //out message         BOX setup (number is dependant of your setup)
#define MSP_MISC                 114   //out message         powermeter trig
#define MSP_MOTOR_PINS           115   //out message         which pins are in use for motors & servos, for GUI 
#define MSP_BOXNAMES             116   //out message         the aux switch names
#define MSP_PIDNAMES             117   //out message         the PID names
#define MSP_WP                   118   //out message         get a WP, WP# is in the payload, returns (WP#, lat, lon, alt, flags) WP#0-home, WP#16-poshold
#define MSP_BOXIDS               119   //out message         get the permanent IDs associated to BOXes

#if defined(HEX_NANO)
#define MSP_SET_RAW_RC_TINY      150   //in message          4 rc chan
#define MSP_ARM                  151
#define MSP_DISARM               152
#define MSP_TRIM_UP              153
#define MSP_TRIM_DOWN            154
#define MSP_TRIM_LEFT            155
#define MSP_TRIM_RIGHT           156
#define MSP_TRIM_UP_FAST         157
#define MSP_TRIM_DOWN_FAST       158
#define MSP_TRIM_LEFT_FAST       159
#define MSP_TRIM_RIGHT_FAST      160

#define MSP_READ_TEST_PARAM      189
#define MSP_SET_TEST_PARAM       190

#define MSP_READ_TEST_PARAM      189
#define MSP_HEX_NANO             199
#endif

#define MSP_SET_RAW_RC           200   //in message          8 rc chan
#define MSP_SET_RAW_GPS          201   //in message          fix, numsat, lat, lon, alt, speed
#define MSP_SET_PID              202   //in message          P I D coeff (9 are used currently)
#define MSP_SET_BOX              203   //in message          BOX setup (number is dependant of your setup)
#define MSP_SET_RC_TUNING        204   //in message          rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_ACC_CALIBRATION      205   //in message          no param
#define MSP_MAG_CALIBRATION      206   //in message          no param
#define MSP_SET_MISC             207   //in message          powermeter trig + 8 free for future use
#define MSP_RESET_CONF           208   //in message          no param
#define MSP_SET_WP               209   //in message          sets a given WP (WP#,lat, lon, alt, flags)
#define MSP_SELECT_SETTING       210   //in message          Select Setting Number (0-2)
#define MSP_SET_HEAD             211   //in message          define a new heading hold direction

#define MSP_BIND                 240   //in message          no param

#define MSP_EEPROM_WRITE         250   //in message          no param

#define MSP_DEBUGMSG             253   //out message         debug string buffer
#define MSP_DEBUG                254   //out message         debug1,debug2,debug3,debug4

static uint8_t checksum[UART_NUMBER];
static uint8_t indRX[UART_NUMBER];
static uint8_t cmdMSP[UART_NUMBER];

static uint8_t CURRENTPORT=0;

uint32_t read32() {
  uint32_t t = read16();
  t+= (uint32_t)read16()<<16;
  return t;
}
uint16_t read16() {
  uint16_t t = read8();
  t+= (uint16_t)read8()<<8;
  return t;
}
uint8_t read8()  {
  return inBuf[indRX[CURRENTPORT]++][CURRENTPORT]&0xff;
}

void headSerialResponse(uint8_t err, uint8_t s) {
  serialize8('$');
  serialize8('M');
  serialize8(err ? '!' : '>');
  checksum[CURRENTPORT] = 0; // start calculating a new checksum
  serialize8(s);
  serialize8(cmdMSP[CURRENTPORT]);
}

void headSerialReply(uint8_t s) {
  headSerialResponse(0, s);
}

void inline headSerialError(uint8_t s) {
  headSerialResponse(1, s);
}

void tailSerialReply() {
  serialize8(checksum[CURRENTPORT]);UartSendData();
}

void serializeNames(PGM_P s) {
  for (PGM_P c = s; pgm_read_byte(c); c++) {
    serialize8(pgm_read_byte(c));
  }
}

void serialCom() {
  uint8_t c,n;  
  static uint8_t offset[UART_NUMBER];
  static uint8_t dataSize[UART_NUMBER];
  static enum _serial_state {
    IDLE,
    HEADER_START,
    HEADER_M,
    HEADER_ARROW,
    HEADER_SIZE,
    HEADER_CMD,
  } c_state[UART_NUMBER];// = IDLE;

  for(n=0;n<UART_NUMBER;n++) {
    #if !defined(PROMINI)
      CURRENTPORT=n;
    #endif
    while (SerialAvailable(CURRENTPORT)) {
      uint8_t bytesTXBuff = ((uint8_t)(serialHeadTX[CURRENTPORT]-serialTailTX[CURRENTPORT]))%TX_BUFFER_SIZE; // indicates the number of occupied bytes in TX buffer
      if (bytesTXBuff > TX_BUFFER_SIZE - 50 ) return; // ensure there is enough free TX buffer to go further (50 bytes margin)
      c = SerialRead(CURRENTPORT);
      #ifdef SUPPRESS_ALL_SERIAL_MSP
        // no MSP handling, so go directly
        evaluateOtherData(c);
      #else
        // regular data handling to detect and handle MSP and other data
        if (c_state[CURRENTPORT] == IDLE) {
          c_state[CURRENTPORT] = (c=='$') ? HEADER_START : IDLE;
          if (c_state[CURRENTPORT] == IDLE) evaluateOtherData(c); // evaluate all other incoming serial data
        } else if (c_state[CURRENTPORT] == HEADER_START) {
          c_state[CURRENTPORT] = (c=='M') ? HEADER_M : IDLE;
        } else if (c_state[CURRENTPORT] == HEADER_M) {
          c_state[CURRENTPORT] = (c=='<') ? HEADER_ARROW : IDLE;
        } else if (c_state[CURRENTPORT] == HEADER_ARROW) {
          if (c > INBUF_SIZE) {  // now we are expecting the payload size
            c_state[CURRENTPORT] = IDLE;
            continue;
          }
          dataSize[CURRENTPORT] = c;
          offset[CURRENTPORT] = 0;
          checksum[CURRENTPORT] = 0;
          indRX[CURRENTPORT] = 0;
          checksum[CURRENTPORT] ^= c;
          c_state[CURRENTPORT] = HEADER_SIZE;  // the command is to follow
        } else if (c_state[CURRENTPORT] == HEADER_SIZE) {
          cmdMSP[CURRENTPORT] = c;
          checksum[CURRENTPORT] ^= c;
          c_state[CURRENTPORT] = HEADER_CMD;
        } else if (c_state[CURRENTPORT] == HEADER_CMD && offset[CURRENTPORT] < dataSize[CURRENTPORT]) {
          checksum[CURRENTPORT] ^= c;
          inBuf[offset[CURRENTPORT]++][CURRENTPORT] = c;
        } else if (c_state[CURRENTPORT] == HEADER_CMD && offset[CURRENTPORT] >= dataSize[CURRENTPORT]) {
          if (checksum[CURRENTPORT] == c) {  // compare calculated and transferred checksum
            evaluateCommand();  // we got a valid packet, evaluate it
          }
          c_state[CURRENTPORT] = IDLE;
        }
      #endif // SUPPRESS_ALL_SERIAL_MSP
    }
  }
}
#ifndef SUPPRESS_ALL_SERIAL_MSP
void evaluateCommand() {
  #if defined(HEX_NANO)
  unsigned char auxChannels;
  unsigned char aux;
  #endif
  
  switch(cmdMSP[CURRENTPORT]) {
   case MSP_SET_RAW_RC:
     for(uint8_t i=0;i<8;i++) {
       rcData[i] = read16();
     }
     
     failsafeCnt = 0;
     
     headSerialReply(0);
     break;
 
   #if defined(HEX_NANO)
   case MSP_READ_TEST_PARAM:
     headSerialReply(12);
     
     blinkLED(15,20,1);
    
     paramList[0] = alpha * 250.f;
     paramList[1] = conf.P8[PIDALT] * 250.f / 200;
     paramList[2] = conf.I8[PIDALT];
     paramList[3] = conf.D8[PIDALT] * 250.f / 100;
     
     for(int idx = 0; idx < 12; idx++){
       serialize8(paramList[idx]);
     } 
     
     break;
   case MSP_SET_TEST_PARAM:
     for(int idx = 0; idx < 12; idx++){
       paramList[idx] = read8();
     }
     
     blinkLED(15,20,1);
     
     alpha = paramList[0] / 250.f;
     conf.P8[PIDALT] = paramList[1] / 250.f * 200;   //0~200
     conf.I8[PIDALT] = paramList[2];                 //0~250
     conf.D8[PIDALT] = paramList[3] / 250.f * 100;   //0~100
     writeParams(0);
     return;
     break;
   case MSP_SET_RAW_RC_TINY:
     for(uint8_t i = 0;i < 4;i++) {
       serialRcValue[i] = 1000 + read8() * 4;
     }
     
     auxChannels = read8();
     
     aux = (auxChannels & 0xc0) >> 6;
     
     if(aux == 0){
       serialRcValue[4] = 1000;
     }
     else if(aux == 1){
       serialRcValue[4] = 1500;
     }
     else{
       serialRcValue[4] = 2000;
     }
     
     
     aux = (auxChannels & 0x30) >> 4;
     
     if(aux == 0){
       serialRcValue[5] = 1000;
     }
     else if(aux == 1){
       serialRcValue[5] = 1500;
     }
     else{
       serialRcValue[5] = 2000;
     }
     
     
     aux = (auxChannels & 0x0c) >> 2;
     
     if(aux == 0){
       serialRcValue[6] = 1000;
     }
     else if(aux == 1){
       serialRcValue[6] = 1500;
     }
     else{
       serialRcValue[6] = 2000;
     }
     
     aux = (auxChannels & 0x03);
     
     if(aux == 0){
       serialRcValue[7] = 1000;
     }
     else if(aux == 1){
       serialRcValue[7] = 1500;
     }
     else{
       serialRcValue[7] = 2000;
     }
     
     failsafeCnt = 0;
     
          
     return;
     
     /*
     headSerialReply(8);
     
     for(uint8_t i = 0; i < 4; i++) {
       serialize16(serialRcValue[i]);
     }*/
     break;
   case MSP_ARM:
     go_arm();
     break;
   case MSP_DISARM:
     go_disarm();
     break;
   case MSP_TRIM_UP:
     if(conf.angleTrim[PITCH] < 120){
       conf.angleTrim[PITCH]+=1;  
       writeParams(1);
     }
     break;
   case MSP_TRIM_DOWN:
     if(conf.angleTrim[PITCH] > -120){
       conf.angleTrim[PITCH]-=1; 
       writeParams(1);
     }
     break;
   case MSP_TRIM_LEFT:
     if(conf.angleTrim[ROLL] > -120){
       conf.angleTrim[ROLL]-=1; 
       writeParams(1);
     }
     break;
   case MSP_TRIM_RIGHT:
     if(conf.angleTrim[ROLL] < 120){
       conf.angleTrim[ROLL]+=1; 
       writeParams(1);
     }
     break;
   case MSP_TRIM_UP_FAST:
     if(conf.angleTrim[PITCH] < 120){
       conf.angleTrim[PITCH]+=10;  
       writeParams(1);
     }
     break;
   case MSP_TRIM_DOWN_FAST:
     if(conf.angleTrim[PITCH] > -120){
       conf.angleTrim[PITCH]-=10; 
       writeParams(1);
     }
     break;
   case MSP_TRIM_LEFT_FAST:
     if(conf.angleTrim[ROLL] > -120){
       conf.angleTrim[ROLL]-=10; 
       writeParams(1);
     }
     break;
   case MSP_TRIM_RIGHT_FAST:
     if(conf.angleTrim[ROLL] < 120){
       conf.angleTrim[ROLL]+=10; 
       writeParams(1);
     }
     break;
   case MSP_HEX_NANO:
     headSerialReply(14);
     serialize8(flightState);
     serialize16(absolutedAccZ);
     //serialize32(EstAlt);
     serialize16((int16_t)EstAlt);
     for(uint8_t i=0;i<2;i++) serialize16(angle[i]);
     serialize16((int16_t)AltHold);
     serialize8(vbat);
     serialize8((int8_t)(conf.angleTrim[PITCH]));
     serialize8((int8_t)(conf.angleTrim[ROLL]));
     break;
   #endif
   case MSP_SET_PID:
     for(uint8_t i=0;i<PIDITEMS;i++) {
       conf.P8[i]=read8();
       conf.I8[i]=read8();
       conf.D8[i]=read8();
     }
     headSerialReply(0);
     break;
   case MSP_SET_BOX:
     for(uint8_t i=0;i<CHECKBOXITEMS;i++) {
       conf.activate[i]=read16();
     }
     headSerialReply(0);
     break;
   case MSP_SET_RC_TUNING:
     conf.rcRate8 = read8();
     conf.rcExpo8 = read8();
     conf.rollPitchRate = read8();
     conf.yawRate = read8();
     conf.dynThrPID = read8();
     conf.thrMid8 = read8();
     conf.thrExpo8 = read8();
     headSerialReply(0);
     break;
   case MSP_SET_MISC:
     headSerialReply(0);
     break;
   #ifdef MULTIPLE_CONFIGURATION_PROFILES
   case MSP_SELECT_SETTING:
     if(!f.ARMED) {
       global_conf.currentSet = read8();
       if(global_conf.currentSet>2) global_conf.currentSet = 0;
       writeGlobalSet(0);
       readEEPROM();
     }
     headSerialReply(0);
     break;
   #endif
   case MSP_SET_HEAD:
     magHold = read16();
     headSerialReply(0);
     break;
   case MSP_IDENT:
     headSerialReply(7);
     serialize8(VERSION);   // multiwii version
     serialize8(MULTITYPE); // type of multicopter
     serialize8(MSP_VERSION);         // MultiWii Serial Protocol Version
     serialize32(pgm_read_dword(&(capability)));        // "capability"
     break;
   case MSP_STATUS:
     headSerialReply(11);
     serialize16(cycleTime);
     serialize16(i2c_errors_count);
     serialize16(ACC|BARO<<1|MAG<<2|GPS<<3|SONAR<<4);
     serialize32(
                 #if ACC
                   f.ANGLE_MODE<<BOXANGLE|
                   f.HORIZON_MODE<<BOXHORIZON|
                 #endif
                 #if defined(BUZZER)
                   rcOptions[BOXBEEPERON]<<BOXBEEPERON|
                 #endif
                 #if defined(INFLIGHT_ACC_CALIBRATION)
                   rcOptions[BOXCALIB]<<BOXCALIB |
                 #endif
                 f.ARMED<<BOXARM);
       serialize8(global_conf.currentSet);   // current setting
     break;
   case MSP_RAW_IMU:
     headSerialReply(18);
     for(uint8_t i=0;i<3;i++) serialize16(accSmooth[i]);
     for(uint8_t i=0;i<3;i++) serialize16(gyroData[i]);
     for(uint8_t i=0;i<3;i++) serialize16(magADC[i]);
     break;
   case MSP_SERVO:
     headSerialReply(16);
     for(uint8_t i=0;i<8;i++)
       serialize16(0);
     break;
   case MSP_MOTOR:
     headSerialReply(16);
     for(uint8_t i=0;i<8;i++) {
       serialize16( (i < NUMBER_MOTOR) ? motor[i] : 0 );
     }
     break;
   case MSP_RC:
     headSerialReply(RC_CHANS * 2);
     for(uint8_t i=0;i<RC_CHANS;i++) serialize16(rcData[i]);
     break;
   case MSP_ATTITUDE:
     headSerialReply(8);
     for(uint8_t i=0;i<2;i++) serialize16(angle[i]);
     serialize16(heading);
     serialize16(headFreeModeHold);
     break;
   case MSP_ALTITUDE:
     headSerialReply(6);
     serialize32(EstAlt);
     serialize16(vario);                  // added since r1172
     break;
   case MSP_ANALOG:
     headSerialReply(5);
     serialize8(vbat);
     serialize16(intPowerMeterSum);
     serialize16(rssi);
     break;
   case MSP_RC_TUNING:
     headSerialReply(7);
     serialize8(conf.rcRate8);
     serialize8(conf.rcExpo8);
     serialize8(conf.rollPitchRate);
     serialize8(conf.yawRate);
     serialize8(conf.dynThrPID);
     serialize8(conf.thrMid8);
     serialize8(conf.thrExpo8);
     break;
   case MSP_PID:
     headSerialReply(3*PIDITEMS);
     for(uint8_t i=0;i<PIDITEMS;i++) {
       serialize8(conf.P8[i]);
       serialize8(conf.I8[i]);
       serialize8(conf.D8[i]);
     }
     break;
   case MSP_PIDNAMES:
     headSerialReply(strlen_P(pidnames));
     serializeNames(pidnames);
     break;
   case MSP_BOX:
     headSerialReply(2*CHECKBOXITEMS);
     for(uint8_t i=0;i<CHECKBOXITEMS;i++) {
       serialize16(conf.activate[i]);
     }
     break;
   case MSP_BOXNAMES:
     headSerialReply(strlen_P(boxnames));
     serializeNames(boxnames);
     break;
   case MSP_BOXIDS:
     headSerialReply(CHECKBOXITEMS);
     for(uint8_t i=0;i<CHECKBOXITEMS;i++) {
       serialize8(pgm_read_byte(&(boxids[i])));
     }
     break;
   case MSP_MISC:
     headSerialReply(2);
     serialize16(intPowerTrigger1);
     break;
   case MSP_MOTOR_PINS:
     headSerialReply(8);
     for(uint8_t i=0;i<8;i++) {
       serialize8(PWM_PIN[i]);
     }
     break;
   #if defined(USE_MSP_WP)    
   case MSP_WP:
     {
       int32_t lat = 0,lon = 0;
       uint8_t wp_no = read8();        //get the wp number  
       headSerialReply(18);
       if (wp_no == 0) {
         lat = GPS_home[LAT];
         lon = GPS_home[LON];
       } else if (wp_no == 16) {
         lat = GPS_hold[LAT];
         lon = GPS_hold[LON];
       }
       serialize8(wp_no);
       serialize32(lat);
       serialize32(lon);
       serialize32(AltHold);           //altitude (cm) will come here -- temporary implementation to test feature with apps
       serialize16(0);                 //heading  will come here (deg)
       serialize16(0);                 //time to stay (ms) will come here 
       serialize8(0);                  //nav flag will come here
     }
     break;
   case MSP_SET_WP:
     {
       int32_t lat = 0,lon = 0,alt = 0;
       uint8_t wp_no = read8();        //get the wp number
       lat = read32();
       lon = read32();
       alt = read32();                 // to set altitude (cm)
       read16();                       // future: to set heading (deg)
       read16();                       // future: to set time to stay (ms)
       read8();                        // future: to set nav flag
       if (wp_no == 0) {
         GPS_home[LAT] = lat;
         GPS_home[LON] = lon;
         f.GPS_HOME_MODE = 0;          // with this flag, GPS_set_next_wp will be called in the next loop -- OK with SERIAL GPS / OK with I2C GPS
         f.GPS_FIX_HOME  = 1;
         if (alt != 0) AltHold = alt;  // temporary implementation to test feature with apps
       } else if (wp_no == 16) {       // OK with SERIAL GPS  --  NOK for I2C GPS / needs more code dev in order to inject GPS coord inside I2C GPS
         GPS_hold[LAT] = lat;
         GPS_hold[LON] = lon;
         if (alt != 0) AltHold = alt;  // temporary implementation to test feature with apps
       }
     }
     headSerialReply(0);
     break;
   #endif
   case MSP_RESET_CONF:
     if(!f.ARMED) LoadDefaults();
     headSerialReply(0);
     break;
   case MSP_ACC_CALIBRATION:
     if(!f.ARMED) calibratingA=512;
     headSerialReply(0);
     break;
   case MSP_MAG_CALIBRATION:
     if(!f.ARMED) f.CALIBRATE_MAG = 1;
     headSerialReply(0);
     break;
   case MSP_EEPROM_WRITE:
     writeParams(0);
     headSerialReply(0);
     break;
   case MSP_DEBUG:
     headSerialReply(8);
     for(uint8_t i=0;i<4;i++) {
       serialize16(debug[i]); // 4 variables are here for general monitoring purpose
     }
     break;
   #ifdef DEBUGMSG
   case MSP_DEBUGMSG:
     {
       uint8_t size = debugmsg_available();
       if (size > 16) size = 16;
       headSerialReply(size);
       debugmsg_serialize(size);
     }
     break;
   #endif
   default:  // we do not know how to handle the (valid) message, indicate error MSP $M!
     headSerialError(0);
     break;
  }
  tailSerialReply();
}
#endif // SUPPRESS_ALL_SERIAL_MSP

// evaluate all other incoming serial data
void evaluateOtherData(uint8_t sr) {
  #ifndef SUPPRESS_OTHER_SERIAL_COMMANDS
    switch (sr) {
    // Note: we may receive weird characters here which could trigger unwanted features during flight.
    //       this could lead to a crash easily.
    //       Please use if (!f.ARMED) where neccessary
      #ifdef LOG_PERMANENT_SHOW_AT_L
        case 'L':
          if (!f.ARMED) dumpPLog(1);
          break;
        #endif
    }
  #endif // SUPPRESS_OTHER_SERIAL_COMMANDS
}

// *******************************************************
// Interrupt driven UART transmitter - using a ring buffer
// *******************************************************

void serialize32(uint32_t a) {
  serialize8((a    ) & 0xFF);
  serialize8((a>> 8) & 0xFF);
  serialize8((a>>16) & 0xFF);
  serialize8((a>>24) & 0xFF);
}

void serialize16(int16_t a) {
  serialize8((a   ) & 0xFF);
  serialize8((a>>8) & 0xFF);
}

void serialize8(uint8_t a) {
  uint8_t t = serialHeadTX[CURRENTPORT];
  if (++t >= TX_BUFFER_SIZE) t = 0;
  serialBufferTX[t][CURRENTPORT] = a;
  checksum[CURRENTPORT] ^= a;
  serialHeadTX[CURRENTPORT] = t;
}

  ISR(USART1_UDRE_vect) { // Serial 1 on a MEGA or on a PROMICRO
    uint8_t t = serialTailTX[1];
    if (serialHeadTX[1] != t) {
      if (++t >= TX_BUFFER_SIZE) t = 0;
      UDR1 = serialBufferTX[t][1];  // Transmit next byte in the ring
      serialTailTX[1] = t;
    }
    if (t == serialHeadTX[1]) UCSR1B &= ~(1<<UDRIE1);
  }

void UartSendData() {
  switch (CURRENTPORT) {
    case 0:
      while(serialHeadTX[0] != serialTailTX[0]) {
         if (++serialTailTX[0] >= TX_BUFFER_SIZE) serialTailTX[0] = 0;
           USB_Send(USB_CDC_TX,serialBufferTX[serialTailTX[0]],1);
       }
      break;
    case 1: UCSR1B |= (1<<UDRIE1); break;
  }
}

void SerialOpen(uint8_t port, uint32_t baud) {
  uint8_t h = ((F_CPU  / 4 / baud -1) / 2) >> 8;
  uint8_t l = ((F_CPU  / 4 / baud -1) / 2);
  switch (port) {
    // disable the USB frame interrupt of arduino (it causes strong jitter and we dont need it)
    case 0: UDIEN &= ~(1<<SOFE); break;
    case 1: UCSR1A  = (1<<U2X1); UBRR1H = h; UBRR1L = l; UCSR1B |= (1<<RXEN1)|(1<<TXEN1)|(1<<RXCIE1); break;
  }
}

/* move to extra.h
static void inline SerialEnd(uint8_t port);
static void inline store_uart_in_buf(uint8_t data, uint8_t portnum);
 */

  ISR(USART1_RX_vect)  { store_uart_in_buf(UDR1, 1); }

uint8_t SerialRead(uint8_t port) {
  if (port == 0) {
    USB_Flush(USB_CDC_TX);
    return USB_Recv(USB_CDC_RX);      
  }

  uint8_t t = serialTailRX[port];
  uint8_t c = serialBufferRX[t][port];
  if (serialHeadRX[port] != t) {
    if (++t >= RX_BUFFER_SIZE) t = 0;
    serialTailRX[port] = t;
  }
  return c;
}

uint8_t SerialAvailable(uint8_t port) {
  if(port == 0)
    return USB_Available(USB_CDC_RX);
  return (serialHeadRX[port] - serialTailRX[port])%RX_BUFFER_SIZE;
}

void SerialWrite(uint8_t port,uint8_t c) {
  CURRENTPORT=port;
  serialize8(c);UartSendData();
}

#ifdef DEBUGMSG
void debugmsg_append_str(const char *str) {
  while(*str) {
    debug_buf[head_debug++] = *str++;
    if (head_debug == DEBUG_MSG_BUFFER_SIZE) {
      head_debug = 0;
    }
  }
}

static uint8_t debugmsg_available() {
  if (head_debug >= tail_debug) {
    return head_debug-tail_debug;
  } else {
    return head_debug + (DEBUG_MSG_BUFFER_SIZE-tail_debug);
  }
}

static void debugmsg_serialize(uint8_t l) {
  for (uint8_t i=0; i<l; i++) {
    if (head_debug != tail_debug) {
      serialize8(debug_buf[tail_debug++]);
      if (tail_debug == DEBUG_MSG_BUFFER_SIZE) {
        tail_debug = 0;
      }
    } else {
      serialize8('\0');
    }
  }
}
#else
void debugmsg_append_str(const char *str) {};
#endif
