static uint8_t cycleDone[5]={0,0,0,0,0}, 
               resourceIsOn[5] = {0,0,0,0,0};
static uint32_t LastToggleTime[5] ={0,0,0,0,0};
static int16_t  i2c_errors_count_old = 0;

static uint8_t SequenceActive[5]={0,0,0,0,0};

#if defined(BUZZER)
  uint8_t isBuzzerON() { return resourceIsOn[1]; } // returns true while buzzer is buzzing; returns 0 for silent periods
#else
  uint8_t isBuzzerON() { return 0; }
#endif  //end of buzzer define
/********************************************************************/
/****                      Alarm Handling                        ****/
/********************************************************************/
/*
AlarmArray
0: toggle
1: failsafe
2: noGPS
3: beeperOn
4: pMeter
5: runtime
6: vBat
7: confirmation
8: Acc
9: I2C Error
*/
/*
Resources:
0: onboard LED
1: Buzzer
2: PL GREEN
3: PL BLUE
4: PL RED
*/
void alarmHandler(){
  
  #if defined(FAILSAFE)
    if ( failsafeCnt > (5*FAILSAFE_DELAY) && f.ARMED) {
      alarmArray[1] = 1;                                                                   //set failsafe warning level to 1 while landing
      if (failsafeCnt > 5*(FAILSAFE_DELAY+FAILSAFE_OFF_DELAY)) alarmArray[1] = 2;          //start "find me" signal after landing   
    }
    if ( failsafeCnt > (5*FAILSAFE_DELAY) && !f.ARMED) alarmArray[1] = 2;                  // tx turned off while motors are off: start "find me" signal
    if ( failsafeCnt == 0) alarmArray[1] = 0;                                              // turn off alarm if TX is okay
  #endif
  
  #if defined(BUZZER)
    if ( rcOptions[BOXBEEPERON] )alarmArray[3] = 1;
    else alarmArray[3] = 0;
  #endif

  #if defined(VBAT)
    if (vbatMin < conf.vbatlevel_crit) alarmArray[6] = 4;
    else if ( (vbat>conf.vbatlevel_warn1)  || (conf.no_vbat > vbat)) alarmArray[6] = 0;
    else if (vbat > conf.vbatlevel_warn2) alarmArray[6] = 2;
    else alarmArray[6] = 4;
  #endif
  
  if (i2c_errors_count > i2c_errors_count_old+100 || i2c_errors_count < -1) alarmArray[9] = 1;
  else alarmArray[9] = 0;
   
  alarmPatternComposer();
}

void alarmPatternComposer(){ 
  static char resource = 0;
  // patternDecode(length1,length2,length3,beeppause,endpause,loop)
  #if defined(BUZZER)
    resource = 1;                                                                                  //buzzer selected
    if (alarmArray[1] == 2)       patternDecode(resource,200,0,0,50,2000);                       //failsafe "find me" signal
    else if (alarmArray[1] == 1 || alarmArray[8] == 1) patternDecode(resource,50,200,200,50,50); //failsafe "panic"  or Acc not calibrated                     
    else if (alarmArray[0] == 1)  patternDecode(resource,50,0,0,50,0);                           //toggle 1
    else if (alarmArray[0] == 2)  patternDecode(resource,50,50,0,50,0);                          //toggle 2       
    else if (alarmArray[0] > 2)   patternDecode(resource,50,50,50,50,0);                         //toggle else         
    else if (alarmArray[2] == 2)  patternDecode(resource,50,50,0,50,50);                         //gps installed but no fix    
    else if (alarmArray[3] == 1)  patternDecode(resource,50,50,50,50,50);                        //BeeperOn
    else if (alarmArray[4] == 1)  patternDecode(resource,50,50,0,50,120);                        //pMeter Warning
    else if (alarmArray[5] == 1)  patternDecode(resource,50,50,50,50,0);                         //Runtime warning      
    else if (alarmArray[6] == 4)  patternDecode(resource,50,50,200,50,2000);                     //vbat critical
    else if (alarmArray[6] == 2)  patternDecode(resource,50,200,0,50,2000);                      //vbat warning      
    else if (alarmArray[6] == 1)  patternDecode(resource,200,0,0,50,2000);                       //vbat info
    else if (alarmArray[7] == 1)  patternDecode(resource,200,0,0,50,200);                        //confirmation indicator 1x
    else if (alarmArray[7] == 2)  patternDecode(resource,200,200,0,50,200);                      //confirmation indicator 2x 
    else if (alarmArray[7] > 2)   patternDecode(resource,200,200,200,50,200);                    //confirmation indicator 3x
    else if (SequenceActive[(uint8_t)resource] == 1) patternDecode(resource,0,0,0,0,0);                   // finish last sequence if not finished yet
    else turnOff(resource);                                                                        // turn off the resource 
    alarmArray[8] = 0;                                                                             //reset acc not calibrated
    
  #endif
}

void patternDecode(uint8_t resource,uint16_t first,uint16_t second,uint16_t third,uint16_t cyclepause, uint16_t endpause){
  static uint16_t pattern[5][5];
  static uint8_t icnt[5] = {0,0,0,0,0};
  
  if(SequenceActive[resource] == 0){
    SequenceActive[resource] = 1; 
    pattern[resource][0] = first; 
    pattern[resource][1] = second;
    pattern[resource][2] = third;
    pattern[resource][3] = endpause;
    pattern[resource][4] = cyclepause;
  }
  if(icnt[resource] <3 ){
    if (pattern[resource][icnt[resource]] != 0){
      setTiming(resource,pattern[resource][icnt[resource]],pattern[resource][4]);
     }
  }
  else if (LastToggleTime[resource] < (millis()-pattern[resource][3]))  {  //sequence is over: reset everything
    icnt[resource]=0;
    SequenceActive[resource] = 0;                               //sequence is now done, cycleDone sequence may begin
    alarmArray[0] = 0;                                //reset toggle bit
    alarmArray[7] = 0;                                //reset confirmation bit
    turnOff(resource);   
    return;
  }
  if (cycleDone[resource] == 1 || pattern[resource][icnt[resource]] == 0) {            //single on off cycle is done
    if (icnt[resource] < 3) {
      icnt[resource]++;
    }
    cycleDone[resource] = 0;
    turnOff(resource);    
  }  
}

void turnOff(uint8_t resource){
  if (resource == 1) {
    if (resourceIsOn[1]) {
      BUZZERPIN_OFF;
      resourceIsOn[1] = 0;
    }
  }else if (resource == 0) {
    if (resourceIsOn[0]) {
      resourceIsOn[0] = 0;
      LEDPIN_OFF;
    }
  }else if (resource == 2) {
    if (resourceIsOn[2]) {
      resourceIsOn[2] = 0;
      #if defined (PILOTLAMP)
        PilotLamp(PL_GRN_OFF);
      #endif
    }
  }else if (resource == 3) {
    if (resourceIsOn[3]) {
      resourceIsOn[3] = 0;
      #if defined (PILOTLAMP)
        PilotLamp(PL_BLU_OFF);
      #endif
    }
  }else if (resource == 4) {
    if (resourceIsOn[4]) {
      resourceIsOn[4] = 0;
      #if defined (PILOTLAMP)
        PilotLamp(PL_RED_OFF);
      #endif
    }
  }
}

/********************************************************************/
/****                         LED Handling                       ****/
/********************************************************************/
//Beware!! Is working with delays, do not use inflight!

void blinkLED(uint8_t num, uint8_t ontime,uint8_t repeat) {
  uint8_t i,r;
  for (r=0;r<repeat;r++) {
    for(i=0;i<num;i++) {
      LEDPIN_TOGGLE; // switch LEDPIN state
      delay(ontime);
    }
    delay(60); //wait 60 ms
  }
}

/********************************************************************/
/****                   Global Resource Handling                 ****/
/********************************************************************/

  void setTiming(uint8_t resource, uint16_t pulse, uint16_t pause){
    if (!resourceIsOn[resource] && (millis() >= (LastToggleTime[resource] + pause))&& pulse != 0) {
      resourceIsOn[resource] = 1;      
      toggleResource(resource,1);
      LastToggleTime[resource]=millis();      
    } else if ( (resourceIsOn[resource] && (millis() >= LastToggleTime[resource] + pulse) ) || (pulse==0 && resourceIsOn[resource]) ) {
      resourceIsOn[resource] = 0;
      toggleResource(resource,0);
      LastToggleTime[resource]=millis();
      cycleDone[resource] = 1;     
    } 
  } 
 
  void toggleResource(uint8_t resource, uint8_t activate){
     switch(resource) {     
        #if defined (BUZZER)   
          case 1:
            if (activate == 1) {BUZZERPIN_ON;}
            else BUZZERPIN_OFF;
            break; 
        #endif
          case 0:
          default:
            if (activate == 1) {LEDPIN_ON;}
            else LEDPIN_OFF;
            break;
      }
      return;
  }

