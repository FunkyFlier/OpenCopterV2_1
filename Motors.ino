void CheckESCFlag(){
  if (EEPROM.read(PWM_FLAG) != 0xAA){
    pwmHigh.val = 2000;
    pwmLow.val = 1000;
    EEPROM.write(PWM_LIM_HIGH_START,pwmHigh.buffer[0]);
    EEPROM.write(PWM_LIM_HIGH_END,pwmHigh.buffer[1]);
    EEPROM.write(PWM_LIM_LOW_START,pwmLow.buffer[0]);
    EEPROM.write(PWM_LIM_LOW_END,pwmLow.buffer[1]);
    EEPROM.write(PWM_FLAG,0xAA);
  }
  int16_u outInt16;
  outInt16.buffer[0] = EEPROM.read(PWM_LIM_HIGH_START);
  outInt16.buffer[1] = EEPROM.read(PWM_LIM_HIGH_END);
  pwmHigh.val = outInt16.val;
  if (pwmHigh.val > 2000){
    pwmHigh.val = 2000;
  }
  if (pwmHigh.val < 1900){
    pwmHigh.val = 1900;
  }
  outInt16.buffer[0] = EEPROM.read(PWM_LIM_LOW_START);
  outInt16.buffer[1] = EEPROM.read(PWM_LIM_LOW_END);
  pwmLow.val = outInt16.val;
  if (pwmLow.val < 1000){
    pwmLow.val = 1000;
  }
  if (pwmLow.val > 1200){
    pwmLow.val = 1200;
  }
  newRC = false;
  while(newRC == false){
    ProcessChannels();
  }
  newRC = false;

  if (EEPROM.read(ESC_CAL_FLAG) == 0xAA){
    digitalWrite(13,HIGH);
    digitalWrite(RED,HIGH);
    digitalWrite(YELLOW,HIGH);
    digitalWrite(GREEN,HIGH);
    while(RCValue[THRO] > 1100 || RCValue[AILE] > 1100 || RCValue[ELEV] > 1100 || RCValue[RUDD] > 1100){
      if (newRC == true){
        newRC = false;
        ProcessChannels();
      } 
    }
    DDRE |= B00111000;
    DDRH |= B00111000;
    DDRB |= B01100000;


    TCCR3A = (1<<WGM31)|(1<<COM3A1)|(1<<COM3B1)|(1<<COM3C1);  
    TCCR3B = (1<<WGM33)|(1<<WGM32)|(1<<CS31);               
    ICR3 = PERIOD;   

    TCCR4A = (1<<WGM41)|(1<<COM4A1)|(1<<COM4B1)|(1<<COM4C1);
    TCCR4B = (1<<WGM43)|(1<<WGM42)|(1<<CS41);
    ICR4 = PERIOD;  

    TCCR1A = (1<<WGM11)|(1<<COM1A1)|(1<<COM1B1);
    TCCR1B = (1<<WGM13)|(1<<WGM12)|(1<<CS11);
    ICR1 = PERIOD; 

    Motor1WriteMicros(pwmHigh.val);//set the output compare value
    Motor2WriteMicros(pwmHigh.val);
    Motor3WriteMicros(pwmHigh.val);
    Motor4WriteMicros(pwmHigh.val);
    Motor5WriteMicros(pwmHigh.val);
    Motor6WriteMicros(pwmHigh.val);
    Motor7WriteMicros(pwmHigh.val);
    Motor8WriteMicros(pwmHigh.val);

    delay(4000);
    Motor1WriteMicros(pwmLow.val);//set the output compare value
    Motor2WriteMicros(pwmLow.val);
    Motor3WriteMicros(pwmLow.val);
    Motor4WriteMicros(pwmLow.val);
    Motor5WriteMicros(pwmLow.val);
    Motor6WriteMicros(pwmLow.val);
    Motor7WriteMicros(pwmLow.val);
    Motor8WriteMicros(pwmLow.val);

    EEPROM.write(ESC_CAL_FLAG,0xFF);
    while(1){
      digitalWrite(13,LOW);
      digitalWrite(RED,HIGH);
      digitalWrite(YELLOW,HIGH);
      digitalWrite(GREEN,HIGH);
      delay(250);
      digitalWrite(13,HIGH);
      digitalWrite(RED,LOW);
      digitalWrite(YELLOW,HIGH);
      digitalWrite(GREEN,HIGH);
      delay(250);
      digitalWrite(13,HIGH);
      digitalWrite(RED,HIGH);
      digitalWrite(YELLOW,LOW);
      digitalWrite(GREEN,HIGH);
      delay(250);
      digitalWrite(13,HIGH);
      digitalWrite(RED,HIGH);
      digitalWrite(YELLOW,HIGH);
      digitalWrite(GREEN,LOW);
      delay(250);
    }
  }
}


void CalibrateESC(){
  delay(100);//wait for new frame

  while(newRC == false){

  }
  ProcessChannels();
  if (RCValue[THRO] > 1900){
    EEPROM.write(HS_FLAG,0xFF);//clear the handshake flag
    EEPROM.write(ESC_CAL_FLAG,0xAA);
    while(1){
      digitalWrite(13,HIGH);
      digitalWrite(RED,LOW);
      digitalWrite(YELLOW,LOW);
      digitalWrite(GREEN,LOW);
      delay(250);
      digitalWrite(13,LOW);
      digitalWrite(RED,HIGH);
      digitalWrite(YELLOW,LOW);
      digitalWrite(GREEN,LOW);
      delay(250);
      digitalWrite(13,LOW);
      digitalWrite(RED,LOW);
      digitalWrite(YELLOW,HIGH);
      digitalWrite(GREEN,LOW);
      delay(250);
      digitalWrite(13,LOW);
      digitalWrite(RED,LOW);
      digitalWrite(YELLOW,LOW);
      digitalWrite(GREEN,HIGH);
      delay(250);
    }

  }

}


void MotorInit(){
  DDRE |= B00111000;
  DDRH |= B00111000;
  DDRB |= B01100000;


  TCCR3A = (1<<WGM31)|(1<<COM3A1)|(1<<COM3B1)|(1<<COM3C1);  
  TCCR3B = (1<<WGM33)|(1<<WGM32)|(1<<CS31);               
  ICR3 = PERIOD;   

  TCCR4A = (1<<WGM41)|(1<<COM4A1)|(1<<COM4B1)|(1<<COM4C1);
  TCCR4B = (1<<WGM43)|(1<<WGM42)|(1<<CS41);
  ICR4 = PERIOD;  

  TCCR1A = (1<<WGM11)|(1<<COM1A1)|(1<<COM1B1);
  TCCR1B = (1<<WGM13)|(1<<WGM12)|(1<<CS11);
  ICR1 = PERIOD;


  Motor1WriteMicros(pwmLow.val);//set the output compare value
  Motor2WriteMicros(pwmLow.val);
  Motor3WriteMicros(pwmLow.val);
  Motor4WriteMicros(pwmLow.val);
  Motor5WriteMicros(pwmLow.val);
  Motor6WriteMicros(pwmLow.val);
  Motor7WriteMicros(pwmLow.val);
  Motor8WriteMicros(pwmLow.val);

}



void SaveGains(){
  uint16_t j_ = GAINS_START;
  for(uint16_t i = KP_PITCH_RATE_; i <= MAG_DEC_; i++){
    EEPROM.write(j_++,(*floatPointerArray[i]).buffer[0]); 
    EEPROM.write(j_++,(*floatPointerArray[i]).buffer[1]); 
    EEPROM.write(j_++,(*floatPointerArray[i]).buffer[2]); 
    EEPROM.write(j_++,(*floatPointerArray[i]).buffer[3]); 
    watchDogFailSafeCounter = 0;

  }
  imu.COS_DEC = cos(imu.declination.val);
  imu.SIN_DEC = sin(imu.declination.val);

  calibrationFlags = EEPROM.read(CAL_FLAGS);
  calibrationFlags &= ~(1<<GAINS_FLAG);
  EEPROM.write(CAL_FLAGS,calibrationFlags);
}
void MotorHandler(){

  switch(motorState){
  case HOLD:

    if (saveGainsFlag == true && (millis() - romWriteDelayTimer) > 2000){
      SaveGains();

      saveGainsFlag = false;
      imuTimer = micros();
      baroTimer = millis();
      _400HzTimer = imuTimer;
    }
    initialYaw.val = imu.yaw.val;
    integrate = false;
    HHState = 0;
    throttleAdjustment.val = 0;
    ZLoiterState = LOITERING;
    XYLoiterState = LOITERING;
    if (throCommand > 1100){
      motorCommand1.val = pwmLow.val;
      motorCommand2.val = pwmLow.val;
      motorCommand3.val = pwmLow.val;
      motorCommand4.val = pwmLow.val;
      motorCommand5.val = pwmLow.val;
      motorCommand6.val = pwmLow.val;
      motorCommand7.val = pwmLow.val;
      motorCommand8.val = pwmLow.val;
      break;
    }
    if (flightMode == RTB){
      motorState = HOLD;
      motorCommand1.val = pwmLow.val;
      motorCommand2.val = pwmLow.val;
      motorCommand3.val = pwmLow.val;
      motorCommand4.val = pwmLow.val;
      motorCommand5.val = pwmLow.val;
      motorCommand6.val = pwmLow.val;
      motorCommand7.val = pwmLow.val;
      motorCommand8.val = pwmLow.val;
      break;
    }

    if (cmdRudd < 1300){
      rudderFlag = true;

    }
    if (rudderFlag == true){

      if (abs(cmdRudd - 1500) < 50){
        rudderFlag = false;
        motorState = TO;


        PitchAngle.reset();
        RollAngle.reset();
        YawAngle.reset();

        PitchRate.reset();
        RollRate.reset();
        YawRate.reset();

        AltHoldPosition.reset();
        AltHoldVelocity.reset();

        WayPointPosition.reset();
        WayPointRate.reset();

        LoiterXPosition.reset();
        LoiterXVelocity.reset();

        LoiterYPosition.reset();
        LoiterYVelocity.reset();
        homeBaseXOffset = imu.XEst.val;
        homeBaseYOffset = imu.YEst.val;
        if (imu.magDetected == true){
          VerifyMag();
        }
        UpdateOffset();
      }
    }
    throttleAdjustment.val = 0;
    motorCommand1.val = pwmLow.val;
    motorCommand2.val = pwmLow.val;
    motorCommand3.val = pwmLow.val;
    motorCommand4.val = pwmLow.val;
    motorCommand5.val = pwmLow.val;
    motorCommand6.val = pwmLow.val;
    motorCommand7.val = pwmLow.val;
    motorCommand8.val = pwmLow.val;
    throttleCheckFlag = false;
    break;
  case TO:
    motorCommand1.val = propIdleCommand;
    motorCommand2.val = propIdleCommand;
    motorCommand3.val = propIdleCommand;
    motorCommand4.val = propIdleCommand;
    motorCommand5.val = propIdleCommand;
    motorCommand6.val = propIdleCommand;
    motorCommand7.val = propIdleCommand;
    motorCommand8.val = propIdleCommand;
    throttleCheckFlag = false;
    initialPressure = pressure.val;
    imu.ZEst.val = 0;
    imu.ZEstUp.val = 0;
    imu.velZ.val = 0;
    imu.velZUp.val = 0;
    prevBaro = 0;
    baroZ.val = 0;
    initialYaw.val = imu.yaw.val;

    if (cmdRudd > 1700){
      motorState = HOLD;
    }
    if (flightMode == RTB){
      motorState = HOLD;
    }

    if (flightMode == RATE || flightMode == ATT){
      if (throCommand > 1150 && throCommand < 1350){
        motorState = FLIGHT;
        integrate = true;
      }
    }
    if (flightMode <= L2 && flightMode >= L0){
      if (throCommand <= 1600 && throCommand >= 1500){
        motorState = FLIGHT;
        zTarget.val = TAKE_OFF_ALT;
        enterState = true;
        throttleAdjustment.val = 0;
        xTarget.val = imu.XEst.val;
        yTarget.val = imu.YEst.val;
        LoiterXPosition.reset();
        LoiterXVelocity.reset();
        LoiterYPosition.reset();
        LoiterYVelocity.reset();
        AltHoldPosition.reset();
        AltHoldVelocity.reset();
        integrate = true;
      }
    }
    if (flightMode == WP || flightMode == FOLLOW){
      if (throCommand <= 1600 && throCommand >= 1500){
        autoMaticReady = true;
      }
    }

    break;
  case FLIGHT:
    if (flightMode == RATE || flightMode == ATT){
      throttleAdjustment.val = 0;
      throttleCommand.val = throCommand;

      if (throttleCommand.val > 1900){
        throttleCommand.val = 1900;
      }
      if (throttleCommand.val < 1050){
        throttleCommand.val = propIdleCommand;
        if (cmdRudd > 1700){
          motorState = HOLD;
        }
        motorState = TO;

      }
    }
    if (flightMode >= L0){
      throttleCommand.val = hoverCommand;
    }
    if (throttleCheckFlag == true){
      if (throCommand <= 1600 && throCommand >= 1500){
        throttleCheckFlag = false;
        throttleCommand.val = hoverCommand;
      }
    }

    landingThroAdjustment.val = 0.997 * landingThroAdjustment.val + 0.003 * throttleAdjustment.val;
#ifdef QUAD
    motorCommand1.val = constrain((throttleCommand.val + throttleAdjustment.val + adjustmentX.val + adjustmentY.val - adjustmentZ.val),pwmLow.val,pwmHigh.val);
    motorCommand2.val = constrain((throttleCommand.val + throttleAdjustment.val - adjustmentX.val + adjustmentY.val + adjustmentZ.val),pwmLow.val,pwmHigh.val);
    motorCommand3.val = constrain((throttleCommand.val + throttleAdjustment.val - adjustmentX.val - adjustmentY.val - adjustmentZ.val),pwmLow.val,pwmHigh.val);
    motorCommand4.val = constrain((throttleCommand.val + throttleAdjustment.val + adjustmentX.val - adjustmentY.val + adjustmentZ.val),pwmLow.val,pwmHigh.val);
    motorCommand5.val = pwmLow.val;
    motorCommand6.val = pwmLow.val;
    motorCommand7.val = pwmLow.val;
    motorCommand8.val = pwmLow.val;
#endif
#ifdef X_8
    motorCommand1.val = constrain((throttleCommand.val + throttleAdjustment.val + adjustmentX.val + adjustmentY.val - adjustmentZ.val),pwmLow.val,pwmHigh.val);
    motorCommand2.val = constrain((throttleCommand.val + throttleAdjustment.val - adjustmentX.val + adjustmentY.val + adjustmentZ.val),pwmLow.val,pwmHigh.val);
    motorCommand3.val = constrain((throttleCommand.val + throttleAdjustment.val - adjustmentX.val - adjustmentY.val - adjustmentZ.val),pwmLow.val,pwmHigh.val);
    motorCommand4.val = constrain((throttleCommand.val + throttleAdjustment.val + adjustmentX.val - adjustmentY.val + adjustmentZ.val),pwmLow.val,pwmHigh.val);

    motorCommand5.val = constrain((throttleCommand.val + throttleAdjustment.val + adjustmentX.val + adjustmentY.val + adjustmentZ.val),pwmLow.val,pwmHigh.val);
    motorCommand6.val = constrain((throttleCommand.val + throttleAdjustment.val - adjustmentX.val + adjustmentY.val - adjustmentZ.val),pwmLow.val,pwmHigh.val);
    motorCommand7.val = constrain((throttleCommand.val + throttleAdjustment.val - adjustmentX.val - adjustmentY.val + adjustmentZ.val),pwmLow.val,pwmHigh.val);
    motorCommand8.val = constrain((throttleCommand.val + throttleAdjustment.val + adjustmentX.val - adjustmentY.val - adjustmentZ.val),pwmLow.val,pwmHigh.val);
#endif
#ifdef HEX_FRAME
    motorCommand1.val = constrain((throttleCommand.val + throttleAdjustment.val + 0.5 * adjustmentX.val + adjustmentY.val - adjustmentZ.val),pwmLow.val,pwmHigh.val);
    motorCommand2.val = constrain((throttleCommand.val + throttleAdjustment.val - 0.5 * adjustmentX.val + adjustmentY.val + adjustmentZ.val),pwmLow.val,pwmHigh.val);
    motorCommand3.val = constrain((throttleCommand.val + throttleAdjustment.val -       adjustmentX.val                   - adjustmentZ.val),pwmLow.val,pwmHigh.val);
    motorCommand4.val = constrain((throttleCommand.val + throttleAdjustment.val - 0.5 * adjustmentX.val - adjustmentY.val + adjustmentZ.val),pwmLow.val,pwmHigh.val);
    motorCommand5.val = constrain((throttleCommand.val + throttleAdjustment.val + 0.5 * adjustmentX.val - adjustmentY.val - adjustmentZ.val),pwmLow.val,pwmHigh.val);
    motorCommand6.val = constrain((throttleCommand.val + throttleAdjustment.val +       adjustmentX.val                   + adjustmentZ.val),pwmLow.val,pwmHigh.val);
    motorCommand7.val = pwmLow.val;
    motorCommand8.val = pwmLow.val;
#endif

    break;
  case LANDING:
    if (flightMode == RATE || flightMode == ATT){
      motorState = FLIGHT;
    }
    if (throttleCheckFlag == true){
      if (throCommand <= 1600 && throCommand >= 1500){
        throttleCheckFlag = false;
      }
    }
    throttleCommand.val = hoverCommand;
    if ( (hoverCommand + throttleAdjustment.val) < 1350){
      motorCommand1.val = pwmLow.val;
      motorCommand2.val = pwmLow.val;
      motorCommand3.val = pwmLow.val;
      motorCommand4.val = pwmLow.val;

      motorCommand5.val = pwmLow.val;
      motorCommand6.val = pwmLow.val;
      motorCommand7.val = pwmLow.val;
      motorCommand8.val = pwmLow.val;
      motorState = HOLD;
      break;
    }
    if (cmdRudd > 1950){
      motorCommand1.val = pwmLow.val;
      motorCommand2.val = pwmLow.val;
      motorCommand3.val = pwmLow.val;
      motorCommand4.val = pwmLow.val;
      motorCommand5.val = pwmLow.val;
      motorCommand6.val = pwmLow.val;
      motorCommand7.val = pwmLow.val;
      motorCommand8.val = pwmLow.val;
      motorState = HOLD;
      break;
    }
    if (fabs(imu.inertialZ.val) > 5.0){
      motorCommand1.val = pwmLow.val;
      motorCommand2.val = pwmLow.val;
      motorCommand3.val = pwmLow.val;
      motorCommand4.val = pwmLow.val;
      motorCommand5.val = pwmLow.val;
      motorCommand6.val = pwmLow.val;
      motorCommand7.val = pwmLow.val;
      motorCommand8.val = pwmLow.val;
      motorState = HOLD;
      break;
    }
    if (throttleAdjustment.val > 0){
      throttleAdjustment.val = 0;
    }

    landingThroAdjustment.val = 0.997 * landingThroAdjustment.val + 0.003 * throttleAdjustment.val;

#ifdef QUAD
    motorCommand1.val = constrain((throttleCommand.val + landingThroAdjustment.val + adjustmentX.val + adjustmentY.val - adjustmentZ.val),pwmLow.val,pwmHigh.val);
    motorCommand2.val = constrain((throttleCommand.val + landingThroAdjustment.val - adjustmentX.val + adjustmentY.val + adjustmentZ.val),pwmLow.val,pwmHigh.val);
    motorCommand3.val = constrain((throttleCommand.val + landingThroAdjustment.val - adjustmentX.val - adjustmentY.val - adjustmentZ.val),pwmLow.val,pwmHigh.val);
    motorCommand4.val = constrain((throttleCommand.val + landingThroAdjustment.val + adjustmentX.val - adjustmentY.val + adjustmentZ.val),pwmLow.val,pwmHigh.val);
    motorCommand5.val = pwmLow.val;
    motorCommand6.val = pwmLow.val;
    motorCommand7.val = pwmLow.val;
    motorCommand8.val = pwmLow.val;
#endif 
#ifdef X_8
    motorCommand1.val = constrain((throttleCommand.val + landingThroAdjustment.val + adjustmentX.val + adjustmentY.val - adjustmentZ.val),pwmLow.val,pwmHigh.val);
    motorCommand2.val = constrain((throttleCommand.val + landingThroAdjustment.val - adjustmentX.val + adjustmentY.val + adjustmentZ.val),pwmLow.val,pwmHigh.val);
    motorCommand3.val = constrain((throttleCommand.val + landingThroAdjustment.val - adjustmentX.val - adjustmentY.val - adjustmentZ.val),pwmLow.val,pwmHigh.val);
    motorCommand4.val = constrain((throttleCommand.val + landingThroAdjustment.val + adjustmentX.val - adjustmentY.val + adjustmentZ.val),pwmLow.val,pwmHigh.val);
    motorCommand5.val = constrain((throttleCommand.val + landingThroAdjustment.val + adjustmentX.val + adjustmentY.val + adjustmentZ.val),pwmLow.val,pwmHigh.val);
    motorCommand6.val = constrain((throttleCommand.val + landingThroAdjustment.val - adjustmentX.val + adjustmentY.val - adjustmentZ.val),pwmLow.val,pwmHigh.val);
    motorCommand7.val = constrain((throttleCommand.val + landingThroAdjustment.val - adjustmentX.val - adjustmentY.val + adjustmentZ.val),pwmLow.val,pwmHigh.val);
    motorCommand8.val = constrain((throttleCommand.val + landingThroAdjustment.val + adjustmentX.val - adjustmentY.val - adjustmentZ.val),pwmLow.val,pwmHigh.val);
#endif
#ifdef HEX_FRAME
    motorCommand1.val = constrain((throttleCommand.val + landingThroAdjustment.val + 0.5 * adjustmentX.val + adjustmentY.val - adjustmentZ.val),pwmLow.val,pwmHigh.val);
    motorCommand2.val = constrain((throttleCommand.val + landingThroAdjustment.val - 0.5 * adjustmentX.val + adjustmentY.val + adjustmentZ.val),pwmLow.val,pwmHigh.val);
    motorCommand3.val = constrain((throttleCommand.val + landingThroAdjustment.val -       adjustmentX.val                   - adjustmentZ.val),pwmLow.val,pwmHigh.val);
    motorCommand4.val = constrain((throttleCommand.val + landingThroAdjustment.val - 0.5 * adjustmentX.val - adjustmentY.val + adjustmentZ.val),pwmLow.val,pwmHigh.val);
    motorCommand5.val = constrain((throttleCommand.val + landingThroAdjustment.val + 0.5 * adjustmentX.val - adjustmentY.val - adjustmentZ.val),pwmLow.val,pwmHigh.val);
    motorCommand6.val = constrain((throttleCommand.val + landingThroAdjustment.val +       adjustmentX.val                   + adjustmentZ.val),pwmLow.val,pwmHigh.val);
    motorCommand7.val = pwmLow.val;
    motorCommand8.val = pwmLow.val;
#endif
    break;
  }
  /*MapVar(&motorCommand1.val,&motorPWM1,1000,2000,pwmLow.val,pwmHigh.val);
   MapVar(&motorCommand2.val,&motorPWM2,1000,2000,pwmLow.val,pwmHigh.val);
   MapVar(&motorCommand3.val,&motorPWM3,1000,2000,pwmLow.val,pwmHigh.val);
   MapVar(&motorCommand4.val,&motorPWM4,1000,2000,pwmLow.val,pwmHigh.val);
   
   MapVar(&motorCommand5.val,&motorPWM5,1000,2000,pwmLow.val,pwmHigh.val);
   MapVar(&motorCommand6.val,&motorPWM6,1000,2000,pwmLow.val,pwmHigh.val);
   MapVar(&motorCommand7.val,&motorPWM7,1000,2000,pwmLow.val,pwmHigh.val);
   MapVar(&motorCommand8.val,&motorPWM8,1000,2000,pwmLow.val,pwmHigh.val);*/
  Motor1WriteMicros(motorCommand1.val);
  Motor2WriteMicros(motorCommand2.val);
  Motor3WriteMicros(motorCommand3.val);
  Motor4WriteMicros(motorCommand4.val);

  Motor5WriteMicros(motorCommand5.val);
  Motor6WriteMicros(motorCommand6.val);
  Motor7WriteMicros(motorCommand7.val);
  Motor8WriteMicros(motorCommand8.val);

}







