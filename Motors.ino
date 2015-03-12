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
  if (EEPROM.read(ESC_CAL_FLAG) == 0xAA){
    DDRE |= B00111000;
    DDRH |= B00111000;


    TCCR3A = (1<<WGM31)|(1<<COM3A1)|(1<<COM3B1)|(1<<COM3C1);  
    TCCR3B = (1<<WGM33)|(1<<WGM32)|(1<<CS31);               
    ICR3 = PERIOD;   

    TCCR4A = (1<<WGM41)|(1<<COM4A1)|(1<<COM4B1)|(1<<COM4C1);
    TCCR4B = (1<<WGM43)|(1<<WGM42)|(1<<CS41);
    ICR4 = PERIOD;  

    Motor1RPMWriteMicros(pwmHigh.val);//set the output compare value
    Motor2RPMWriteMicros(pwmHigh.val);

    delay(4000);
    Motor1RPMWriteMicros(pwmLow.val);//set the output compare value
    Motor2RPMWriteMicros(pwmLow.val);
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
  //DDRB |= B01100000;


  TCCR3A = (1<<WGM31)|(1<<COM3A1)|(1<<COM3B1)|(1<<COM3C1);  
  TCCR3B = (1<<WGM33)|(1<<WGM32)|(1<<CS31);               
  ICR3 = PERIOD;   

  TCCR4A = (1<<WGM41)|(1<<COM4A1)|(1<<COM4B1)|(1<<COM4C1);
  TCCR4B = (1<<WGM43)|(1<<WGM42)|(1<<CS41);
  ICR4 = PERIOD;  

  //TCCR1A = (1<<WGM11)|(1<<COM1A1)|(1<<COM1B1);
  //TCCR1B = (1<<WGM13)|(1<<WGM12)|(1<<CS11);
  //ICR1 = PERIOD;

  Motor1RPMWriteMicros(pwmLow.val);//set the output compare value
  Motor2RPMWriteMicros(pwmLow.val);
  Motor1TILTWriteMicros(1500);
  Motor2TILTWriteMicros(1500);
  TailWriteMicros(1500);

}



void SaveGains(){
  uint16_t j_ = GAINS_START;
  for(uint16_t i = KP_PITCH_RATE_; i <= MAG_DEC_; i++){
    EEPROM.write(j_++,(*floatPointerArray[i]).buffer[0]); 
    EEPROM.write(j_++,(*floatPointerArray[i]).buffer[1]); 
    EEPROM.write(j_++,(*floatPointerArray[i]).buffer[2]); 
    EEPROM.write(j_++,(*floatPointerArray[i]).buffer[3]); 
    watchDogFailSafeCounter = 0;
    calibrationFlags = EEPROM.read(CAL_FLAGS);
    calibrationFlags &= ~(1<<GAINS_FLAG);
    EEPROM.write(CAL_FLAGS,calibrationFlags);
  }
}
void MotorHandler(){
  ffCommand = ffGain * pitchSetPoint.val;
  switch(motorState){
  case HOLD:

    if (saveGainsFlag == true && (millis() - romWriteDelayTimer) > 2000){
      SaveGains();

      saveGainsFlag = false;
      imuTimer = micros();
      baroTimer = millis();
      _400HzTimer = imuTimer;
    }
    //pressureInitial = pressure;
    initialYaw.val = imu.yaw.val;
    integrate = false;
    //HHState = 0;
    throttleAdjustment.val = 0;
    ZLoiterState = LOITERING;
    XYLoiterState = LOITERING;
    if (RCValue[THRO] > 1100){
      motorCommand1RPM.val = pwmLow.val;
      motorCommand2RPM.val = pwmLow.val;
      motorCommand1Tilt.val = constrain(1500 + ffCommand - adjustmentZ.val + adjustmentY.val,1000,2000);
      motorCommand2Tilt.val = constrain(1500 - ffCommand - adjustmentZ.val - adjustmentY.val,1000,2000);
      break;
    }
    if (flightMode == RTB){
      motorState = HOLD;
      motorCommand1RPM.val = pwmLow.val;
      motorCommand2RPM.val = pwmLow.val;
      motorCommand1Tilt.val = constrain(1500 + ffCommand - adjustmentZ.val + adjustmentY.val,1000,2000);
      motorCommand2Tilt.val = constrain(1500 - ffCommand - adjustmentZ.val - adjustmentY.val,1000,2000);
      break;
    }

    if (RCValue[RUDD] < 1300){
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
      UpdateOffset();
    }
    throttleAdjustment.val = 0;
    motorCommand1RPM.val = pwmLow.val;
    motorCommand2RPM.val = pwmLow.val;
    motorCommand1Tilt.val = constrain(1500 + ffCommand - adjustmentZ.val + adjustmentY.val,1000,2000);
    motorCommand2Tilt.val = constrain(1500 - ffCommand - adjustmentZ.val - adjustmentY.val,1000,2000);
    throttleCheckFlag = false;
    break;
  case TO:
    motorCommand1RPM.val = propIdleCommand;
    motorCommand2RPM.val = propIdleCommand;
    motorCommand1Tilt.val = constrain(1500 + ffCommand - adjustmentZ.val + adjustmentY.val,1000,2000);
    motorCommand2Tilt.val = constrain(1500 - ffCommand - adjustmentZ.val - adjustmentY.val,1000,2000);
    throttleCheckFlag = false;
    pressureInitial = pressure.val;
    imu.ZEst.val = 0;
    imu.ZEstUp.val = 0;
    imu.velZ.val = 0;
    imu.velZUp.val = 0;
    prevBaro = 0;
    baroZ.val = 0;
    //baroTimer = millis();
    initialYaw.val = imu.yaw.val;

    if (RCValue[RUDD] > 1700){
      motorState = HOLD;
    }
    if (flightMode == RTB){
      motorState = HOLD;
    }

    if (flightMode == RATE || flightMode == ATT){
      if (RCValue[THRO] > 1150 && RCValue[THRO] < 1350){
        motorState = FLIGHT;
        integrate = true;
      }
    }
    if (flightMode <= L2 && flightMode >= L0){
      if (RCValue[THRO] <= 1600 && RCValue[THRO] >= 1500){
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
      if (RCValue[THRO] <= 1600 && RCValue[THRO] >= 1500){
        autoMaticReady = true;
      }
    }

    break;
  case FLIGHT:
    if (flightMode == RATE || flightMode == ATT){
      throttleAdjustment.val = 0;
      throttleCommand.val = RCValue[THRO];
      if (throttleCommand.val > 1900){
        throttleCommand.val = 1900;
      }
      if (throttleCommand.val < 1050){
        motorState = HOLD;
      }
    }

    if (flightMode >= L0){
      throttleCommand.val = hoverCommand;
    }
    if (throttleCheckFlag == true){
      if (RCValue[THRO] <= 1600 && RCValue[THRO] >= 1500){
        throttleCheckFlag = false;
        throttleCommand.val = hoverCommand;
      }
    }

    motorCommand1RPM.val = constrain((throttleCommand.val + throttleAdjustment.val + adjustmentX.val),pwmLow.val,2000);
    motorCommand2RPM.val = constrain((throttleCommand.val + throttleAdjustment.val - adjustmentX.val),pwmLow.val,2000);
    motorCommand1Tilt.val = constrain(1500 + ffCommand - adjustmentZ.val + adjustmentY.val,1000,2000);
    motorCommand2Tilt.val = constrain(1500 - ffCommand - adjustmentZ.val - adjustmentY.val,1000,2000);

    /*motorCommand1.val = constrain((throttleCommand.val + throttleAdjustment.val + adjustmentX.val + adjustmentY.val - adjustmentZ.val),1000,2000);
     motorCommand2.val = constrain((throttleCommand.val + throttleAdjustment.val - adjustmentX.val + adjustmentY.val + adjustmentZ.val),1000,2000);
     motorCommand3.val = constrain((throttleCommand.val + throttleAdjustment.val - adjustmentX.val - adjustmentY.val - adjustmentZ.val),1000,2000);
     motorCommand4.val = constrain((throttleCommand.val + throttleAdjustment.val + adjustmentX.val - adjustmentY.val + adjustmentZ.val),1000,2000);*/


    break;
  case LANDING:
    if (flightMode == RATE || flightMode == ATT){
      motorState = FLIGHT;
    }
    if (throttleCheckFlag == true){
      if (RCValue[THRO] <= 1600 && RCValue[THRO] >= 1500){
        throttleCheckFlag = false;
      }
    }
    throttleCommand.val = hoverCommand;
    if ( (hoverCommand + throttleAdjustment.val) < 1200){
      motorCommand1RPM.val = pwmLow.val;
      motorCommand2RPM.val = pwmLow.val;
      motorCommand1Tilt.val = constrain(1500 + ffCommand - adjustmentZ.val + adjustmentY.val,1000,2000);
      motorCommand2Tilt.val = constrain(1500 - ffCommand - adjustmentZ.val - adjustmentY.val,1000,2000);
      motorState = HOLD;
      break;
    }
    if (RCValue[RUDD] > 1950){
      motorCommand1RPM.val = pwmLow.val;
      motorCommand2RPM.val = pwmLow.val;
      motorCommand1Tilt.val = constrain(1500 + ffCommand - adjustmentZ.val + adjustmentY.val,1000,2000);
      motorCommand2Tilt.val = constrain(1500 - ffCommand - adjustmentZ.val - adjustmentY.val,1000,2000);
      motorState = HOLD;
      break;
    }
    if (fabs(imu.inertialZ.val) > 5.0){
      motorCommand1RPM.val = pwmLow.val;
      motorCommand2RPM.val = pwmLow.val;
      motorCommand1Tilt.val = constrain(1500 + ffCommand - adjustmentZ.val + adjustmentY.val,1000,2000);
      motorCommand2Tilt.val = constrain(1500 - ffCommand - adjustmentZ.val - adjustmentY.val,1000,2000);
      motorState = HOLD;
      break;
    }
    motorCommand1RPM.val = constrain((throttleCommand.val + throttleAdjustment.val + adjustmentX.val),pwmLow.val,2000);
    motorCommand2RPM.val = constrain((throttleCommand.val + throttleAdjustment.val - adjustmentX.val),pwmLow.val,2000);
    motorCommand1Tilt.val = constrain(1500 + ffCommand - adjustmentZ.val + adjustmentY.val,1000,2000);
    motorCommand2Tilt.val = constrain(1500 - ffCommand - adjustmentZ.val - adjustmentY.val,1000,2000);
    /*motorCommand1.val = constrain((throttleCommand.val + throttleAdjustment.val + adjustmentX.val + adjustmentY.val - adjustmentZ.val),1000,2000);
     motorCommand2.val = constrain((throttleCommand.val + throttleAdjustment.val - adjustmentX.val + adjustmentY.val + adjustmentZ.val),1000,2000);
     motorCommand3.val = constrain((throttleCommand.val + throttleAdjustment.val - adjustmentX.val - adjustmentY.val - adjustmentZ.val),1000,2000);
     motorCommand4.val = constrain((throttleCommand.val + throttleAdjustment.val + adjustmentX.val - adjustmentY.val + adjustmentZ.val),1000,2000);*/

    break;
  }
  MapVar(&motorCommand1RPM.val,&motorPWM1,1000,2000,pwmLow.val,pwmHigh.val);
  MapVar(&motorCommand2RPM.val,&motorPWM2,1000,2000,pwmLow.val,pwmHigh.val);


  Motor1RPMWriteMicros(motorPWM1);//set the output compare value
  Motor2RPMWriteMicros(motorPWM2);
  Motor1TILTWriteMicros(motorCommand1Tilt.val);
  Motor2TILTWriteMicros(motorCommand2Tilt.val);
  tailPitch = -1.0 * imu.pitch.val;//fix the mapping function 
  if (tailPitch > 35.0){
    tailPitch = 35.0;
  }
  if (tailPitch < -35.0){
    tailPitch = -35.0;
  }
  MapVar(&tailPitch,&tailCommand.val,-35,35,1000,2000);
  //tailCommandFilt.val = 0.99 * tailCommandFilt.val + 0.01 * tailCommand.val;
  TailWriteMicros(tailCommand.val);
  //TailWriteMicros(RCValue[AUX3]);
}

















