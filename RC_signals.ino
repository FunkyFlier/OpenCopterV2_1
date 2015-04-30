#include "AUXMATH.h"



void GetSwitchPositions() {
  //value from gear switch
  if (RCValue[GEAR] < 1250) {
    switchPositions = 0;
  }
  if (RCValue[GEAR] < 1650 && RCValue[GEAR] > 1350) {
    switchPositions = 3;
  }
  if (RCValue[GEAR] > 1750) {
    switchPositions = 6;
  }

  //value from aux 1
  if (RCValue[AUX1] < 1250) {
    switchPositions += 0;
  }
  if (RCValue[AUX1] < 1650 && RCValue[AUX1] > 1350) {
    switchPositions += 1;
  }
  if (RCValue[AUX1] > 1750) {
    switchPositions += 2;
  }

}

void ModeSelect() {
  uint8_t selectState = 0;
  uint32_t timeDiff = 0;
  //wait for mode input
  generalPurposeTimer = millis();
  while (modeSelect == false) { //---

    if (newRC == true) { //+++
      newRC = false;
      ProcessChannels();
      switch (selectState) {

      case 0://wait for input
        digitalWrite(RED, HIGH);
        digitalWrite(YELLOW, LOW);
        digitalWrite(GREEN, LOW);
        digitalWrite(13, LOW);
        timeDiff = millis() - generalPurposeTimer;
        if (timeDiff > 5000) {

          modeSelect = true;
          trimMode = true;
          break;
        }
        if (RCValue[AILE] > 1800) {
          digitalWrite(RED, HIGH);
          digitalWrite(YELLOW, LOW);
          digitalWrite(GREEN, HIGH);
          digitalWrite(13, LOW);
        }
        if (RCValue[ELEV] > 1800) {
          digitalWrite(RED, HIGH);
          digitalWrite(YELLOW, LOW);
          digitalWrite(GREEN, LOW);
          digitalWrite(13, HIGH);
        }
        if (RCValue[AILE] > 1800 && RCValue[ELEV] > 1800) {
          selectState = 1;
          generalPurposeTimer = millis();
        }
        break;

      case 1:
        digitalWrite(RED, LOW);
        digitalWrite(YELLOW, HIGH);
        digitalWrite(GREEN, LOW);
        digitalWrite(13, LOW);

        if (RCValue[AILE] < 1800 || RCValue[ELEV] < 1800) {
          selectState = 0;
        }
        if (((int32_t)millis() - (int32_t)generalPurposeTimer) - (int32_t)timeDiff > 2500) {
          modeSelect = true;
          trimMode = false;
          break;
        }
        break;
      }
    }//+++

  }//---
}

void CheckTXPositions() {
  bool positionOK = false;
  if (gsCTRL == false){
    while (positionOK == false) {
      digitalWrite(RED, LOW);
      digitalWrite(YELLOW, LOW);
      digitalWrite(GREEN, HIGH);
      digitalWrite(13, HIGH);
      if (newRC == true) {
        newRC = false;
        ProcessChannels();
      }
      positionOK = true;
      if (RCValue[THRO] > 1050) {
        positionOK = false;
      }
      if (RCValue[GEAR] > 1050) {
        positionOK = false;
      }
      if (RCValue[AUX1] > 1050) {
        positionOK = false;
      }
      if (RCValue[AUX2] > 1050) {
        positionOK = false;
      }
      if (RCValue[AUX3] > 1050) {
        positionOK = false;
      }
    }
  }
}
void ProcessChannels() {

  for (uint8_t i = 0; i < 8; i++)  {
    switch (rcData[i].chan) {
    case THRO:
      if (rcData[i].reverse == 0) {
        RCValue[THRO] = (rcData[i].rcvd - rcData[i].min) * rcData[i].scale + 1000;
      }
      else {
        RCValue[THRO] = (rcData[i].rcvd - rcData[i].min) * - rcData[i].scale + 2000;
      }
      break;
    case AILE:
      if (rcData[i].reverse == 0) {
        RCValue[AILE] = (rcData[i].rcvd - rcData[i].mid) * rcData[i].scale + 1500;
      }
      else {
        RCValue[AILE] = (rcData[i].rcvd - rcData[i].mid) * - rcData[i].scale + 1500;
      }
      break;
    case ELEV:
      if (rcData[i].reverse == 0) {
        RCValue[ELEV] = (rcData[i].rcvd - rcData[i].mid) * rcData[i].scale + 1500;
      }
      else {
        RCValue[ELEV] = (rcData[i].rcvd - rcData[i].mid) * - rcData[i].scale + 1500;
      }

      break;
    case RUDD:
      if (rcData[i].reverse == 0) {
        RCValue[RUDD] = (rcData[i].rcvd - rcData[i].mid) * rcData[i].scale + 1500;
      }
      else {
        RCValue[RUDD] = (rcData[i].rcvd - rcData[i].mid) * - rcData[i].scale + 1500;
      }
      break;
    case GEAR:
      if (rcData[i].reverse == 0) {
        RCValue[GEAR] = (rcData[i].rcvd - rcData[i].min) * rcData[i].scale + 1000;
      }
      else {
        RCValue[GEAR] = (rcData[i].rcvd - rcData[i].min) * - rcData[i].scale + 2000;
      }

      break;
    case AUX1:
      if (rcData[i].reverse == 0) {
        RCValue[AUX1] = (rcData[i].rcvd - rcData[i].min) * rcData[i].scale + 1000;
      }
      else {
        RCValue[AUX1] = (rcData[i].rcvd - rcData[i].min) * - rcData[i].scale + 2000;
      }

      break;
    case AUX2:
      if (rcData[i].reverse == 0) {
        RCValue[AUX2] = (rcData[i].rcvd - rcData[i].min) * rcData[i].scale + 1000;
      }
      else {
        RCValue[AUX2] = (rcData[i].rcvd - rcData[i].min) * - rcData[i].scale + 2000;
      }

      break;
    case AUX3:
      if (rcData[i].reverse == 0) {
        RCValue[AUX3] = (rcData[i].rcvd - rcData[i].min) * rcData[i].scale + 1000;
      }
      else {
        RCValue[AUX3] = (rcData[i].rcvd - rcData[i].min) * - rcData[i].scale + 2000;
      }

      break;


    }
  }



  if (txFailSafe == true) {
    switch (clearTXRTB) {

    case 0:
      if (RCValue[GEAR] > 1850) {
        clearTXRTB = 1;
      }
      return;
      break;

    case 1:
      if (RCValue[GEAR] < 1150) {
        txFailSafe = false;
        failSafe = false;
        RCFailSafeCounter = 0;
        clearTXRTB = 0;
        break;
      }
      return;
      break;
    default:
      clearTXRTB = 0;
      return;
      break;

    }


  }
  else {
    ProcessModes();
  }




}

void ProcessModes() {

  previousFlightMode = flightMode;
  if (RCValue[AUX2] > 1750) {
    gsCTRL = false;
    flightMode = ATT;
    setTrim = true;
    trimComplete = true;
    cmdElev = RCValue[ELEV];
    cmdAile = RCValue[AILE];
    cmdRudd = RCValue[RUDD];
    throCommand = RCValue[THRO];
    MapVar(&cmdAile, &rollSetPointTX.val, 1000, 2000, -60, 60);
    MapVar(&cmdElev, &pitchSetPointTX.val, 1000, 2000, -60, 60);
    MapVar(&cmdRudd, &yawInput, 1000, 2000, -300, 300);
    if (rollSetPoint.val < 1 && rollSetPoint.val > -1) {
      rollSetPoint.val = 0;
    }
    if (pitchSetPoint.val < 1 && pitchSetPoint.val > -1) {
      pitchSetPoint.val = 0;
    }
    if (yawInput < 5 && yawInput > -5) {
      yawInput = 0;
    }
    if (flightMode != previousFlightMode) {
      enterState = true;
    }

    return;

  }


  if (RCValue[AUX3] > 1750) {
    gsCTRL = false;
    flightMode = RTB;
    cmdElev = RCValue[ELEV];
    cmdAile = RCValue[AILE];
    cmdRudd = RCValue[RUDD];
    throCommand = RCValue[THRO];
    MapVar(&cmdAile, &rollSetPointTX.val, 1000, 2000, -60, 60);
    MapVar(&cmdElev, &pitchSetPointTX.val, 1000, 2000, -60, 60);
    MapVar(&cmdRudd, &yawInput, 1000, 2000, -300, 300);
    if (rollSetPointTX.val < 1 && rollSetPointTX.val > -1) {
      rollSetPointTX.val = 0;
    }
    if (pitchSetPointTX.val < 1 && pitchSetPointTX.val > -1) {
      pitchSetPointTX.val = 0;
    }
    if (yawInput < 5 && yawInput > -5) {
      yawInput = 0;
    }
    if (flightMode != previousFlightMode) {
      enterState = true;
    }
    return;

  }

  if (gsCTRL == false){
    cmdElev = RCValue[ELEV];
    cmdAile = RCValue[AILE];
    cmdRudd = RCValue[RUDD];
    throCommand = RCValue[THRO];
    flightMode = modeArray[switchPositions];
  }
  else{
    cmdElev = GSRCValue[ELEV];
    cmdAile = GSRCValue[AILE];
    cmdRudd = GSRCValue[RUDD];
    throCommand = GSRCValue[THRO];
    flightMode = GSRCValue[GEAR];
    if (flightMode < 2){
      flightMode = 2;
    }
    if (flightMode > 9){
      flightMode = 2;
    }
  }  

  switch(flightMode){
  case RATE:
    flightMode = RATE;
    setTrim = false;
    trimComplete = false;
    MapVar(&cmdElev, &rateSetPointY.val, 1000, 2000, -400, 400);
    MapVar(&cmdAile, &rateSetPointX.val, 1000, 2000, -400, 400);
    MapVar(&cmdRudd, &rateSetPointZ.val, 1000, 2000, -400, 400);
    if (rateSetPointY.val < 5 && rateSetPointY.val > -5) {
      rateSetPointY.val = 0;
    }
    if (rateSetPointX.val < 5 && rateSetPointX.val > -5) {
      rateSetPointX.val = 0;
    }
    if (rateSetPointZ.val < 5 && rateSetPointZ.val > -5) {
      rateSetPointZ.val = 0;
    }
    break;
  case RATE_TRIM:
    setTrim = true;
    flightMode = RATE;
    MapVar(&cmdElev, &rateSetPointY.val, 1000, 2000, -400, 400);
    MapVar(&cmdAile, &rateSetPointX.val, 1000, 2000, -400, 400);
    MapVar(&cmdRudd, &rateSetPointZ.val, 1000, 2000, -400, 400);
    if (rateSetPointY.val < 5 && rateSetPointY.val > -5) {
      rateSetPointY.val = 0;
    }
    if (rateSetPointX.val < 5 && rateSetPointX.val > -5) {
      rateSetPointX.val = 0;
    }
    if (rateSetPointZ.val < 5 && rateSetPointZ.val > -5) {
      rateSetPointZ.val = 0;
    }
    break;
  case ATT:
    flightMode = ATT;
    setTrim = false;
    trimComplete = false;
    MapVar(&cmdElev, &pitchSetPoint.val, 1000, 2000, -60, 60);
    MapVar(&cmdAile, &rollSetPoint.val, 1000, 2000, -60, 60);
    MapVar(&cmdRudd, &yawInput, 1000, 2000, -300, 300);
    if (rollSetPoint.val < 1 && rollSetPoint.val > -1) {
      rollSetPoint.val = 0;
    }
    if (pitchSetPoint.val < 1 && pitchSetPoint.val > -1) {
      pitchSetPoint.val = 0;
    }
    if (yawInput < 5 && yawInput > -5) {
      yawInput = 0;
    }
    break;
  case ATT_TRIM:
    flightMode = ATT;
    setTrim = true;
    MapVar(&cmdElev, &pitchSetPoint.val, 1000, 2000, -60, 60);
    MapVar(&cmdAile, &rollSetPoint.val, 1000, 2000, -60, 60);
    MapVar(&cmdRudd, &yawInput, 1000, 2000, -300, 300);
    if (rollSetPoint.val < 1 && rollSetPoint.val > -1) {
      rollSetPoint.val = 0;
    }
    if (pitchSetPoint.val < 1 && pitchSetPoint.val > -1) {
      pitchSetPoint.val = 0;
    }
    if (yawInput < 5 && yawInput > -5) {
      yawInput = 0;
    }
    break;
  case L0:
    flightMode = L0;
    MapVar(&cmdAile, &rollSetPointTX.val, 1000, 2000, -60, 60);
    MapVar(&cmdElev, &pitchSetPointTX.val, 1000, 2000, -60, 60);
    MapVar(&cmdRudd, &yawInput, 1000, 2000, -300, 300);
    if (rollSetPointTX.val < 1 && rollSetPointTX.val > -1) {
      rollSetPointTX.val = 0;
    }
    if (pitchSetPointTX.val < 1 && pitchSetPointTX.val > -1) {
      pitchSetPointTX.val = 0;
    }
    if (yawInput < 5 && yawInput > -5) {
      yawInput = 0;
    }
    break;
  case L1:
    flightMode = L1;
    MapVar(&cmdAile, &rollSetPointTX.val, 1000, 2000, -60, 60);
    MapVar(&cmdElev, &pitchSetPointTX.val, 1000, 2000, -60, 60);
    MapVar(&cmdRudd, &yawInput, 1000, 2000, -300, 300);
    if (rollSetPointTX.val < 1 && rollSetPointTX.val > -1) {
      rollSetPointTX.val = 0;
    }
    if (pitchSetPointTX.val < 1 && pitchSetPointTX.val > -1) {
      pitchSetPointTX.val = 0;
    }
    if (yawInput < 5 && yawInput > -5) {
      yawInput = 0;
    }
    break;
  case L2:
    flightMode = L2;
    MapVar(&cmdAile, &rollSetPointTX.val, 1000, 2000, -60, 60);
    MapVar(&cmdElev, &pitchSetPointTX.val, 1000, 2000, -60, 60);
    MapVar(&cmdRudd, &yawInput, 1000, 2000, -300, 300);
    if (rollSetPointTX.val < 1 && rollSetPointTX.val > -1) {
      rollSetPointTX.val = 0;
    }
    if (pitchSetPointTX.val < 1 && pitchSetPointTX.val > -1) {
      pitchSetPointTX.val = 0;
    }
    if (yawInput < 5 && yawInput > -5) {
      yawInput = 0;
    }
    if (gpsFailSafe == true) {
      flightMode = L1;

    }
    break;
  case FOLLOW://TBD impliment FOLLOW and WP modes currently operate as ATT
    flightMode = ATT;
    setTrim = false;
    trimComplete = false;
    MapVar(&cmdElev, &pitchSetPoint.val, 1000, 2000, -60, 60);
    MapVar(&cmdAile, &rollSetPoint.val, 1000, 2000, -60, 60);
    MapVar(&cmdRudd, &yawInput, 1000, 2000, -300, 300);
    if (rollSetPoint.val < 1 && rollSetPoint.val > -1) {
      rollSetPoint.val = 0;
    }
    if (pitchSetPoint.val < 1 && pitchSetPoint.val > -1) {
      pitchSetPoint.val = 0;
    }
    if (yawInput < 5 && yawInput > -5) {
      yawInput = 0;
    }
    break;
  case WP:
    flightMode = ATT;
    setTrim = false;
    trimComplete = false;
    MapVar(&cmdElev, &pitchSetPoint.val, 1000, 2000, -60, 60);
    MapVar(&cmdAile, &rollSetPoint.val, 1000, 2000, -60, 60);
    MapVar(&cmdRudd, &yawInput, 1000, 2000, -300, 300);
    if (rollSetPoint.val < 1 && rollSetPoint.val > -1) {
      rollSetPoint.val = 0;
    }
    if (pitchSetPoint.val < 1 && pitchSetPoint.val > -1) {
      pitchSetPoint.val = 0;
    }
    if (yawInput < 5 && yawInput > -5) {
      yawInput = 0;
    }
    break;
  case RTB:
    flightMode = RTB;
    MapVar(&cmdAile, &rollSetPointTX.val, 1000, 2000, -60, 60);
    MapVar(&cmdElev, &pitchSetPointTX.val, 1000, 2000, -60, 60);
    MapVar(&cmdRudd, &yawInput, 1000, 2000, -300, 300);
    if (rollSetPointTX.val < 1 && rollSetPointTX.val > -1) {
      rollSetPointTX.val = 0;
    }
    if (pitchSetPointTX.val < 1 && pitchSetPointTX.val > -1) {
      pitchSetPointTX.val = 0;
    }
    if (yawInput < 5 && yawInput > -5) {
      yawInput = 0;
    }
    if (flightMode != previousFlightMode) {
      enterState = true;
    }
    break;

  }
  if (flightMode > L0 && imu.magDetected == false) {
    flightMode = L0;
    MapVar(&cmdElev, &pitchSetPoint.val, 1000, 2000, -60, 60);
    MapVar(&cmdAile, &rollSetPoint.val, 1000, 2000, -60, 60);
    MapVar(&cmdRudd, &yawInput, 1000, 2000, -300, 300);
    if (rollSetPoint.val < 1 && rollSetPoint.val > -1) {
      rollSetPoint.val = 0;
    }
    if (pitchSetPoint.val < 1 && pitchSetPoint.val > -1) {
      pitchSetPoint.val = 0;
    }
    if (yawInput < 5 && yawInput > -5) {
      yawInput = 0;
    }
  }
  if (flightMode != previousFlightMode) {
    enterState = true;
  }
}

void FeedLine() {

  switch (rcType) {
  case 1:
    DSMXParser();
    break;
  case 2:
    SBusParser();
    break;
  }

}
void SBusParser() {
  while (Serial1.available() > 0) {
    if (millis() - frameTime > 8) {
      readState = 0;
    }
    inByte = Serial1.read();
    frameTime = millis();
    switch (readState) {
    case 0:
      if (inByte == 0x0F) {
        bufferIndex = 0;
        sBusData[bufferIndex] = inByte;
        sBusData[24] = 0xff;
        readState = 1;
      }

      break;
    case 1:
      bufferIndex ++;
      sBusData[bufferIndex] = inByte;

      if (bufferIndex == 24) {
        readState = 0;
        if (sBusData[0] == 0x0f && sBusData[24] == 0x00) {
          newRC = true;
          rcData[0].rcvd = (sBusData[1] | sBusData[2] << 8) & 0x07FF ;
          rcData[1].rcvd = (sBusData[2] >> 3 | sBusData[3] << 5) & 0x07FF;
          rcData[2].rcvd = (sBusData[3] >> 6 | sBusData[4] << 2 | sBusData[5] << 10) & 0x07FF;
          rcData[3].rcvd = (sBusData[5] >> 1 | sBusData[6] << 7) & 0x07FF;
          rcData[4].rcvd = (sBusData[6] >> 4 | sBusData[7] << 4) & 0x07FF;
          rcData[5].rcvd = (sBusData[7] >> 7 | sBusData[8] << 1 | sBusData[9] << 9) & 0x07FF;
          rcData[6].rcvd = (sBusData[9] >> 2 | sBusData[10] << 6) & 0x07FF;
          rcData[7].rcvd = (sBusData[10] >> 5 | sBusData[11] << 3) & 0x07FF;

          if (sBusData[23] & (1 << 3)) {
            failSafe = true;
          }
        }
      }
      break;
    }
  }


}

void DSMXParser() {
  if (Serial.available() > 14) {
    while (Serial.available() > 14) {
      Serial.read();
    }
    byteCount = 0;
    bufferIndex = 0;
  }
  while (Serial1.available() > 0) {
    if (millis() - frameTime > 8) {
      byteCount = 0;
      bufferIndex = 0;
    }
    inByte = Serial1.read();
    frameTime = millis();
    byteCount++;


    if (bufferIndex > 14) {
      bufferIndex = 0;
      byteCount = 0;
    }
    if (byteCount > 2) {
      spekBuffer[bufferIndex] = inByte;
      bufferIndex++;
    }
    if (byteCount == 16 && bufferIndex == 14) {
      newRC = true;
      byteCount = 0;
      bufferIndex = 0;
      for (uint8_t i = 0; i < 14; i = i + 2) {
        channelNumber = (spekBuffer[i] >> 3) & 0x0F;
        if (channelNumber < 8) {
          rcData[channelNumber].rcvd = ((spekBuffer[i] << 8) | (spekBuffer[i + 1])) & 0x07FF;
        }
      }
    }
  }
}

void DetectRC() {
  readState = 0;
  RC_SSHigh();
  SBus();
  readState = 0;
  if (detected == true) {
    FrameCheck();
    readState = 0;
    if (failSafe == true){
      rcDetected = false;
    }
    return;
  }
  readState = 0;
  RC_SSLow();
  Spektrum();
  readState = 0;

  if (detected == true) {
    FrameCheck();
    readState = 0;
    return;
  }
  else {
    rcType = RC;
  }
  readState = 0;
  if (rcType == RC) { //figure out the best way to handle this redundant code
    DDRK = 0;//PORTK as input
    PORTK |= 0xFF;//turn on pull ups
    PCMSK2 |= 0xFF;//set interrupt mask for all of PORTK
    PCICR = 1 << 2; //enable the pin change interrupt for K
    delay(100);//wait for a few frames
    if (rcData[0].rcvd == 0 && rcData[1].rcvd == 0 && rcData[2].rcvd == 0 && rcData[3].rcvd == 0 && rcData[4].rcvd == 0 && rcData[5].rcvd == 0 && rcData[6].rcvd == 0) {
      ISRState = PPM;
      PORTK |= 0x80;
      PCMSK2 |= 0x80;
    }

  }



}

ISR(PCINT2_vect) {
  switch (ISRState) {
  case STAND:
    currentPinState = PINK;
    changeMask = currentPinState ^ lastPinState;
    lastPinState = currentPinState;
    currentTime = micros();
    for (uint8_t i = 0; i < 8; i++) {
      if (changeMask & 1 << i) { //has there been a change
        if (!(currentPinState & 1 << i)) { //is the pin in question logic low?
          timeDifference = currentTime - changeTime[i];//if so then calculate the pulse width
          if (900 < timeDifference && timeDifference < 2200) { //check to see if it is a valid length
            rcData[i].rcvd = timeDifference;
            if (rcData[i].chan == THRO && ((timeDifference ) < ((uint16_t)rcData[i].min - 50) )) {
              failSafe = true;
            }
            else {
              newRC = true;
            }

          }
        }
        else { //the pin is logic high implying that this is the start of the pulse
          changeTime[i] = currentTime;
        }
      }
    }
    break;
  case PPM:
    currentTime = micros();
    if ((PINK & 0x80) == 0x80) { //is the PPM pin high
      previousTime = currentTime;
    }
    else {
      timeDifference = currentTime - previousTime;
      if (timeDifference > 2500) {
        channelCount = 0;
      }
      else {
        rcData[channelCount].rcvd = timeDifference;
        if (rcData[channelCount].chan == THRO && ((timeDifference ) < ((uint16_t)rcData[channelCount].min - 50) )) {
          failSafe = true;
        }
        else {
          newRC = true;
        }
        channelCount++;
      }
    }
    break;
  }

}

void FrameCheck() { //checks if serial RC was incorrectly detected
  newRC = false;
  uint32_t frameCheckTimer;
  frameCheckTimer =  millis();
  delay(100);
  rcDetected = true;
  while (newRC == false) {
    if (rcType == RC) {
      delay(100);
    }
    if (rcType != RC) {
      FeedLine();
    }

    if (millis() - frameCheckTimer > 1000) { //in case it has incorrectly detected serial RC
      rcDetected = false;
      return;
      /*rcType = RC;
       DDRK = 0;//PORTK as input
       PORTK |= 0xFF;//turn on pull ups
       PCMSK2 |= 0xFF;//set interrupt mask for all of PORTK
       PCICR = 1 << 2;
       delay(100);//wait for a few frames
       if (rcData[0].rcvd == 0 && rcData[1].rcvd == 0 && rcData[2].rcvd == 0 && rcData[3].rcvd == 0 && rcData[4].rcvd == 0 && rcData[5].rcvd == 0 && rcData[6].rcvd == 0) {
       ISRState = PPM;
       PORTK |= 0x80;
       PCMSK2 |= 0x80;
       }
       
       generalPurposeTimer = millis();*/
    }
  }
  newRC = false;

}


void SBus() {

  Serial1.begin(100000);
  generalPurposeTimer = millis();

  while (Serial1.available() == 0) {
    if (millis() - generalPurposeTimer > 1000) {
      return;
    }
  }

  delay(20);
  while (Serial1.available() > 0) {
    inByte = Serial1.read();
    switch (readState) {
    case 0:
      if (inByte == 0x0f) {
        bufferIndex = 0;
        sBusData[bufferIndex] = inByte;
        sBusData[24] = 0xff;
        readState = 1;
      }
      break;
    case 1:
      bufferIndex ++;
      sBusData[bufferIndex] = inByte;
      if (bufferIndex == 24) {
        readState = 0;
        if (sBusData[0] == 0x0f && sBusData[24] == 0x00) {
          rcType = SBUS;
          detected = true;
        }
      }
      break;
    }
  }
  frameTime = millis();
}
void Spektrum() {
  Serial1.begin(115200);
  generalPurposeTimer = millis();
  while (Serial1.available() == 0) {
    if (millis() - generalPurposeTimer > 1000) {
      return;
    }
  }
  delay(5);
  while (Serial1.available() > 0) {
    Serial1.read();
  }
  frameTime = millis();
  rcType = DSMX;
  detected = true;
}









































