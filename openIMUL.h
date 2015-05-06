// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//This code is provided under the GNU General Public Licence
//12/19/2012
//Gyro must be in RAD/s
//Sensors must be in the North East Down convention
#ifndef openIMUL_h
#define openIMUL_h
#include "AUXMATH.h"
#include <Streaming.h>

#include <Arduino.h>

#define ToRad(x) ((x)*0.01745329252)  // *pi/180
#define ToDeg(x) ((x)*57.2957795131)  // *180/pi


#define LAG_SIZE 56
#define LAG_SIZE_BARO 27


class openIMU{
public:
  openIMU(float*, float*, float*, float*, float*, 
  float*, float*, float*, float*, float*, float*, float*,
  float*, float*, float* , float*, float*, float*, float*);

  void AHRSupdate(void);
  void GetEuler(void);
  void GetPitch(void);
  void GetRoll(void);
  void GetYaw(void);
  void InitialQuat(void);
  void GetGravOffset(void);
  void UpdateLagIndex(void);  
  void GetInertial(void);
  void Predict(void);
  void CorrectGPS(void);
  void CorrectAlt(void);
  void GenerateRotationMatrix(void);

  float_u q0,q1,q2,q3;
  float_u pitch,roll,yaw;
  float_u XEst,YEst,ZEst,ZEstUp;
  float_u velX,velY,velZ,velZUp;
  float_u accelBiasX,accelBiasY,accelBiasZ;
  float accelBiasXEF,accelBiasYEF,accelBiasZEF;
  float_u inertialX,inertialY,inertialZ,inertialZGrav;
  float_u inertialXBiased,inertialYBiased,inertialZBiased;
  float_u xPosError,yPosError,zPosError;
  float_u xVelError,yVelError,zVelError;
  float XEstHist[LAG_SIZE],YEstHist[LAG_SIZE],ZEstHist[LAG_SIZE_BARO];
  float XVelHist[LAG_SIZE],YVelHist[LAG_SIZE],ZVelHist[LAG_SIZE_BARO];
  int16_t currentEstIndex,lagIndex,currentEstIndex_z,lagIndex_z;  
  float_u rawPitch,rawRoll;
  float_u pitchOffset,rollOffset;
  float_u declination;
  float_u magnitude,initialAccMagnitude,magnitudeDifference;
  float kpAcc;
  float kiAcc;
  float kpMag;
  float kiMag;
  float FEEDBACK_LIMIT;
  float COS_DEC,SIN_DEC;
  float kPosGPS,kVelGPS,kAccGPS,kPosBaro,kVelBaro,kAccBaro;
  float R11,R12,R13,R21,R22,R23,R31,R32,R33;
  float inertialSumX,inertialSumY,inertialSumZ,inertialAvgX,inertialAvgY,inertialAvgZ;
  uint8_t lagAmount;
  float_u biasedX,biasedY,biasedZ;
  uint8_t magDetected;
private:

  
  float *gx;
  float *gy;
  float *gz;
  float *ax;
  float *ay;
  float *az;
  float *mx;
  float *my;
  float *mz;
  float *dt;
  float *sax;//scaled accelerometer value
  float *say;
  float *saz;
  float *XPosGPS,*YPosGPS,*ZPosBaro;
  float *XVelGPS,*YVelGPS,*ZVelBaro;

  float recipNorm;
  float q0q0,q1q1,q2q2,q3q3,q0q1,q0q2,q0q3,q1q2,q1q3,q2q3;
  float integralFBX,integralFBY,integralFBZ;
  float qa,qb,qc;

  float kiDTAcc,kiDTMag,dtby2;
  float bx,by,bz,wx,wy,wz,vx,vy,vz;
  
  float hx,hy,hz,exm,eym,ezm,exa,eya,eza;
  float_u radPitch,radRoll,radYaw;
};


#endif









