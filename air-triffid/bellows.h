
#ifndef BELLOWS_H
#define BELLOWS_H

//#define SF

#ifdef SF
#include "SparkFunBME280.h"
// NB - we have frobbed the LibraryManager version of this to support the BMP280 as well as the BME280
// by adding chip ID 0x58 as an alternative 
//This was a terrible idea, and we moved to Adafruit's library
#else
// Adafruit
#include <Adafruit_BMP280.h>

#endif


class Bellows
{
  public:
  Bellows( int n, float x, float y, int _muxAddress, int _inflateServo, int _deflateServo );
  void driveServoAngle();
  void driveServoAngle(int servoNum, float openFraction);
  void setDrive( float drive );
  void incrementTarget( float delta );
  void incrementTargetFromPosition( float delta );
  void setup();
  void loop();

  float targetPressure;
  float currentPressure;
  float error; // -1 to 1
  float frustration; // integral of recent error, zeroed when we are on-target

  int n;
  float x;
  float y;
  float drive;
  
  private:
  
  int muxAddress;
  int inflateServo;
  int deflateServo;
  #ifdef SF
  BME280 bmp280;
  #else
  Adafruit_BMP280 bmp280;
  #endif
};
#endif

