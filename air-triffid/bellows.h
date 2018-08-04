#ifndef BELLOWS_H
#define BELLOWS_H

#include "SparkFunBME280.h"
// NB - we have frobbed the LibraryManager version of this to support the BMP280 as well as the BME280
// by adding chip ID 0x58 as an alternative 

class Bellows
{
  public:
  Bellows( float x, float y, int _muxAddress, int _inflateServo, int _deflateServo );
  void driveServoAngle();
  void driveServoAngle(int servoNum, float openFraction);
  void drive( float drive );
  void incrementTarget( float delta );
  void incrementTargetFromPosition( float delta );
  void loop();

  float targetPressure;
  float currentPressure;
  float error; // -1 to 1
  float frustration; // integral of recent error, zeroed when we are on-target

  float x;
  float y;
  
  private:
  
  int muxAddress;
  int inflateServo;
  int deflateServo;
  BME280 bme280;
};
#endif

