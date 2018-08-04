#include "bellows.h"


// Servo pos for valve open/close
#define SERVO_CLOSE_DEGREES 60.0
#define SERVO_OPEN_DEGREES 120.0

#define DRIVE_GAIN -1.0

// values tuned for the tentacle valve servos - 130/550
#define SERVOMIN  130 //150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  550 //600 // this is the 'maximum' pulse length count (out of 4096)


Bellows::Bellows( float _x, float _y, int _muxAddress, int _inflateServo, int _deflateServo )
{
  x = _x;
  y = _y;
  muxAddress = _muxAddress;
  inflateServo = _inflateServo;
  deflateServo = _deflateServo;
  error = -2;
  targetPressure = 0.0;
  currentPressure = 0.0;
  frustration = 0;

  
  muxSelect(muxAddress);
  bme280.setI2CAddress(0x76);
  if (! bme280.beginI2C() )
  {
    Serial.println("Failed to read BME280 for bellows");
  }

}


void Bellows::loop()
{

  muxSelect(muxAddress);
  currentPressure = bme280.readFloatPressure() - atmosphericPressure;

  error = targetPressure - currentPressure;
  if( trace ){Serial.print("targetPressure "); Serial.println(targetPressure);}
  if( trace ){Serial.print("currentPressure "); Serial.println(currentPressure);}
  if( trace ){Serial.print("error fraction "); Serial.println(error);}
  if( trace ){Serial.print("frustration "); Serial.println(frustration);}

  if( fabs(error) < FRUSTRATION_LIMIT )
    frustration = 0;
  else
    frustration += error * loopSeconds;
      
  // simple linear feedback
  drive( error * DRIVE_GAIN);
  
}

void Bellows::drive( float drive ) // 0 is off, 1.0 is full pressure
{
  if( drive > 0.0 ) //increase pressure
  {
    driveServoAngle(inflateServo, drive);
    driveServoAngle(deflateServo, 0.0);
  }
  else // decrease pressure
  {
    driveServoAngle(inflateServo, 0.0);
    driveServoAngle(deflateServo, -drive);
  }
  
}



void Bellows::driveServoAngle(int servoNum, float openFraction)
{

  if( trace ){Serial.print("openFraction "); Serial.println(openFraction);}

  float servoAngle = fmap(openFraction, 0.0, 1.0, SERVO_CLOSE_DEGREES, SERVO_OPEN_DEGREES);
  
  float pulseLen = fmap( servoAngle, 0, 180, SERVOMIN, SERVOMAX ); // map angle to pulse length in PWM count units

  noMux();
  
  pwm.setPWM(servoNum, 0, pulseLen);
  

}

int printOneBellows( int y, int fh, Bellows*b )
{
  oled.print(twodigits(b->currentPressure/1000.0));

  oled.print(">");
  oled.print(twodigits(b->targetPressure/1000.0));
  
  y += fh; 
  oled.setCursor(0,y); 

   
  //oled.print("  S");
  //oled.print(threedigits( b->servoAngle));
  
  oled.print(" f");
  oled.print(twodigits( b->frustration * 100.0));

  y += fh; 
  oled.setCursor(0,y); 
  
  return y;
}

void  printBellows(  )
{
  int fh = oled.getFontHeight();
  int y = 0;

  oled.setCursor(0,y);

  oled.print("Bellows:");
  
  y += fh; 
  oled.setCursor(0,y); 
  
  if( enableBellows )
  {
    for( Bellows b : bellows )
      y = printOneBellows( y, fh, &b );
    
  } 
 
}





