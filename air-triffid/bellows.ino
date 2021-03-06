#include "bellows.h"


// Servo pos for valve open/close
#define SERVO_CLOSE_DEGREES 40.0 
#define SERVO_OPEN_DEGREES 130.0

#define DRIVE_GAIN -4.0

// values tuned for the tentacle valve servos - 130/550
#define SERVOMIN  130 //150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  550 //600 // this is the 'maximum' pulse length count (out of 4096)

#ifdef SF
  BME280 bmp280Atmospheric;
  BME280 bmp280Airbox;
#else
  Adafruit_BMP280 bmp280Atmospheric;
  Adafruit_BMP280 bmp280Airbox;
#endif


int muxAddressAtmospheric = 4;
int muxAddressAirbox = 3;

// resting offset values to calibrate for specific sensors
float airboxCal = -90;
float bellowsCal[] = {10, -47, -53 }; // by observation

// in degrees, added to servo angles to get less leakiness
float inflateServoOffset[] = {0.0, 0.0, 0.0};
float deflateServoOffset[] = {5.0, 0.0, 0.0};

#ifdef SF
void startBmp280(BME280 *b)
#else
void startBmp280( char* label, Adafruit_BMP280 *b)
#endif

{
  #ifdef SF
  b->setI2CAddress(0x76);
  if (! b->beginI2C() )
  {
    Serial.print(label); Serial.print(" - ");
    Serial.println("failed to read BMP280");
    delay(1000);
  }
  b->setStandbyTime(0); //0 to 7 valid. Time between readings. See table 27.

  b->setTempOverSample(1); //0 to 16 are valid. 0 disables temp sensing. See table 24.
  b->setPressureOverSample(16); //0 to 16 are valid. 0 disables pressure sensing. See table 23.
  b->setHumidityOverSample(1); //0 to 16 are valid. 0 disables humidity sensing. See table 19.
  
  b->setMode(MODE_NORMAL); //MODE_SLEEP, MODE_FORCED, MODE_NORMAL is valid. See 3.3

  b->setFilter(16); //0 to 4 is valid. Filter coefficient. See 3.4.4
 
  
  #else
  if (! b->begin(0x76)) {
        Serial.print(label); Serial.print(" - ");
        Serial.println("Could not find a BMP280 sensor");
        
    }
  else
  {
    Serial.print(label); Serial.print(" - ");
    Serial.print("Found BMP280 sensor");
    float pressure = bmp280Atmospheric.readPressure();
    Serial.print("  pressure "); Serial.println(pressure);
  }

   /* b->setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X0, //X2,  // temperature
                    Adafruit_BMP280::SAMPLING_X16, // pressure
                    Adafruit_BMP280::SAMPLING_X0, //X1,  // humidity
                    Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_0_5 );*/
                    
  #endif

}


float readBmp280( Adafruit_BMP280 *b )
{
  startT(TPRESSURE);
  
  float p = b->readPressure();   
  if( ! goodPressure( p ))
    i2cIsSickly = true;

  endT();
  
  return p;
}

void setupFixedPressures()
{
    
  muxSelect(muxAddressAtmospheric);
  startBmp280("atmospheric", &bmp280Atmospheric);
  
  
  muxSelect(muxAddressAirbox);

  startBmp280("airbox",&bmp280Airbox);

}

boolean goodPressure( float pressure )
{
  return pressure > minGoodPressure && pressure < maxGoodPressure; 
}

void readFixedPressures()
{
  
  //if( openLoop )
  //  return;
    
  muxSelect(muxAddressAtmospheric);
  
  if( alwaysRestartSensors ) startBmp280("atmospheric", &bmp280Atmospheric);
  
  
  #ifdef SF
  atmosphericAbsPressure = bmp280Atmospheric.readFloatPressure() ;
  #else
  atmosphericAbsPressure = readBmp280(&bmp280Atmospheric);
  #endif
  
  if( tracePressures ) { Serial.print(" atmosphericAbsPressure "); Serial.println(atmosphericAbsPressure);}

  
  muxSelect(muxAddressAirbox);
  
  if( alwaysRestartSensors ) startBmp280("muxAddressAirbox", &bmp280Airbox);
 
    #ifdef SF
  airboxAbsPressure = bmp280Airbox.readFloatPressure() - airboxCal;
  #else
airboxAbsPressure = readBmp280(&bmp280Airbox) - airboxCal;
#endif

  //if( tracePressures ) { Serial.print(" airboxAbsPressure "); Serial.println(airboxAbsPressure);}
  if( tracePressures ) { Serial.print(" airboxdelta  "); Serial.println(airboxAbsPressure - atmosphericAbsPressure);}


  if( goodPressure(airboxAbsPressure) && goodPressure(atmosphericAbsPressure))
  {
    if( airboxAbsPressure - atmosphericAbsPressure > nominalMaxPressure )
        nominalMaxPressure = airboxAbsPressure - atmosphericAbsPressure;  // go up quickly
    else
        nominalMaxPressure = nominalMaxPressure * 0.9 + 0.1 * (airboxAbsPressure - atmosphericAbsPressure); // go down slowly
    
    //baselinePressure = (airboxAbsPressure - atmosphericAbsPressure) * baselinePressureFraction * breatheFraction;
    baselinePressure = nominalMaxPressure * baselinePressureFraction * breatheFraction;
  }
  else
    baselinePressure = nominalMaxPressure * baselinePressureFraction * breatheFraction;

  if( tracePressures ) { Serial.print(" baselinePressure "); Serial.println(baselinePressure);}

}


Bellows::Bellows( int _n, float _x, float _y, int _muxAddress, int _inflateServo, int _deflateServo )
{
  n = _n;
  x = _x;
  y = _y;
  muxAddress = _muxAddress;
  inflateServo = _inflateServo;
  deflateServo = _deflateServo;
  error = -2;
  targetPressure = 0.0;
  currentPressure = 0.0;
  frustration = 0;
}


void Bellows::setup()
{
  
  muxSelect(muxAddress);

  char snum[5];

  itoa(n, snum, 10);
  startBmp280(snum, &bmp280);
}

void servoTest()
{
  Serial.println("selftest bellows ");
  for( int i = 0; i < 3; i++ )
  {
    Serial.println("Deflate");
    yield();
    for( int b = 0; b < BELLOWS; b ++ )
    {
      Bellows *bellow = &(bellows[b]);
      bellow->setDrive(-1.0);
      /*delay(500);
      yield(); 
      delay(500);
      yield();*/
    }
    delay(500);
    yield(); 
    delay(500);
    yield();
    Serial.println("Inflate");
    for( int b = 0; b < BELLOWS; b ++ )
    {
      Bellows *bellow = &(bellows[b]);
      bellow->setDrive(1.0);
      /*delay(500);
      yield(); 
      delay(500);
      yield();*/
    }
    
    delay(500);
    yield();
    delay(500);
    yield();
  }
  
}

long openLoopPeriod = 5000;

void Bellows::loop()
{

  if( disablePressureFeedback )
    return; // means we are in supermanual

  if( openLoop )
  {

     float phase = (float)(millis() % openLoopPeriod)/(float)openLoopPeriod;
    if( phase < reduction )
      setDrive(-0.4);
    else
      setDrive(1.0);
    return;
  }
  
  muxSelect(muxAddress);

  if( alwaysRestartSensors ) startBmp280("bellows", &bmp280);

 
  #ifdef SF
  float pressure = bmp280.readFloatPressure() -  bellowsCal[n];
  #else
  float pressure = readBmp280(&bmp280) -  bellowsCal[n];
  #endif

  if( tracePressures ) { Serial.print("n "); Serial.print(n);  Serial.print(" pressure "); Serial.println(pressure);}

  
  pressure = pressure - atmosphericAbsPressure;

  // a bit of smoothing
  currentPressure = 0.9 * currentPressure + 0.1 * pressure;
  
  error = (targetPressure - currentPressure)/baselinePressure;


  float d = - error * DRIVE_GAIN;

  //drive = 1.0;

  
  
  if( traceBellows ){Serial.print("n "); Serial.println(n);}
  if( traceBellows ){Serial.print("targetPressure "); Serial.println(targetPressure);}
  if( traceBellows ){Serial.print("currentPressure "); Serial.println(currentPressure);}
  if( traceBellows ){Serial.print("error fraction "); Serial.println(error);}
  if( traceBellows ){Serial.print("drive "); Serial.println(drive);}

  if( fabs(error) < FRUSTRATION_LIMIT )
    frustration = 0;
  else
    frustration += error * loopSeconds;
      
  // simple linear feedback
  setDrive( d );
  
}

void Bellows::setDrive( float d ) // -1.0 to deflate, 1.0 to inflate
{
  drive = fconstrain( d, -1.0, 1.0 );

   //driveServoAngle(inflateServo, 1.0);
   // driveServoAngle(deflateServo, 1.0);
    //return;
    
  if( drive > 0.0 ) //increase pressure
  {
    driveServoAngle(inflateServo, drive, inflateServoOffset[n]);
    driveServoAngle(deflateServo, 0.0, deflateServoOffset[n]);
    if( traceBellows ){Serial.print(n); Serial.print(" inflate "); Serial.print(drive);Serial.print(" deflate "); Serial.println(0);}
  }
  else // decrease pressure
  {
    driveServoAngle(inflateServo, 0.0, inflateServoOffset[n]);
    driveServoAngle(deflateServo, -drive, deflateServoOffset[n]);
    if( traceBellows ){Serial.print(n); Serial.print(" inflate "); Serial.print(0);Serial.print(" deflate "); Serial.println(-drive);}

  }
  
}



void Bellows::driveServoAngle(int servoNum, float openFraction, float offset)
{

  if( trace )
    {Serial.print("openFraction "); Serial.println(openFraction);}

  float servoAngle = fmap(openFraction, 0.0, 1.0, SERVO_CLOSE_DEGREES, SERVO_OPEN_DEGREES);
  servoAngle += offset;
  
  float pulseLen = fmap( servoAngle, 0, 180, SERVOMIN, SERVOMAX ); // map angle to pulse length in PWM count units

  noMux();

  startT(TSERVO);
  pwm.setPWM(servoNum, 0, pulseLen);
  endT();

}

int printOneBellows( int y, int fh, Bellows*b )
{
  oled.print(twodigits(b->currentPressure/1000.0));

  oled.print(">");
  oled.print(twodigits(b->targetPressure/1000.0));
  
  y += fh; 
  oled.setCursor(0,y); 


  oled.print("dr ");
  oled.print(twodigits( b->drive * 100.0));

  y += fh; 
  oled.setCursor(0,y); 
  
  return y;
}

void  printBellows(  )
{

  
  int fh = oled.getFontHeight();
  int y = 0;

  oled.setCursor(0,y);

  /*oled.print("Bellows:");
  
  y += fh;
   
  oled.setCursor(0,y); 
  */
  if( enableBellows )
  {
    for( Bellows b : bellows )
      y = printOneBellows( y, fh, &b );
    
  } 
 
}





