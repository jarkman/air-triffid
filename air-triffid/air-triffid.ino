
// add this to Preferences/Addiotnal Board Manager URLs
// http://arduino.esp8266.com/stable/package_esp8266com_index.json
// and select LOLIN (WEMOS) D1 mini Pro

#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>


#include <NintendoExtensionCtrl.h>


// This runs the air triffid with 6 servo-controlled air valves,
// taking user input from a wii nunchuck,
// with pressure feeback from BME280 pressure sensors in each chamber
// position feedback from one LSM303 compass sensor (specifically, the Adafruit board)
// a tca9548a i2c multiplexer on the Adafruit breakout
// a MCP23017 GPIO to read PIR signals
// and a Adafruit PWMServoDriver board 

// We use Polulu libraries for the LSM303 because they use a lot less memory than the Adafruit libraries

// Code lives in https://github.com/jarkman/air-triffid

// It runs on a Wemos D1 mini with an oled screen


// Wiring:

// "Boot from Flash" requires some pin state to be right:

// D3/GPIO0 = 1 (high)
// D4/GPIO2 = 1 (high)
// D8/GPIO15 = 0 (low)

// so we'd better use these as outputs

// Pins are
//  I2C
//    D1 SCL
//    D2 SDA
//      wired to OLED and to mux and to servo board
// 
// Neopixels are on D3 D4 D6
// Encoder is on D5 & D7, switch on D0, pulling to ground

// or, in order:
// D0 Encoder switch
// D1 SCL
// D2 SDA
// D3 Neopixels 0  via CD40109BE A p3 -> E p4   http://www.ti.com/lit/ds/symlink/cd40109b.pdf
// D4 Neopixels 1                B p6 -> F p5
// D5 Encoder A    
// D6 Neopixels 2                C  p10 -> G p11
// D7 Encoder B
// D8 - 




// i2c addresses:
// Oled   : 0x3C
// Adafruit_PWMServoDriver: 0x40
// wii nunchuck: 0x52
// MCP23017 GPIO: 0x20                   // http://ww1.microchip.com/downloads/en/DeviceDoc/20001952C.pdf

// Mux    : 0x70, 0x71
// and beyond the mux:
//  LSM303 : 0x19 & 0x1E
//  BMP280: 0x76
//  ADS1015 for supermanual control: 0x48 


// Mux channels:
// 0: bellows 0  BMP280 and low LSM303 and ADS1015
// 1: bellows 1  BMP280 and high LSM303
// 2: bellows 2  BMP280
// 3: airbox pressure BMP280
// 4: atmospheric pressure BMP280 (on control board)


#include <Wire.h>
//#include <Servo.h>
#include <SFE_MicroOLED.h>  // Include the SFE_MicroOLED library
#include <Adafruit_PWMServoDriver.h>

#define FRUSTRATION_LIMIT 0.01 // error value above which frustration accumulates


#include "bellows.h"

boolean openLoop = false; // run without using pressure sensors, using duty-cycle modulation on the valves
boolean alwaysRestartSensors = false;

boolean trace = false;          // activity tracing for finding crashes - leave off to get better response times
boolean traceTime = false;      // time logging for our major consumers
boolean traceBehaviour = false;
boolean traceBellows = false;

boolean traceNodes = false;
boolean tracePressures = false;
boolean tracePirs = false;
boolean traceNunchuck = false;
boolean traceSupermanual = false;
boolean traceSelftest = false;

boolean enableDisplay = false; // costs about 200ms / loop

boolean enableLeds = true; // costs about 150ms / loop, of which 100 is sin/cos fns

boolean enablePoseTrack = false;
boolean enableBellows = true;  // turn on/off bellows code
boolean enableBehaviour = true;
boolean calibrateCompasses = false; // turn on then rotate each compass smoothly about all axes to get the individual compass min/max values for compass setup

bool disablePressureFeedback = false; // turned off by supermanual


void setupNodes();
void loopNodes();
void setupEncoder();



#define PIN_RESET 255  //
#define DC_JUMPER 0  // I2C Addres: 0 - 0x3C, 1 - 0x3D

MicroOLED oled(PIN_RESET, DC_JUMPER);  // I2C Example

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();



#define BELLOWS 3
// servo numbers on the PWM servo driver
Bellows bellows[] = {Bellows(0, 0.0,                     1.0,                     0, 0, 1), 
                     Bellows(1, sin(30.0*3.1415/180.0),  -cos(30.0*3.1415/180.0), 1, 2, 3), 
                     Bellows(2, -sin(30.0*3.1415/180.0), -cos(30.0*3.1415/180.0), 2, 4, 5)};


float supermanual[]={0,0,0};

float attentionAngle = 0.0; // net attention direction, degrees clockwise from bellows 0
float attentionAmount = 0.0; // 0.0 to 1.1


float tiltPeople = 0.0;
int numPeople = 0;
float slowNumPeople = 0.0;
float fastNumPeople = 0.0;

// UI screens accessible via encoder
#define UI_STATES 3
// 0 - pirs
// 1 - pressures
long uiState = 1; // pirs

char report[80];

float bendTargetX = 0.0; // -1.0 to 1.0
float bendTargetY = 0.0;

boolean gotNunchuck = false;

boolean gotSupermanual = false;
boolean supermanualDrive = true; // true to control drive, false to control pressure


float joyX = 0.0; // -1.0 to 1.0
float joyY = 0.0;
float baselinePressureFraction = 1.0;
float breatheFraction = 1.0;

long breatheStartT = 0;
float breathePeriod = 10000.0; // in millis
float breatheAmplitude = 0.1;

float maxGoodPressure = 101000.0 + 1500.0; // Pa, about what we expect from our blower
float minGoodPressure = 101000.0 - 1500.0;

boolean i2cIsSickly = false; // have we had terrible pressure readings suggesting the i2c bus is mad ?


float airboxAbsPressure = 0.0; //101000.0 + 1500.0; // Pa, about what we expect from our blower
float atmosphericAbsPressure = 0.0; //101000.0; // Pa
float nominalMaxPressure = 800.0; // what we expect to make in the airbox
float baselinePressure = 0.0;


 
float wavePeriod = 10000.0; // in millis
float waveFraction = 0.5; // 0 to 1.0

float waveAmplitude = 1.0; // 0 to 1.0
long waveStartT = 0;


float loopSeconds = 0.1; // duration of our loop() in seconds, used for normalising assorted constants

// pose targets for a boot-time wriggle selftest
#define SELFTEST_MILLIS (5 * 1000) 
#define NUM_SELFTEST 5
// target pressure ratios for chambers
float low = 0.5;
float selftest[NUM_SELFTEST][3] = {{1.0,1.0,1.0},{1.0,1.0,low},{low,1.0,1.0},{1.0,low,1.0}/*,{1.0,low,low},{low,1.0,low},{low,low,1.0}*/,{1.0,1.0,1.0}};
int nextSelftest = -1;
long selftestStartMillis = -1;


// time profiling


#define TPRESSURE 0
#define TSERVO 1
#define TNUNCHUCK 2
#define TSLIDERS 3
#define TDISPLAY 4
#define TMUX 5
#define TLEDS 6

#define NBINS 7

String tName[] = {"Pressure", "Servos", "Nunchuck", "Sliders", "Display", "Mux", "LEDs"};
long tBin[NBINS];
long binStart;
int currentBin = -1;



void setupI2C()
{
  Wire.begin();
  Wire.setClock(10000);  // 10k for a 10m wire length limit - esp seems to ignore this! Currently getting a 53khz clock
  Wire.setClockStretchLimit(40000); 
}



void setup() {
  setupDisplay();
  
  delay(2000);
  Serial.begin(115200);


  Serial.println("");
  Serial.println("---Setup---");
  Serial.println("..i2c");
  setupI2C();
  Serial.println("..pir");
  setupPir();


  Serial.println("..fixed pressures");
  setupFixedPressures();
  
  Serial.println("..servo driver");
  setupServoDriver();

  Serial.println("..encoder");
  setupEncoder();

  Serial.println("..leds");
  
  setupLeds();

  Serial.println("..nunchuck");
  setupNunchuck();

  Serial.println("..supermanual");
  setupSupermanual();
  
  Serial.println("..wifi");
  setupWifi();
  
  //baseServo.attach(D7); 
  //tipServo.attach(D8); 
  Serial.println("..bellows");
   
  for( int b = 0; b < BELLOWS; b ++ )
  {
    Bellows *bellow = &(bellows[b]);
    bellow->setup();
  }

  //servoTest();
  
  waveStartT = millis();
  breatheStartT = millis();

  Serial.println("..setup done");
}

void setupServoDriver()
{
  noMux();
  
  pwm.begin();
  
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
}


void loopBreathe()
{
  // modulate overall pressure up & down a bit ihn a slow sine
  float theta = 2 * PI * (millis() - breatheStartT)/breathePeriod;

   
  breatheFraction = 1.0 - breatheAmplitude * 0.5 * sin(theta);
 
}

float wave(float phaseOffset) // -1.0 to 1.0
{
  float theta = 2 * PI * (millis() - waveStartT)/wavePeriod;

   
  return waveAmplitude * sin(theta + phaseOffset);
  
}

boolean loopSelftest()
{
  return false;
  if( nextSelftest >= NUM_SELFTEST )
    return false; // selftest done


  float error = 0;
  for( int b = 0; b < BELLOWS; b ++ )
  {
    Bellows *bellow = &(bellows[b]);
  
    error += fabs( bellow->error);
  }
  
  if( nextSelftest < 0 ||  // first time round the loop
    //error < 0.1 ||          // arrived at target pose
    (selftestStartMillis > 0 && millis() - selftestStartMillis > SELFTEST_MILLIS) ) // time has been too long, must be broken
  {
    //move on to next pose
    Serial.println("------------------------------------------");
    Serial.print("Starting selftest pose ");
    Serial.println(nextSelftest);
    nextSelftest ++;
    selftestStartMillis = millis();
    
    if( nextSelftest >= NUM_SELFTEST )
      return false; // selftest done
  }
  
    int i = 0;
    for( int b = 0; b < BELLOWS; b ++ )
    {
      Bellows *bellow = &(bellows[b]);
      bellow->targetPressure = baselinePressure * selftest[nextSelftest][i++]; 

      if( traceSelftest )
      {
        Serial.print("baselinePressure ");
        Serial.println(baselinePressure);
    
        Serial.print("set bellows to ");
        Serial.println(bellow->targetPressure);
      }
    }
    

  return true;
}

boolean loopManual()
{
  //return false;

  
  if( ! gotNunchuck )
    return false;

  if( nunchuckIdle())
    return false;
  
  setBendDirection(joyX, joyY);


  printBellowsPressures("Manual: ");
  return true;
}

boolean loopSupermanualControl()
{

  if( ! gotSupermanual || supermanualIdle())
  {
    disablePressureFeedback = false;
    return false;
  }

   
  if( supermanualDrive )
  {
    for( int b = 0; b < BELLOWS; b ++ )
    {
      Bellows *bellow = &(bellows[b]);
      bellow->setDrive(supermanual[b]);
    }
  
    disablePressureFeedback = true;
    printBellowsPressures("Supermanual drive: ");
  }
  else
  { // pressure
    for( int b = 0; b < BELLOWS; b ++ )
    {
      Bellows *bellow = &(bellows[b]);
      bellow->setDrive(supermanual[b]);
      bellow->targetPressure = baselinePressure * fmap( supermanual[b], -1.0, 1.0, 0.0, 1.0); 
    }
  
    
    printBellowsPressures("Supermanual pressure: ");
  }
  return true;
}

long lastPrintBellowsPressures = 0;
void printBellowsPressures(char*label)
{
  if( millis() - lastPrintBellowsPressures < 1000 )
    return;

  lastPrintBellowsPressures = millis();
    
  if( openLoop )
  {
    Serial.print(label);
    for( int b = 0; b < BELLOWS; b ++ )
   {
    Bellows *bellow = &(bellows[b]);
    Serial.print(bellow->n);
     Serial.print(" R: ");
    Serial.print(bellow->reduction);
   }

   
  Serial.println(" ");
  
    return;
    
  }
  Serial.print(label);
  Serial.print(" Atm: ");
  Serial.print(atmosphericAbsPressure);
  Serial.print("  Abx: ");
  Serial.print(airboxAbsPressure);
  Serial.print("  Delta: ");
    Serial.print((int)(airboxAbsPressure-atmosphericAbsPressure));
  Serial.print("  : ");

  
   for( int b = 0; b < BELLOWS; b ++ )
   {
    Bellows *bellow = &(bellows[b]);
    Serial.print(bellow->n);
     Serial.print(" T:");
    Serial.print((int)bellow->targetPressure);
     Serial.print(" C:");
    Serial.print((int)bellow->currentPressure);
      Serial.print(" D:");
    Serial.print(bellow->drive);
     Serial.print("    ");
   }
   Serial.println(" ");
  
}
void setBendDirection(float x, float y)
{
  // at 0,0, set all three to nominal pressure
  for( int b = 0; b < BELLOWS; b ++ )
  {
    Bellows *bellow = &(bellows[b]);
    bendTargetX = x;
    bendTargetY = y;
    float reduction = 0.8* (bendTargetX * bellow->x + bendTargetY * bellow->y); // not at all sure this is sensible
    bellow->reduction = fconstrain( reduction, 0.0, 0.6 );
    bellow->targetPressure = baselinePressure * (1.0 - reduction);
  }

}

void setBendAngle( float angle, float amount )
{
  setBendDirection( cos( radians( angle )) * amount, sin(radians(angle))*amount );
}

void loopWave()
{
  float angle = fmod( (((float) millis()) /  10000.0), 1.0 ) * 360.0 ;
  setBendAngle( angle, 0.8 );
  printBellowsPressures("Wave: ");
   /*
  //float controlVal = analogRead(controlPotPin);   

  // waveFraction sets the amount of autowave we are going to add to the stick motion
  //float waveFraction = fmap(controlVal, 0.0,905.0, 0.0, 1.0); // magic numbers for control pot

  float effectiveWaveFraction = waveFraction;

  long stickDelay = millis() - lastStickMoveMillis; // how long since we last wiggled the stick ?
  if( stickDelay > 0 && stickDelay < 20000 )
    effectiveWaveFraction *= (float) (stickDelay + 2000) / (20000.0 + 2000.0); // fade back in over 20 secs, but always keep 10%

  //Serial.println(effectiveWaveFraction);
  
  float tipPhaseDelay = - fmap( effectiveWaveFraction, 0.0, 1.0, - PI / 2.0, 0.3 ); // 90 degrees delay at small amplitude, for a wiggle, no delay at large amplitude, for a whole-tentacle curl

  //wavePeriod = fmap( waveFraction, 0.0, 1.0, 10000.0, 20000.0 );


  float tipDrive = stickLeftX * (1.5 * stickLeftY - 0.5) + stickRightX;
  
  baseBellows.target(stickLeftX +  effectiveWaveFraction * wave(0));
  tipBellows.target(tipDrive +  effectiveWaveFraction * wave(tipPhaseDelay));
  */

}

int i2cResetCount = 0;

void manageI2C()
{
  if( ! i2cIsSickly )
    return;

  int fails = 0;

  i2cResetCount ++;

 
  if( i2cResetCount > 5 )
  {
    return;
  }

   Serial.print("i2c reset # ");  Serial.println(i2cResetCount);

  noMux();
  int i2cResult = Wire.endTransmission(true);
  Serial.print("i2c master reset"); Serial.print(" - " ); Serial.println(i2cResult);
  // we get 4 when our odd things happens
  if( i2cResult != 0 )
  {
    fails ++;
    delay( 500 );
    yield();
  } 
  for( int m = 0; m < 8; m ++ )
  {
    
    muxSelect( m );
    int i2cResult = Wire.endTransmission(true);
    Serial.print("i2c reset result mux "); Serial.print(m); Serial.print(" - " ); Serial.println(i2cResult);
    // we get 4 when our odd things happens
    if( i2cResult != 0 )
    {
      fails ++;
      delay( 500 );
      yield();
    } 
  }



      
  if( fails == 0 )
      i2cIsSickly = false; //this is set when we read bad pressure numbers

}

void loop() {

  long start = millis();

  manageI2C();
 
  loopBreathe();
  loopWifi();
  loopPir();
  
  loopEncoder();
  loopDisplay();
  loopLeds();

  loopNunchuck();
  loopSupermanual();
 

  //delay(100);

  //return;

  readFixedPressures();
  
  
  if( trace ) Serial.println("---Loop---");
  if( trace ) Serial.println("selftest/wave"); 
  if( ! loopManual())
    if( ! loopSupermanualControl())
      if( ! loopSelftest())  
        if( ! loopPoseTrack())
          if( ! loopBehaviour())
            loopWave();


  for( int b = 0; b < BELLOWS; b ++ )
    {
      Bellows *bellow = &(bellows[b]);
      bellow->loop();
    }

  long end = millis();
  if( traceTime ) {Serial.print("loop took ") ; Serial.println( end-start );}

  loopSeconds = 0.001 * (float)( end-start );
  
  if( traceTime ) {printT();}
  
}



float fmapConstrained(float x, float in_min, float in_max, float out_min, float out_max)
{
  float f = fmap( x,  in_min, in_max, out_min, out_max);

  if( f < out_min )
    f = out_min;

  if( f > out_max )
    f = out_max;

  return f;
}

float fmap(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


float fconstrain(float f, float out_min, float out_max)
{
  if( f < out_min )
    f = out_min;

  if( f > out_max )
    f = out_max;

  return f;
}

// https://learn.adafruit.com/adafruit-tca9548a-1-to-8-i2c-multiplexer-breakout/wiring-and-test
#define TCAADDR0 0x70
#define NO_MUX 16

 
void muxSelect(uint8_t i) {
  uint8_t m0;
  //delay(10); // let i2cc calm down - totally speculative
  
  if( i== NO_MUX )
  {
    // disable both
    m0 = 0;
    
  }
  else
  {
    
    m0 = 1 << i;
 
  }
 
 startT(TMUX);
  Wire.beginTransmission(TCAADDR0);
  Wire.write(m0);
  Wire.endTransmission(); 
  endT();
 
}

// disable mux 
void noMux()
{
  muxSelect( NO_MUX );
}



void clearT()
{
  for( int i = 0; i< NBINS; i ++)
    tBin[i] = 0;
}

void printT()
{
  long total = 0;
  
  for( int i = 0; i< NBINS; i ++)
  {
    
    total += tBin[i];
  }
  
  for( int i = 0; i< NBINS; i ++)
  {
    Serial.print( tName[i] );
    Serial.print( " : ");
    Serial.print(tBin[i] /1000);
    Serial.print( " ms ");
    Serial.print((100 * tBin[i]) /total);
    Serial.println( " %");
    
  }

  Serial.print( "Total : ");
  Serial.print(total/1000 );
  Serial.println( " ms ");
  
  clearT();
}


void startT(int bin)
{
  currentBin = bin;
  binStart = micros();
  
}

void endT()
{
  if( currentBin < 0 )
    return;

  tBin[currentBin] += micros() - binStart;
}


