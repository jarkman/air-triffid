#include <NintendoExtensionCtrl.h>

Nunchuk nchuk;

long lastNunchuckTime = 0;

#define NUNCHUCK_ADDRESS 5

void setupNunchuck() {

  muxSelect(NUNCHUCK_ADDRESS);
  nchuk.begin();

  if (!nchuk.connect()) {
    Serial.println("Nunchuk not detected!");
    
  }
  else
  {
    gotNunchuck = true;
    
  }

  lastNunchuckTime = millis() - 20000;
  noMux();
  
}

boolean nunchuckIdle()
{
  return millis() - lastNunchuckTime > 20000;
}

void loopNunchuck() {

  if( ! gotNunchuck )
  {
    if( traceNunchuck) Serial.println("no nunchuck");
    startT(TNUNCHUCK);
    endT();
    return;
  } 


  muxSelect(NUNCHUCK_ADDRESS);

  startT(TNUNCHUCK);
  boolean success = nchuk.update();  // Get new data from the controller

  if (success == true) {  // We've got data!
    if( traceNunchuck) nchuk.printDebug();  // Print all of the values!
    joyX = fmap(nchuk.joyX(),0.0,255.0,-1.0,1.0);
    joyY = fmap(nchuk.joyY(),0.0,255.0,-1.0,1.0);
    if( fabs( joyX ) > 0.1 || fabs( joyY ) > 0.1 )
      lastNunchuckTime = millis();
      
    //baselinePressureFraction = fmap(nchuk.pitchAngle(), -180.0, 180.0, 0.0, 1.0 );
  }
  else {  // Data is bad :(
    if( traceNunchuck)  Serial.println("Nunchuck disconnected, reconnecting");
    nchuk.reconnect();
    joyX = 0.0;
    joyY = 0.0;
  }
  endT();
  
  noMux();
}
