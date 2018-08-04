#include <NintendoExtensionCtrl.h>

Nunchuk nchuk;

void setupNunchuck() {

    noMux();
  nchuk.begin();

  while (!nchuk.connect()) {
    Serial.println("Nunchuk not detected!");
    delay(1000);
  }
}

void loopNunchuck() {
    noMux();
  boolean success = nchuk.update();  // Get new data from the controller

  if (success == true) {  // We've got data!
    nchuk.printDebug();  // Print all of the values!
    joyX = fmap(nchuk.joyX(),0.0,255.0,-1.0,1.0);
    joyY = fmap(nchuk.joyY(),0.0,255.0,-1.0,1.0);
  }
  else {  // Data is bad :(
    Serial.println("Controller Disconnected!");
    nchuk.reconnect();
    joyX = 0.0;
    joyY = 0.0;
  }
}
