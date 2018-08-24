
#include "Adafruit_MCP23017.h"

Adafruit_MCP23017 mcp;

#define PIRS 16
boolean pir[PIRS];


void setupPir()
{
  noMux();
  mcp.begin();      // use default address 0
  for( int p = 0; p < PIRS; p++)
  {
    mcp.pinMode(p, INPUT);
    //mcp.pinMode(p, OUTPUT);
    //mcp.pullUp(p, HIGH);
  }
}

boolean s = false;
void loopPir()
{

  s = ! s;
  
  noMux();
  for( int p = 0; p < PIRS; p++)
  {
    // mcp.digitalWrite(p, s);
     
    pir[p] = mcp.digitalRead(p);
    Serial.print(p);
    Serial.print(":");
    Serial.print(pir[p]);
    Serial.print(" ");
   
  }

   Serial.println("");
  
}

