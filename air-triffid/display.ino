
void setupDisplay()
{
  if( ! enableDisplay )
  {
    Serial.println("Display disabled !!!!!!!!!!!!!!!");
    return;
  }

  noMux();
  
  oled.begin();     // Initialize the OLED
  oled.clear(PAGE); // Clear the display's internal memory
  oled.clear(ALL);  // Clear the library's display buffer

   oled.setFontType(0);
  int y = 0; //oled.getLCDHeight();
  int fh = oled.getFontHeight();

  oled.setCursor(0,y);
  oled.print("[triffid]!");
  oled.display();   


}

void loopDisplay()
{
  if( ! enableDisplay )
  {
    startT(TDISPLAY);
    endT();
    return;
  }
  
  noMux();

  startT(TDISPLAY);
  oled.clear(PAGE); // Clear the display's internal memory
  
  oled.setFontType(0);
  
  switch( uiState )
  {
    case 0: 
      printPirs( );
      break;
    case 1:
      printBellows();
      break;
    case 2:
      //drawPose();
    default:
      break;
  }
  

  oled.display();   

 endT();
 
}


char *threedigits( float f )
{
  snprintf(report, sizeof(report), "%3d", (int) f);
  return report;

}

char *twodigits( float f )
{
  snprintf(report, sizeof(report), "%2d", (int) f);
  return report;

}
