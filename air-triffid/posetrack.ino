
// if we have pose-tracking inout, use that to control pose
boolean loopPoseTrack()
 {
  if( ! enablePoseTrack )
    return false;

  if( ! Serial.available())
    return false;

  char c;
  // take the most recent char
  while(Serial.available())
    c = Serial.read();

  // sent tilt as a single byte between 'a' and 'y', with 'n' in the middle
  if( c < 'a' || c > 'y')
    return false; // shouldn't happen

  float tilt = ((float)c - (float)'n')/(float)'a' - (float)'n'; // convert to range -1 to +1

  float amount = fabs(tilt);
  float angle = (tilt > 0.0) ? 180 : 0; // TODO = probably the wrong way round

   setBendAngle( angle, amount );


      Serial.print("tilt setting angle ");
      Serial.print(angle);
      Serial.print(" amount ");
      Serial.print(amount);
      Serial.println(" ");
     
    return true;
  }

