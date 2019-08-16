
long tiltPeopleTime = millis();
long numPeopleTime = millis();
long peopleTimeout = 5000;

// if we have pose-tracking inout, use that to control pose
boolean loopPoseTrack()
{

  if( ! enablePoseTrack )
    return false;

  if( ! Serial.available())
    return false;

  long now = millis();
  
  char c;
  // take the most recent char
  while(Serial.available())
  {
    c = Serial.read();

    // sent tilt as a single byte between 'a' and 'y', with 'm' in the middle
    if( c >= 'a' &&  c <= 'y')
    {  
      tiltPeople = ((float)c - (float)'m')/(float)'a' - (float)'m'; // convert to range -1 to +1
      tiltPeopleTime = now;
    }
    
    // sent people count as a single byte between '0' and '9'
    if( c >= '0' &&  c <= '9')
    {  
      numPeople = ((int)c - (int)'0');
      if( numPeople > 0 )
        numPeopleTime = now;
    }

  }

  if( now - tiltPeopleTime < peopleTimeout )
  {
      if( numPeople < 1 )
        numPeople = 1; // so we get speeded-up behaviour even if I don't get round to writing the num-sending code
        
      float amount = fabs(tiltPeople);
      float angle = (tiltPeople > 0.0) ? 180 : 0; // TODO = probably the wrong way round
    
      setBendAngle( angle, amount );
      
      
      Serial.print("tilt setting angle ");
      Serial.print(angle);
      Serial.print(" amount ");
      Serial.print(amount);
      Serial.println(" ");

      return true;
   }
  else
    return false;
}

