
long tiltPeopleTime = millis();
long numPeopleTime = millis();
long peopleTimeout = 5000;

float fastFactor = 0.1;
float slowFactor = 0.01;

// if we have pose-tracking inout, use that to control pose
boolean loopPoseTrack()
{

  if( ! enablePoseTrack )
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


  slowNumPeople = (slowFactor*loopSeconds) * numPeople + (1.0-slowFactor*loopSeconds) * slowNumPeople;
  fastNumPeople = (fastFactor*loopSeconds) * numPeople + (1.0-fastFactor*loopSeconds) * fastNumPeople;
  
  if( now - tiltPeopleTime < peopleTimeout )
  {
      if( numPeople < 1 )
        numPeople = 1; // so we get speeded-up behaviour even if I don't get round to writing the num-sending code
        
      float x = tiltPeople;
      float y = fastNumPeople - slowNumPeople; // lean toward when people arrive, away when they leave
      
      //float amount = fabs(tiltPeople);
      //float angle = (tiltPeople > 0.0) ? 180 : 0; // TODO = probably the wrong way round
    
      setBendDirection( x, y );
      
      
      Serial.print("tilt setting x ");
      Serial.print(x);
      Serial.print(" y ");
      Serial.print(y);
      Serial.println(" ");

      return true;
   }
  else
  {
    numPeople = 0; // turn off LED behaviour
    return false;
  }
}

