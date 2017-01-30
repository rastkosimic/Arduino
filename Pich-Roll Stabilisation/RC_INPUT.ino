void calcInput1()
{
  // if the pin is high, its the start of an interrupt
  if(digitalRead(RECEIVER_SIGNAL_IN_1_PIN) == HIGH)
  { 
    // get the time using micros - when our code gets really busy this will become inaccurate, but for the current application its 
    // easy to understand and works very well
    ulStartPeriod1 = micros();
  }
  else
  {
    // if the pin is low, its the falling edge of the pulse so now we can calculate the pulse duration by subtracting the 
    // start time ulStartPeriod1 from the current time returned by micros()
    if(ulStartPeriod1 && (bNewChannelSignal1 == false))
    {
      nChannel1In = (int)(micros() - ulStartPeriod1);
      ulStartPeriod1 = 0;

      // tell loop we have a new signal on the channel
      // we will not update nChannel1In until loop sets
      // bNewChannelSignal back to false
      bNewChannelSignal1 = true;
    }
  }
}

void calcInput2()
{
  // if the pin is high, its the start of an interrupt
  if(digitalRead(RECEIVER_SIGNAL_IN_2_PIN) == HIGH)
  { 
    // get the time using micros - when our code gets really busy this will become inaccurate, but for the current application its 
    // easy to understand and works very well
    ulStartPeriod2 = micros();
  }
  else
  {
    // if the pin is low, its the falling edge of the pulse so now we can calculate the pulse duration by subtracting the 
    // start time ulStartPeriod from the current time returned by micros()
    if(ulStartPeriod2 && (bNewChannelSignal2 == false))
    {
      nChannel2In = (int)(micros() - ulStartPeriod2);
      ulStartPeriod2 = 0;

      // tell loop we have a new signal on the channel
      // we will not update nChannel2In until loop sets
      // bNewThrottleSignal back to false
      bNewChannelSignal2 = true;
    }
  }
}
