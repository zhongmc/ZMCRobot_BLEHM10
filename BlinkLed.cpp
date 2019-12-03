#include "BlinkLed.h"

BlinkLed::BlinkLed()
{
  m_onTime= 700;
  m_offTime = 300;
  m_ledPin = 13;

  pinMode(m_ledPin, OUTPUT);
  digitalWrite( m_ledPin, HIGH);
  isLedOn = true;
  m_onTimer = millis();
}

BlinkLed::BlinkLed(int ledPin, int onTime, int offTime)
{
  m_onTime= onTime;
  m_offTime = offTime;
  m_ledPin = ledPin;

  pinMode(m_ledPin, OUTPUT);
  digitalWrite( m_ledPin, HIGH);
  isLedOn = true;
  m_onTimer = millis();
  
}

void BlinkLed::beSureToBlink()
{
  long curTime = millis();
  if( isLedOn)
  {
     if( (curTime - m_onTimer) > m_onTime )
     {
      isLedOn = false;
       digitalWrite( m_ledPin, LOW);
       m_offTimer = curTime;
     }
  }
  else
  {
     if( (curTime - m_offTimer) > m_offTime )
     {
      isLedOn = true;
       digitalWrite( m_ledPin, HIGH);
       m_onTimer = curTime;
     }
    
  }
}

        
void BlinkLed::normalBlink()
{
  m_onTime= 100;
  m_offTime = 500;
  digitalWrite( m_ledPin, HIGH);
  isLedOn = true;
  m_onTimer = millis();
}

void BlinkLed::fastBlink()
{
  m_onTime= 100;
  m_offTime = 300;
  digitalWrite( m_ledPin, HIGH);
  isLedOn = true;
  m_onTimer = millis();
  
}

void BlinkLed::slowBlink()
{
  m_onTime= 200;
  m_offTime = 800;
  digitalWrite( m_ledPin, HIGH);
  isLedOn = true;
  m_onTimer = millis();  
}

  



