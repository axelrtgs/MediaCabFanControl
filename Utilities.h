#pragma once

#ifndef Utilities_h
#define Utilities_h

void printLine(String text) 
{
  Serial.println (text);
}

void printLine(String text, String val) 
{
  Serial.print (text);
  Serial.println (val);
}

void printLine(const __FlashStringHelper* text) 
{
  Serial.println (text);
}

void printLine(const __FlashStringHelper* text, String val) 
{
  Serial.print (text);
  Serial.println (val);
}
#endif
