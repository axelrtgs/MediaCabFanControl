#pragma once

template<typename T>
T clamp(const T &Value, const T &Min, const T &Max)
{
  return (Value < Min)? Min : (Value > Max)? Max : Value;
}
