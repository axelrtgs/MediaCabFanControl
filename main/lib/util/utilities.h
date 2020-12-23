#pragma once

#ifndef Utilities_h
#define Utilities_h

template <typename T>
T clamp_val(const T &Value, const T &Min, const T &Max) {
  return std::max(Min, std::min(Value, Max))
}
#endif
