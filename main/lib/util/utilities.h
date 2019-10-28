#pragma once

#include <iterator>
#include <numeric>
#include <vector>

#ifndef Utilities_h
#define Utilities_h

template <typename T>
inline auto average_list(const std::initializer_list<T> &v) -> decltype(T() / 1.0)
{
  return std::accumulate(std::begin(v), std::end(v), T()) / static_cast<float>(std::distance(std::begin(v), std::end(v)));
}

template <typename T>
inline auto  average_vector(std::vector<T> const& v) -> decltype(T() / 1.0) 
{
    return std::accumulate(v.begin(), v.end(), 0LL) / v.size();
}

inline double average(const double numbers[], const int &count )
{
  double sum = 0;

  double average;

  for (int i = 0 ; i < count; i++)
    sum += numbers[i];

  average = static_cast<double>(sum) / count;
  return average;
}

template<typename T>
T clamp(const T &Value, const T &Min, const T &Max)
{
    return (Value < Min)? Min : (Value > Max)? Max : Value;
}
#endif
