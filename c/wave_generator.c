#include "wave_generator.h"

#define M_PI (3.1415926)

double_t get_sin_wave(double_t A, double_t T, double_t b, double_t dt)
{
  static double_t time = 0, out = 0;
  out = (A * sin((2 * M_PI / T) * time) + b);
  time += dt;
  if (time >= T) // prevention overflow
  {
    time = 0;
  }
  return out;
}

double_t get_square_wave(double_t A, double_t T, double_t b, double_t dt)
{
  static double_t time = 0, out = 0;
  if (time < (T / 2.0))
  {
    out = A + b;
  }
  else
  {
    out = -A + b;
  }
  time += dt;
  if (time >= T)
  {
    time = 0;
  }
  return out;
}

double_t get_triangular_wave(double_t A, double_t T, double_t b, double_t dt)
{
  static double_t time = 0, out = 0;
  if (time < (T / 2.0))
  {
    out = (A / (T / 2.0)) * time + b;
  }
  else
  {
    out = (A / (T / 2.0)) * (T - time) + b;
  }
  time += dt;
  if (time >= T)
  {
    time = 0;
  }
  return out;
}
