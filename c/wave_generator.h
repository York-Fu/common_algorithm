#ifndef _wave_generator_h_
#define _wave_generator_h_

#include <stdio.h>
#include <math.h>

double_t get_sin_wave(double_t A, double_t T, double_t b, double_t dt);
double_t get_square_wave(double_t A, double_t T, double_t b, double_t dt);
double_t get_triangular_wave(double_t A, double_t T, double_t b, double_t dt);

#endif
