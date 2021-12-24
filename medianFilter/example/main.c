#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#include "median_filter.h"

int main(int argv, char **argc)
{
  uint32_t size = 10;
  double data[size];
  time_t t;
  uint32_t bigen = 5, end = 8;

  printf("array: ");
  for (uint32_t i = 0; i < size; i++)
  {
    data[i] = i;
    printf("%f  ", data[i]);
  }
  printf("\n");
  printf("Median: %f\n", MedianFilter(data, size));
  printf("AverageMedian: %f\n", AverageMedianFilter(data, size));

  printf("\n");
  srand((unsigned int)time(&t));
  printf("array: ");
  for (uint32_t i = 0; i < size; i++)
  {
    data[i] = bigen + (rand() / (double)RAND_MAX) * (end - bigen);
    printf("%f  ", data[i]);
  }
  printf("\n");
  printf("Median: %f\n", MedianFilter(data, size));
  printf("AverageMedian: %f\n", AverageMedianFilter(data, size));

  return 0;
}
