#ifndef _median_filter_h_
#define _median_filter_h_

#include <stdio.h>
#include <math.h>
#include "string.h"

double MedianFilter(double *pData, int iFilterLen);
double AverageMedianFilter(double *pData, int nSize);

#endif
