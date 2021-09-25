#ifndef _filter_algorithm_h_
#define _filter_algorithm_h_

#include <stdio.h>
#include <math.h>

double_t GetMedianNum(double_t *dataArray, int iFilterLen);
double_t MedianFilter(double_t *pData, int nSize);

#endif
