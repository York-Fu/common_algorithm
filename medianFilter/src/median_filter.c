#include "median_filter.h"

double MedianFilter(double *pData, int iFilterLen)
{
	int i, j; // 循环变量
	double bTemp;
	double bArray[iFilterLen];
	memcpy(bArray, pData, sizeof(double) * iFilterLen);
	// 用冒泡法对数组进行排序
	for (j = 0; j < iFilterLen - 1; j++)
	{
		for (i = 0; i < iFilterLen - j - 1; i++)
		{
			if (bArray[i] > bArray[i + 1])
			{
				// 互换
				bTemp = bArray[i];
				bArray[i] = bArray[i + 1];
				bArray[i + 1] = bTemp;
			}
		}
	}
	// 计算中值
	if ((iFilterLen & 1) > 0)
	{
		// 数组有奇数个元素，返回中间一个元素
		bTemp = bArray[(iFilterLen + 1) / 2];
	}
	else
	{
		// 数组有偶数个元素，返回中间两个元素平均值
		bTemp = (bArray[iFilterLen / 2 - 1] + bArray[iFilterLen / 2]) / 2.0;
	}
	return bTemp;
}

double AverageMedianFilter(double *pData, int nSize)
{
	double max, min;
	double sum;
	int i = 0;
	if (nSize > 2)
	{
		max = pData[0];
		min = max;
		sum = 0;
		for (i = 0; i < nSize; i++)
		{
			sum += pData[i];
			if (pData[i] > max)
			{
				max = pData[i]; //一个循环之后max就是最大的值
			}

			if (pData[i] < min)
			{
				min = pData[i]; //一个循环之后min就是最小的值
			}
		}

		sum = sum - max - min;							//去掉最大的值和最小的值
		return sum / (double)(nSize - 2); //对N-2个数求平均值
	}

	return 0;
}
