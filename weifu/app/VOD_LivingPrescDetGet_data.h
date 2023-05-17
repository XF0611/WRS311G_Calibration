/*
 * File: VOD_LivingPrescDetGet_data.h
 *
 * MATLAB Coder version            : 4.3
 * C/C++ source code generated on  : 04-Aug-2021 16:42:33
 */

#ifndef VOD_LIVINGPRESCDETGET_DATA_H
#define VOD_LIVINGPRESCDETGET_DATA_H

/* Include Files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "VOD_LivingPrescDetGet_types.h"

/* Variable Declarations */
extern float tpVector[1000];//累积得到的峰值位置依次排列
extern unsigned char frqVector[1000];//累积得到的峰值位置频次依次排列
extern float VOD_D1FFTdataAbsDiff_M[5000];//前一次累积的D1FFTdata和当前累积D1FFTdata矩阵的差值求模矩阵
extern unsigned char VOD_FrmNum_C;
extern creal32_T fft1d_data[50];//1帧一维fft结果
extern creal32_T VOD_D1FFTData_M[5000];//累积的100帧一维fft结果，每帧有50个复数
extern float VOD_peakLoc_M[1000];//累积得到的峰值位置按帧排列
extern float VOD_D1FFTAbsLoc_M[100];//根据峰值位置取出的一维fft数据


#endif

/*
 * File trailer for VOD_LivingPrescDetGet_data.h
 *
 * [EOF]
 */
