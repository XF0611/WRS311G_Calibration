/*
 * File: VOD_LivingPrescDetGet_data.c
 * MATLAB Coder version            : 4.3
 *Description:函数VOD_LivingPrescDetGet的变量定义
 *Author:liang.yue
 *Version:1.0
 * C/C++ source code generated on  : 04-Aug-2021 16:42:33
 * Arguments    : unsigned char VOD_LivingPrescDet
 *                unsigned char VOD_DelFrmThres_C
 * Return Type  : unsigned char
 */

/* Include Files */
#include "VOD_LivingPrescDetGet_data.h"
#include "VOD_D1FFTDataGet.h"
#include "VOD_D1FFTdataAbsDiffGet.h"
#include "VOD_DetRsltShow.h"
#include "VOD_LivingPrescDetGet.h"
#include "rt_nonfinite.h"
#include "uniqueYLfrq.h"

/* Variable Definitions */
float tpVector[1000];//累积得到的峰值位置依次排列
unsigned char frqVector[1000];//累积得到的峰值位置频次依次排列
float VOD_D1FFTdataAbsDiff_M[5000];//前一次累积的D1FFTdata和当前累积D1FFTdata矩阵的差值求模矩阵
extern creal32_T fft1d_data[50];//1帧一维fft结果
creal32_T VOD_D1FFTData_M[5000];//累积的100帧一维fft结果，每帧有50个复数
float VOD_peakLoc_M[1000];//累积得到的峰值位置按帧排列
float VOD_D1FFTAbsLoc_M[100];//根据峰值位置取出的一维fft数据


/*
 * File trailer for VOD_LivingPrescDetGet_data.c
 *
 * [EOF]
 */
