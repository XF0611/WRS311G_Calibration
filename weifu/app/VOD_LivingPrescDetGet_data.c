/*
 * File: VOD_LivingPrescDetGet_data.c
 * MATLAB Coder version            : 4.3
 *Description:����VOD_LivingPrescDetGet�ı�������
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
float tpVector[1000];//�ۻ��õ��ķ�ֵλ����������
unsigned char frqVector[1000];//�ۻ��õ��ķ�ֵλ��Ƶ����������
float VOD_D1FFTdataAbsDiff_M[5000];//ǰһ���ۻ���D1FFTdata�͵�ǰ�ۻ�D1FFTdata����Ĳ�ֵ��ģ����
extern creal32_T fft1d_data[50];//1֡һάfft���
creal32_T VOD_D1FFTData_M[5000];//�ۻ���100֡һάfft�����ÿ֡��50������
float VOD_peakLoc_M[1000];//�ۻ��õ��ķ�ֵλ�ð�֡����
float VOD_D1FFTAbsLoc_M[100];//���ݷ�ֵλ��ȡ����һάfft����


/*
 * File trailer for VOD_LivingPrescDetGet_data.c
 *
 * [EOF]
 */
