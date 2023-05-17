/*
 * File: VOD_LivingPrescDetGet.c
 * MATLAB Coder version            : 4.3
 *Description:根据包含有目标微动信息的数据进行目标特征提取，将活体和静止非活体目标区分开，1表示有，0表示无
 *Author:liang.yue
 *Version:1.0
 * C/C++ source code generated on  : 04-Aug-2021 16:42:33
 * 输入VOD_ThresData_M  阈值参数　　VOD_DelFrmThres_C阈值参数
 * 输出　VOD_D1FFTdataAbsDiff_M　包含目标微动信息的数据矩阵
 * Arguments    : unsigned char VOD_LivingPrescDet
 *                unsigned char VOD_DelFrmThres_C
 * Return Type  : unsigned char
 */

/* Include Files */
#include "VOD_LivingPrescDetGet.h"
#include "VOD_D1FFTDataGet.h"
#include "VOD_D1FFTdataAbsDiffGet.h"
#include "VOD_DetRsltShow.h"
#include "VOD_LivingPrescDetGet_data.h"
#include "findpeaks.h"
#include "rt_nonfinite.h"
#include "uniqueYLfrq.h"
#include <math.h>
#include <string.h>
#include "embARC_debug.h"


unsigned char VOD_LivingPrescDetGet(const float VOD_ThresData_M[11])
{
  unsigned char VOD_LivingPrescDet;
  float VOD_MinLoc_C;
  float VOD_MaxLoc_C;
  unsigned char VOD_EmptyCnt;
  int i;
  int k;
  int i1;
  int i2;
  int i3;
  int loop_ub_tmp;
  unsigned char VOD_Frq_M[20];
  float VOD_Unique_M[20];
  float x_data[50];
  float VOD_Mean_tp;
  int i4;
  int b_k;
  int i5;
  boolean_T exitg1;
  int b_loop_ub_tmp;
  int i6;
  int n;
  float VOD_Max_tp;
  int idx;
  float f;
  int x_size[2];
  float VOD_peakLoc_tp_data[100];
  int VOD_peakLoc_tp_size[2];
  double b_VOD_peakLoc_tp_data[100];
  int b_VOD_peakLoc_tp_size[2];

  /*变量初始化 */
  VOD_MinLoc_C = VOD_ThresData_M[0]; /* 搜峰时限制的最小位置（实车标定） */
  VOD_MaxLoc_C = VOD_ThresData_M[1];  /* 搜峰时限制的最大位置（实车标定） */

  VOD_EmptyCnt = 0U;  /* 底噪计数 */
  VOD_LivingPrescDet = 0U; /*活体检测结果0无 1有 */
  memset(&VOD_peakLoc_M[0], 0, 1000U * sizeof(float));/*峰值位置矩阵 */
  memset(&VOD_D1FFTAbsLoc_M[0], 0, 100U * sizeof(float));/*根据峰值位置索引提取的数据 */
  memset(&tpVector[0], 0, 1000U * sizeof(float));/*统计数据唯一性 */
  memset(&frqVector[0], 0, 1000U * sizeof(unsigned char));/*统计数据出现频次 */

  /*  */
  /*  VOD_peakGet VOD_peakGet 查找符合条件的峰值索引位置 */
  i = VOD_FrmNum_C;
  if (0 <= VOD_FrmNum_C - 1) {
    if (VOD_ThresData_M[0] > VOD_ThresData_M[1]) {
      i1 = -1;
      i2 = -1;
    } else {
      i1 = (int)VOD_ThresData_M[0] - 2;
      i2 = (int)VOD_ThresData_M[1] - 1;
    }

    loop_ub_tmp = i2 - i1;
    if (VOD_ThresData_M[0] > VOD_ThresData_M[1]) {
      i4 = 0;
      i5 = -1;
    } else {
      i4 = (int)VOD_ThresData_M[0] - 1;
      i5 = (int)VOD_ThresData_M[1] - 1;
    }

    b_loop_ub_tmp = i5 - i4;
    n = b_loop_ub_tmp + 1;
  }
  /* 每一帧数据的平均值计算  */
  for (k = 0; k < i; k++) {
    for (i3 = 0; i3 < loop_ub_tmp; i3++) {
      x_data[i3] = VOD_D1FFTdataAbsDiff_M[k + 100 * ((i1 + i3) + 1)];
    }

    if (i2 - i1 == 0) {
      VOD_Mean_tp = 0.0F;
    } else {
      VOD_Mean_tp = VOD_D1FFTdataAbsDiff_M[k + 100 * (i1 + 1)];
      for (b_k = 2; b_k <= loop_ub_tmp; b_k++) {
        VOD_Mean_tp += x_data[b_k - 1];
      }
    }

    VOD_Mean_tp /= (float)(i2 - i1);
    i3 = (i5 - i4) + 1;
    /* 每一帧数据的最大值计算  */
    for (i6 = 0; i6 <= b_loop_ub_tmp; i6++) {
      x_data[i6] = VOD_D1FFTdataAbsDiff_M[k + 100 * (i4 + i6)];
    }

    if ((i5 - i4) + 1 <= 2) {
      if (i3 == 1) {
        VOD_Max_tp = VOD_D1FFTdataAbsDiff_M[k + 100 * i4];
      } else {
        i3 = k + 100 * i4;
        i6 = k + 100 * (i4 + 1);
        if ((VOD_D1FFTdataAbsDiff_M[i3] < VOD_D1FFTdataAbsDiff_M[i6]) ||
            (rtIsNaNF(VOD_D1FFTdataAbsDiff_M[i3]) && (!rtIsNaNF
              (VOD_D1FFTdataAbsDiff_M[i6])))) {
          VOD_Max_tp = VOD_D1FFTdataAbsDiff_M[i6];
        } else {
          VOD_Max_tp = VOD_D1FFTdataAbsDiff_M[i3];
        }
      }
    } else {
      i6 = k + 100 * i4;
      if (!rtIsNaNF(VOD_D1FFTdataAbsDiff_M[i6])) {
        idx = 1;
      } else {
        idx = 0;
        b_k = 2;
        exitg1 = false;
        while ((!exitg1) && (b_k <= i3)) {
          if (!rtIsNaNF(x_data[b_k - 1])) {
            idx = b_k;
            exitg1 = true;
          } else {
            b_k++;
          }
        }
      }

      if (idx == 0) {
        VOD_Max_tp = VOD_D1FFTdataAbsDiff_M[i6];
      } else {
        VOD_Max_tp = VOD_D1FFTdataAbsDiff_M[k + 100 * ((i4 + idx) - 1)];
        i3 = idx + 1;
        for (b_k = i3; b_k <= n; b_k++) {
          f = x_data[b_k - 1];
          if (VOD_Max_tp < f) {
            VOD_Max_tp = f;
          }
        }
      }
    }
    /* 底噪帧数统计 */
    if ((VOD_Mean_tp <= VOD_ThresData_M[7]) && (VOD_Max_tp <= VOD_ThresData_M[8]))
    {
      if (k + 1 >= 50) {
        i3 = (int)(VOD_EmptyCnt + 1U);
        if ((unsigned int)i3 > 255U) {
          i3 = 255;
        }

        VOD_EmptyCnt = (unsigned char)i3;
      }
    } else {
      if (VOD_MinLoc_C > VOD_MaxLoc_C) {
        i3 = 0;
        i6 = 0;
      } else {
        i3 = (int)VOD_MinLoc_C - 1;
        i6 = (int)VOD_MaxLoc_C;
      }

      x_size[0] = 1;
      b_k = i6 - i3;
      x_size[1] = b_k;
      for (i6 = 0; i6 < b_k; i6++) {
        x_data[i6] = VOD_D1FFTdataAbsDiff_M[k + 100 * (i3 + i6)];
      }

      findpeaks(x_data, x_size, VOD_ThresData_M[2], VOD_Mean_tp *
                VOD_ThresData_M[3], VOD_ThresData_M[4], VOD_peakLoc_tp_data,
                VOD_peakLoc_tp_size, b_VOD_peakLoc_tp_data,
                b_VOD_peakLoc_tp_size);
      b_k = b_VOD_peakLoc_tp_size[0] * b_VOD_peakLoc_tp_size[1];
      for (i3 = 0; i3 < b_k; i3++) {
        VOD_peakLoc_tp_data[i3] = (float)b_VOD_peakLoc_tp_data[i3];
      }

      i3 = b_VOD_peakLoc_tp_size[1];
      for (b_k = 0; b_k < i3; b_k++) {
        VOD_peakLoc_tp_data[b_k] = (VOD_peakLoc_tp_data[b_k] + VOD_MinLoc_C) -
          1.0F;
      }

      b_k = b_VOD_peakLoc_tp_size[1];
      for (i3 = 0; i3 < b_k; i3++) {
        VOD_peakLoc_M[k + 100 * i3] = VOD_peakLoc_tp_data[i3];
      }
    }
  }

  memcpy(&tpVector[0], &VOD_peakLoc_M[0], 1000U * sizeof(float));
  uniqueYLfrq(VOD_Frq_M, VOD_Unique_M);  /* 统计出现过的数值及其出现频次 */


  /*  根据一维累计结果进行活体遗留检测，根据之前得到的峰值索引位置进行多帧的数据提取
并关注多帧数据的强度变化趋势，若变化较大，则判断为有活体遗留，若变化不大，则判断为无活体遗留*/
  b_k = 0;
  exitg1 = false;
  while ((!exitg1) && (b_k < 20)) {
    if ((VOD_Unique_M[b_k] == 0.0F) || (VOD_Frq_M[b_k] <= VOD_ThresData_M[6])) {
      b_k++;
    } else {
      for (i = 0; i < 100; i++) {
        VOD_D1FFTAbsLoc_M[i] = VOD_D1FFTdataAbsDiff_M[i + 100 * ((int)
          VOD_Unique_M[b_k] - 1)];
      }

      VOD_Mean_tp = VOD_D1FFTAbsLoc_M[0];
      for (k = 0; k < 99; k++) {
        VOD_Mean_tp += VOD_D1FFTAbsLoc_M[k + 1];
      }

      VOD_Mean_tp /= 100.0F;
      VOD_Max_tp = 0.0F;
      VOD_MinLoc_C = 1.29246971E-26F;
      for (k = 0; k < 100; k++) {
        f = (float)fabs(VOD_D1FFTAbsLoc_M[k] - VOD_Mean_tp);
        if (f > VOD_MinLoc_C) {
          VOD_MaxLoc_C = VOD_MinLoc_C / f;
          VOD_Max_tp = VOD_Max_tp * VOD_MaxLoc_C * VOD_MaxLoc_C + 1.0F;
          VOD_MinLoc_C = f;
        } else {
          VOD_MaxLoc_C = f / VOD_MinLoc_C;
          VOD_Max_tp += VOD_MaxLoc_C * VOD_MaxLoc_C;
        }
      }

      if (VOD_MinLoc_C * (float)sqrt(VOD_Max_tp) / 9.94987392F >=
          VOD_ThresData_M[5]) {
        VOD_LivingPrescDet = 1U; /* 有活体*/
        exitg1 = true;
      } else {
        b_k++;/* 无活体*/
      }
    }
  }

  /*  若低噪帧数统计超过阈值，则强制判断为无活体 */
  if (VOD_EmptyCnt > VOD_ThresData_M[9]) {
    VOD_LivingPrescDet = 0U;/* 无活体*/

  }

  return VOD_LivingPrescDet;
}

/*
 * File trailer for VOD_LivingPrescDetGet.c
 *
 * [EOF]
 */
