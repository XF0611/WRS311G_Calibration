/*
 * File: uniqueYLfrq.c
 * Description:统计出现的数据及其频次矩阵
 * MATLAB Coder version            : 4.3
 * C/C++ source code generated on  : 04-Aug-2021 16:42:33
 * 输入输出说明
 * tpVector：需要统计频次的矩阵 frqVector：统计的频次矩阵初始化
 * frq:%统计的频次矩阵，uniqueTp删除重复的数据之后的矩阵
 * Arguments    : unsigned char frq[20]
 *                float uniqueTp[20]
 * Return Type  : void
 */

/* Include Files */
#include "uniqueYLfrq.h"
#include "VOD_D1FFTDataGet.h"
#include "VOD_D1FFTdataAbsDiffGet.h"
#include "VOD_DetRsltShow.h"
#include "VOD_LivingPrescDetGet.h"
#include "VOD_LivingPrescDetGet_data.h"
#include "rt_nonfinite.h"
#include <string.h>


void uniqueYLfrq(unsigned char frq[20], float uniqueTp[20])
{
  int i;
  unsigned short u;
  unsigned short m;
  int j;
  boolean_T exitg1;
  /*  矩阵初始化 */
  memset(&uniqueTp[0], 0, 20U * sizeof(float));
  for (i = 0; i < 20; i++) {
    frq[i] = 0U;
  }
  /*  根据输入的数据进行筛选，只要出现过一次的数据就记录保存下来，并统计其出现频次*/
  for (i = 0; i < 1000; i++) {
    u = (unsigned short)(i + 2);
    for (m = u; m < 1001; m++) {
      j = m - 1;
      if (tpVector[j] == tpVector[i]) {
        tpVector[j] = 0.0F;
        if (tpVector[i] != 0.0F) {
          j = (int)(frqVector[i] + 1U);
          if ((unsigned int)j > 255U) {
            j = 255;
          }

          frqVector[i] = (unsigned char)j;
        }
      }
    }
  }

  /*  将输出矩阵大小控制在20以内 */
  i = 0;
  j = 0;
  exitg1 = false;
  while ((!exitg1) && (j < 1000)) {
    if (tpVector[j] != 0.0F) {
      uniqueTp[i] = tpVector[j];
      frq[i] = frqVector[j];
      i++;
      if (i + 1 > 20) {
        exitg1 = true;
      } else {
        j++;
      }
    } else {
      j++;
    }
  }
}

/*
 * File trailer for uniqueYLfrq.c
 *
 * [EOF]
 */
