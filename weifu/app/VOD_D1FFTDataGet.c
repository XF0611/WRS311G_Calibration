/*
 * File: VOD_D1FFTDataGet.c
 *Description:累积一维FFT结果至100帧
 *Author:liang.yue
 *Version:1.0
 *Date:07/19/2021
 * 输入输出说明
 * fft1d_data：当前帧一维fft数据；
 * VOD_D1FFTData_M：累积的100帧一维fft数据矩阵
 *History:修改历史记录列表， 每条修改记录应包括修改日期、修改者及修改内容简述。
 * MATLAB Coder version            : 4.3
 * C/C++ source code generated on  : 04-Aug-2021 16:42:33
 */

/* Include Files */
#include "VOD_D1FFTDataGet.h"
#include "VOD_D1FFTdataAbsDiffGet.h"
#include "VOD_DetRsltShow.h"
#include "VOD_LivingPrescDetGet.h"
#include "VOD_LivingPrescDetGet_data.h"
#include "rt_nonfinite.h"
#include "uniqueYLfrq.h"

/* Function Definitions */

/*
 * const define %%
 * Arguments    : void
 * Return Type  : void
 */
void VOD_D1FFTDataGet(void)
{
  unsigned char fr;
  int i;
  int VOD_D1FFTData_M_tmp;


  for (fr = 2; fr <= VOD_FrmNum_C; fr++) {
	  /* 删除第一帧，保存当前帧数据*/
    for (i = 0; i < 50; i++) {
      VOD_D1FFTData_M_tmp = fr + 100 * i;
      VOD_D1FFTData_M[VOD_D1FFTData_M_tmp - 2] =
        VOD_D1FFTData_M[VOD_D1FFTData_M_tmp - 1];
    }

    /*          VS_D1FFT(fr-1,:)=VS_D1FFT(fr,:); */
  }
  /* 将当前帧一维数据保存在矩阵VOD_D1FFTData_M对应位置上*/
  for (i = 0; i < 50; i++) {
    VOD_D1FFTData_M[(VOD_FrmNum_C + 100 * i) - 1] = fft1d_data[i];
  }
}

/*
 * File trailer for VOD_D1FFTDataGet.c
 *
 * [EOF]
 */
