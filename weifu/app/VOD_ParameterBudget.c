/*
 * File: VOD_ParameterBudget.c
 *
 * MATLAB Coder version            : 5.1
 * C/C++ source code generated on  : 02-Dec-2021 14:52:29
 */

/* Include Files */
#include "VOD_ParameterBudget.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>
#include "CalVariable.h"
#include "target_doa.h"
#include "VOD_LivingPrescDetGet_data.h"

/* Variable Definitions */
static unsigned char Frame_cnt;
static unsigned char diffStart;
static unsigned char BudgetCnt;
static float VOD_ThresData_sum[11];

/* Function Declarations */
static float rt_hypotf_snf(float u0, float u1);

/* Function Definitions */
/*
 * Arguments    : float u0
 *                float u1
 * Return Type  : float
 */
static float rt_hypotf_snf(float u0, float u1)
{
  float a;
  float y;
  a = fabsf(u0);
  y = fabsf(u1);
  if (a < y) {
    a /= y;
    y *= sqrtf(a * a + 1.0F);
  } else if (a > y) {
    y /= a;
    y = a * sqrtf(y * y + 1.0F);
  } else {
    if (!rtIsNaNF(y)) {
      y = a * 1.41421354F;
    }
  }

  return y;
}

/*
 * Arguments    : void
 * Return Type  : unsigned char
 */
unsigned char VOD_ParameterBudget(void)
{
  float fv[11];
  float edges[6];
  float val[5];
  float delta1;
  float delta2;
  float maxData;
  float miny;
  int exponent;
  int i;
  int idx;
  int k;
  unsigned int qY;
  unsigned char BudgetFlag;
  unsigned char fr;
  bool exitg1;
  bool guard1 = false;
  bool guard2 = false;


  BudgetFlag = 0U;
  guard1 = false;
  guard2 = false;
  if (diffStart == 1) {
    i = (int)((unsigned int)VOD_windowstep_C + VOD_FrmNum_C);
    if ((unsigned int)i > 255U) {
      i = 255;
    }

    if (Frame_cnt_Parameter > i) {
      i = (int)(VOD_windowstep_C + 1U);
      if (VOD_windowstep_C + 1U > 255U) {
        i = 255;
      }

      Frame_cnt_Parameter = (unsigned char)i;
      i = (int)(BudgetCnt + 1U);
      if (BudgetCnt + 1U > 255U) {
        i = 255;
      }

      BudgetCnt = (unsigned char)i;
      if (!rtIsNaNF(VOD_D1FFTdataAbsDiff_M[0])) {
        idx = 1;
      } else {
        idx = 0;
        k = 2;
        exitg1 = false;
        while ((!exitg1) && (k < 5001)) {
          if (!rtIsNaNF(VOD_D1FFTdataAbsDiff_M[k - 1])) {
            idx = k;
            exitg1 = true;
          } else {
            k++;
          }
        }
      }

      if (idx == 0) {
        maxData = VOD_D1FFTdataAbsDiff_M[0];
      } else {
        maxData = VOD_D1FFTdataAbsDiff_M[idx - 1];
        i = idx + 1;
        for (k = i; k < 5001; k++) {
          delta2 = VOD_D1FFTdataAbsDiff_M[k - 1];
          if (maxData < delta2) {
            maxData = delta2;
          }
        }
      }

      /*  meanData=single(mean(VOD_D1FFTdataAbsDiff_M(:))); */
      k = 0;
      while ((k + 1 <= 5000) && (rtIsInfF(VOD_D1FFTdataAbsDiff_M[k]) || rtIsNaNF
              (VOD_D1FFTdataAbsDiff_M[k]))) {
        k++;
      }

      if (k + 1 > 5000) {
        miny = 0.0F;
        delta2 = 0.0F;
      } else {
        miny = VOD_D1FFTdataAbsDiff_M[k];
        delta2 = VOD_D1FFTdataAbsDiff_M[k];
        while (k + 1 <= 5000) {
          if ((!rtIsInfF(VOD_D1FFTdataAbsDiff_M[k])) && (!rtIsNaNF
               (VOD_D1FFTdataAbsDiff_M[k]))) {
            if (VOD_D1FFTdataAbsDiff_M[k] < miny) {
              miny = VOD_D1FFTdataAbsDiff_M[k];
            }

            if (VOD_D1FFTdataAbsDiff_M[k] > delta2) {
              delta2 = VOD_D1FFTdataAbsDiff_M[k];
            }
          }

          k++;
        }
      }

      if (miny == delta2) {
        miny = (miny - 2.0F) - 0.5F;
        delta2 = (delta2 + 3.0F) - 0.5F;
      }

      edges[5] = delta2;
      edges[0] = miny;
      if (miny == -delta2) {
        edges[1] = delta2 * -0.6F;
        edges[2] = delta2 * -0.2F;
        edges[3] = delta2 * 0.2F;
        edges[4] = delta2 * 0.6F;
      } else if (((miny < 0.0F) != (delta2 < 0.0F)) && ((fabsf(miny) >
                   1.70141173E+38F) || (fabsf(delta2) > 1.70141173E+38F))) {
        delta1 = miny / 5.0F;
        delta2 /= 5.0F;
        edges[1] = (miny + delta2) - delta1;
        edges[2] = (miny + delta2 * 2.0F) - delta1 * 2.0F;
        edges[3] = (miny + delta2 * 3.0F) - delta1 * 3.0F;
        edges[4] = (miny + delta2 * 4.0F) - delta1 * 4.0F;
      } else {
        delta1 = (delta2 - miny) / 5.0F;
        edges[1] = miny + delta1;
        edges[2] = miny + 2.0F * delta1;
        edges[3] = miny + 3.0F * delta1;
        edges[4] = miny + 4.0F * delta1;
      }

      delta2 = (edges[1] - miny) / 2.0F;
      for (i = 0; i < 5; i++) {
        val[i] = edges[i] + delta2;
      }

      delta2 = fabsf(edges[1]);
      if ((!rtIsInfF(delta2)) && (!rtIsNaNF(delta2)) && (!(delta2 <=
            1.17549435E-38F))) {
        frexpf(delta2, &exponent);
      }

      delta2 = fabsf(edges[2]);
      if ((!rtIsInfF(delta2)) && (!rtIsNaNF(delta2)) && (!(delta2 <=
            1.17549435E-38F))) {
        frexpf(delta2, &exponent);
      }

      delta2 = fabsf(edges[3]);
      if ((!rtIsInfF(delta2)) && (!rtIsNaNF(delta2)) && (!(delta2 <=
            1.17549435E-38F))) {
        frexpf(delta2, &exponent);
      }

      delta2 = fabsf(edges[4]);
      if ((!rtIsInfF(delta2)) && (!rtIsNaNF(delta2)) && (!(delta2 <=
            1.17549435E-38F))) {
        frexpf(delta2, &exponent);
      }

      fv[0] = VOD_ThresData_sum[0] + 5.0F;
      fv[1] = VOD_ThresData_sum[1] + 36.0F;
      fv[2] = VOD_ThresData_sum[2] + 1.0F;
      fv[3] = VOD_ThresData_sum[3] + 1.0F;
      fv[4] = VOD_ThresData_sum[4] + 2.0F;
      fv[5] = VOD_ThresData_sum[5] + 4.0F;
      fv[6] = VOD_ThresData_sum[6] + 10.0F;
      fv[7] = VOD_ThresData_sum[7] + val[2];
      fv[8] = VOD_ThresData_sum[8] + maxData * VOD_maxValueRadio;
      fv[9] = VOD_ThresData_sum[9] + 25.0F;
      fv[10] = VOD_ThresData_sum[10] + 5.0F;
      for (i = 0; i < 11; i++) {
        VOD_ThresData_sum[i] = fv[i];
      }

      if (BudgetCnt == VOD_BudgetNum) {
        for (i = 0; i < 11; i++) {
          VOD_ThresData_M[i] = VOD_ThresData_sum[i] / (float)VOD_BudgetNum;
        }

        if (VOD_ThresData_M[8] <= VOD_ParameterMaxP) {
          BudgetFlag = 1U;
          Frame_cnt_Parameter = 1U;
          memset(&VOD_D1FFTData_M[0], 0, 5000U * sizeof(creal32_T));
          memset(&VOD_D1FFTdataAbsDiff_M[0], 0, 5000U * sizeof(float));
        } else {
          BudgetFlag = 2U;
          Frame_cnt_Parameter = 1U;
          memset(&VOD_D1FFTData_M[0], 0, 5000U * sizeof(creal32_T));
          memset(&VOD_D1FFTdataAbsDiff_M[0], 0, 5000U * sizeof(float));
          for (i = 0; i < 11; i++) {
            VOD_ThresData_M[i] = iv[i];
          }
        }
      } else {
        guard2 = true;
      }
    } else {
      guard2 = true;
    }
  } else {
    guard1 = true;
  }

  if (guard2) {
    /*  VOD_D1FFTdataAbsDiffGet用于得到并累积差分之后的FFT数据 */
    /*  每一帧得到fft1d_data数据之后，将其与上一帧的fft数据均求模后再相减， */
    /*  最后再求模，得到这一帧的差分fft数据，并把它存放在矩阵VOD_D1FFTdataAbsDiff_Ｍ */
    /*  的对应位置 */
    /*  输入　fft1d_data当前帧一维FFT结果　VOD_D1FFTData_M累计的一维FFT结果 */
    /*  输出  VOD_D1FFTdataAbsDiff_Ｍ　累计的差分之后的FFT结果 */
    /*  滑窗VOD_windowstep_C帧 */
    if (VOD_D1FFTdataAbsDiff_M[VOD_FrmNum_C - 1] != 0.0F) {
      for (fr = 2; fr <= VOD_FrmNum_C; fr++) {
        for (i = 0; i < 50; i++) {
          idx = fr + 100 * i;
          VOD_D1FFTdataAbsDiff_M[idx - 2] = VOD_D1FFTdataAbsDiff_M[idx - 1];
        }
      }

      qY = (unsigned int)VOD_FrmNum_C - VOD_windowstep_C;
      if (qY > VOD_FrmNum_C) {
        qY = 0U;
      }

      i = (int)(qY + 1U);
      if (qY + 1U > 255U) {
        i = 255;
      }

      for (k = 0; k < 50; k++) {
        idx = ((unsigned char)i + 100 * k) - 1;
        VOD_D1FFTdataAbsDiff_M[(VOD_FrmNum_C + 100 * k) - 1] = fabsf
          (rt_hypotf_snf(fft1d_data[k].re, fft1d_data[k].im) - rt_hypotf_snf
           (VOD_D1FFTData_M[idx].re, VOD_D1FFTData_M[idx].im));
      }
    } else {
      qY = (unsigned int)VOD_FrmNum_C - VOD_windowstep_C;
      if (qY > VOD_FrmNum_C) {
        qY = 0U;
      }

      i = (int)(qY + 1U);
      if (qY + 1U > 255U) {
        i = 255;
      }

      qY = (unsigned int)Frame_cnt_Parameter - VOD_windowstep_C;
      if (qY > Frame_cnt_Parameter) {
        qY = 0U;
      }

      for (k = 0; k < 50; k++) {
        idx = ((unsigned char)i + 100 * k) - 1;
        VOD_D1FFTdataAbsDiff_M[((int)qY + 100 * k) - 1] = fabsf(rt_hypotf_snf
          (fft1d_data[k].re, fft1d_data[k].im) - rt_hypotf_snf
          (VOD_D1FFTData_M[idx].re, VOD_D1FFTData_M[idx].im));
      }
    }

    guard1 = true;
  }

  if (guard1) {
    /*  VOD_D1FFTDataGet用于逐帧得到并累积的一维FFT数据, */
    /*  输入 fft1d_data当前帧一维FFT结果 */
    /*  输出　VOD_D1FFTData_M累计的一维FFT结果 */
    for (fr = 2; fr <= VOD_FrmNum_C; fr++) {
      for (i = 0; i < 50; i++) {
        idx = fr + 100 * i;
        VOD_D1FFTData_M[idx - 2] = VOD_D1FFTData_M[idx - 1];
      }

      /*          VS_D1FFT(fr-1,:)=VS_D1FFT(fr,:); */
    }

    for (i = 0; i < 50; i++) {
      VOD_D1FFTData_M[(VOD_FrmNum_C + 100 * i) - 1] = fft1d_data[i];
    }

    i = (int)(Frame_cnt + 1U);
    if (Frame_cnt + 1U > 255U) {
      i = 255;
    }

    Frame_cnt = (unsigned char)i;
    i = (int)(Frame_cnt_Parameter + 1U);
    if (Frame_cnt_Parameter + 1U > 255U) {
      i = 255;
    }

    Frame_cnt_Parameter = (unsigned char)i;
    if (Frame_cnt > VOD_windowstep_C) {
      Frame_cnt = 1U;
      diffStart = 1U;
    }
  }

  return BudgetFlag;
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void VOD_ParameterBudget_init(void)
{
  int i;

  /* 初始化方法局部变量 */
  Frame_cnt = 1U;
  diffStart = 0U;
  BudgetCnt = 0U;
  for (i = 0; i < 11; i++) {
    VOD_ThresData_sum[i] = 0.0F;
  }
}

/*
 * File trailer for VOD_ParameterBudget.c
 *
 * [EOF]
 */
