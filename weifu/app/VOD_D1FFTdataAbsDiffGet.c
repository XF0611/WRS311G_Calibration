/*
 * File: VOD_D1FFTdataAbsDiffGet.c
 *Description:�ۻ����һάFFT�����100֡
 *Author:liang.yue
 *Version:1.0
 *Date:07/19/2021
 * �������˵��
 * fft1d_data����ǰ֡һάfft���ݣ�VOD_D1FFTData_M���ۻ���100֡һάfft���ݾ���
 * VOD_D1FFTdataAbsDiff_M:�ۻ���100֡��һάfft���ݲ����ģ�������
 *History:�޸���ʷ��¼�б� ÿ���޸ļ�¼Ӧ�����޸����ڡ��޸��߼��޸����ݼ�����
 * MATLAB Coder version            : 4.3
 * C/C++ source code generated on  : 04-Aug-2021 16:42:33
 */

/* Include Files */
#include "VOD_D1FFTdataAbsDiffGet.h"
#include "VOD_D1FFTDataGet.h"
#include "VOD_DetRsltShow.h"
#include "VOD_LivingPrescDetGet.h"
#include "VOD_LivingPrescDetGet_data.h"
#include "rt_nonfinite.h"
#include "uniqueYLfrq.h"
#include <math.h>
#include "target_doa.h"

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
  float y;
  float a;
  a = (float)fabs(u0);
  y = (float)fabs(u1);
  if (a < y) {
    a /= y;
    y *= (float)sqrt(a * a + 1.0F);
  } else if (a > y) {
    y /= a;
    y = a * (float)sqrt(y * y + 1.0F);
  } else {
    if (!rtIsNaNF(y)) {
      y = a * 1.41421354F;
    }
  }

  return y;
}

/*
 * const define %%
 * Arguments    : void
 * Return Type  : void
 */
void VOD_D1FFTdataAbsDiffGet(void)
{
  unsigned int qY;
  unsigned char fr;
  int i;
  int k;
  int re_tmp;


  /*  ����VOD_windowstep_C֡ */
  if (VOD_D1FFTdataAbsDiff_M[VOD_FrmNum_C - 1] != 0.0F) {
	  /* ɾ����һ֡���ݣ����浱ǰ֡����*/
    for (fr = 2; fr <= VOD_FrmNum_C; fr++) {
      for (i = 0; i < 50; i++) {
        k = fr + 100 * i;
        VOD_D1FFTdataAbsDiff_M[k - 2] = VOD_D1FFTdataAbsDiff_M[k - 1];
      }
    }
    /* ���㵱ǰ֡���ݸô�ŵ�����λ��*/
    qY = (unsigned int)VOD_FrmNum_C - VOD_windowstep_C;
    if (qY > VOD_FrmNum_C) {
      qY = 0U;
    }

    i = (int)(qY + 1U);
    if ((unsigned int)i > 255U) {
      i = 255;
    }

    for (k = 0; k < 50; k++) {
      re_tmp = (i + 100 * k) - 1;
      VOD_D1FFTdataAbsDiff_M[(VOD_FrmNum_C + 100 * k) - 1] = (float)fabs
        (rt_hypotf_snf(fft1d_data[k].re, fft1d_data[k].im) - rt_hypotf_snf
         (VOD_D1FFTData_M[re_tmp].re, VOD_D1FFTData_M[re_tmp].im));
    }
  } else {
    qY = (unsigned int)VOD_FrmNum_C - VOD_windowstep_C;
    if (qY > VOD_FrmNum_C) {
      qY = 0U;
    }

    i = (int)(qY + 1U);
    if ((unsigned int)i > 255U) {
      i = 255;
    }

    qY = (unsigned int)Frame_cnt2 - VOD_windowstep_C;
    if (qY > Frame_cnt2) {
      qY = 0U;
    }
    /*  �õ�ǰ֡��һά�����ģ��ȥ��һ֡��һά�����ģ�������ģ */
    for (k = 0; k < 50; k++) {
      re_tmp = (i + 100 * k) - 1;
      VOD_D1FFTdataAbsDiff_M[((int)qY + 100 * k) - 1] = (float)fabs
        (rt_hypotf_snf(fft1d_data[k].re, fft1d_data[k].im) - rt_hypotf_snf
         (VOD_D1FFTData_M[re_tmp].re, VOD_D1FFTData_M[re_tmp].im));
    }
  }
}

/*
 * File trailer for VOD_D1FFTdataAbsDiffGet.c
 *
 * [EOF]
 */
