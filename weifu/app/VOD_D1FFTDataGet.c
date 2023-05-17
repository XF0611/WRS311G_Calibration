/*
 * File: VOD_D1FFTDataGet.c
 *Description:�ۻ�һάFFT�����100֡
 *Author:liang.yue
 *Version:1.0
 *Date:07/19/2021
 * �������˵��
 * fft1d_data����ǰ֡һάfft���ݣ�
 * VOD_D1FFTData_M���ۻ���100֡һάfft���ݾ���
 *History:�޸���ʷ��¼�б� ÿ���޸ļ�¼Ӧ�����޸����ڡ��޸��߼��޸����ݼ�����
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
	  /* ɾ����һ֡�����浱ǰ֡����*/
    for (i = 0; i < 50; i++) {
      VOD_D1FFTData_M_tmp = fr + 100 * i;
      VOD_D1FFTData_M[VOD_D1FFTData_M_tmp - 2] =
        VOD_D1FFTData_M[VOD_D1FFTData_M_tmp - 1];
    }

    /*          VS_D1FFT(fr-1,:)=VS_D1FFT(fr,:); */
  }
  /* ����ǰ֡һά���ݱ����ھ���VOD_D1FFTData_M��Ӧλ����*/
  for (i = 0; i < 50; i++) {
    VOD_D1FFTData_M[(VOD_FrmNum_C + 100 * i) - 1] = fft1d_data[i];
  }
}

/*
 * File trailer for VOD_D1FFTDataGet.c
 *
 * [EOF]
 */
