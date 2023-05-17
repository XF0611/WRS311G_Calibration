/*
 * File: VOD_DetRsltShow.c
 * MATLAB Coder version            : 4.3
 *Description:�Լ�������������Ƚ���˸���ȶ�����������ȶ���������1��ʾ�У�0��ʾ��
 *Author:liang.yue
 *Version:1.0
 * C/C++ source code generated on  : 04-Aug-2021 16:42:33
 * ���롡VOD_LivingPrescDet��ǰ�����������VOD_DelFrmThres_C��ֵ����
 * �����LivingPrescDetEnd_tp�����յ����������
 * Arguments    : unsigned char VOD_LivingPrescDet
 *                unsigned char VOD_DelFrmThres_C
 * Return Type  : unsigned char
 */


/* Include Files */
#include "VOD_DetRsltShow.h"
#include "VOD_D1FFTDataGet.h"
#include "VOD_D1FFTdataAbsDiffGet.h"
#include "VOD_LivingPrescDetGet.h"
#include "VOD_LivingPrescDetGet_data.h"
#include "rt_nonfinite.h"
#include "uniqueYLfrq.h"

/* Variable Definitions */
static unsigned char VOD_LivingPrescDetEnd;
static unsigned char VOD_ConvLen;
static unsigned short VOD_DurLen;

/* Function Definitions */


unsigned char VOD_DetRsltShow(unsigned char VOD_LivingPrescDet, unsigned char
  VOD_DelFrmThres_C)
{
  unsigned int u;
  int i;


  /*  method1�����ǰ������ϴν��һ�£�����³������ȣ� */
  /*  �����һ���ҳ�������С�ڵ�����ֵ�������������ǰ�������ʼ���������Ⱥ�ת�䳤��; */
  /*  �����һ���ҳ������ȴ�����ֵ�������������һ�ν��������ת�䳤��; */
  /*   ���ת�䳤�ȴ��ڵ�����ֵ�������������ǰ�������ʼ���������Ⱥ�ת�䳤��; */
  if (VOD_LivingPrescDetEnd == VOD_LivingPrescDet) {
    u = VOD_DurLen + 1U;
    if (u > 65535U) {
      u = 65535U;
    }

    VOD_DurLen = (unsigned short)u;
    VOD_ConvLen = 0U;
  } else if (VOD_DurLen <= VOD_DelFrmThres_C) {
    VOD_LivingPrescDetEnd = VOD_LivingPrescDet;
    VOD_DurLen = 1U;
    VOD_ConvLen = 0U;
  } else {
    if (VOD_DurLen > VOD_DelFrmThres_C) {
      i = (int)(VOD_ConvLen + 1U);
      if ((unsigned int)i > 255U) {
        i = 255;
      }

      VOD_ConvLen = (unsigned char)i;
    }
  }

  if (VOD_ConvLen >= VOD_DelFrmThres_C) {
    VOD_LivingPrescDetEnd = VOD_LivingPrescDet;
    VOD_ConvLen = 0U;
    VOD_DurLen = 1U;
  }

  return VOD_LivingPrescDetEnd;
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void VOD_DetRsltShow_init(void)
{
  /* ��ʼ�������ֲ����� */
  VOD_DurLen = 1U;
  VOD_ConvLen = 1U;
  VOD_LivingPrescDetEnd = 0U;
}

/*
 * File trailer for VOD_DetRsltShow.c
 *
 * [EOF]
 */
