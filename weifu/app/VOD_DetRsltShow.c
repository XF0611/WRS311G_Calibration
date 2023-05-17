/*
 * File: VOD_DetRsltShow.c
 * MATLAB Coder version            : 4.3
 *Description:对检测活体遗留结果比较闪烁不稳定的情况进行稳定和延续，1表示有，0表示无
 *Author:liang.yue
 *Version:1.0
 * C/C++ source code generated on  : 04-Aug-2021 16:42:33
 * 输入　VOD_LivingPrescDet当前遗留检测结果　VOD_DelFrmThres_C阈值参数
 * 输出　LivingPrescDetEnd_tp　最终的遗留检测结果
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


  /*  method1如果当前结果与上次结果一致，则更新持续长度； */
  /*  如果不一致且持续长度小于等于阈值参数，则输出当前结果并初始化持续长度和转变长度; */
  /*  如果不一致且持续长度大于阈值参数，则输出上一次结果并更新转变长度; */
  /*   如果转变长度大于等于阈值参数，则输出当前结果并初始化持续长度和转变长度; */
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
  /* 初始化方法局部变量 */
  VOD_DurLen = 1U;
  VOD_ConvLen = 1U;
  VOD_LivingPrescDetEnd = 0U;
}

/*
 * File trailer for VOD_DetRsltShow.c
 *
 * [EOF]
 */
