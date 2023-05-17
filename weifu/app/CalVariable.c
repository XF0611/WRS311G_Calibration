/**************************
File name:CalVariable.c
Description:后排遗留检测全局变量
Author:liang.yue
Version:1.0
Date:07/19/2021
History:修改历史记录列表， 每条修改记录应包括修改日期、修改者及修改内容简述。
****************************/

#include "CalVariable.h"
uint16_T  VS_uf_C[9] = {6, 19, 50, 15, 20, 5, 5, 5, 30};
float     VS_ff_C[6] = {5, 2, 0.3, 1.5, 0, 0};
float     VOD_ThresData_M[11] = {5,36,1,1,2,4,10,8,12,25,5};
uint8_T  Car_Indoor_Calib[8]  = {0,0,0,0,0,0,0,0};
unsigned char VOD_FrmNum_C    = 100;

/* Variable Definitions */
unsigned char VOD_BudgetNum=5;
unsigned char VOD_ParameterMaxP=25;
float VOD_maxValueRadio=0.9;
const signed char iv[11] = { 5, 36, 1, 1, 2, 4, 10, 8, 15, 25, 5 };

