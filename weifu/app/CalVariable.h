/**************************
File name:CalVariable.h
Description:后排遗留检测全局变量定义
Author:liang.yue
Version:1.0
Date:07/19/2021
History:修改历史记录列表， 每条修改记录应包括修改日期、修改者及修改内容简述。
****************************/


#include "rtwtypes.h"
#include "EECU_Types.h"

extern uint16_T  VS_uf_C[9];
extern float     VS_ff_C[6];
extern float     VOD_ThresData_M[11];
extern unsigned char VOD_FrmNum_C;
extern uint8_T  Car_Indoor_Calib[8];

/* Variable Declarations */
extern unsigned char VOD_BudgetNum;
extern unsigned char VOD_ParameterMaxP;
extern float VOD_maxValueRadio;
extern const signed char iv[11];
