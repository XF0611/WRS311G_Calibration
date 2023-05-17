/*
 * File: sort.c
 *
 * MATLAB Coder version            : 4.3
 * C/C++ source code generated on  : 04-Aug-2021 16:42:33
 */

/* Include Files */
#include "sort.h"
#include "VOD_D1FFTDataGet.h"
#include "VOD_D1FFTdataAbsDiffGet.h"
#include "VOD_DetRsltShow.h"
#include "VOD_LivingPrescDetGet.h"
#include "rt_nonfinite.h"
#include "sortIdx.h"
#include "uniqueYLfrq.h"

/* Type Definitions */
#ifndef struct_emxArray_int32_T_100
#define struct_emxArray_int32_T_100

struct emxArray_int32_T_100
{
  int data[100];
  int size[1];
};

#endif                                 /*struct_emxArray_int32_T_100*/

#ifndef typedef_emxArray_int32_T_100
#define typedef_emxArray_int32_T_100

typedef struct emxArray_int32_T_100 emxArray_int32_T_100;

#endif                                 /*typedef_emxArray_int32_T_100*/

/* Function Definitions */

/*
 * Arguments    : int x_data[]
 *                const int x_size[1]
 * Return Type  : void
 */
void sort(int x_data[], const int x_size[1])
{
  int dim;
  int j;
  int vlen;
  int vwork_size[1];
  int vstride;
  int k;
  int vwork_data[100];
  emxArray_int32_T_100 b_vwork_data;
  dim = 0;
  if (x_size[0] != 1) {
    dim = -1;
  }

  if (dim + 2 <= 1) {
    j = x_size[0];
  } else {
    j = 1;
  }

  vlen = j - 1;
  vwork_size[0] = j;
  vstride = 1;
  for (k = 0; k <= dim; k++) {
    vstride *= x_size[0];
  }

  for (j = 0; j < vstride; j++) {
    for (k = 0; k <= vlen; k++) {
      vwork_data[k] = x_data[j + k * vstride];
    }

    sortIdx(vwork_data, vwork_size, b_vwork_data.data, b_vwork_data.size);
    for (k = 0; k <= vlen; k++) {
      x_data[j + k * vstride] = vwork_data[k];
    }
  }
}

/*
 * File trailer for sort.c
 *
 * [EOF]
 */
