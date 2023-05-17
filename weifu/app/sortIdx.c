/*
 * File: sortIdx.c
 *
 * MATLAB Coder version            : 4.3
 * C/C++ source code generated on  : 04-Aug-2021 16:42:33
 */

/* Include Files */
#include "sortIdx.h"
#include "VOD_D1FFTDataGet.h"
#include "VOD_D1FFTdataAbsDiffGet.h"
#include "VOD_DetRsltShow.h"
#include "VOD_LivingPrescDetGet.h"
#include "rt_nonfinite.h"
#include "uniqueYLfrq.h"
#include <string.h>

/* Function Declarations */
static void merge(int idx_data[], int x_data[], int offset, int np, int nq, int
                  iwork_data[], int xwork_data[]);

/* Function Definitions */

/*
 * Arguments    : int idx_data[]
 *                int x_data[]
 *                int offset
 *                int np
 *                int nq
 *                int iwork_data[]
 *                int xwork_data[]
 * Return Type  : void
 */
static void merge(int idx_data[], int x_data[], int offset, int np, int nq, int
                  iwork_data[], int xwork_data[])
{
  int n_tmp;
  int iout;
  int p;
  int i;
  int q;
  int exitg1;
  if (nq != 0) {
    n_tmp = np + nq;
    for (iout = 0; iout < n_tmp; iout++) {
      i = offset + iout;
      iwork_data[iout] = idx_data[i];
      xwork_data[iout] = x_data[i];
    }

    p = 0;
    q = np;
    iout = offset - 1;
    do {
      exitg1 = 0;
      iout++;
      if (xwork_data[p] <= xwork_data[q]) {
        idx_data[iout] = iwork_data[p];
        x_data[iout] = xwork_data[p];
        if (p + 1 < np) {
          p++;
        } else {
          exitg1 = 1;
        }
      } else {
        idx_data[iout] = iwork_data[q];
        x_data[iout] = xwork_data[q];
        if (q + 1 < n_tmp) {
          q++;
        } else {
          q = iout - p;
          for (iout = p + 1; iout <= np; iout++) {
            i = q + iout;
            idx_data[i] = iwork_data[iout - 1];
            x_data[i] = xwork_data[iout - 1];
          }

          exitg1 = 1;
        }
      }
    } while (exitg1 == 0);
  }
}

/*
 * Arguments    : int x_data[]
 *                const int x_size[1]
 *                int idx_data[]
 *                int idx_size[1]
 * Return Type  : void
 */
void sortIdx(int x_data[], const int x_size[1], int idx_data[], int idx_size[1])
{
  signed char unnamed_idx_0;
  int i3;
  int n;
  int x4[4];
  unsigned char idx4[4];
  int iwork_data[100];
  int xwork_data[100];
  int nQuartets;
  int j;
  int i4;
  int i;
  int nLeft;
  int i1;
  int nPairs;
  signed char perm[4];
  int i2;
  unnamed_idx_0 = (signed char)x_size[0];
  idx_size[0] = unnamed_idx_0;
  i3 = unnamed_idx_0;
  if (0 <= i3 - 1) {
    memset(&idx_data[0], 0, i3 * sizeof(int));
  }

  if (x_size[0] != 0) {
    n = x_size[0];
    x4[0] = 0;
    idx4[0] = 0U;
    x4[1] = 0;
    idx4[1] = 0U;
    x4[2] = 0;
    idx4[2] = 0U;
    x4[3] = 0;
    idx4[3] = 0U;
    i3 = unnamed_idx_0;
    if (0 <= i3 - 1) {
      memset(&iwork_data[0], 0, i3 * sizeof(int));
    }

    i3 = x_size[0];
    if (0 <= i3 - 1) {
      memset(&xwork_data[0], 0, i3 * sizeof(int));
    }

    nQuartets = x_size[0] >> 2;
    for (j = 0; j < nQuartets; j++) {
      i = j << 2;
      idx4[0] = (unsigned char)(i + 1);
      idx4[1] = (unsigned char)(i + 2);
      idx4[2] = (unsigned char)(i + 3);
      idx4[3] = (unsigned char)(i + 4);
      x4[0] = x_data[i];
      i3 = x_data[i + 1];
      x4[1] = i3;
      i4 = x_data[i + 2];
      x4[2] = i4;
      nLeft = x_data[i + 3];
      x4[3] = nLeft;
      if (x_data[i] <= i3) {
        i1 = 1;
        i2 = 2;
      } else {
        i1 = 2;
        i2 = 1;
      }

      if (i4 <= nLeft) {
        i3 = 3;
        i4 = 4;
      } else {
        i3 = 4;
        i4 = 3;
      }

      nLeft = x4[i1 - 1];
      nPairs = x4[i3 - 1];
      if (nLeft <= nPairs) {
        nLeft = x4[i2 - 1];
        if (nLeft <= nPairs) {
          perm[0] = (signed char)i1;
          perm[1] = (signed char)i2;
          perm[2] = (signed char)i3;
          perm[3] = (signed char)i4;
        } else if (nLeft <= x4[i4 - 1]) {
          perm[0] = (signed char)i1;
          perm[1] = (signed char)i3;
          perm[2] = (signed char)i2;
          perm[3] = (signed char)i4;
        } else {
          perm[0] = (signed char)i1;
          perm[1] = (signed char)i3;
          perm[2] = (signed char)i4;
          perm[3] = (signed char)i2;
        }
      } else {
        nPairs = x4[i4 - 1];
        if (nLeft <= nPairs) {
          if (x4[i2 - 1] <= nPairs) {
            perm[0] = (signed char)i3;
            perm[1] = (signed char)i1;
            perm[2] = (signed char)i2;
            perm[3] = (signed char)i4;
          } else {
            perm[0] = (signed char)i3;
            perm[1] = (signed char)i1;
            perm[2] = (signed char)i4;
            perm[3] = (signed char)i2;
          }
        } else {
          perm[0] = (signed char)i3;
          perm[1] = (signed char)i4;
          perm[2] = (signed char)i1;
          perm[3] = (signed char)i2;
        }
      }

      nPairs = perm[0] - 1;
      idx_data[i] = idx4[nPairs];
      i2 = perm[1] - 1;
      idx_data[i + 1] = idx4[i2];
      i3 = perm[2] - 1;
      idx_data[i + 2] = idx4[i3];
      i4 = perm[3] - 1;
      idx_data[i + 3] = idx4[i4];
      x_data[i] = x4[nPairs];
      x_data[i + 1] = x4[i2];
      x_data[i + 2] = x4[i3];
      x_data[i + 3] = x4[i4];
    }

    i4 = nQuartets << 2;
    nLeft = (x_size[0] - i4) - 1;
    if (nLeft + 1 > 0) {
      for (i1 = 0; i1 <= nLeft; i1++) {
        i3 = i4 + i1;
        idx4[i1] = (unsigned char)(i3 + 1);
        x4[i1] = x_data[i3];
      }

      perm[1] = 0;
      perm[2] = 0;
      perm[3] = 0;
      if (nLeft + 1 == 1) {
        perm[0] = 1;
      } else if (nLeft + 1 == 2) {
        if (x4[0] <= x4[1]) {
          perm[0] = 1;
          perm[1] = 2;
        } else {
          perm[0] = 2;
          perm[1] = 1;
        }
      } else if (x4[0] <= x4[1]) {
        if (x4[1] <= x4[2]) {
          perm[0] = 1;
          perm[1] = 2;
          perm[2] = 3;
        } else if (x4[0] <= x4[2]) {
          perm[0] = 1;
          perm[1] = 3;
          perm[2] = 2;
        } else {
          perm[0] = 3;
          perm[1] = 1;
          perm[2] = 2;
        }
      } else if (x4[0] <= x4[2]) {
        perm[0] = 2;
        perm[1] = 1;
        perm[2] = 3;
      } else if (x4[1] <= x4[2]) {
        perm[0] = 2;
        perm[1] = 3;
        perm[2] = 1;
      } else {
        perm[0] = 3;
        perm[1] = 2;
        perm[2] = 1;
      }

      for (i1 = 0; i1 <= nLeft; i1++) {
        nPairs = perm[i1] - 1;
        i2 = i4 + i1;
        idx_data[i2] = idx4[nPairs];
        x_data[i2] = x4[nPairs];
      }
    }

    if (n > 1) {
      nPairs = n >> 2;
      nLeft = 4;
      while (nPairs > 1) {
        if ((nPairs & 1) != 0) {
          nPairs--;
          i3 = nLeft * nPairs;
          i4 = n - i3;
          if (i4 > nLeft) {
            merge(idx_data, x_data, i3, nLeft, i4 - nLeft, iwork_data,
                  xwork_data);
          }
        }

        i3 = nLeft << 1;
        nPairs >>= 1;
        for (i1 = 0; i1 < nPairs; i1++) {
          merge(idx_data, x_data, i1 * i3, nLeft, nLeft, iwork_data, xwork_data);
        }

        nLeft = i3;
      }

      if (n > nLeft) {
        merge(idx_data, x_data, 0, nLeft, n - nLeft, iwork_data, xwork_data);
      }
    }
  }
}

/*
 * File trailer for sortIdx.c
 *
 * [EOF]
 */
