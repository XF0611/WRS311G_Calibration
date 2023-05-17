/*
 * File: findpeaks.c
 *
 * MATLAB Coder version            : 4.3
 * C/C++ source code generated on  : 04-Aug-2021 16:42:33
 */

/* Include Files */
#include "findpeaks.h"
#include "VOD_D1FFTDataGet.h"
#include "VOD_D1FFTdataAbsDiffGet.h"
#include "VOD_DetRsltShow.h"
#include "VOD_LivingPrescDetGet.h"
#include "eml_setop.h"
#include "rt_nonfinite.h"
#include "sort.h"
#include "uniqueYLfrq.h"
#include <string.h>

/* Function Declarations */
static void c_findPeaksSeparatedByMoreThanM(const float y_data[], const double
  x_data[], const int iPk_data[], const int iPk_size[1], float Pd, int idx_data[],
  int idx_size[1]);

/* Function Definitions */

/*
 * Arguments    : const float y_data[]
 *                const double x_data[]
 *                const int iPk_data[]
 *                const int iPk_size[1]
 *                float Pd
 *                int idx_data[]
 *                int idx_size[1]
 * Return Type  : void
 */
static void c_findPeaksSeparatedByMoreThanM(const float y_data[], const double
  x_data[], const int iPk_data[], const int iPk_size[1], float Pd, int idx_data[],
  int idx_size[1])
{
  int n;
  int sortIdx_size_idx_0;
  int yk;
  int sortIdx_data[100];
  int i;
  int k;
  int b_i;
  int i2;
  int j;
  double locs_temp_data[100];
  int pEnd;
  boolean_T idelete_data[100];
  int p;
  int q;
  int qEnd;
  double x;
  double b_x;
  int kEnd;
  boolean_T tmp_data[100];
  signed char b_tmp_data[100];
  int iwork_data[100];
  if ((iPk_size[0] == 0) || (Pd == 0.0F)) {
    if (iPk_size[0] < 1) {
      n = 0;
    } else {
      n = iPk_size[0];
    }

    if (n > 0) {
      sortIdx_data[0] = 1;
      yk = 1;
      for (k = 2; k <= n; k++) {
        yk++;
        sortIdx_data[k - 1] = yk;
      }
    }

    idx_size[0] = n;
    if (0 <= n - 1) {
      memcpy(&idx_data[0], &sortIdx_data[0], n * sizeof(int));
    }
  } else {
    n = iPk_size[0] + 1;
    sortIdx_size_idx_0 = (signed char)iPk_size[0];
    yk = (signed char)iPk_size[0];
    if (0 <= yk - 1) {
      memset(&sortIdx_data[0], 0, yk * sizeof(int));
    }

    i = iPk_size[0] - 1;
    for (k = 1; k <= i; k += 2) {
      yk = iPk_data[k - 1] - 1;
      if ((y_data[yk] >= y_data[iPk_data[k] - 1]) || rtIsNaNF(y_data[yk])) {
        sortIdx_data[k - 1] = k;
        sortIdx_data[k] = k + 1;
      } else {
        sortIdx_data[k - 1] = k + 1;
        sortIdx_data[k] = k;
      }
    }

    if ((iPk_size[0] & 1) != 0) {
      sortIdx_data[iPk_size[0] - 1] = iPk_size[0];
    }

    b_i = 2;
    while (b_i < n - 1) {
      i2 = b_i << 1;
      j = 1;
      for (pEnd = b_i + 1; pEnd < n; pEnd = qEnd + b_i) {
        p = j - 1;
        q = pEnd;
        qEnd = j + i2;
        if (qEnd > n) {
          qEnd = n;
        }

        k = 0;
        kEnd = qEnd - j;
        while (k + 1 <= kEnd) {
          i = iPk_data[sortIdx_data[p] - 1] - 1;
          yk = sortIdx_data[q - 1];
          if ((y_data[i] >= y_data[iPk_data[yk - 1] - 1]) || rtIsNaNF(y_data[i]))
          {
            iwork_data[k] = sortIdx_data[p];
            p++;
            if (p + 1 == pEnd) {
              while (q < qEnd) {
                k++;
                iwork_data[k] = sortIdx_data[q - 1];
                q++;
              }
            }
          } else {
            iwork_data[k] = yk;
            q++;
            if (q == qEnd) {
              while (p + 1 < pEnd) {
                k++;
                iwork_data[k] = sortIdx_data[p];
                p++;
              }
            }
          }

          k++;
        }

        for (k = 0; k < kEnd; k++) {
          sortIdx_data[(j + k) - 1] = iwork_data[k];
        }

        j = qEnd;
      }

      b_i = i2;
    }

    for (i = 0; i < sortIdx_size_idx_0; i++) {
      locs_temp_data[i] = x_data[iPk_data[sortIdx_data[i] - 1] - 1];
    }

    if (0 <= sortIdx_size_idx_0 - 1) {
      memset(&idelete_data[0], 0, sortIdx_size_idx_0 * sizeof(boolean_T));
    }

    for (b_i = 0; b_i < sortIdx_size_idx_0; b_i++) {
      if (!idelete_data[b_i]) {
        yk = iPk_data[sortIdx_data[b_i] - 1] - 1;
        x = (float)x_data[yk] - Pd;
        b_x = (float)x_data[yk] + Pd;
        for (i = 0; i < sortIdx_size_idx_0; i++) {
          tmp_data[i] = ((locs_temp_data[i] >= x) && (locs_temp_data[i] <= b_x));
        }

        for (i = 0; i < sortIdx_size_idx_0; i++) {
          idelete_data[i] = (idelete_data[i] || tmp_data[i]);
        }

        idelete_data[b_i] = false;
      }
    }

    yk = sortIdx_size_idx_0 - 1;
    j = 0;
    for (b_i = 0; b_i <= yk; b_i++) {
      if (!idelete_data[b_i]) {
        j++;
      }
    }

    i2 = 0;
    for (b_i = 0; b_i <= yk; b_i++) {
      if (!idelete_data[b_i]) {
        b_tmp_data[i2] = (signed char)(b_i + 1);
        i2++;
      }
    }

    idx_size[0] = j;
    for (i = 0; i < j; i++) {
      idx_data[i] = sortIdx_data[b_tmp_data[i] - 1];
    }

    sort(idx_data, idx_size);
  }
}

/*
 * Arguments    : const float Yin_data[]
 *                const int Yin_size[2]
 *                float varargin_2
 *                float varargin_4
 *                float varargin_6
 *                float Ypk_data[]
 *                int Ypk_size[2]
 *                double Xpk_data[]
 *                int Xpk_size[2]
 * Return Type  : void
 */
void findpeaks(const float Yin_data[], const int Yin_size[2], float varargin_2,
               float varargin_4, float varargin_6, float Ypk_data[], int
               Ypk_size[2], double Xpk_data[], int Xpk_size[2])
{
  int ny;
  int nPk;
  int nInf;
  char dir;
  int kfirst;
  float ykfirst;
  boolean_T isinfykfirst;
  int k;
  float yk;
  int i;
  boolean_T isinfyk;
  int iInfinite_size[1];
  int iInfinite_data[50];
  char previousdir;
  int iFinite_data[50];
  int iPk_size[1];
  int iPk_data[50];
  int c_data[100];
  int c_size[1];
  int iInflect_data[50];
  int iInflect_size[1];
  int iFinite_size[1];
  double tmp_data[50];
  int idx_data[100];
  int b_idx_data[100];
  ny = Yin_size[1];
  nPk = 0;
  nInf = 0;
  dir = 'n';
  kfirst = 0;
  ykfirst = rtInfF;
  isinfykfirst = true;
  for (k = 1; k <= ny; k++) {
    yk = Yin_data[k - 1];
    if (rtIsNaNF(yk)) {
      yk = rtInfF;
      isinfyk = true;
    } else if (rtIsInfF(yk) && (yk > 0.0F)) {
      isinfyk = true;
      nInf++;
      iInfinite_data[nInf - 1] = k;
    } else {
      isinfyk = false;
    }

    if (yk != ykfirst) {
      previousdir = dir;
      if (isinfyk || isinfykfirst) {
        dir = 'n';
      } else if (yk < ykfirst) {
        dir = 'd';
        if (('d' != previousdir) && (previousdir == 'i')) {
          nPk++;
          iFinite_data[nPk - 1] = kfirst;
        }
      } else {
        dir = 'i';
      }

      ykfirst = yk;
      kfirst = k;
      isinfykfirst = isinfyk;
    }
  }

  if (1 > nPk) {
    i = 0;
  } else {
    i = nPk;
  }

  if (1 > nInf) {
    iInfinite_size[0] = 0;
  } else {
    iInfinite_size[0] = nInf;
  }

  nPk = 0;
  for (k = 0; k < i; k++) {
    ykfirst = Yin_data[iFinite_data[k] - 1];
    if (ykfirst > varargin_4) {
      yk = Yin_data[iFinite_data[k] - 2];
      if ((!(yk > Yin_data[iFinite_data[k]])) && (!rtIsNaNF
           (Yin_data[iFinite_data[k]]))) {
        yk = Yin_data[iFinite_data[k]];
      }

      if (ykfirst - yk >= varargin_6) {
        nPk++;
        iPk_data[nPk - 1] = iFinite_data[k];
      }
    }
  }

  if (1 > nPk) {
    iPk_size[0] = 0;
  } else {
    iPk_size[0] = nPk;
  }

  do_vectors(iPk_data, iPk_size, iInfinite_data, iInfinite_size, c_data, c_size,
             iInflect_data, iInflect_size, iFinite_data, iFinite_size);
  ny = (int)((double)Yin_size[1] - 1.0);
  for (i = 0; i <= ny; i++) {
    tmp_data[i] = (double)i + 1.0;
  }

  c_findPeaksSeparatedByMoreThanM(Yin_data, tmp_data, c_data, c_size, varargin_2,
    idx_data, iInfinite_size);
  if (iInfinite_size[0] > 10) {
    for (i = 0; i < 10; i++) {
      b_idx_data[i] = idx_data[i];
    }

    iInfinite_size[0] = 10;
    for (i = 0; i < 10; i++) {
      idx_data[i] = b_idx_data[i];
    }
  }

  kfirst = iInfinite_size[0];
  ny = iInfinite_size[0];
  for (i = 0; i < ny; i++) {
    b_idx_data[i] = c_data[idx_data[i] - 1];
  }

  Ypk_size[0] = 1;
  Ypk_size[1] = iInfinite_size[0];
  for (i = 0; i < kfirst; i++) {
    Ypk_data[i] = Yin_data[b_idx_data[i] - 1];
  }

  Xpk_size[0] = 1;
  Xpk_size[1] = iInfinite_size[0];
  for (i = 0; i < kfirst; i++) {
    Xpk_data[i] = (signed char)((signed char)(b_idx_data[i] - 1) + 1);
  }
}

/*
 * File trailer for findpeaks.c
 *
 * [EOF]
 */
