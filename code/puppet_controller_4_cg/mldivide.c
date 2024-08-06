/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * mldivide.c
 *
 * Code generation for function 'mldivide'
 *
 */

/* Include files */
#include "mldivide.h"
#include "rt_nonfinite.h"
#include "xgetrf.h"
#include <string.h>

/* Function Definitions */
void mldivide(const double A[256], double B[16])
{
  double b_A[256];
  double temp;
  int ipiv[16];
  int i;
  int info;
  int k;
  int kAcol;
  memcpy(&b_A[0], &A[0], 256U * sizeof(double));
  xgetrf(b_A, ipiv);
  for (i = 0; i < 15; i++) {
    info = ipiv[i];
    if (info != i + 1) {
      temp = B[i];
      B[i] = B[info - 1];
      B[info - 1] = temp;
    }
  }
  for (k = 0; k < 16; k++) {
    kAcol = k << 4;
    if (B[k] != 0.0) {
      info = k + 2;
      for (i = info; i < 17; i++) {
        B[i - 1] -= B[k] * b_A[(i + kAcol) - 1];
      }
    }
  }
  for (k = 15; k >= 0; k--) {
    kAcol = k << 4;
    temp = B[k];
    if (temp != 0.0) {
      temp /= b_A[k + kAcol];
      B[k] = temp;
      for (i = 0; i < k; i++) {
        B[i] -= B[k] * b_A[i + kAcol];
      }
    }
  }
}

/* End of code generation (mldivide.c) */
