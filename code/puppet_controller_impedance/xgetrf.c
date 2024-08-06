/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * xgetrf.c
 *
 * Code generation for function 'xgetrf'
 *
 */

/* Include files */
#include "xgetrf.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Function Definitions */
int xgetrf(double A[256], int ipiv[16])
{
  int i;
  int info;
  int j;
  int jA;
  int jp1j;
  int k;
  int temp_tmp;
  for (i = 0; i < 16; i++) {
    ipiv[i] = i + 1;
  }
  info = 0;
  for (j = 0; j < 15; j++) {
    double smax;
    int a;
    int b_tmp;
    int mmj_tmp;
    mmj_tmp = 14 - j;
    b_tmp = j * 17;
    jp1j = b_tmp + 2;
    jA = 16 - j;
    a = 0;
    smax = fabs(A[b_tmp]);
    for (k = 2; k <= jA; k++) {
      double s;
      s = fabs(A[(b_tmp + k) - 1]);
      if (s > smax) {
        a = k - 1;
        smax = s;
      }
    }
    if (A[b_tmp + a] != 0.0) {
      if (a != 0) {
        a += j;
        ipiv[j] = a + 1;
        for (k = 0; k < 16; k++) {
          jA = k << 4;
          temp_tmp = j + jA;
          smax = A[temp_tmp];
          i = a + jA;
          A[temp_tmp] = A[i];
          A[i] = smax;
        }
      }
      i = (b_tmp - j) + 16;
      for (jA = jp1j; jA <= i; jA++) {
        A[jA - 1] /= A[b_tmp];
      }
    } else {
      info = j + 1;
    }
    jA = b_tmp;
    for (temp_tmp = 0; temp_tmp <= mmj_tmp; temp_tmp++) {
      smax = A[(b_tmp + (temp_tmp << 4)) + 16];
      if (smax != 0.0) {
        i = jA + 18;
        a = (jA - j) + 32;
        for (jp1j = i; jp1j <= a; jp1j++) {
          A[jp1j - 1] += A[((b_tmp + jp1j) - jA) - 17] * -smax;
        }
      }
      jA += 16;
    }
  }
  if ((info == 0) && (!(A[255] != 0.0))) {
    info = 16;
  }
  return info;
}

/* End of code generation (xgetrf.c) */
