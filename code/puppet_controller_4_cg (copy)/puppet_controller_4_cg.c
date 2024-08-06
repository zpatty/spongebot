/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * puppet_controller_4_cg.c
 *
 * Code generation for function 'puppet_controller_4_cg'
 *
 */

/* Include files */
#include "puppet_controller_4_cg.h"
#include "MC_4_cg.h"
#include "puppet_controller_4_cg_rtwutil.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
void puppet_controller_4_cg(const double q[16], const double dq[16],
                            const double qd[16], const double dqd[16],
                            const double ddqd[16], double d, double m,
                            double mm, double hm, double rm, double r,
                            double kb, double ks, double bb, double bs,
                            double L0, const double Kp[256],
                            const double KD[256], double kc, double ka,
                            double offset, double contact, double conv_pcc,
                            double conv_motor, double tau[16], double cvec[4])
{
  static const signed char inds[12] = {2,  3,  4,  6,  7,  8,
                                       10, 11, 12, 14, 15, 16};
  static const signed char b[9] = {-1, 0, 0, 0, -1, 0, 0, 0, -1};
  static const signed char iv[9] = {0, 1, 0, -1, 0, 0, 0, 0, 1};
  double A[256];
  double D[256];
  double K[256];
  double M[256];
  double conversion[256];
  double Ai[36];
  double C[16];
  double b_C[16];
  double b_dqd[16];
  double b_qd[16];
  double c_K[16];
  double Fci[12];
  double qdp[12];
  double qp[12];
  double b_d;
  double c;
  double d1;
  double d2;
  double theta;
  int A_tmp;
  int K_tmp;
  int b_i;
  int i;
  int ibcol;
  int j;
  int k;
  int kAcol;
  signed char ipiv[16];
  signed char a_tmp_tmp;
  (void)ka;
  qp[0] = q[1];
  qp[3] = q[5];
  qp[6] = q[9];
  qp[9] = q[13];
  qp[1] = q[2];
  qp[4] = q[6];
  qp[7] = q[10];
  qp[10] = q[14];
  qp[2] = q[3];
  qp[5] = q[7];
  qp[8] = q[11];
  qp[11] = q[15];
  MC_4_cg(q, dq, m, mm, hm, rm, r, L0, d, M, C);
  for (kAcol = 0; kAcol < 3; kAcol++) {
    qdp[kAcol] = qd[kAcol + 1];
    qdp[kAcol + 3] = qd[kAcol + 5];
    qdp[kAcol + 6] = qd[kAcol + 9];
    qdp[kAcol + 9] = qd[kAcol + 13];
    ibcol = kAcol << 2;
    Fci[ibcol] = kb;
    Fci[ibcol + 1] = kb;
    Fci[ibcol + 2] = ks;
    Fci[ibcol + 3] = 0.0;
  }
  b_C[0] = 0.0;
  memcpy(&b_C[1], &Fci[0], 12U * sizeof(double));
  b_C[13] = kb;
  b_C[14] = kb;
  b_C[15] = ks;
  memset(&K[0], 0, 256U * sizeof(double));
  for (j = 0; j < 16; j++) {
    K[j + (j << 4)] = b_C[j];
  }
  for (kAcol = 0; kAcol < 3; kAcol++) {
    ibcol = kAcol << 2;
    Fci[ibcol] = bb;
    Fci[ibcol + 1] = bb;
    Fci[ibcol + 2] = bs;
    Fci[ibcol + 3] = 5.0;
  }
  b_C[0] = 0.0;
  memcpy(&b_C[1], &Fci[0], 12U * sizeof(double));
  b_C[13] = bb;
  b_C[14] = bb;
  b_C[15] = bs;
  memset(&D[0], 0, 256U * sizeof(double));
  for (j = 0; j < 16; j++) {
    D[j + (j << 4)] = b_C[j];
  }
  /*  Fci = cell(1,4); */
  /*  Ai = cell(1,4); */
  for (i = 0; i < 4; i++) {
    double Aq[9];
    double b_K[9];
    double b_Kp[9];
    double Dq;
    double Dq_tmp;
    double b_del_tmp;
    double del;
    double del_tmp;
    double sig;
    signed char a_tmp[3];
    signed char b_a_tmp_tmp;
    signed char c_a_tmp_tmp;
    b_d = qp[3 * i];
    b_i = 3 * i + 1;
    d1 = qp[b_i];
    del_tmp = b_d * b_d;
    b_del_tmp = d1 * d1;
    del = sqrt(del_tmp + b_del_tmp);
    theta = del / d;
    if (theta < 1.0E-6) {
      d2 = qp[3 * i + 2];
      c = (L0 + d2) / 3.0;
    } else {
      d2 = qp[3 * i + 2];
      c = 2.0 * ((L0 + d2) / theta - d) * sin(theta / 6.0);
    }
    cvec[i] = c;
    theta = exp(-kc * (c + offset));
    sig = theta / (theta + 1.0);
    Dq_tmp = sin(del);
    Dq = del - Dq_tmp;
    theta = d2 + L0;
    if ((b_d < 1.0E-5) && (d1 < 1.0E-5)) {
      for (A_tmp = 0; A_tmp < 9; A_tmp++) {
        Aq[A_tmp] = iv[A_tmp];
      }
    } else {
      c = rt_powd_snf(del, 3.0);
      Aq[0] = b_d * d1 * Dq / c;
      Aq[3] = (-del_tmp * del - b_del_tmp * Dq_tmp) / c;
      Aq[6] = b_d * Dq * theta / c;
      Aq[1] = (b_del_tmp * del + del_tmp * Dq_tmp) / c;
      Aq[4] = -b_d * d1 * Dq / c;
      Aq[7] = d1 * Dq * theta / c;
      Aq[2] = 0.0;
      Aq[5] = 0.0;
      Aq[8] = Dq_tmp / del;
    }
    a_tmp_tmp = inds[3 * i];
    a_tmp[0] = a_tmp_tmp;
    b_a_tmp_tmp = inds[b_i];
    a_tmp[1] = b_a_tmp_tmp;
    c_a_tmp_tmp = inds[3 * i + 2];
    a_tmp[2] = c_a_tmp_tmp;
    for (b_i = 0; b_i < 3; b_i++) {
      K_tmp = (a_tmp[b_i] - 1) << 4;
      b_K[3 * b_i] = K[(a_tmp_tmp + K_tmp) - 1];
      ibcol = (inds[b_i + 3 * i] - 1) << 4;
      b_Kp[3 * b_i] = Kp[(a_tmp_tmp + ibcol) - 1];
      kAcol = 3 * b_i + 1;
      b_K[kAcol] = K[(b_a_tmp_tmp + K_tmp) - 1];
      b_Kp[kAcol] = Kp[(b_a_tmp_tmp + ibcol) - 1];
      kAcol = 3 * b_i + 2;
      b_K[kAcol] = K[(c_a_tmp_tmp + K_tmp) - 1];
      b_Kp[kAcol] = Kp[(c_a_tmp_tmp + ibcol) - 1];
    }
    for (b_i = 0; b_i < 3; b_i++) {
      b_d = 0.0;
      d1 = 0.0;
      d2 = Aq[b_i];
      theta = Aq[b_i + 3];
      c = Aq[b_i + 6];
      for (A_tmp = 0; A_tmp < 3; A_tmp++) {
        Dq_tmp = qdp[A_tmp + 3 * i];
        K_tmp = b_i + 3 * A_tmp;
        b_d += b_K[K_tmp] * Dq_tmp;
        d1 += b_Kp[K_tmp] * Dq_tmp;
        b_Kp[K_tmp] =
            (d2 * (double)b[3 * A_tmp] + theta * (double)b[3 * A_tmp + 1]) +
            c * (double)b[3 * A_tmp + 2];
      }
      kAcol = b_i + 3 * i;
      Fci[kAcol] = -sig * ((b_d + d1) - qp[kAcol]);
    }
    b_K[0] = 1 * 0.86602540378443871;
    b_K[3] = 1 * 0.86602540378443871;
    b_K[6] = -1;
    b_K[1] = -1 * 0.49999999999999994;
    b_K[4] = 1 * 0.49999999999999994;
    b_K[7] = 0.0;
    b_K[2] = 1.0;
    b_K[5] = 1.0;
    b_K[8] = 1.0;
    for (b_i = 0; b_i < 3; b_i++) {
      b_d = b_Kp[b_i];
      d1 = b_Kp[b_i + 3];
      d2 = b_Kp[b_i + 6];
      for (A_tmp = 0; A_tmp < 3; A_tmp++) {
        Ai[(b_i + 3 * A_tmp) + 9 * i] =
            (b_d * b_K[3 * A_tmp] + d1 * b_K[3 * A_tmp + 1]) +
            d2 * b_K[3 * A_tmp + 2];
      }
    }
  }
  memset(&A[0], 0, 256U * sizeof(double));
  A[0] = 1.0;
  A[68] = 1.0;
  A[136] = 1.0;
  A[204] = 1.0;
  for (b_i = 0; b_i < 3; b_i++) {
    A_tmp = (b_i + 1) << 4;
    A[A_tmp + 1] = Ai[3 * b_i];
    ibcol = (b_i + 5) << 4;
    A[ibcol + 5] = Ai[3 * b_i + 9];
    kAcol = (b_i + 9) << 4;
    A[kAcol + 9] = Ai[3 * b_i + 18];
    K_tmp = (b_i + 13) << 4;
    A[K_tmp + 13] = Ai[3 * b_i + 27];
    A[A_tmp + 2] = Ai[3 * b_i + 1];
    A[ibcol + 6] = Ai[3 * b_i + 10];
    A[kAcol + 10] = Ai[3 * b_i + 19];
    A[K_tmp + 14] = Ai[3 * b_i + 28];
    A[A_tmp + 3] = Ai[3 * b_i + 2];
    A[ibcol + 7] = Ai[3 * b_i + 11];
    A[kAcol + 11] = Ai[3 * b_i + 20];
    A[K_tmp + 15] = Ai[3 * b_i + 29];
  }
  for (kAcol = 0; kAcol < 4; kAcol++) {
    ibcol = kAcol << 2;
    b_C[ibcol] = conv_motor;
    b_C[ibcol + 1] = conv_pcc;
    b_C[ibcol + 2] = conv_pcc;
    b_C[ibcol + 3] = conv_pcc;
  }
  memset(&conversion[0], 0, 256U * sizeof(double));
  for (j = 0; j < 16; j++) {
    conversion[j + (j << 4)] = b_C[j];
    b_d = 0.0;
    d1 = 0.0;
    d2 = 0.0;
    for (b_i = 0; b_i < 16; b_i++) {
      A_tmp = j + (b_i << 4);
      b_d += M[A_tmp] * ddqd[b_i];
      d2 += K[A_tmp] * qd[b_i];
      d1 += D[A_tmp] * dqd[b_i];
    }
    b_d += C[j];
    b_C[j] = b_d;
    C[j] = (b_d + d2) + d1;
  }
  c_K[0] = 0.0 * contact;
  c_K[4] = 0.0 * contact;
  c_K[8] = 0.0 * contact;
  c_K[12] = 0.0 * contact;
  c_K[1] = Fci[0] * contact;
  c_K[5] = Fci[3] * contact;
  c_K[9] = Fci[6] * contact;
  c_K[13] = Fci[9] * contact;
  c_K[2] = Fci[1] * contact;
  c_K[6] = Fci[4] * contact;
  c_K[10] = Fci[7] * contact;
  c_K[14] = Fci[10] * contact;
  c_K[3] = Fci[2] * contact;
  c_K[7] = Fci[5] * contact;
  c_K[11] = Fci[8] * contact;
  c_K[15] = Fci[11] * contact;
  for (b_i = 0; b_i < 16; b_i++) {
    b_qd[b_i] = qd[b_i] - q[b_i];
    b_dqd[b_i] = dqd[b_i] - dq[b_i];
  }
  for (b_i = 0; b_i < 16; b_i++) {
    b_d = 0.0;
    d1 = 0.0;
    for (A_tmp = 0; A_tmp < 16; A_tmp++) {
      kAcol = b_i + (A_tmp << 4);
      b_d += Kp[kAcol] * b_qd[A_tmp];
      d1 += KD[kAcol] * b_dqd[A_tmp];
    }
    b_d += C[b_i] + c_K[b_i];
    c_K[b_i] = d1;
    b_d += d1;
    b_C[b_i] = b_d;
    ipiv[b_i] = (signed char)(b_i + 1);
  }
  for (j = 0; j < 15; j++) {
    int b_tmp;
    int jp1j;
    int mmj_tmp;
    mmj_tmp = 14 - j;
    b_tmp = j * 17;
    jp1j = b_tmp + 2;
    ibcol = 16 - j;
    kAcol = 0;
    theta = fabs(A[b_tmp]);
    for (k = 2; k <= ibcol; k++) {
      c = fabs(A[(b_tmp + k) - 1]);
      if (c > theta) {
        kAcol = k - 1;
        theta = c;
      }
    }
    if (A[b_tmp + kAcol] != 0.0) {
      if (kAcol != 0) {
        kAcol += j;
        ipiv[j] = (signed char)(kAcol + 1);
        for (k = 0; k < 16; k++) {
          ibcol = k << 4;
          K_tmp = j + ibcol;
          theta = A[K_tmp];
          A_tmp = kAcol + ibcol;
          A[K_tmp] = A[A_tmp];
          A[A_tmp] = theta;
        }
      }
      b_i = (b_tmp - j) + 16;
      for (i = jp1j; i <= b_i; i++) {
        A[i - 1] /= A[b_tmp];
      }
    }
    ibcol = b_tmp;
    for (kAcol = 0; kAcol <= mmj_tmp; kAcol++) {
      theta = A[(b_tmp + (kAcol << 4)) + 16];
      if (theta != 0.0) {
        b_i = ibcol + 18;
        A_tmp = (ibcol - j) + 32;
        for (K_tmp = b_i; K_tmp <= A_tmp; K_tmp++) {
          A[K_tmp - 1] += A[((b_tmp + K_tmp) - ibcol) - 17] * -theta;
        }
      }
      ibcol += 16;
    }
    a_tmp_tmp = ipiv[j];
    if (a_tmp_tmp != j + 1) {
      theta = b_C[j];
      b_C[j] = b_C[a_tmp_tmp - 1];
      b_C[a_tmp_tmp - 1] = theta;
    }
  }
  for (k = 0; k < 16; k++) {
    kAcol = k << 4;
    if (b_C[k] != 0.0) {
      b_i = k + 2;
      for (i = b_i; i < 17; i++) {
        b_C[i - 1] -= b_C[k] * A[(i + kAcol) - 1];
      }
    }
  }
  for (k = 15; k >= 0; k--) {
    kAcol = k << 4;
    b_d = b_C[k];
    if (b_d != 0.0) {
      b_d /= A[k + kAcol];
      b_C[k] = b_d;
      for (i = 0; i < k; i++) {
        b_C[i] -= b_C[k] * A[i + kAcol];
      }
    }
  }
  for (b_i = 0; b_i < 16; b_i++) {
    b_d = 0.0;
    for (A_tmp = 0; A_tmp < 16; A_tmp++) {
      b_d += conversion[b_i + (A_tmp << 4)] * b_C[A_tmp];
    }
    tau[b_i] = b_d;
  }
}

/* End of code generation (puppet_controller_4_cg.c) */
