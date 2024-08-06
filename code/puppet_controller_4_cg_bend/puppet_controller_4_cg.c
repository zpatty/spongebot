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
                            double L0, const double Kp[256], double KD,
                            double kc, double ka, double offset, double contact,
                            double conv_pcc, double conv_motor, double tau[16],
                            double cq[4])
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
  double b_K[16];
  double v[16];
  double Fci[12];
  double qdp[12];
  double qp[12];
  double b_d;
  double d1;
  double dc_dtheta;
  double theta;
  int A_tmp;
  int b_Ki_tmp;
  int b_i;
  int i;
  int ibcol;
  int ibtile;
  int j;
  int k;
  signed char ipiv[16];
  signed char Ki_tmp_tmp;
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
  for (ibtile = 0; ibtile < 3; ibtile++) {
    qdp[ibtile] = qd[ibtile + 1];
    qdp[ibtile + 3] = qd[ibtile + 5];
    qdp[ibtile + 6] = qd[ibtile + 9];
    qdp[ibtile + 9] = qd[ibtile + 13];
    ibcol = ibtile << 2;
    Fci[ibcol] = kb;
    Fci[ibcol + 1] = kb;
    Fci[ibcol + 2] = ks;
    Fci[ibcol + 3] = 0.0;
  }
  v[0] = 0.0;
  memcpy(&v[1], &Fci[0], 12U * sizeof(double));
  v[13] = kb;
  v[14] = kb;
  v[15] = ks;
  memset(&K[0], 0, 256U * sizeof(double));
  for (j = 0; j < 16; j++) {
    K[j + (j << 4)] = v[j];
  }
  for (ibtile = 0; ibtile < 3; ibtile++) {
    ibcol = ibtile << 2;
    Fci[ibcol] = bb;
    Fci[ibcol + 1] = bb;
    Fci[ibcol + 2] = bs;
    Fci[ibcol + 3] = 5.0;
  }
  v[0] = 0.0;
  memcpy(&v[1], &Fci[0], 12U * sizeof(double));
  v[13] = bb;
  v[14] = bb;
  v[15] = bs;
  memset(&D[0], 0, 256U * sizeof(double));
  for (j = 0; j < 16; j++) {
    D[j + (j << 4)] = v[j];
  }
  /*  Fci = cell(1,4); */
  /*  Ai = cell(1,4); */
  for (i = 0; i < 4; i++) {
    double Aq[9];
    double Ki[9];
    double Kpi[9];
    double b_y_tmp[9];
    double y_tmp[9];
    double dv[3];
    double Dq;
    double Dq_tmp;
    double L_tmp;
    double b_del_tmp;
    double c;
    double dc_dL;
    double dc_dx;
    double dc_dy;
    double del;
    double del_tmp;
    signed char Ki_tmp[3];
    signed char b_Ki_tmp_tmp;
    signed char c_Ki_tmp_tmp;
    b_d = qp[3 * i];
    b_i = 3 * i + 1;
    d1 = qp[b_i];
    del_tmp = b_d * b_d;
    b_del_tmp = d1 * d1;
    del = sqrt(del_tmp + b_del_tmp);
    theta = del / d;
    Dq_tmp = sin(del);
    Dq = del - Dq_tmp;
    A_tmp = 3 * i + 2;
    L_tmp = qp[A_tmp] + L0;
    if ((b_d < 1.0E-5) && (d1 < 1.0E-5)) {
      c = L_tmp / 3.0;
      dc_dx = 0.0;
      dc_dy = 0.0;
      dc_dL = 0.33333333333333331;
      for (ibcol = 0; ibcol < 9; ibcol++) {
        Aq[ibcol] = iv[ibcol];
      }
    } else {
      dc_dL = sin(theta / 6.0);
      c = 2.0 * (L_tmp / theta - d) * dc_dL;
      dc_dtheta = -(cos(theta / 6.0) * (2.0 * d - 2.0 * L_tmp / theta)) / 6.0 -
                  2.0 * dc_dL * L_tmp / (theta * theta);
      dc_dx = dc_dtheta * (b_d / del / d);
      dc_dy = dc_dtheta * (d1 / del / d);
      dc_dL *= 2.0 / theta;
      theta = rt_powd_snf(del, 3.0);
      Aq[0] = b_d * d1 * Dq / theta;
      Aq[3] = (-del_tmp * del - b_del_tmp * Dq_tmp) / theta;
      Aq[6] = b_d * Dq * L_tmp / theta;
      Aq[1] = (b_del_tmp * del + del_tmp * Dq_tmp) / theta;
      Aq[4] = -b_d * d1 * Dq / theta;
      Aq[7] = d1 * Dq * L_tmp / theta;
      Aq[2] = 0.0;
      Aq[5] = 0.0;
      Aq[8] = Dq_tmp / del;
    }
    cq[i] = c;
    theta = c + offset;
    dc_dtheta = exp(-kc * theta);
    del = dc_dtheta / (dc_dtheta + 1.0);
    dc_dtheta = exp(-theta * kc);
    Ki_tmp_tmp = inds[3 * i];
    Ki_tmp[0] = Ki_tmp_tmp;
    b_Ki_tmp_tmp = inds[b_i];
    Ki_tmp[1] = b_Ki_tmp_tmp;
    c_Ki_tmp_tmp = inds[A_tmp];
    Ki_tmp[2] = c_Ki_tmp_tmp;
    dc_dtheta = 0.5 * (kc * exp(-2.0 * theta * kc) /
                           ((dc_dtheta + 1.0) * (dc_dtheta + 1.0)) -
                       kc * dc_dtheta / (dc_dtheta + 1.0));
    b_d = dc_dtheta * dc_dx;
    d1 = dc_dtheta * dc_dy;
    dc_dtheta *= dc_dL;
    for (b_i = 0; b_i < 3; b_i++) {
      A_tmp = b_i + 3 * i;
      theta = qdp[A_tmp] - qp[A_tmp];
      dv[b_i] = theta;
      b_Ki_tmp = (Ki_tmp[b_i] - 1) << 4;
      Ki[3 * b_i] = K[(Ki_tmp_tmp + b_Ki_tmp) - 1];
      ibcol = (inds[A_tmp] - 1) << 4;
      Kpi[3 * b_i] = Kp[(Ki_tmp_tmp + ibcol) - 1];
      y_tmp[3 * b_i] = b_d * theta;
      ibtile = 3 * b_i + 1;
      Ki[ibtile] = K[(b_Ki_tmp_tmp + b_Ki_tmp) - 1];
      Kpi[ibtile] = Kp[(b_Ki_tmp_tmp + ibcol) - 1];
      y_tmp[ibtile] = d1 * theta;
      ibtile = 3 * b_i + 2;
      Ki[ibtile] = K[(c_Ki_tmp_tmp + b_Ki_tmp) - 1];
      Kpi[ibtile] = Kp[(c_Ki_tmp_tmp + ibcol) - 1];
      y_tmp[ibtile] = dc_dtheta * theta;
    }
    for (b_i = 0; b_i < 3; b_i++) {
      b_d = 0.0;
      d1 = y_tmp[b_i + 3];
      dc_dtheta = y_tmp[b_i + 6];
      theta = y_tmp[b_i];
      dc_dL = 0.0;
      del_tmp = 0.0;
      for (A_tmp = 0; A_tmp < 3; A_tmp++) {
        ibcol = 3 * A_tmp + 1;
        ibtile = 3 * A_tmp + 2;
        b_del_tmp = qdp[A_tmp + 3 * i];
        b_Ki_tmp = b_i + 3 * A_tmp;
        dc_dL += del * Ki[b_Ki_tmp] * b_del_tmp;
        del_tmp += ((theta * Ki[3 * A_tmp] + d1 * Ki[ibcol]) +
                    dc_dtheta * Ki[ibtile]) *
                   b_del_tmp;
        b_y_tmp[b_Ki_tmp] = (theta * Kpi[3 * A_tmp] + d1 * Kpi[ibcol]) +
                            dc_dtheta * Kpi[ibtile];
        b_d += del * Kpi[b_Ki_tmp] * dv[A_tmp];
      }
      Fci[b_i + 3 * i] =
          -ka * (((dc_dL - del_tmp) - b_d) +
                 ((b_y_tmp[b_i] * dv[0] + b_y_tmp[b_i + 3] * dv[1]) +
                  b_y_tmp[b_i + 6] * dv[2]));
    }
    Ki[0] = d * 0.86602540378443871;
    Ki[3] = d * 0.86602540378443871;
    Ki[6] = -d;
    Ki[1] = -d * 0.49999999999999994;
    Ki[4] = d * 0.49999999999999994;
    Ki[7] = 0.0;
    for (b_i = 0; b_i < 3; b_i++) {
      b_d = Aq[b_i];
      d1 = Aq[b_i + 3];
      dc_dtheta = Aq[b_i + 6];
      for (A_tmp = 0; A_tmp < 3; A_tmp++) {
        Kpi[b_i + 3 * A_tmp] =
            (b_d * (double)b[3 * A_tmp] + d1 * (double)b[3 * A_tmp + 1]) +
            dc_dtheta * (double)b[3 * A_tmp + 2];
      }
      Ki[3 * b_i + 2] = 1.0;
    }
    for (b_i = 0; b_i < 3; b_i++) {
      b_d = Kpi[b_i];
      d1 = Kpi[b_i + 3];
      dc_dtheta = Kpi[b_i + 6];
      for (A_tmp = 0; A_tmp < 3; A_tmp++) {
        Ai[(b_i + 3 * A_tmp) + 9 * i] =
            (b_d * Ki[3 * A_tmp] + d1 * Ki[3 * A_tmp + 1]) +
            dc_dtheta * Ki[3 * A_tmp + 2];
      }
    }
    ibtile = i << 2;
    v[ibtile] = conv_motor;
    v[ibtile + 1] = conv_pcc;
    v[ibtile + 2] = conv_pcc;
    v[ibtile + 3] = conv_pcc;
  }
  memset(&conversion[0], 0, 256U * sizeof(double));
  for (j = 0; j < 16; j++) {
    conversion[j + (j << 4)] = v[j];
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
    ibtile = (b_i + 9) << 4;
    A[ibtile + 9] = Ai[3 * b_i + 18];
    b_Ki_tmp = (b_i + 13) << 4;
    A[b_Ki_tmp + 13] = Ai[3 * b_i + 27];
    A[A_tmp + 2] = Ai[3 * b_i + 1];
    A[ibcol + 6] = Ai[3 * b_i + 10];
    A[ibtile + 10] = Ai[3 * b_i + 19];
    A[b_Ki_tmp + 14] = Ai[3 * b_i + 28];
    A[A_tmp + 3] = Ai[3 * b_i + 2];
    A[ibcol + 7] = Ai[3 * b_i + 11];
    A[ibtile + 11] = Ai[3 * b_i + 20];
    A[b_Ki_tmp + 15] = Ai[3 * b_i + 29];
  }
  b_C[0] = 0.0 * contact;
  b_C[4] = 0.0 * contact;
  b_C[8] = 0.0 * contact;
  b_C[12] = 0.0 * contact;
  b_C[1] = Fci[0] * contact;
  b_C[5] = Fci[3] * contact;
  b_C[9] = Fci[6] * contact;
  b_C[13] = Fci[9] * contact;
  b_C[2] = Fci[1] * contact;
  b_C[6] = Fci[4] * contact;
  b_C[10] = Fci[7] * contact;
  b_C[14] = Fci[10] * contact;
  b_C[3] = Fci[2] * contact;
  b_C[7] = Fci[5] * contact;
  b_C[11] = Fci[8] * contact;
  b_C[15] = Fci[11] * contact;
  for (b_i = 0; b_i < 16; b_i++) {
    b_d = 0.0;
    d1 = 0.0;
    dc_dtheta = 0.0;
    for (A_tmp = 0; A_tmp < 16; A_tmp++) {
      ibcol = b_i + (A_tmp << 4);
      b_d += M[ibcol] * ddqd[A_tmp];
      dc_dtheta += K[ibcol] * qd[A_tmp];
      d1 += D[ibcol] * dqd[A_tmp];
    }
    C[b_i] = ((C[b_i] + b_d) + dc_dtheta) + d1;
    b_K[b_i] = qd[b_i] - q[b_i];
  }
  for (b_i = 0; b_i < 16; b_i++) {
    b_d = 0.0;
    for (A_tmp = 0; A_tmp < 16; A_tmp++) {
      b_d += Kp[b_i + (A_tmp << 4)] * b_K[A_tmp];
    }
    v[b_i] = ((C[b_i] + b_C[b_i]) + b_d) + KD * (dqd[b_i] - dq[b_i]);
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
    ibtile = 0;
    dc_dtheta = fabs(A[b_tmp]);
    for (k = 2; k <= ibcol; k++) {
      theta = fabs(A[(b_tmp + k) - 1]);
      if (theta > dc_dtheta) {
        ibtile = k - 1;
        dc_dtheta = theta;
      }
    }
    if (A[b_tmp + ibtile] != 0.0) {
      if (ibtile != 0) {
        ibtile += j;
        ipiv[j] = (signed char)(ibtile + 1);
        for (k = 0; k < 16; k++) {
          ibcol = k << 4;
          b_Ki_tmp = j + ibcol;
          dc_dtheta = A[b_Ki_tmp];
          A_tmp = ibtile + ibcol;
          A[b_Ki_tmp] = A[A_tmp];
          A[A_tmp] = dc_dtheta;
        }
      }
      b_i = (b_tmp - j) + 16;
      for (i = jp1j; i <= b_i; i++) {
        A[i - 1] /= A[b_tmp];
      }
    }
    ibcol = b_tmp;
    for (ibtile = 0; ibtile <= mmj_tmp; ibtile++) {
      dc_dtheta = A[(b_tmp + (ibtile << 4)) + 16];
      if (dc_dtheta != 0.0) {
        b_i = ibcol + 18;
        A_tmp = (ibcol - j) + 32;
        for (b_Ki_tmp = b_i; b_Ki_tmp <= A_tmp; b_Ki_tmp++) {
          A[b_Ki_tmp - 1] += A[((b_tmp + b_Ki_tmp) - ibcol) - 17] * -dc_dtheta;
        }
      }
      ibcol += 16;
    }
    Ki_tmp_tmp = ipiv[j];
    if (Ki_tmp_tmp != j + 1) {
      dc_dtheta = v[j];
      v[j] = v[Ki_tmp_tmp - 1];
      v[Ki_tmp_tmp - 1] = dc_dtheta;
    }
  }
  for (k = 0; k < 16; k++) {
    ibcol = k << 4;
    if (v[k] != 0.0) {
      b_i = k + 2;
      for (i = b_i; i < 17; i++) {
        v[i - 1] -= v[k] * A[(i + ibcol) - 1];
      }
    }
  }
  for (k = 15; k >= 0; k--) {
    ibcol = k << 4;
    b_d = v[k];
    if (b_d != 0.0) {
      b_d /= A[k + ibcol];
      v[k] = b_d;
      for (i = 0; i < k; i++) {
        v[i] -= v[k] * A[i + ibcol];
      }
    }
  }
  for (b_i = 0; b_i < 16; b_i++) {
    b_d = 0.0;
    for (A_tmp = 0; A_tmp < 16; A_tmp++) {
      b_d += conversion[b_i + (A_tmp << 4)] * v[A_tmp];
    }
    tau[b_i] = b_d;
  }
}

/* End of code generation (puppet_controller_4_cg.c) */
