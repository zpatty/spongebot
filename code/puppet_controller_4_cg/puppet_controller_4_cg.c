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
#include "J_r.h"
#include "MC_4_cg.h"
#include "mldivide.h"
#include "puppet_controller_4_cg_rtwutil.h"
#include "rt_nonfinite.h"
#include "xgetrf.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
void puppet_controller_4_cg(
    const double q[16], const double dq[16], const double qd[16],
    const double dqd[16], const double ddqd[16], double d, double m, double mm,
    double hm, double rm, double r, double kb, double ks, double bb, double bs,
    double bm, double L0, const double Kp[256], const double KD[256],
    double Kpx, double KDx, const double xd[3], const double dxd[3],
    const double dxr[3], double kc, double ka, double offset, double contact,
    double conv_pcc, double conv_motor, double tau[16], double tau_r[16],
    double x[3], double cq[4])
{
  static const signed char inds[12] = {2,  3,  4,  6,  7,  8,
                                       10, 11, 12, 14, 15, 16};
  static const signed char b[9] = {-1, 0, 0, 0, -1, 0, 0, 0, -1};
  static const signed char iv[9] = {0, 1, 0, -1, 0, 0, 0, 0, 1};
  double A[256];
  double D[256];
  double K[256];
  double KDr[256];
  double Kpr[256];
  double M[256];
  double conversion[256];
  double B[48];
  double J[48];
  double b_tmp[48];
  double b_tmp_tmp[48];
  double Ai[36];
  double C[16];
  double b_C[16];
  double b_K[16];
  double c_C[16];
  double dv[16];
  double v[16];
  double Fci[12];
  double qdp[12];
  double qp[12];
  double Aq[9];
  double Ki[9];
  double a[3];
  double b_d;
  double b_del_tmp;
  double d1;
  double dc_dL;
  double dc_dtheta;
  double del;
  double del_tmp;
  double theta;
  int ipiv[16];
  int b_Ki_tmp;
  int b_i;
  int i;
  int i1;
  int ibcol;
  int kAcol;
  int r2;
  int r3;
  int rtemp;
  signed char b_I[256];
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
  for (rtemp = 0; rtemp < 3; rtemp++) {
    qdp[rtemp] = qd[rtemp + 1];
    qdp[rtemp + 3] = qd[rtemp + 5];
    qdp[rtemp + 6] = qd[rtemp + 9];
    qdp[rtemp + 9] = qd[rtemp + 13];
    ibcol = rtemp << 2;
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
  for (r3 = 0; r3 < 16; r3++) {
    K[r3 + (r3 << 4)] = v[r3];
  }
  for (rtemp = 0; rtemp < 3; rtemp++) {
    ibcol = rtemp << 2;
    Fci[ibcol] = bb;
    Fci[ibcol + 1] = bb;
    Fci[ibcol + 2] = bs;
    Fci[ibcol + 3] = bm;
  }
  v[0] = bm;
  memcpy(&v[1], &Fci[0], 12U * sizeof(double));
  v[13] = bb;
  v[14] = bb;
  v[15] = bs;
  memset(&D[0], 0, 256U * sizeof(double));
  for (r3 = 0; r3 < 16; r3++) {
    D[r3 + (r3 << 4)] = v[r3];
  }
  /*  Fci = cell(1,4); */
  /*  Ai = cell(1,4); */
  for (i = 0; i < 4; i++) {
    double Kpi[9];
    double b_y_tmp[9];
    double y_tmp[9];
    double Dq;
    double Dq_tmp;
    double L_tmp;
    double c;
    double dc_dx;
    double dc_dy;
    signed char Ki_tmp[3];
    signed char Ki_tmp_tmp;
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
    i1 = 3 * i + 2;
    L_tmp = qp[i1] + L0;
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
    c_Ki_tmp_tmp = inds[i1];
    Ki_tmp[2] = c_Ki_tmp_tmp;
    dc_dtheta = 0.5 * (kc * exp(-2.0 * theta * kc) /
                           ((dc_dtheta + 1.0) * (dc_dtheta + 1.0)) -
                       kc * dc_dtheta / (dc_dtheta + 1.0));
    b_d = dc_dtheta * dc_dx;
    d1 = dc_dtheta * dc_dy;
    dc_dtheta *= dc_dL;
    for (b_i = 0; b_i < 3; b_i++) {
      i1 = b_i + 3 * i;
      theta = qdp[i1] - qp[i1];
      a[b_i] = theta;
      b_Ki_tmp = (Ki_tmp[b_i] - 1) << 4;
      Ki[3 * b_i] = K[(Ki_tmp_tmp + b_Ki_tmp) - 1];
      ibcol = (inds[i1] - 1) << 4;
      Kpi[3 * b_i] = Kp[(Ki_tmp_tmp + ibcol) - 1];
      y_tmp[3 * b_i] = b_d * theta;
      rtemp = 3 * b_i + 1;
      Ki[rtemp] = K[(b_Ki_tmp_tmp + b_Ki_tmp) - 1];
      Kpi[rtemp] = Kp[(b_Ki_tmp_tmp + ibcol) - 1];
      y_tmp[rtemp] = d1 * theta;
      rtemp = 3 * b_i + 2;
      Ki[rtemp] = K[(c_Ki_tmp_tmp + b_Ki_tmp) - 1];
      Kpi[rtemp] = Kp[(c_Ki_tmp_tmp + ibcol) - 1];
      y_tmp[rtemp] = dc_dtheta * theta;
    }
    for (b_i = 0; b_i < 3; b_i++) {
      b_d = 0.0;
      d1 = y_tmp[b_i + 3];
      dc_dtheta = y_tmp[b_i + 6];
      theta = y_tmp[b_i];
      dc_dL = 0.0;
      del_tmp = 0.0;
      for (i1 = 0; i1 < 3; i1++) {
        ibcol = 3 * i1 + 1;
        rtemp = 3 * i1 + 2;
        b_del_tmp = qdp[i1 + 3 * i];
        r2 = b_i + 3 * i1;
        dc_dL += del * Ki[r2] * b_del_tmp;
        del_tmp +=
            ((theta * Ki[3 * i1] + d1 * Ki[ibcol]) + dc_dtheta * Ki[rtemp]) *
            b_del_tmp;
        b_y_tmp[r2] =
            (theta * Kpi[3 * i1] + d1 * Kpi[ibcol]) + dc_dtheta * Kpi[rtemp];
        b_d += del * Kpi[r2] * a[i1];
      }
      Fci[b_i + 3 * i] =
          -ka * (((dc_dL - del_tmp) - b_d) +
                 ((b_y_tmp[b_i] * a[0] + b_y_tmp[b_i + 3] * a[1]) +
                  b_y_tmp[b_i + 6] * a[2]));
    }
    Ki[0] = d * 0.49999999999999994;
    Ki[3] = d * 0.49999999999999994;
    Ki[6] = -d;
    Ki[1] = -d * 0.86602540378443871;
    Ki[4] = d * 0.86602540378443871;
    Ki[7] = 0.0;
    for (b_i = 0; b_i < 3; b_i++) {
      b_d = Aq[b_i];
      d1 = Aq[b_i + 3];
      dc_dtheta = Aq[b_i + 6];
      for (i1 = 0; i1 < 3; i1++) {
        Kpi[b_i + 3 * i1] =
            (b_d * (double)b[3 * i1] + d1 * (double)b[3 * i1 + 1]) +
            dc_dtheta * (double)b[3 * i1 + 2];
      }
      Ki[3 * b_i + 2] = 1.0;
    }
    for (b_i = 0; b_i < 3; b_i++) {
      b_d = Kpi[b_i];
      d1 = Kpi[b_i + 3];
      dc_dtheta = Kpi[b_i + 6];
      for (i1 = 0; i1 < 3; i1++) {
        Ai[(b_i + 3 * i1) + 9 * i] = (b_d * Ki[3 * i1] + d1 * Ki[3 * i1 + 1]) +
                                     dc_dtheta * Ki[3 * i1 + 2];
      }
    }
  }
  memset(&A[0], 0, 256U * sizeof(double));
  A[0] = 1.0;
  A[68] = 1.0;
  A[136] = 1.0;
  A[204] = 1.0;
  for (b_i = 0; b_i < 3; b_i++) {
    ibcol = (b_i + 1) << 4;
    A[ibcol + 1] = Ai[3 * b_i];
    rtemp = (b_i + 5) << 4;
    A[rtemp + 5] = Ai[3 * b_i + 9];
    r2 = (b_i + 9) << 4;
    A[r2 + 9] = Ai[3 * b_i + 18];
    kAcol = (b_i + 13) << 4;
    A[kAcol + 13] = Ai[3 * b_i + 27];
    A[ibcol + 2] = Ai[3 * b_i + 1];
    A[rtemp + 6] = Ai[3 * b_i + 10];
    A[r2 + 10] = Ai[3 * b_i + 19];
    A[kAcol + 14] = Ai[3 * b_i + 28];
    A[ibcol + 3] = Ai[3 * b_i + 2];
    A[rtemp + 7] = Ai[3 * b_i + 11];
    A[r2 + 11] = Ai[3 * b_i + 20];
    A[kAcol + 15] = Ai[3 * b_i + 29];
  }
  for (rtemp = 0; rtemp < 4; rtemp++) {
    ibcol = rtemp << 2;
    v[ibcol] = conv_motor;
    v[ibcol + 1] = conv_pcc;
    v[ibcol + 2] = conv_pcc;
    v[ibcol + 3] = conv_pcc;
  }
  memset(&conversion[0], 0, 256U * sizeof(double));
  for (r3 = 0; r3 < 16; r3++) {
    conversion[r3 + (r3 << 4)] = v[r3];
    v[r3] = qd[r3] - q[r3];
    b_d = 0.0;
    d1 = 0.0;
    dc_dtheta = 0.0;
    for (b_i = 0; b_i < 16; b_i++) {
      i1 = r3 + (b_i << 4);
      b_d += M[i1] * ddqd[b_i];
      dc_dtheta += K[i1] * qd[b_i];
      d1 += D[i1] * dqd[b_i];
    }
    c_C[r3] = ((C[r3] + b_d) + dc_dtheta) + d1;
  }
  dv[0] = 0.0 * contact;
  dv[4] = 0.0 * contact;
  dv[8] = 0.0 * contact;
  dv[12] = 0.0 * contact;
  dv[1] = Fci[0] * contact;
  dv[5] = Fci[3] * contact;
  dv[9] = Fci[6] * contact;
  dv[13] = Fci[9] * contact;
  dv[2] = Fci[1] * contact;
  dv[6] = Fci[4] * contact;
  dv[10] = Fci[7] * contact;
  dv[14] = Fci[10] * contact;
  dv[3] = Fci[2] * contact;
  dv[7] = Fci[5] * contact;
  dv[11] = Fci[8] * contact;
  dv[15] = Fci[11] * contact;
  for (b_i = 0; b_i < 16; b_i++) {
    b_K[b_i] = dqd[b_i] - dq[b_i];
    b_d = 0.0;
    for (i1 = 0; i1 < 16; i1++) {
      b_d += Kp[b_i + (i1 << 4)] * v[i1];
    }
    b_C[b_i] = (c_C[b_i] + dv[b_i]) + b_d;
  }
  for (b_i = 0; b_i < 16; b_i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 16; i1++) {
      b_d += KD[b_i + (i1 << 4)] * b_K[i1];
    }
    b_C[b_i] += b_d;
  }
  mldivide(A, b_C);
  for (b_i = 0; b_i < 16; b_i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 16; i1++) {
      b_d += conversion[b_i + (i1 << 4)] * b_C[i1];
    }
    tau[b_i] = b_d;
  }
  memcpy(&Kpr[0], &Kp[0], 256U * sizeof(double));
  Kpr[0] = 0.0;
  Kpr[68] = 0.0;
  Kpr[136] = 0.0;
  Kpr[204] = 0.0;
  memcpy(&KDr[0], &KD[0], 256U * sizeof(double));
  KDr[0] = 0.0;
  KDr[68] = 0.0;
  KDr[136] = 0.0;
  KDr[204] = 0.0;
  memcpy(&b_C[0], &q[0], 16U * sizeof(double));
  J_r(b_C, L0, d, hm, J, x);
  for (b_i = 0; b_i < 3; b_i++) {
    for (i1 = 0; i1 < 16; i1++) {
      b_tmp_tmp[i1 + (b_i << 4)] = J[b_i + 3 * i1];
    }
  }
  memcpy(&b_tmp[0], &b_tmp_tmp[0], 48U * sizeof(double));
  xgetrf(M, ipiv);
  for (i = 0; i < 15; i++) {
    b_i = ipiv[i];
    if (b_i != i + 1) {
      dc_dtheta = b_tmp[i];
      b_tmp[i] = b_tmp[b_i - 1];
      b_tmp[b_i - 1] = dc_dtheta;
      dc_dtheta = b_tmp[i + 16];
      b_tmp[i + 16] = b_tmp[b_i + 15];
      b_tmp[b_i + 15] = dc_dtheta;
      dc_dtheta = b_tmp[i + 32];
      b_tmp[i + 32] = b_tmp[b_i + 31];
      b_tmp[b_i + 31] = dc_dtheta;
    }
  }
  for (r3 = 0; r3 < 3; r3++) {
    ibcol = r3 << 4;
    for (rtemp = 0; rtemp < 16; rtemp++) {
      kAcol = rtemp << 4;
      b_i = rtemp + ibcol;
      if (b_tmp[b_i] != 0.0) {
        i1 = rtemp + 2;
        for (i = i1; i < 17; i++) {
          r2 = (i + ibcol) - 1;
          b_tmp[r2] -= b_tmp[b_i] * M[(i + kAcol) - 1];
        }
      }
    }
  }
  for (r3 = 0; r3 < 3; r3++) {
    ibcol = r3 << 4;
    for (rtemp = 15; rtemp >= 0; rtemp--) {
      kAcol = rtemp << 4;
      b_i = rtemp + ibcol;
      b_d = b_tmp[b_i];
      if (b_d != 0.0) {
        b_tmp[b_i] = b_d / M[rtemp + kAcol];
        for (i = 0; i < rtemp; i++) {
          r2 = i + ibcol;
          b_tmp[r2] -= b_tmp[b_i] * M[i + kAcol];
        }
      }
    }
  }
  for (b_i = 0; b_i < 3; b_i++) {
    for (i1 = 0; i1 < 3; i1++) {
      b_d = 0.0;
      for (ibcol = 0; ibcol < 16; ibcol++) {
        b_d += J[b_i + 3 * ibcol] * b_tmp[ibcol + (i1 << 4)];
      }
      Aq[b_i + 3 * i1] = b_d;
    }
  }
  kAcol = 0;
  r2 = 1;
  r3 = 2;
  dc_dtheta = fabs(Aq[0]);
  theta = fabs(Aq[1]);
  if (theta > dc_dtheta) {
    dc_dtheta = theta;
    kAcol = 1;
    r2 = 0;
  }
  if (fabs(Aq[2]) > dc_dtheta) {
    kAcol = 2;
    r2 = 1;
    r3 = 0;
  }
  Aq[r2] /= Aq[kAcol];
  Aq[r3] /= Aq[kAcol];
  Aq[r2 + 3] -= Aq[r2] * Aq[kAcol + 3];
  Aq[r3 + 3] -= Aq[r3] * Aq[kAcol + 3];
  Aq[r2 + 6] -= Aq[r2] * Aq[kAcol + 6];
  Aq[r3 + 6] -= Aq[r3] * Aq[kAcol + 6];
  if (fabs(Aq[r3 + 3]) > fabs(Aq[r2 + 3])) {
    rtemp = r2;
    r2 = r3;
    r3 = rtemp;
  }
  Aq[r3 + 3] /= Aq[r2 + 3];
  Aq[r3 + 6] -= Aq[r3 + 3] * Aq[r2 + 6];
  Ki[3 * kAcol] = 1.0 / Aq[kAcol];
  dc_dtheta = Aq[kAcol + 3];
  Ki[3 * r2] = 0.0 - Ki[3 * kAcol] * dc_dtheta;
  theta = Aq[kAcol + 6];
  Ki[3 * r3] = 0.0 - Ki[3 * kAcol] * theta;
  dc_dL = Aq[r2 + 3];
  Ki[3 * r2] /= dc_dL;
  del_tmp = Aq[r2 + 6];
  Ki[3 * r3] -= Ki[3 * r2] * del_tmp;
  b_del_tmp = Aq[r3 + 6];
  Ki[3 * r3] /= b_del_tmp;
  del = Aq[r3 + 3];
  Ki[3 * r2] -= Ki[3 * r3] * del;
  Ki[3 * kAcol] -= Ki[3 * r3] * Aq[r3];
  Ki[3 * kAcol] -= Ki[3 * r2] * Aq[r2];
  b_Ki_tmp = 3 * kAcol + 1;
  Ki[b_Ki_tmp] = 0.0 / Aq[kAcol];
  rtemp = 3 * r2 + 1;
  Ki[rtemp] = 1.0 - Ki[b_Ki_tmp] * dc_dtheta;
  ibcol = 3 * r3 + 1;
  Ki[ibcol] = 0.0 - Ki[b_Ki_tmp] * theta;
  Ki[rtemp] /= dc_dL;
  Ki[ibcol] -= Ki[rtemp] * del_tmp;
  Ki[ibcol] /= b_del_tmp;
  Ki[rtemp] -= Ki[ibcol] * del;
  Ki[b_Ki_tmp] -= Ki[ibcol] * Aq[r3];
  Ki[b_Ki_tmp] -= Ki[rtemp] * Aq[r2];
  b_Ki_tmp = 3 * kAcol + 2;
  Ki[b_Ki_tmp] = 0.0 / Aq[kAcol];
  rtemp = 3 * r2 + 2;
  Ki[rtemp] = 0.0 - Ki[b_Ki_tmp] * dc_dtheta;
  ibcol = 3 * r3 + 2;
  Ki[ibcol] = 1.0 - Ki[b_Ki_tmp] * theta;
  Ki[rtemp] /= dc_dL;
  Ki[ibcol] -= Ki[rtemp] * del_tmp;
  Ki[ibcol] /= b_del_tmp;
  Ki[rtemp] -= Ki[ibcol] * del;
  Ki[b_Ki_tmp] -= Ki[ibcol] * Aq[r3];
  Ki[b_Ki_tmp] -= Ki[rtemp] * Aq[r2];
  for (b_i = 0; b_i < 16; b_i++) {
    b_d = b_tmp[b_i];
    d1 = b_tmp[b_i + 16];
    dc_dtheta = b_tmp[b_i + 32];
    for (i1 = 0; i1 < 3; i1++) {
      B[b_i + (i1 << 4)] =
          (b_d * Ki[3 * i1] + d1 * Ki[3 * i1 + 1]) + dc_dtheta * Ki[3 * i1 + 2];
    }
  }
  for (r3 = 0; r3 < 16; r3++) {
    ibcol = r3 << 4;
    b_d = B[r3];
    d1 = B[r3 + 16];
    dc_dtheta = B[r3 + 32];
    for (i = 0; i < 16; i++) {
      rtemp = i * 3;
      M[ibcol + i] =
          (J[rtemp] * b_d + J[rtemp + 1] * d1) + J[rtemp + 2] * dc_dtheta;
    }
  }
  memset(&b_I[0], 0, 256U * sizeof(signed char));
  for (rtemp = 0; rtemp < 16; rtemp++) {
    b_I[rtemp + (rtemp << 4)] = 1;
    b_d = 0.0;
    d1 = 0.0;
    for (b_i = 0; b_i < 16; b_i++) {
      i1 = rtemp + (b_i << 4);
      b_d += K[i1] * q[b_i];
      d1 += D[i1] * dq[b_i];
    }
    b_K[rtemp] = b_d + d1;
    b_d = b_tmp_tmp[rtemp];
    d1 = b_tmp_tmp[rtemp + 16];
    dc_dtheta = b_tmp_tmp[rtemp + 32];
    for (b_i = 0; b_i < 3; b_i++) {
      b_tmp[rtemp + (b_i << 4)] = (b_d * Ki[3 * b_i] + d1 * Ki[3 * b_i + 1]) +
                                  dc_dtheta * Ki[3 * b_i + 2];
    }
  }
  a[0] = Kpx * (xd[0] - x[0]) + KDx * (dxd[0] - dxr[0]);
  a[1] = Kpx * (xd[1] - x[1]) + KDx * (dxd[1] - dxr[1]);
  a[2] = Kpx * (xd[2] - x[2]) + KDx * (dxd[2] - dxr[2]);
  for (b_i = 0; b_i < 16; b_i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 16; i1++) {
      b_d += M[b_i + (i1 << 4)] * b_K[i1];
    }
    b_C[b_i] = (C[b_i] + b_d) + ((b_tmp[b_i] * a[0] + b_tmp[b_i + 16] * a[1]) +
                                 b_tmp[b_i + 32] * a[2]);
  }
  mldivide(A, b_C);
  for (b_i = 0; b_i < 16; b_i++) {
    b_d = 0.0;
    d1 = 0.0;
    for (i1 = 0; i1 < 16; i1++) {
      ibcol = b_i + (i1 << 4);
      b_d += Kpr[ibcol] * v[i1];
      d1 += KDr[ibcol] * dq[i1];
    }
    c_C[b_i] = d1;
    b_K[b_i] = b_d;
  }
  for (b_i = 0; b_i < 256; b_i++) {
    M[b_i] = (double)b_I[b_i] - M[b_i];
  }
  for (b_i = 0; b_i < 16; b_i++) {
    b_K[b_i] -= c_C[b_i];
  }
  for (b_i = 0; b_i < 16; b_i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 16; i1++) {
      b_d += M[b_i + (i1 << 4)] * b_K[i1];
    }
    dv[b_i] = b_C[b_i] + b_d;
  }
  for (b_i = 0; b_i < 16; b_i++) {
    b_d = 0.0;
    for (i1 = 0; i1 < 16; i1++) {
      b_d += conversion[b_i + (i1 << 4)] * dv[i1];
    }
    tau_r[b_i] = b_d;
  }
}

/* End of code generation (puppet_controller_4_cg.c) */
