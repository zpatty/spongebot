/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * MC_4_cg.c
 *
 * Code generation for function 'MC_4_cg'
 *
 */

/* Include files */
#include "MC_4_cg.h"
#include "PCC_jacobian.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Declarations */
static void binary_expand_op(double in1[48], int in2, const double in3[288],
                             const double in4_data[], const int *in4_size,
                             const double in5[36], const double in6[6]);

/* Function Definitions */
static void binary_expand_op(double in1[48], int in2, const double in3[288],
                             const double in4_data[], const int *in4_size,
                             const double in5[36], const double in6[6])
{
  double b_in3[6];
  double b_in5[6];
  int i;
  int i1;
  int stride_0_0;
  for (i = 0; i < 6; i++) {
    double d;
    double d1;
    d = 0.0;
    d1 = 0.0;
    for (i1 = 0; i1 < 6; i1++) {
      stride_0_0 = i + 6 * i1;
      d += in3[stride_0_0 + 36 * (in2 + 1)] * in1[i1 + 6 * in2];
      d1 += in5[stride_0_0] * in6[i1];
    }
    b_in5[i] = d1;
    b_in3[i] = d;
  }
  stride_0_0 = (*in4_size != 1);
  for (i = 0; i < 6; i++) {
    in1[i + 6 * (in2 + 1)] = (b_in3[i] + in4_data[i * stride_0_0]) + b_in5[i];
  }
}

void MC_4_cg(const double q[16], const double qd[16], double m, double mm,
             double hm, double rm, double r, double L0, double d, double M[256],
             double C[16])
{
  static const double c_b[6] = {-0.0, -0.0, 9.81, -0.0, -0.0, -0.0};
  static const signed char b_b[36] = {1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
                                      0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0,
                                      0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1};
  static const signed char b[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  static const signed char b_a[6] = {0, 0, 0, 0, 0, 1};
  static const signed char c_a[6] = {0, 0, 0, 0, 2, 0};
  static const signed char d_a[6] = {0, 0, 0, 0, 0, 2};
  static const signed char d_b[6] = {0, 0, 0, 0, 1, 0};
  static const signed char iv[6] = {0, 0, 0, 0, 1, 0};
  static const signed char iv1[6] = {0, 0, 0, 0, 0, 1};
  double Xup[288];
  double b_I[288];
  double Smod[72];
  double a[48];
  double f[48];
  double v[48];
  double Xtree[36];
  double b_a_tmp[36];
  double c_I[36];
  double dJ[18];
  double dJ_data[18];
  double g[16];
  double b_c[9];
  double c[9];
  double c_c[9];
  double y_tmp[9];
  double C_data[6];
  double a_tmp[6];
  double b_Xup[6];
  double vJ[6];
  double inertia_motor[3];
  double b_d;
  double inertia_motor_tmp;
  double vJ_tmp;
  double vJ_tmp_tmp;
  int aoffset;
  int b_i;
  int c_i;
  int dJ_size_idx_0;
  int dJ_tmp;
  int i;
  int i1;
  int j;
  int qi_size_idx_1;
  int qj;
  signed char qi_data[17];
  signed char tmp_data[17];
  /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
  /*  This function calculates the Mass and Coriolis + Gravity Matrices */
  /*  given the current state and geometric parameters of the pushpuppet robot
   */
  /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
  /*  q = state */
  /*  qd = velocity */
  /*  qdd = acceleration */
  /*  m = mass of a module; */
  /*  mm = mass of a motor; */
  /*  hm = height of a motor; */
  /*  rm = "radius" a motor; */
  /*  r = radius of the hex plate; */
  /*  L0 = Length of a module; */
  /*  d = distance to cable; */
  /*  N = number of links (number of modules plus number of motors) */
  /*  qi = {1, 2:4, 5, 6:8, 9, 10:12, 13, 14:16}; */
  memset(&b_I[0], 0, 288U * sizeof(double));
  vJ[0] = m;
  vJ[1] = m;
  vJ[2] = m;
  vJ_tmp_tmp = r * r;
  vJ_tmp = 0.25 * m * vJ_tmp_tmp;
  vJ[3] = vJ_tmp;
  vJ[4] = vJ_tmp;
  vJ[5] = 0.5 * m * vJ_tmp_tmp;
  memset(&b_I[0], 0, 36U * sizeof(double));
  for (j = 0; j < 6; j++) {
    b_I[j + 6 * j] = vJ[j];
  }
  for (i = 0; i < 6; i++) {
    for (i1 = 0; i1 < 6; i1++) {
      b_i = i1 + 6 * i;
      b_d = b_I[b_i];
      b_I[b_i + 36] = b_d;
      b_I[b_i + 108] = b_d;
      b_I[b_i + 180] = b_d;
      b_I[b_i + 252] = b_d;
    }
  }
  vJ_tmp_tmp = rm * rm;
  inertia_motor_tmp = 0.25 * mm * vJ_tmp_tmp;
  inertia_motor[2] = 0.5 * mm * vJ_tmp_tmp;
  vJ_tmp_tmp = hm / 2.0;
  c[0] = 0.0;
  c[3] = -vJ_tmp_tmp;
  c[6] = 0.0;
  c[1] = vJ_tmp_tmp;
  c[4] = 0.0;
  c[7] = -0.0;
  c[2] = -0.0;
  c[5] = -0.0;
  c[8] = -0.0;
  for (i = 0; i < 3; i++) {
    y_tmp[3 * i] = c[i];
    y_tmp[3 * i + 1] = c[i + 3];
    y_tmp[3 * i + 2] = c[i + 6];
  }
  for (i = 0; i < 3; i++) {
    b_d = c[i];
    vJ_tmp = c[i + 3];
    i1 = (int)c[i + 6];
    for (b_i = 0; b_i < 3; b_i++) {
      b_c[i + 3 * b_i] = (b_d * y_tmp[3 * b_i] + vJ_tmp * y_tmp[3 * b_i + 1]) +
                         (double)i1 * y_tmp[3 * b_i + 2];
    }
  }
  for (i = 0; i < 3; i++) {
    b_d = inertia_motor_tmp + mm * b_c[3 * i];
    b_c[3 * i] = b_d;
    b_I[6 * i + 72] = mm * (double)b[3 * i];
    aoffset = 6 * (i + 3);
    b_I[aoffset + 72] = mm * y_tmp[3 * i];
    b_I[6 * i + 75] = mm * c[3 * i];
    b_I[aoffset + 75] = b_d;
    i1 = 3 * i + 1;
    b_d = inertia_motor_tmp + mm * b_c[i1];
    b_c[i1] = b_d;
    b_I[6 * i + 73] = mm * (double)b[i1];
    b_I[aoffset + 73] = mm * y_tmp[i1];
    b_I[6 * i + 76] = mm * c[i1];
    b_I[aoffset + 76] = b_d;
    i1 = 3 * i + 2;
    b_d = inertia_motor[2] + mm * b_c[i1];
    b_c[i1] = b_d;
    b_I[6 * i + 74] = mm * (double)b[i1];
    b_I[aoffset + 74] = mm * y_tmp[i1];
    b_I[6 * i + 77] = mm * c[i1];
    b_I[aoffset + 77] = b_d;
  }
  for (i = 0; i < 6; i++) {
    for (i1 = 0; i1 < 6; i1++) {
      b_i = i1 + 6 * i;
      b_d = b_I[b_i + 72];
      b_I[b_i + 144] = b_d;
      b_I[b_i + 216] = b_d;
    }
  }
  memset(&Smod[0], 0, 72U * sizeof(double));
  memset(&Xup[0], 0, 288U * sizeof(double));
  memset(&v[0], 0, 48U * sizeof(double));
  memset(&a[0], 0, 48U * sizeof(double));
  memset(&f[0], 0, 48U * sizeof(double));
  memset(&C[0], 0, 16U * sizeof(double));
  for (i = 0; i < 36; i++) {
    Xtree[i] = b_b[i];
  }
  /* [diag(ones(3,1)) [0;0;0]; 0 0 0 1]; */
  vJ_tmp_tmp = sin(q[0]);
  vJ_tmp = cos(q[0]);
  g[0] = vJ_tmp;
  g[4] = -vJ_tmp_tmp;
  g[8] = 0.0;
  g[12] = 0.0;
  g[1] = vJ_tmp_tmp;
  g[5] = vJ_tmp;
  g[9] = 0.0;
  g[13] = 0.0;
  g[2] = 0.0;
  g[3] = 0.0;
  g[6] = 0.0;
  g[7] = 0.0;
  g[10] = 1.0;
  g[11] = 0.0;
  g[14] = 0.0;
  g[15] = 1.0;
  /*  Adjoint calculator */
  /*  This function calculates the adjoint given simply the transform between */
  /*  joints */
  /*  convert transform to rotation and translation */
  /*  get skew symmetric matrix of translation */
  for (i = 0; i < 3; i++) {
    c[3 * i] = g[i];
    c[3 * i + 1] = g[i + 4];
    c[3 * i + 2] = g[i + 8];
  }
  for (i = 0; i < 9; i++) {
    b_c[i] = -c[i];
  }
  y_tmp[0] = 0.0;
  y_tmp[3] = -0.0;
  y_tmp[6] = 0.0;
  y_tmp[1] = 0.0;
  y_tmp[4] = 0.0;
  y_tmp[7] = -0.0;
  y_tmp[2] = -0.0;
  y_tmp[5] = 0.0;
  y_tmp[8] = 0.0;
  for (i = 0; i < 3; i++) {
    b_d = b_c[i];
    vJ_tmp = b_c[i + 3];
    vJ_tmp_tmp = b_c[i + 6];
    for (i1 = 0; i1 < 3; i1++) {
      c_c[i + 3 * i1] = (b_d * y_tmp[3 * i1] + vJ_tmp * y_tmp[3 * i1 + 1]) +
                        vJ_tmp_tmp * y_tmp[3 * i1 + 2];
      c_I[i1 + 6 * i] = c[i1 + 3 * i];
    }
  }
  for (i = 0; i < 3; i++) {
    aoffset = 6 * (i + 3);
    c_I[aoffset] = c_c[3 * i];
    c_I[6 * i + 3] = 0.0;
    c_I[aoffset + 3] = c[3 * i];
    qj = 3 * i + 1;
    c_I[aoffset + 1] = c_c[qj];
    c_I[6 * i + 4] = 0.0;
    c_I[aoffset + 4] = c[qj];
    qj = 3 * i + 2;
    c_I[aoffset + 2] = c_c[qj];
    c_I[6 * i + 5] = 0.0;
    c_I[aoffset + 5] = c[qj];
  }
  for (c_i = 0; c_i < 6; c_i++) {
    for (i = 0; i < 6; i++) {
      b_d = 0.0;
      for (i1 = 0; i1 < 6; i1++) {
        b_d += c_I[c_i + 6 * i1] * (double)b_b[i1 + 6 * i];
      }
      Xup[c_i + 6 * i] = b_d;
    }
    b_d = (double)b_a[c_i] * qd[0];
    vJ[c_i] = b_d;
    v[c_i] = b_d;
  }
  memset(&c_I[0], 0, 36U * sizeof(double));
  c[0] = 0.0;
  c[3] = -v[5];
  c[6] = v[4];
  c[1] = v[5];
  c[4] = 0.0;
  c[7] = -v[3];
  c[2] = -v[4];
  c[5] = v[3];
  c[8] = 0.0;
  for (i = 0; i < 3; i++) {
    vJ_tmp_tmp = c[3 * i];
    c_I[6 * i] = vJ_tmp_tmp;
    aoffset = 6 * (i + 3);
    c_I[aoffset + 3] = vJ_tmp_tmp;
    vJ_tmp_tmp = c[3 * i + 1];
    c_I[6 * i + 1] = vJ_tmp_tmp;
    c_I[aoffset + 4] = vJ_tmp_tmp;
    vJ_tmp_tmp = c[3 * i + 2];
    c_I[6 * i + 2] = vJ_tmp_tmp;
    c_I[aoffset + 5] = vJ_tmp_tmp;
  }
  c_I[18] = 0.0;
  c_I[24] = -v[2];
  c_I[30] = v[1];
  c_I[19] = v[2];
  c_I[25] = 0.0;
  c_I[31] = -v[0];
  c_I[20] = -v[1];
  c_I[26] = v[0];
  c_I[32] = 0.0;
  for (i = 0; i < 6; i++) {
    b_d = 0.0;
    vJ_tmp = 0.0;
    for (i1 = 0; i1 < 6; i1++) {
      b_i = i + 6 * i1;
      b_d += Xup[b_i] * c_b[i1];
      vJ_tmp_tmp = c_I[b_i];
      vJ_tmp += vJ_tmp_tmp * vJ[i1];
      b_a_tmp[i1 + 6 * i] = -vJ_tmp_tmp;
    }
    a[i] = b_d + vJ_tmp;
  }
  for (i = 0; i < 6; i++) {
    b_Xup[i] = 0.0;
    a_tmp[i] = 0.0;
    for (i1 = 0; i1 < 6; i1++) {
      b_d = 0.0;
      for (b_i = 0; b_i < 6; b_i++) {
        b_d += b_a_tmp[i + 6 * b_i] * b_I[b_i + 6 * i1];
      }
      b_Xup[i] += b_I[i + 6 * i1] * a[i1];
      a_tmp[i] += b_d * v[i1];
    }
    f[i] = b_Xup[i] + a_tmp[i];
  }
  /*  Recursive Newton Euler to Calculate C+G */
  for (c_i = 0; c_i < 7; c_i++) {
    double q_data[15];
    int dJ_size_idx_1;
    if (fmod((double)c_i + 2.0, 2.0) == 0.0) {
      double qd_data[15];
      qj = (c_i + 2) << 1;
      if (qj < qj - 2) {
        qi_size_idx_1 = 0;
      } else {
        qi_size_idx_1 = 3;
        for (i = 0; i < 3; i++) {
          qi_data[i] = (signed char)((qj + i) - 2);
        }
      }
      for (i = 0; i < qi_size_idx_1; i++) {
        i1 = qi_data[i];
        q_data[i] = q[i1 - 1];
        qd_data[i] = qd[i1 - 1];
      }
      i = 18 * ((int)(((double)c_i + 2.0) / 2.0) - 1);
      PCC_jacobian(q_data, d, L0, qd_data, c_I, *(double(*)[18]) & Smod[i], g,
                   dJ);
      dJ_size_idx_0 = 6;
      dJ_size_idx_1 = 3;
      memcpy(&dJ_data[0], &dJ[0], 18U * sizeof(double));
      for (i1 = 0; i1 < 6; i1++) {
        for (b_i = 0; b_i < 6; b_i++) {
          b_d = 0.0;
          for (qj = 0; qj < 6; qj++) {
            b_d += c_I[i1 + 6 * qj] * Xtree[qj + 6 * b_i];
          }
          Xup[(i1 + 6 * b_i) + 36 * (c_i + 1)] = b_d;
        }
      }
      memset(&Xtree[0], 0, 36U * sizeof(double));
      for (j = 0; j < 6; j++) {
        Xtree[j + 6 * j] = 1.0;
      }
      for (i1 = 0; i1 < 3; i1++) {
        for (b_i = 0; b_i < 6; b_i++) {
          dJ_tmp = b_i + 6 * i1;
          dJ[dJ_tmp] = Smod[dJ_tmp + i];
        }
        inertia_motor[i1] = qd[qi_data[i1] - 1];
      }
      b_d = inertia_motor[0];
      vJ_tmp = inertia_motor[1];
      vJ_tmp_tmp = inertia_motor[2];
      for (i = 0; i < 6; i++) {
        vJ[i] = (dJ[i] * b_d + dJ[i + 6] * vJ_tmp) + dJ[i + 12] * vJ_tmp_tmp;
      }
    } else {
      qi_size_idx_1 = 1;
      qi_data[0] = (signed char)(2 * (c_i + 2) - 1);
      j = ((c_i + 2) << 1) - 2;
      vJ_tmp_tmp = q[j];
      vJ_tmp = vJ_tmp_tmp;
      vJ_tmp_tmp = cos(vJ_tmp_tmp);
      vJ_tmp = sin(vJ_tmp);
      dJ_size_idx_0 = 1;
      dJ_size_idx_1 = 1;
      dJ_data[0] = 0.0;
      g[0] = vJ_tmp_tmp;
      g[4] = 0.0;
      g[8] = vJ_tmp;
      g[12] = 0.0;
      g[2] = -vJ_tmp;
      g[6] = 0.0;
      g[10] = vJ_tmp_tmp;
      g[14] = 0.0;
      g[1] = 0.0;
      g[3] = 0.0;
      g[5] = 1.0;
      g[7] = 0.0;
      g[9] = 0.0;
      g[11] = 0.0;
      g[13] = 0.0;
      g[15] = 1.0;
      /*  Adjoint calculator */
      /*  This function calculates the adjoint given simply the transform
       * between */
      /*  joints */
      /*  convert transform to rotation and translation */
      /*  get skew symmetric matrix of translation */
      for (i = 0; i < 3; i++) {
        c[3 * i] = g[i];
        c[3 * i + 1] = g[i + 4];
        c[3 * i + 2] = g[i + 8];
      }
      for (i = 0; i < 9; i++) {
        b_c[i] = -c[i];
      }
      y_tmp[0] = 0.0;
      y_tmp[3] = -0.0;
      y_tmp[6] = 0.0;
      y_tmp[1] = 0.0;
      y_tmp[4] = 0.0;
      y_tmp[7] = -0.0;
      y_tmp[2] = -0.0;
      y_tmp[5] = 0.0;
      y_tmp[8] = 0.0;
      for (i = 0; i < 3; i++) {
        b_d = b_c[i];
        vJ_tmp = b_c[i + 3];
        vJ_tmp_tmp = b_c[i + 6];
        for (i1 = 0; i1 < 3; i1++) {
          c_c[i + 3 * i1] = (b_d * y_tmp[3 * i1] + vJ_tmp * y_tmp[3 * i1 + 1]) +
                            vJ_tmp_tmp * y_tmp[3 * i1 + 2];
          c_I[i1 + 6 * i] = c[i1 + 3 * i];
        }
      }
      for (i = 0; i < 3; i++) {
        aoffset = 6 * (i + 3);
        c_I[aoffset] = c_c[3 * i];
        c_I[6 * i + 3] = 0.0;
        c_I[aoffset + 3] = c[3 * i];
        qj = 3 * i + 1;
        c_I[aoffset + 1] = c_c[qj];
        c_I[6 * i + 4] = 0.0;
        c_I[aoffset + 4] = c[qj];
        qj = 3 * i + 2;
        c_I[aoffset + 2] = c_c[qj];
        c_I[6 * i + 5] = 0.0;
        c_I[aoffset + 5] = c[qj];
      }
      for (i = 0; i < 6; i++) {
        for (i1 = 0; i1 < 6; i1++) {
          b_d = 0.0;
          for (b_i = 0; b_i < 6; b_i++) {
            b_d += c_I[i + 6 * b_i] * Xtree[b_i + 6 * i1];
          }
          Xup[(i + 6 * i1) + 36 * (c_i + 1)] = b_d;
        }
      }
      g[2] = -0.0;
      g[6] = 0.0;
      g[10] = 1.0;
      g[14] = hm;
      g[0] = 1.0;
      g[1] = 0.0;
      g[3] = 0.0;
      g[4] = 0.0;
      g[5] = 1.0;
      g[7] = 0.0;
      g[8] = 0.0;
      g[9] = 0.0;
      g[11] = 0.0;
      g[12] = 0.0;
      g[13] = 0.0;
      g[15] = 1.0;
      /*  Adjoint calculator */
      /*  This function calculates the adjoint given simply the transform
       * between */
      /*  joints */
      /*  convert transform to rotation and translation */
      /*  get skew symmetric matrix of translation */
      for (i = 0; i < 3; i++) {
        c[3 * i] = g[i];
        c[3 * i + 1] = g[i + 4];
        c[3 * i + 2] = g[i + 8];
      }
      for (i = 0; i < 9; i++) {
        b_c[i] = -c[i];
      }
      y_tmp[0] = 0.0;
      y_tmp[3] = -hm;
      y_tmp[6] = 0.0;
      y_tmp[1] = hm;
      y_tmp[4] = 0.0;
      y_tmp[7] = -0.0;
      y_tmp[2] = -0.0;
      y_tmp[5] = 0.0;
      y_tmp[8] = 0.0;
      for (i = 0; i < 3; i++) {
        b_d = b_c[i];
        vJ_tmp = b_c[i + 3];
        vJ_tmp_tmp = b_c[i + 6];
        for (i1 = 0; i1 < 3; i1++) {
          c_c[i + 3 * i1] = (b_d * y_tmp[3 * i1] + vJ_tmp * y_tmp[3 * i1 + 1]) +
                            vJ_tmp_tmp * y_tmp[3 * i1 + 2];
          Xtree[i1 + 6 * i] = c[i1 + 3 * i];
        }
      }
      for (i = 0; i < 3; i++) {
        qj = 6 * (i + 3);
        Xtree[qj] = c_c[3 * i];
        Xtree[6 * i + 3] = 0.0;
        Xtree[qj + 3] = c[3 * i];
        aoffset = 3 * i + 1;
        Xtree[qj + 1] = c_c[aoffset];
        Xtree[6 * i + 4] = 0.0;
        Xtree[qj + 4] = c[aoffset];
        aoffset = 3 * i + 2;
        Xtree[qj + 2] = c_c[aoffset];
        Xtree[6 * i + 5] = 0.0;
        Xtree[qj + 5] = c[aoffset];
      }
      for (b_i = 0; b_i < 6; b_i++) {
        vJ[b_i] = (double)d_b[b_i] * qd[j];
      }
    }
    /*        if i == 1 */
    /*          v(:,i) = vJ; */
    /*          a(:,i) = Xup{i}*(-a_grav) + dJ * qd(qi) +
     * spatial_cross(v(:,i))*vJ; */
    /*        else */
    for (i = 0; i < 6; i++) {
      b_d = 0.0;
      for (i1 = 0; i1 < 6; i1++) {
        b_d += Xup[(i + 6 * i1) + 36 * (c_i + 1)] * v[i1 + 6 * c_i];
      }
      b_Xup[i] = b_d + vJ[i];
    }
    for (i = 0; i < 6; i++) {
      v[i + 6 * (c_i + 1)] = b_Xup[i];
    }
    memset(&c_I[0], 0, 36U * sizeof(double));
    c[0] = 0.0;
    i = 6 * (c_i + 1);
    b_d = v[i + 5];
    c[3] = -b_d;
    vJ_tmp = v[i + 4];
    c[6] = vJ_tmp;
    c[1] = b_d;
    c[4] = 0.0;
    b_d = v[i + 3];
    c[7] = -b_d;
    c[2] = -vJ_tmp;
    c[5] = b_d;
    c[8] = 0.0;
    for (i1 = 0; i1 < 3; i1++) {
      vJ_tmp_tmp = c[3 * i1];
      c_I[6 * i1] = vJ_tmp_tmp;
      aoffset = 6 * (i1 + 3);
      c_I[aoffset + 3] = vJ_tmp_tmp;
      vJ_tmp_tmp = c[3 * i1 + 1];
      c_I[6 * i1 + 1] = vJ_tmp_tmp;
      c_I[aoffset + 4] = vJ_tmp_tmp;
      vJ_tmp_tmp = c[3 * i1 + 2];
      c_I[6 * i1 + 2] = vJ_tmp_tmp;
      c_I[aoffset + 5] = vJ_tmp_tmp;
    }
    c_I[18] = 0.0;
    b_d = v[i + 2];
    c_I[24] = -b_d;
    vJ_tmp = v[i + 1];
    c_I[30] = vJ_tmp;
    c_I[19] = b_d;
    c_I[25] = 0.0;
    b_d = v[i];
    c_I[31] = -b_d;
    c_I[20] = -vJ_tmp;
    c_I[26] = b_d;
    c_I[32] = 0.0;
    for (i1 = 0; i1 < qi_size_idx_1; i1++) {
      q_data[i1] = qd[qi_data[i1] - 1];
    }
    qj = dJ_size_idx_0 - 1;
    memset(&C_data[0], 0, (unsigned int)(qj + 1) * sizeof(double));
    for (j = 0; j < dJ_size_idx_1; j++) {
      aoffset = j * dJ_size_idx_0;
      for (b_i = 0; b_i <= qj; b_i++) {
        C_data[b_i] += dJ_data[aoffset + b_i] * q_data[j];
      }
    }
    if (dJ_size_idx_0 == 6) {
      for (i1 = 0; i1 < 6; i1++) {
        b_d = 0.0;
        vJ_tmp = 0.0;
        for (b_i = 0; b_i < 6; b_i++) {
          qj = i1 + 6 * b_i;
          b_d += Xup[qj + 36 * (c_i + 1)] * a[b_i + 6 * c_i];
          vJ_tmp += c_I[qj] * vJ[b_i];
        }
        a_tmp[i1] = vJ_tmp;
        b_Xup[i1] = b_d + C_data[i1];
      }
      for (i1 = 0; i1 < 6; i1++) {
        a[i1 + i] = b_Xup[i1] + a_tmp[i1];
      }
    } else {
      binary_expand_op(a, c_i, Xup, C_data, &dJ_size_idx_0, c_I, vJ);
    }
    /*        end */
    for (i1 = 0; i1 < 6; i1++) {
      for (b_i = 0; b_i < 6; b_i++) {
        b_a_tmp[b_i + 6 * i1] = -c_I[i1 + 6 * b_i];
      }
    }
    for (i1 = 0; i1 < 6; i1++) {
      b_Xup[i1] = 0.0;
      a_tmp[i1] = 0.0;
      for (b_i = 0; b_i < 6; b_i++) {
        b_d = 0.0;
        for (qj = 0; qj < 6; qj++) {
          b_d += b_a_tmp[i1 + 6 * qj] * b_I[(qj + 6 * b_i) + 36 * (c_i + 1)];
        }
        qj = b_i + i;
        b_Xup[i1] += b_I[(i1 + 6 * b_i) + 36 * (c_i + 1)] * a[qj];
        a_tmp[i1] += b_d * v[qj];
      }
      f[i1 + i] = b_Xup[i1] + a_tmp[i1];
    }
  }
  /*  Composite Rigid Body Algorithm to calculate M */
  /*  composite inertia calculation */
  for (c_i = 0; c_i < 8; c_i++) {
    if (fmod(-(double)c_i + 8.0, 2.0) == 0.0) {
      qj = (8 - c_i) << 1;
      if (qj < qj - 2) {
        qi_size_idx_1 = 0;
      } else {
        qi_size_idx_1 = 3;
        for (i = 0; i < 3; i++) {
          qi_data[i] = (signed char)((qj + i) - 2);
        }
      }
      b_i = (int)((-(double)c_i + 8.0) / 2.0);
      for (i = 0; i < 3; i++) {
        for (i1 = 0; i1 < 6; i1++) {
          dJ_tmp = i1 + 6 * i;
          dJ[dJ_tmp] = Smod[dJ_tmp + 18 * (b_i - 1)];
        }
      }
      for (i = 0; i < qi_size_idx_1; i++) {
        tmp_data[i] = (signed char)(qi_data[i] - 1);
      }
      for (i = 0; i < 3; i++) {
        b_d = 0.0;
        for (i1 = 0; i1 < 6; i1++) {
          b_d += 2.0 * dJ[i1 + 6 * i] * f[i1 + 6 * (7 - c_i)];
        }
        inertia_motor[i] = b_d;
      }
      for (i = 0; i < qi_size_idx_1; i++) {
        C[tmp_data[i]] = inertia_motor[i];
      }
    } else if (8 - c_i == 1) {
      vJ_tmp_tmp = 0.0;
      for (i = 0; i < 6; i++) {
        vJ_tmp_tmp += (double)d_a[i] * f[i];
      }
      C[0] = vJ_tmp_tmp;
    } else {
      vJ_tmp_tmp = 0.0;
      for (i = 0; i < 6; i++) {
        vJ_tmp_tmp += (double)c_a[i] * f[i + 6 * (7 - c_i)];
      }
      C[((8 - c_i) << 1) - 2] = vJ_tmp_tmp;
    }
    if (8 - c_i != 1) {
      for (i = 0; i < 6; i++) {
        b_d = 0.0;
        for (i1 = 0; i1 < 6; i1++) {
          b_d += Xup[(i1 + 6 * i) + 36 * (7 - c_i)] * f[i1 + 6 * (7 - c_i)];
        }
        b_Xup[i] = f[i + 6 * (6 - c_i)] + b_d;
      }
      i = 36 * (7 - c_i);
      for (i1 = 0; i1 < 6; i1++) {
        f[i1 + 6 * (6 - c_i)] = b_Xup[i1];
        for (b_i = 0; b_i < 6; b_i++) {
          b_d = 0.0;
          for (qj = 0; qj < 6; qj++) {
            b_d += Xup[(qj + 6 * i1) + i] * b_I[(qj + 6 * b_i) + i];
          }
          b_a_tmp[i1 + 6 * b_i] = b_d;
        }
        for (b_i = 0; b_i < 6; b_i++) {
          b_d = 0.0;
          for (qj = 0; qj < 6; qj++) {
            b_d += b_a_tmp[i1 + 6 * qj] * Xup[(qj + 6 * b_i) + 36 * (7 - c_i)];
          }
          aoffset = i1 + 6 * b_i;
          c_I[aoffset] = b_I[aoffset + 36 * (6 - c_i)] + b_d;
        }
      }
      memcpy(&b_I[c_i * -36 + 216], &c_I[0], 36U * sizeof(double));
    }
  }
  memset(&M[0], 0, 256U * sizeof(double));
  /*  fh3 = zeros(6,3); */
  /*  fh1 = zeros(6,1); */
  for (c_i = 0; c_i < 8; c_i++) {
    if (fmod((double)c_i + 1.0, 2.0) == 0.0) {
      signed char c_tmp_data[17];
      qj = (c_i + 1) << 1;
      if (qj < qj - 2) {
        qi_size_idx_1 = 0;
      } else {
        qi_size_idx_1 = 3;
        for (i = 0; i < 3; i++) {
          qi_data[i] = (signed char)((qj + i) - 2);
        }
      }
      qj = (int)(((double)c_i + 1.0) / 2.0);
      for (i = 0; i < 3; i++) {
        for (i1 = 0; i1 < 6; i1++) {
          dJ_tmp = i1 + 6 * i;
          dJ[dJ_tmp] = Smod[dJ_tmp + 18 * (qj - 1)];
        }
      }
      for (i = 0; i < 6; i++) {
        for (i1 = 0; i1 < 3; i1++) {
          b_d = 0.0;
          for (b_i = 0; b_i < 6; b_i++) {
            b_d += b_I[(i + 6 * b_i) + 36 * c_i] * dJ[b_i + 6 * i1];
          }
          dJ_data[i + 6 * i1] = b_d;
        }
      }
      for (i = 0; i < 3; i++) {
        for (i1 = 0; i1 < 6; i1++) {
          dJ_tmp = i1 + 6 * i;
          dJ[dJ_tmp] = Smod[dJ_tmp + 18 * (qj - 1)];
        }
      }
      aoffset = qi_size_idx_1;
      for (i = 0; i < qi_size_idx_1; i++) {
        i1 = qi_data[i];
        tmp_data[i] = (signed char)(i1 - 1);
        c_tmp_data[i] = (signed char)(i1 - 1);
      }
      for (i = 0; i < 3; i++) {
        for (i1 = 0; i1 < 3; i1++) {
          b_d = 0.0;
          for (b_i = 0; b_i < 6; b_i++) {
            b_d += dJ[b_i + 6 * i] * dJ_data[b_i + 6 * i1];
          }
          y_tmp[i + 3 * i1] = b_d;
        }
      }
      for (i = 0; i < qi_size_idx_1; i++) {
        for (i1 = 0; i1 < qi_size_idx_1; i1++) {
          M[tmp_data[i1] + (c_tmp_data[i] << 4)] =
              y_tmp[i1 + qi_size_idx_1 * i];
        }
      }
      j = c_i + 1;
      while (j > 1) {
        for (i = 0; i < 6; i++) {
          for (i1 = 0; i1 < 3; i1++) {
            b_d = 0.0;
            for (b_i = 0; b_i < 6; b_i++) {
              b_d += Xup[(b_i + 6 * i) + 36 * (j - 1)] * dJ_data[b_i + 6 * i1];
            }
            dJ[i + 6 * i1] = b_d;
          }
        }
        memcpy(&dJ_data[0], &dJ[0], 18U * sizeof(double));
        j--;
        if (fmod(j, 2.0) == 0.0) {
          signed char b_tmp_data[17];
          qj = j << 1;
          if (qj < qj - 2) {
            qi_size_idx_1 = 0;
          } else {
            qi_size_idx_1 = 3;
            for (i = 0; i < 3; i++) {
              qi_data[i] = (signed char)((qj + i) - 2);
            }
          }
          for (i = 0; i < qi_size_idx_1; i++) {
            i1 = qi_data[i];
            b_tmp_data[i] = (signed char)i1;
            tmp_data[i] = (signed char)(i1 - 1);
          }
          qj = (int)((double)j / 2.0);
          for (i = 0; i < 3; i++) {
            for (i1 = 0; i1 < 6; i1++) {
              dJ_tmp = i1 + 6 * i;
              dJ[dJ_tmp] = Smod[dJ_tmp + 18 * (qj - 1)];
            }
          }
          for (i = 0; i < 3; i++) {
            for (i1 = 0; i1 < 3; i1++) {
              b_d = 0.0;
              for (b_i = 0; b_i < 6; b_i++) {
                b_d += dJ_data[b_i + 6 * i] * dJ[b_i + 6 * i1];
              }
              y_tmp[i + 3 * i1] = b_d;
            }
          }
          for (i = 0; i < qi_size_idx_1; i++) {
            for (i1 = 0; i1 < aoffset; i1++) {
              M[c_tmp_data[i1] + (tmp_data[i] << 4)] = y_tmp[i1 + aoffset * i];
            }
          }
          /* (S{j}' * fh).'; */
          for (i = 0; i < 3; i++) {
            for (i1 = 0; i1 < 6; i1++) {
              dJ_tmp = i1 + 6 * i;
              dJ[dJ_tmp] = Smod[dJ_tmp + 18 * (qj - 1)];
            }
          }
          for (i = 0; i < qi_size_idx_1; i++) {
            tmp_data[i] = (signed char)(b_tmp_data[i] - 1);
          }
          for (i = 0; i < 3; i++) {
            for (i1 = 0; i1 < 3; i1++) {
              b_d = 0.0;
              for (b_i = 0; b_i < 6; b_i++) {
                b_d += dJ[b_i + 6 * i] * dJ_data[b_i + 6 * i1];
              }
              y_tmp[i + 3 * i1] = b_d;
            }
          }
          for (i = 0; i < aoffset; i++) {
            for (i1 = 0; i1 < qi_size_idx_1; i1++) {
              M[tmp_data[i1] + (c_tmp_data[i] << 4)] =
                  y_tmp[i1 + qi_size_idx_1 * i];
            }
          }
        } else if (j == 1) {
          for (i = 0; i < 3; i++) {
            b_d = 0.0;
            for (i1 = 0; i1 < 6; i1++) {
              b_d += dJ_data[i1 + 6 * i] * (double)b_a[i1];
            }
            inertia_motor[i] = b_d;
          }
          for (i = 0; i < aoffset; i++) {
            M[c_tmp_data[i]] = inertia_motor[i];
          }
          /* (S{j}' * fh).'; */
          for (i = 0; i < 3; i++) {
            b_d = 0.0;
            for (i1 = 0; i1 < 6; i1++) {
              b_d += (double)iv1[i1] * dJ_data[i1 + 6 * i];
            }
            inertia_motor[i] = b_d;
          }
          for (i = 0; i < aoffset; i++) {
            M[c_tmp_data[i] << 4] = inertia_motor[i];
          }
        } else {
          qj = (j << 1) - 2;
          for (i = 0; i < 3; i++) {
            b_d = 0.0;
            for (i1 = 0; i1 < 6; i1++) {
              b_d += dJ_data[i1 + 6 * i] * (double)d_b[i1];
            }
            inertia_motor[i] = b_d;
          }
          for (i = 0; i < aoffset; i++) {
            M[c_tmp_data[i] + (qj << 4)] = inertia_motor[i];
          }
          /* (S{j}' * fh).'; */
          for (i = 0; i < 3; i++) {
            b_d = 0.0;
            for (i1 = 0; i1 < 6; i1++) {
              b_d += (double)iv[i1] * dJ_data[i1 + 6 * i];
            }
            inertia_motor[i] = b_d;
          }
          for (i = 0; i < aoffset; i++) {
            M[qj + (c_tmp_data[i] << 4)] = inertia_motor[i];
          }
        }
      }
    } else {
      aoffset = ((c_i + 1) << 1) - 2;
      if (c_i + 1 == 1) {
        b_d = 0.0;
        for (i = 0; i < 6; i++) {
          vJ_tmp = 0.0;
          for (i1 = 0; i1 < 6; i1++) {
            vJ_tmp += b_I[i + 6 * i1] * (double)b_a[i1];
          }
          vJ[i] = vJ_tmp;
          b_d += (double)iv1[i] * vJ_tmp;
        }
        M[aoffset + (aoffset << 4)] = b_d;
      } else {
        b_d = 0.0;
        for (i = 0; i < 6; i++) {
          vJ_tmp = 0.0;
          for (i1 = 0; i1 < 6; i1++) {
            vJ_tmp += b_I[(i + 6 * i1) + 36 * c_i] * (double)d_b[i1];
          }
          vJ[i] = vJ_tmp;
          b_d += (double)iv[i] * vJ_tmp;
        }
        M[aoffset + (aoffset << 4)] = b_d;
      }
      j = c_i + 1;
      while (j > 1) {
        for (i = 0; i < 6; i++) {
          b_d = 0.0;
          for (i1 = 0; i1 < 6; i1++) {
            b_d += Xup[(i1 + 6 * i) + 36 * (j - 1)] * vJ[i1];
          }
          b_Xup[i] = b_d;
        }
        for (i = 0; i < 6; i++) {
          vJ[i] = b_Xup[i];
        }
        j--;
        if (fmod(j, 2.0) == 0.0) {
          signed char b_tmp_data[17];
          qj = j << 1;
          if (qj < qj - 2) {
            qi_size_idx_1 = 0;
          } else {
            qi_size_idx_1 = 3;
            for (i = 0; i < 3; i++) {
              qi_data[i] = (signed char)((qj + i) - 2);
            }
          }
          for (i = 0; i < qi_size_idx_1; i++) {
            i1 = qi_data[i];
            b_tmp_data[i] = (signed char)i1;
            tmp_data[i] = (signed char)(i1 - 1);
          }
          qj = (int)((double)j / 2.0);
          for (i = 0; i < 3; i++) {
            b_d = 0.0;
            for (i1 = 0; i1 < 6; i1++) {
              b_d += vJ[i1] * Smod[(i1 + 6 * i) + 18 * (qj - 1)];
            }
            inertia_motor[i] = b_d;
          }
          for (i = 0; i < qi_size_idx_1; i++) {
            M[aoffset + (tmp_data[i] << 4)] = inertia_motor[i];
          }
          /* (S{j}' * fh).'; */
          for (i = 0; i < 3; i++) {
            for (i1 = 0; i1 < 6; i1++) {
              dJ_tmp = i1 + 6 * i;
              dJ[dJ_tmp] = Smod[dJ_tmp + 18 * (qj - 1)];
            }
          }
          for (i = 0; i < qi_size_idx_1; i++) {
            tmp_data[i] = (signed char)(b_tmp_data[i] - 1);
          }
          for (i = 0; i < 3; i++) {
            b_d = 0.0;
            for (i1 = 0; i1 < 6; i1++) {
              b_d += dJ[i1 + 6 * i] * vJ[i1];
            }
            inertia_motor[i] = b_d;
          }
          for (i = 0; i < qi_size_idx_1; i++) {
            M[tmp_data[i] + (aoffset << 4)] = inertia_motor[i];
          }
        } else if (j == 1) {
          vJ_tmp = 0.0;
          /* (S{j}' * fh).'; */
          b_d = 0.0;
          for (i = 0; i < 6; i++) {
            vJ_tmp += vJ[i] * (double)b_a[i];
            b_d = vJ_tmp;
          }
          M[aoffset] = vJ_tmp;
          M[aoffset << 4] = b_d;
        } else {
          qj = (j << 1) - 2;
          vJ_tmp = 0.0;
          /* (S{j}' * fh).'; */
          b_d = 0.0;
          for (i = 0; i < 6; i++) {
            vJ_tmp += vJ[i] * (double)d_b[i];
            b_d = vJ_tmp;
          }
          M[aoffset + (qj << 4)] = vJ_tmp;
          M[qj + (aoffset << 4)] = b_d;
        }
      }
    }
  }
}

/* End of code generation (MC_4_cg.c) */
