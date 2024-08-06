/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * PCC_jacobian.c
 *
 * Code generation for function 'PCC_jacobian'
 *
 */

/* Include files */
#include "PCC_jacobian.h"
#include "puppet_controller_4_cg_rtwutil.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
void PCC_jacobian(const double q_data[], double d, double L0,
                  const double qd_data[], double X[36], double J[18],
                  double T_q[16], double dJ[18])
{
  double b_I[9];
  double c_I[9];
  double d_I[9];
  double dv[9];
  double T_q_tmp_tmp;
  double a;
  double a_tmp;
  double b_del_tmp;
  double c_del_tmp;
  double d_del_tmp;
  double del_tmp;
  int T_q_tmp;
  int i;
  del_tmp = q_data[0] * q_data[0];
  b_del_tmp = q_data[1] * q_data[1];
  c_del_tmp = del_tmp + b_del_tmp;
  d_del_tmp = sqrt(c_del_tmp);
  if (d_del_tmp < 1.0E-6) {
    memset(&b_I[0], 0, 9U * sizeof(double));
    b_I[0] = 1.0;
    b_I[4] = 1.0;
    b_I[8] = 1.0;
    for (i = 0; i < 3; i++) {
      T_q_tmp = i << 2;
      T_q[T_q_tmp] = b_I[3 * i];
      T_q[T_q_tmp + 1] = b_I[3 * i + 1];
      T_q[T_q_tmp + 2] = b_I[3 * i + 2];
    }
    double J_tmp;
    double b_T_q_tmp;
    double dJ_tmp;
    double dJ_tmp_tmp;
    T_q[12] = 0.0;
    T_q[13] = 0.0;
    b_T_q_tmp = L0 + q_data[2];
    T_q[14] = b_T_q_tmp;
    T_q[3] = 0.0;
    T_q[7] = 0.0;
    T_q[11] = 0.0;
    T_q[15] = 1.0;
    J_tmp = b_T_q_tmp / (2.0 * d);
    J[0] = J_tmp;
    J[6] = 0.0;
    J[12] = 0.0;
    J[1] = 0.0;
    J[7] = J_tmp;
    J[13] = 0.0;
    J[3] = 0.0;
    J[9] = -1.0 / d;
    J[15] = 0.0;
    J[4] = 1.0 / d;
    J[10] = 0.0;
    J[16] = 0.0;
    dJ_tmp = qd_data[2] / (2.0 * d);
    dJ[0] = dJ_tmp;
    dJ[6] = 0.0;
    dJ[12] = -qd_data[0] / (2.0 * d);
    dJ[1] = 0.0;
    dJ[7] = dJ_tmp;
    dJ[13] = -qd_data[1] / (2.0 * d);
    dJ_tmp_tmp = d * d;
    dJ_tmp = 6.0 * dJ_tmp_tmp;
    dJ[2] = qd_data[0] * b_T_q_tmp / dJ_tmp;
    dJ[8] = (L0 * qd_data[1] + qd_data[1] * q_data[2]) / dJ_tmp;
    dJ[14] = 0.0;
    J[2] = 0.0;
    J[5] = 0.0;
    dJ[3] = 0.0;
    dJ[4] = 0.0;
    J[8] = 0.0;
    J[11] = 0.0;
    dJ[9] = 0.0;
    dJ[10] = 0.0;
    J[14] = 1.0;
    J[17] = 0.0;
    dJ[15] = 0.0;
    dJ[16] = 0.0;
    dJ_tmp = 2.0 * dJ_tmp_tmp;
    dJ[5] = qd_data[1] / dJ_tmp;
    dJ[11] = -qd_data[0] / dJ_tmp;
    dJ[17] = 0.0;
  } else {
    double J_tmp;
    double J_tmp_tmp;
    double J_tmp_tmp_tmp;
    double b_J_tmp;
    double b_J_tmp_tmp;
    double b_T_q_tmp;
    double b_T_q_tmp_tmp;
    double b_a_tmp;
    double b_dJ_tmp;
    double b_dJ_tmp_tmp;
    double c_J_tmp;
    double c_J_tmp_tmp;
    double c_T_q_tmp;
    double c_T_q_tmp_tmp;
    double c_a_tmp;
    double c_dJ_tmp;
    double c_dJ_tmp_tmp;
    double dJ_tmp;
    double dJ_tmp_tmp;
    double d_J_tmp;
    double d_J_tmp_tmp;
    double d_dJ_tmp;
    double d_dJ_tmp_tmp;
    double e_J_tmp;
    double e_J_tmp_tmp;
    double e_dJ_tmp;
    double e_dJ_tmp_tmp;
    double f_J_tmp;
    double f_J_tmp_tmp;
    double f_dJ_tmp;
    double g_J_tmp;
    double g_J_tmp_tmp;
    double g_dJ_tmp;
    double h_J_tmp;
    double h_J_tmp_tmp;
    double h_dJ_tmp;
    double i_J_tmp;
    double i_dJ_tmp;
    double j_J_tmp;
    double j_dJ_tmp;
    double k_J_tmp;
    double k_dJ_tmp;
    double l_J_tmp;
    double l_dJ_tmp;
    double m_J_tmp;
    double m_dJ_tmp;
    double n_J_tmp;
    double n_dJ_tmp;
    double o_J_tmp;
    double o_dJ_tmp;
    double p_J_tmp;
    double q_J_tmp;
    double r_J_tmp;
    a_tmp = d_del_tmp * d_del_tmp;
    b_a_tmp = L0 + q_data[2];
    c_a_tmp = d * b_a_tmp;
    a = c_a_tmp / a_tmp;
    T_q_tmp_tmp = d_del_tmp / d;
    b_T_q_tmp_tmp = cos(T_q_tmp_tmp);
    T_q[0] = del_tmp / a_tmp * (b_T_q_tmp_tmp - 1.0) + 1.0;
    c_T_q_tmp_tmp = q_data[0] * q_data[1];
    b_T_q_tmp = c_T_q_tmp_tmp / a_tmp * (b_T_q_tmp_tmp - 1.0);
    T_q[1] = b_T_q_tmp;
    c_T_q_tmp = sin(T_q_tmp_tmp);
    T_q[2] = -q_data[0] / d_del_tmp * c_T_q_tmp;
    T_q[4] = b_T_q_tmp;
    T_q[5] = b_del_tmp / a_tmp * (b_T_q_tmp_tmp - 1.0) + 1.0;
    T_q[6] = -q_data[1] / d_del_tmp * c_T_q_tmp;
    T_q[8] = q_data[0] / d_del_tmp * c_T_q_tmp;
    T_q[9] = q_data[1] / d_del_tmp * c_T_q_tmp;
    T_q[10] = b_T_q_tmp_tmp;
    T_q[12] = a * (q_data[0] * (1.0 - b_T_q_tmp_tmp));
    T_q[13] = a * (q_data[1] * (1.0 - b_T_q_tmp_tmp));
    b_T_q_tmp = d_del_tmp * c_T_q_tmp;
    T_q[14] = a * b_T_q_tmp;
    T_q[3] = 0.0;
    T_q[7] = 0.0;
    T_q[11] = 0.0;
    T_q[15] = 1.0;
    J_tmp_tmp = del_tmp * c_T_q_tmp;
    J_tmp = J_tmp_tmp * b_a_tmp;
    b_J_tmp = c_del_tmp * c_del_tmp;
    c_J_tmp = rt_powd_snf(c_del_tmp, 1.5);
    T_q_tmp_tmp = c_a_tmp * (b_T_q_tmp_tmp - 1.0);
    d_J_tmp = T_q_tmp_tmp / c_del_tmp;
    b_J_tmp_tmp = d * c_T_q_tmp;
    e_J_tmp = b_T_q_tmp_tmp * d_del_tmp - b_J_tmp_tmp;
    c_J_tmp_tmp = rt_powd_snf(c_del_tmp, 3.0);
    J_tmp_tmp_tmp = del_tmp * b_del_tmp;
    a_tmp = J_tmp_tmp_tmp * b_a_tmp * (b_T_q_tmp_tmp - 1.0);
    d_J_tmp_tmp = (b_T_q_tmp - 2.0 * d) + 2.0 * d * b_T_q_tmp_tmp;
    f_J_tmp = a_tmp * d_J_tmp_tmp / c_J_tmp_tmp;
    e_J_tmp_tmp = del_tmp * b_T_q_tmp_tmp;
    g_J_tmp = b_del_tmp + e_J_tmp_tmp;
    h_J_tmp = (J_tmp / c_J_tmp - d_J_tmp) +
              2.0 * d * del_tmp * b_a_tmp * (b_T_q_tmp_tmp - 1.0) / b_J_tmp;
    J[0] =
        (g_J_tmp * h_J_tmp / c_del_tmp - J_tmp * e_J_tmp / b_J_tmp) + f_J_tmp;
    J[6] = 0.0;
    J_tmp = d * q_data[0] * (b_T_q_tmp_tmp - 1.0);
    J[12] = J_tmp / c_del_tmp;
    J[1] = 0.0;
    f_J_tmp_tmp = b_del_tmp * c_T_q_tmp;
    i_J_tmp = f_J_tmp_tmp * b_a_tmp;
    g_J_tmp_tmp = b_del_tmp * b_T_q_tmp_tmp;
    j_J_tmp = del_tmp + g_J_tmp_tmp;
    d_J_tmp = (i_J_tmp / c_J_tmp - d_J_tmp) +
              2.0 * d * b_del_tmp * b_a_tmp * (b_T_q_tmp_tmp - 1.0) / b_J_tmp;
    J[7] =
        (j_J_tmp * d_J_tmp / c_del_tmp - i_J_tmp * e_J_tmp / b_J_tmp) + f_J_tmp;
    f_J_tmp = d * q_data[1] * (b_T_q_tmp_tmp - 1.0);
    J[13] = f_J_tmp / c_del_tmp;
    i_J_tmp = c_J_tmp - b_J_tmp_tmp * c_del_tmp;
    k_J_tmp = rt_powd_snf(c_del_tmp, 2.5);
    l_J_tmp = q_data[0] * b_a_tmp;
    J[2] = l_J_tmp * i_J_tmp / k_J_tmp;
    m_J_tmp = q_data[1] * b_a_tmp;
    J[8] = m_J_tmp * i_J_tmp / k_J_tmp;
    J[14] = b_J_tmp_tmp / d_del_tmp;
    n_J_tmp = d * c_J_tmp;
    h_J_tmp_tmp = d_del_tmp - b_J_tmp_tmp;
    o_J_tmp = c_T_q_tmp_tmp * h_J_tmp_tmp;
    J[3] = -o_J_tmp / n_J_tmp;
    p_J_tmp = b_del_tmp * d_del_tmp + d * del_tmp * c_T_q_tmp;
    J[9] = -p_J_tmp / n_J_tmp;
    J[15] = 0.0;
    q_J_tmp = del_tmp * d_del_tmp + d * b_del_tmp * c_T_q_tmp;
    J[4] = q_J_tmp / n_J_tmp;
    J[10] = o_J_tmp / n_J_tmp;
    J[16] = 0.0;
    o_J_tmp = q_data[1] * (b_T_q_tmp_tmp - 1.0);
    J[5] = -o_J_tmp / c_del_tmp;
    r_J_tmp = q_data[0] * (b_T_q_tmp_tmp - 1.0);
    J[11] = r_J_tmp / c_del_tmp;
    J[17] = 0.0;
    /*      J = [                                                       -(d*(L0
     * + dL)*(cos(del/d)*dx^2 + dy^2)*(cos(del/d) - 1))/del^4, -(d*dx*dy*(L0 +
     * dL)*(cos(del/d) - 1)^2)/del^4, - (d*dx*sin(del/d)^2)/del^2 -
     * (d*dx*dy^2*(cos(del/d) - 1)^2)/del^4 - (d*dx*(cos(del/d)*dx^2 +
     * dy^2)*(cos(del/d) - 1))/del^4; */
    /*                                                                                  -(d*dx*dy*(L0
     * + dL)*(cos(del/d) - 1)^2)/del^4,                               -(d*(L0 +
     * dL)*(dx^2 + cos(del/d)*dy^2)*(cos(del/d) - 1))/del^4,
     * (d*dy*sin(del/d)^2)/del^2 - (d*dx^2*dy*(cos(del/d) - 1)^2)/del^4 -
     * (d*dy*(dx^2 + cos(del/d)*dy^2)*(cos(del/d) - 1))/del^4; */
    /*                                                                            -(d*dx*sin(del/d)*(L0
     * + dL)*(cos(del/d) - 1))/del^3, (d*dy*sin(del/d)*(L0 + dL)*(cos(del/d) -
     * 1))/del^3, (d*sin(del/d)*(del^2*cos(del/d) - dx^2*cos(del/d) +
     * dy^2*cos(del/d) + dx^2 - dy^2))/del^3; */
    /*                                                                                      (dx*dy*sin(del/d)*(cos(del/d)
     * - 1))/del^3, -(sin(del/d)*(del^2*cos(del/d) - dx^2*cos(del/d) +
     * 2*dy^2*cos(del/d) + dx^2 - 2*dy^2))/del^3, 0; */
    /*                                                                                    (sin(del/d)*(cos(del/d)*dx^2
     * + dy^2))/del^3, (dx*dy*sin(del/d)*(cos(del/d) - 1))/del^3, 0; */
    /*          (dy*(dx^2 + cos(del/d)*dy^2)*(cos(del/d) - 1))/del^4 -
     * (dy*sin(del/d)^2)/del^2 + (2*dx^2*dy*(cos(del/d) - 1)^2)/del^4, (dx*(dx^2
     * + cos(del/d)*dy^2)*(cos(del/d) - 1))/del^4, 0]; */
    dJ_tmp_tmp = 2.0 * q_data[0] * qd_data[0];
    b_dJ_tmp_tmp = 2.0 * q_data[1] * qd_data[1];
    dJ_tmp = dJ_tmp_tmp + b_dJ_tmp_tmp;
    c_dJ_tmp_tmp = qd_data[2] * del_tmp;
    b_dJ_tmp = c_dJ_tmp_tmp * c_T_q_tmp;
    c_dJ_tmp = 2.0 * d * b_J_tmp;
    d_dJ_tmp = dJ_tmp_tmp * c_T_q_tmp * b_a_tmp;
    e_dJ_tmp = e_J_tmp_tmp * b_a_tmp;
    f_dJ_tmp = 2.0 * del_tmp;
    g_dJ_tmp = 2.0 * c_J_tmp;
    h_dJ_tmp = c_T_q_tmp * b_a_tmp * dJ_tmp / g_dJ_tmp -
               d * qd_data[2] * (b_T_q_tmp_tmp - 1.0) / c_del_tmp;
    i_dJ_tmp = 2.0 * k_J_tmp;
    j_dJ_tmp = 2.0 * d * qd_data[2];
    k_dJ_tmp = T_q_tmp_tmp * dJ_tmp / b_J_tmp;
    l_dJ_tmp = 2.0 * d * d_del_tmp;
    m_dJ_tmp = c_T_q_tmp * c_T_q_tmp;
    d_dJ_tmp_tmp = b_T_q_tmp_tmp * dJ_tmp;
    e_dJ_tmp_tmp = 2.0 * d_del_tmp;
    a_tmp = a_tmp *
            (c_T_q_tmp * dJ_tmp / e_dJ_tmp_tmp - d_dJ_tmp_tmp / (2.0 * d)) /
            c_J_tmp_tmp;
    a = c_dJ_tmp_tmp * b_del_tmp * (b_T_q_tmp_tmp - 1.0) * d_J_tmp_tmp /
        c_J_tmp_tmp;
    n_dJ_tmp = 2.0 * d * k_J_tmp;
    c_a_tmp = 3.0 * del_tmp * b_del_tmp * b_a_tmp * (b_T_q_tmp_tmp - 1.0) *
              dJ_tmp * d_J_tmp_tmp / rt_powd_snf(c_del_tmp, 4.0);
    b_T_q_tmp = dJ_tmp_tmp * b_del_tmp * b_a_tmp * (b_T_q_tmp_tmp - 1.0) *
                d_J_tmp_tmp / c_J_tmp_tmp;
    o_dJ_tmp = f_dJ_tmp * q_data[1] * qd_data[1] * b_a_tmp *
               (b_T_q_tmp_tmp - 1.0) * d_J_tmp_tmp / c_J_tmp_tmp;
    c_dJ_tmp_tmp = rt_powd_snf(c_del_tmp, 3.5);
    T_q_tmp_tmp = J_tmp_tmp_tmp * c_T_q_tmp * b_a_tmp * dJ_tmp * d_J_tmp_tmp /
                  (2.0 * d * c_dJ_tmp_tmp);
    dJ[0] =
        ((((((((((((g_J_tmp *
                        ((((((((h_dJ_tmp + b_dJ_tmp / c_J_tmp) -
                               5.0 * del_tmp * c_T_q_tmp * b_a_tmp * dJ_tmp /
                                   i_dJ_tmp) +
                              d_dJ_tmp / c_J_tmp) +
                             j_dJ_tmp * del_tmp * (b_T_q_tmp_tmp - 1.0) /
                                 b_J_tmp) +
                            k_dJ_tmp) +
                           4.0 * d * q_data[0] * qd_data[0] * b_a_tmp *
                               (b_T_q_tmp_tmp - 1.0) / b_J_tmp) -
                          4.0 * d * del_tmp * b_a_tmp * (b_T_q_tmp_tmp - 1.0) *
                              dJ_tmp / c_J_tmp_tmp) +
                         e_dJ_tmp * dJ_tmp / c_dJ_tmp) /
                        c_del_tmp +
                    h_J_tmp *
                        ((b_dJ_tmp_tmp + dJ_tmp_tmp * b_T_q_tmp_tmp) -
                         J_tmp_tmp * dJ_tmp / l_dJ_tmp) /
                        c_del_tmp) -
                   g_J_tmp * dJ_tmp * h_J_tmp / b_J_tmp) -
                  b_dJ_tmp * e_J_tmp / b_J_tmp) +
                 del_tmp * m_dJ_tmp * b_a_tmp * dJ_tmp / c_dJ_tmp) +
                f_dJ_tmp * c_T_q_tmp * b_a_tmp * e_J_tmp * dJ_tmp /
                    c_J_tmp_tmp) -
               a_tmp) -
              d_dJ_tmp * e_J_tmp / b_J_tmp) +
             a) -
            e_dJ_tmp * e_J_tmp * dJ_tmp / n_dJ_tmp) -
           c_a_tmp) +
          b_T_q_tmp) +
         o_dJ_tmp) -
        T_q_tmp_tmp;
    dJ[6] = 0.0;
    b_dJ_tmp = q_data[0] * c_T_q_tmp * dJ_tmp;
    dJ[12] = (d * qd_data[0] * (b_T_q_tmp_tmp - 1.0) / c_del_tmp -
              b_dJ_tmp / g_dJ_tmp) -
             J_tmp * dJ_tmp / b_J_tmp;
    dJ[1] = 0.0;
    d_dJ_tmp = qd_data[2] * b_del_tmp * c_T_q_tmp;
    e_dJ_tmp = b_dJ_tmp_tmp * c_T_q_tmp * b_a_tmp;
    f_dJ_tmp = g_J_tmp_tmp * b_a_tmp;
    dJ[7] =
        ((((((((((((j_J_tmp *
                        ((((((((h_dJ_tmp + d_dJ_tmp / c_J_tmp) -
                               5.0 * b_del_tmp * c_T_q_tmp * b_a_tmp * dJ_tmp /
                                   i_dJ_tmp) +
                              e_dJ_tmp / c_J_tmp) +
                             j_dJ_tmp * b_del_tmp * (b_T_q_tmp_tmp - 1.0) /
                                 b_J_tmp) +
                            k_dJ_tmp) +
                           4.0 * d * q_data[1] * qd_data[1] * b_a_tmp *
                               (b_T_q_tmp_tmp - 1.0) / b_J_tmp) -
                          4.0 * d * b_del_tmp * b_a_tmp *
                              (b_T_q_tmp_tmp - 1.0) * dJ_tmp / c_J_tmp_tmp) +
                         f_dJ_tmp * dJ_tmp / c_dJ_tmp) /
                        c_del_tmp +
                    d_J_tmp *
                        ((dJ_tmp_tmp + b_dJ_tmp_tmp * b_T_q_tmp_tmp) -
                         f_J_tmp_tmp * dJ_tmp / l_dJ_tmp) /
                        c_del_tmp) -
                   j_J_tmp * dJ_tmp * d_J_tmp / b_J_tmp) -
                  d_dJ_tmp * e_J_tmp / b_J_tmp) +
                 b_del_tmp * m_dJ_tmp * b_a_tmp * dJ_tmp / c_dJ_tmp) +
                2.0 * b_del_tmp * c_T_q_tmp * b_a_tmp * e_J_tmp * dJ_tmp /
                    c_J_tmp_tmp) -
               a_tmp) -
              e_dJ_tmp * e_J_tmp / b_J_tmp) +
             a) -
            f_dJ_tmp * e_J_tmp * dJ_tmp / n_dJ_tmp) -
           c_a_tmp) +
          b_T_q_tmp) +
         o_dJ_tmp) -
        T_q_tmp_tmp;
    c_dJ_tmp = q_data[1] * c_T_q_tmp * dJ_tmp;
    dJ[13] = (d * qd_data[1] * (b_T_q_tmp_tmp - 1.0) / c_del_tmp -
              c_dJ_tmp / g_dJ_tmp) -
             f_J_tmp * dJ_tmp / b_J_tmp;
    T_q_tmp_tmp = b_J_tmp_tmp * dJ_tmp;
    d_dJ_tmp =
        (d_dJ_tmp_tmp * d_del_tmp / 2.0 - 3.0 * dJ_tmp * d_del_tmp / 2.0) +
        T_q_tmp_tmp;
    e_dJ_tmp = 2.0 * c_dJ_tmp_tmp;
    dJ[2] = ((qd_data[0] * b_a_tmp * i_J_tmp / k_J_tmp -
              l_J_tmp * d_dJ_tmp / k_J_tmp) +
             q_data[0] * qd_data[2] * i_J_tmp / k_J_tmp) -
            5.0 * q_data[0] * b_a_tmp * i_J_tmp * dJ_tmp / e_dJ_tmp;
    dJ[8] = ((qd_data[1] * b_a_tmp * i_J_tmp / k_J_tmp -
              m_J_tmp * d_dJ_tmp / k_J_tmp) +
             q_data[1] * qd_data[2] * i_J_tmp / k_J_tmp) -
            5.0 * q_data[1] * b_a_tmp * i_J_tmp * dJ_tmp / e_dJ_tmp;
    dJ[14] = d_dJ_tmp_tmp / (2.0 * c_del_tmp) - T_q_tmp_tmp / g_dJ_tmp;
    d_dJ_tmp = c_T_q_tmp_tmp *
               (dJ_tmp / e_dJ_tmp_tmp - d_dJ_tmp_tmp / e_dJ_tmp_tmp) / n_J_tmp;
    e_dJ_tmp = q_data[0] * qd_data[1] * h_J_tmp_tmp / n_J_tmp;
    f_dJ_tmp = qd_data[0] * q_data[1] * h_J_tmp_tmp / n_J_tmp;
    g_dJ_tmp = 3.0 * q_data[0] * q_data[1] * h_J_tmp_tmp * dJ_tmp / n_dJ_tmp;
    dJ[3] = ((g_dJ_tmp - e_dJ_tmp) - f_dJ_tmp) - d_dJ_tmp;
    dJ[9] = 3.0 * p_J_tmp * dJ_tmp / n_dJ_tmp -
            (((b_del_tmp * dJ_tmp / e_dJ_tmp_tmp + b_dJ_tmp_tmp * d_del_tmp) +
              2.0 * d * q_data[0] * qd_data[0] * c_T_q_tmp) +
             e_J_tmp_tmp * dJ_tmp / e_dJ_tmp_tmp) /
                n_J_tmp;
    dJ[15] = 0.0;
    dJ[4] = (((del_tmp * dJ_tmp / e_dJ_tmp_tmp + dJ_tmp_tmp * d_del_tmp) +
              2.0 * d * q_data[1] * qd_data[1] * c_T_q_tmp) +
             g_J_tmp_tmp * dJ_tmp / e_dJ_tmp_tmp) /
                n_J_tmp -
            3.0 * q_J_tmp * dJ_tmp / n_dJ_tmp;
    dJ[10] = ((d_dJ_tmp + e_dJ_tmp) + f_dJ_tmp) - g_dJ_tmp;
    dJ[16] = 0.0;
    d_dJ_tmp = 2.0 * d * c_J_tmp;
    dJ[5] = (o_J_tmp * dJ_tmp / b_J_tmp -
             qd_data[1] * (b_T_q_tmp_tmp - 1.0) / c_del_tmp) +
            c_dJ_tmp / d_dJ_tmp;
    dJ[11] = (qd_data[0] * (b_T_q_tmp_tmp - 1.0) / c_del_tmp -
              r_J_tmp * dJ_tmp / b_J_tmp) -
             b_dJ_tmp / d_dJ_tmp;
    dJ[17] = 0.0;
  }
  /*  Adjoint calculator */
  /*  This function calculates the adjoint given simply the transform between */
  /*  joints */
  /*  convert transform to rotation and translation */
  /*  get skew symmetric matrix of translation */
  for (i = 0; i < 3; i++) {
    b_I[3 * i] = T_q[i];
    b_I[3 * i + 1] = T_q[i + 4];
    b_I[3 * i + 2] = T_q[i + 8];
  }
  for (i = 0; i < 9; i++) {
    c_I[i] = -b_I[i];
  }
  dv[0] = 0.0;
  dv[3] = -T_q[14];
  dv[6] = T_q[13];
  dv[1] = T_q[14];
  dv[4] = 0.0;
  dv[7] = -T_q[12];
  dv[2] = -T_q[13];
  dv[5] = T_q[12];
  dv[8] = 0.0;
  for (i = 0; i < 3; i++) {
    T_q_tmp_tmp = c_I[i];
    a_tmp = c_I[i + 3];
    a = c_I[i + 6];
    for (T_q_tmp = 0; T_q_tmp < 3; T_q_tmp++) {
      d_I[i + 3 * T_q_tmp] =
          (T_q_tmp_tmp * dv[3 * T_q_tmp] + a_tmp * dv[3 * T_q_tmp + 1]) +
          a * dv[3 * T_q_tmp + 2];
      X[T_q_tmp + 6 * i] = b_I[T_q_tmp + 3 * i];
    }
  }
  for (i = 0; i < 3; i++) {
    int X_tmp;
    T_q_tmp = 6 * (i + 3);
    X[T_q_tmp] = d_I[3 * i];
    X[6 * i + 3] = 0.0;
    X[T_q_tmp + 3] = b_I[3 * i];
    X_tmp = 3 * i + 1;
    X[T_q_tmp + 1] = d_I[X_tmp];
    X[6 * i + 4] = 0.0;
    X[T_q_tmp + 4] = b_I[X_tmp];
    X_tmp = 3 * i + 2;
    X[T_q_tmp + 2] = d_I[X_tmp];
    X[6 * i + 5] = 0.0;
    X[T_q_tmp + 5] = b_I[X_tmp];
  }
}

/* End of code generation (PCC_jacobian.c) */
