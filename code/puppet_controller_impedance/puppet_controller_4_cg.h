/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * puppet_controller_4_cg.h
 *
 * Code generation for function 'puppet_controller_4_cg'
 *
 */

#ifndef PUPPET_CONTROLLER_4_CG_H
#define PUPPET_CONTROLLER_4_CG_H

/* Include files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void puppet_controller_4_cg(
    const double q[16], const double dq[16], const double qd[16],
    const double dqd[16], const double ddqd[16], double d, double m, double mm,
    double hm, double rm, double r, double kb, double ks, double bb, double bs,
    double bm, double L0, const double Kp[256], const double KD[256],
    double Kpx, double KDx, const double xd[3], const double dxd[3],
    const double dxr[3], double kc, double ka, double offset, double contact,
    double conv_pcc, double conv_motor, double tau[16], double tau_r[16],
    double x[3], double cq[4]);

#ifdef __cplusplus
}
#endif

#endif
/* End of code generation (puppet_controller_4_cg.h) */
