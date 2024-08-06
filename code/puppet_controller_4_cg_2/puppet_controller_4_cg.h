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
extern void puppet_controller_4_cg(const double q[16], const double dq[16],
                                   const double qd[16], const double dqd[16],
                                   const double ddqd[16], double d, double m,
                                   double mm, double hm, double rm, double r,
                                   double kb, double ks, double bb, double bs,
                                   double L0, const double Kp[256], double KD,
                                   double kc, double ka, double offset,
                                   double contact, double conv_pcc,
                                   double conv_motor, double tau[16]);

#ifdef __cplusplus
}
#endif

#endif
/* End of code generation (puppet_controller_4_cg.h) */
