/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * PCC_jacobian.h
 *
 * Code generation for function 'PCC_jacobian'
 *
 */

#ifndef PCC_JACOBIAN_H
#define PCC_JACOBIAN_H

/* Include files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void PCC_jacobian(const double q_data[], double d, double L0,
                  const double qd_data[], double X[36], double J[18],
                  double T_q[16], double dJ[18]);

#ifdef __cplusplus
}
#endif

#endif
/* End of code generation (PCC_jacobian.h) */
