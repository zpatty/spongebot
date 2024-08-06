/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * MC_4_cg.h
 *
 * Code generation for function 'MC_4_cg'
 *
 */

#ifndef MC_4_CG_H
#define MC_4_CG_H

/* Include files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void MC_4_cg(const double q[16], const double qd[16], double m, double mm,
             double hm, double rm, double r, double L0, double d, double M[256],
             double C[16]);

#ifdef __cplusplus
}
#endif

#endif
/* End of code generation (MC_4_cg.h) */
