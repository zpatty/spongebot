/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_puppet_controller_4_cg_api.h
 *
 * Code generation for function 'puppet_controller_4_cg'
 *
 */

#ifndef _CODER_PUPPET_CONTROLLER_4_CG_API_H
#define _CODER_PUPPET_CONTROLLER_4_CG_API_H

/* Include files */
#include "emlrt.h"
#include "tmwtypes.h"
#include <string.h>

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void puppet_controller_4_cg(real_T q[16], real_T dq[16], real_T qd[16],
                            real_T dqd[16], real_T ddqd[16], real_T d, real_T m,
                            real_T mm, real_T hm, real_T rm, real_T r,
                            real_T kb, real_T ks, real_T bb, real_T bs,
                            real_T bm, real_T L0, real_T Kp[256],
                            real_T KD[256], real_T Kpx, real_T KDx,
                            real_T xd[3], real_T dxd[3], real_T dxr[3],
                            real_T kc, real_T ka, real_T offset, real_T contact,
                            real_T conv_pcc, real_T conv_motor, real_T tau[16],
                            real_T tau_r[16], real_T x[3], real_T cq[4]);

void puppet_controller_4_cg_api(const mxArray *const prhs[30], int32_T nlhs,
                                const mxArray *plhs[4]);

void puppet_controller_4_cg_atexit(void);

void puppet_controller_4_cg_initialize(void);

void puppet_controller_4_cg_terminate(void);

void puppet_controller_4_cg_xil_shutdown(void);

void puppet_controller_4_cg_xil_terminate(void);

#ifdef __cplusplus
}
#endif

#endif
/* End of code generation (_coder_puppet_controller_4_cg_api.h) */
