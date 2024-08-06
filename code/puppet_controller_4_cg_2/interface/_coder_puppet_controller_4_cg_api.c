/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_puppet_controller_4_cg_api.c
 *
 * Code generation for function 'puppet_controller_4_cg'
 *
 */

/* Include files */
#include "_coder_puppet_controller_4_cg_api.h"
#include "_coder_puppet_controller_4_cg_mex.h"

/* Variable Definitions */
emlrtCTX emlrtRootTLSGlobal = NULL;

emlrtContext emlrtContextGlobal = {
    true,                                                 /* bFirstTime */
    false,                                                /* bInitialized */
    131642U,                                              /* fVersionInfo */
    NULL,                                                 /* fErrorFunction */
    "puppet_controller_4_cg",                             /* fFunctionName */
    NULL,                                                 /* fRTCallStack */
    false,                                                /* bDebugMode */
    {2045744189U, 2170104910U, 2743257031U, 4284093946U}, /* fSigWrd */
    NULL                                                  /* fSigMem */
};

/* Function Declarations */
static real_T (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[16];

static real_T c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *d,
                                 const char_T *identifier);

static real_T d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId);

static real_T (*e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *Kp,
                                   const char_T *identifier))[256];

static real_T (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *q,
                                 const char_T *identifier))[16];

static const mxArray *emlrt_marshallOut(const real_T u[16]);

static real_T (*f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[256];

static real_T (*g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[16];

static real_T h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                 const emlrtMsgIdentifier *msgId);

static real_T (*i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[256];

/* Function Definitions */
static real_T (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[16]
{
  real_T(*y)[16];
  y = g_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static real_T c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *d,
                                 const char_T *identifier)
{
  emlrtMsgIdentifier thisId;
  real_T y;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = d_emlrt_marshallIn(sp, emlrtAlias(d), &thisId);
  emlrtDestroyArray(&d);
  return y;
}

static real_T d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId)
{
  real_T y;
  y = h_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static real_T (*e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *Kp,
                                   const char_T *identifier))[256]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[256];
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = f_emlrt_marshallIn(sp, emlrtAlias(Kp), &thisId);
  emlrtDestroyArray(&Kp);
  return y;
}

static real_T (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *q,
                                 const char_T *identifier))[16]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[16];
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = b_emlrt_marshallIn(sp, emlrtAlias(q), &thisId);
  emlrtDestroyArray(&q);
  return y;
}

static const mxArray *emlrt_marshallOut(const real_T u[16])
{
  static const int32_T i = 0;
  static const int32_T i1 = 16;
  const mxArray *m;
  const mxArray *y;
  y = NULL;
  m = emlrtCreateNumericArray(1, (const void *)&i, mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m, (void *)&u[0]);
  emlrtSetDimensions((mxArray *)m, &i1, 1);
  emlrtAssign(&y, m);
  return y;
}

static real_T (*f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[256]
{
  real_T(*y)[256];
  y = i_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static real_T (*g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[16]
{
  static const int32_T dims = 16;
  real_T(*ret)[16];
  int32_T i;
  boolean_T b = false;
  emlrtCheckVsBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 1U,
                            (const void *)&dims, &b, &i);
  ret = (real_T(*)[16])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static real_T h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                 const emlrtMsgIdentifier *msgId)
{
  static const int32_T dims = 0;
  real_T ret;
  emlrtCheckBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 0U,
                          (const void *)&dims);
  ret = *(real_T *)emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static real_T (*i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[256]
{
  static const int32_T dims[2] = {16, 16};
  real_T(*ret)[256];
  int32_T iv[2];
  boolean_T bv[2] = {false, false};
  emlrtCheckVsBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 2U,
                            (const void *)&dims[0], &bv[0], &iv[0]);
  ret = (real_T(*)[256])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

void puppet_controller_4_cg_api(const mxArray *const prhs[24],
                                const mxArray **plhs)
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  real_T(*Kp)[256];
  real_T(*ddqd)[16];
  real_T(*dq)[16];
  real_T(*dqd)[16];
  real_T(*q)[16];
  real_T(*qd)[16];
  real_T(*tau)[16];
  real_T KD;
  real_T L0;
  real_T bb;
  real_T bs;
  real_T contact;
  real_T conv_motor;
  real_T conv_pcc;
  real_T d;
  real_T hm;
  real_T ka;
  real_T kb;
  real_T kc;
  real_T ks;
  real_T m;
  real_T mm;
  real_T offset;
  real_T r;
  real_T rm;
  st.tls = emlrtRootTLSGlobal;
  tau = (real_T(*)[16])mxMalloc(sizeof(real_T[16]));
  /* Marshall function inputs */
  q = emlrt_marshallIn(&st, emlrtAlias(prhs[0]), "q");
  dq = emlrt_marshallIn(&st, emlrtAlias(prhs[1]), "dq");
  qd = emlrt_marshallIn(&st, emlrtAlias(prhs[2]), "qd");
  dqd = emlrt_marshallIn(&st, emlrtAlias(prhs[3]), "dqd");
  ddqd = emlrt_marshallIn(&st, emlrtAlias(prhs[4]), "ddqd");
  d = c_emlrt_marshallIn(&st, emlrtAliasP(prhs[5]), "d");
  m = c_emlrt_marshallIn(&st, emlrtAliasP(prhs[6]), "m");
  mm = c_emlrt_marshallIn(&st, emlrtAliasP(prhs[7]), "mm");
  hm = c_emlrt_marshallIn(&st, emlrtAliasP(prhs[8]), "hm");
  rm = c_emlrt_marshallIn(&st, emlrtAliasP(prhs[9]), "rm");
  r = c_emlrt_marshallIn(&st, emlrtAliasP(prhs[10]), "r");
  kb = c_emlrt_marshallIn(&st, emlrtAliasP(prhs[11]), "kb");
  ks = c_emlrt_marshallIn(&st, emlrtAliasP(prhs[12]), "ks");
  bb = c_emlrt_marshallIn(&st, emlrtAliasP(prhs[13]), "bb");
  bs = c_emlrt_marshallIn(&st, emlrtAliasP(prhs[14]), "bs");
  L0 = c_emlrt_marshallIn(&st, emlrtAliasP(prhs[15]), "L0");
  Kp = e_emlrt_marshallIn(&st, emlrtAlias(prhs[16]), "Kp");
  KD = c_emlrt_marshallIn(&st, emlrtAliasP(prhs[17]), "KD");
  kc = c_emlrt_marshallIn(&st, emlrtAliasP(prhs[18]), "kc");
  ka = c_emlrt_marshallIn(&st, emlrtAliasP(prhs[19]), "ka");
  offset = c_emlrt_marshallIn(&st, emlrtAliasP(prhs[20]), "offset");
  contact = c_emlrt_marshallIn(&st, emlrtAliasP(prhs[21]), "contact");
  conv_pcc = c_emlrt_marshallIn(&st, emlrtAliasP(prhs[22]), "conv_pcc");
  conv_motor = c_emlrt_marshallIn(&st, emlrtAliasP(prhs[23]), "conv_motor");
  /* Invoke the target function */
  puppet_controller_4_cg(*q, *dq, *qd, *dqd, *ddqd, d, m, mm, hm, rm, r, kb, ks,
                         bb, bs, L0, *Kp, KD, kc, ka, offset, contact, conv_pcc,
                         conv_motor, *tau);
  /* Marshall function outputs */
  *plhs = emlrt_marshallOut(*tau);
}

void puppet_controller_4_cg_atexit(void)
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtEnterRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  puppet_controller_4_cg_xil_terminate();
  puppet_controller_4_cg_xil_shutdown();
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

void puppet_controller_4_cg_initialize(void)
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, NULL);
  emlrtEnterRtStackR2012b(&st);
  emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

void puppet_controller_4_cg_terminate(void)
{
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/* End of code generation (_coder_puppet_controller_4_cg_api.c) */
