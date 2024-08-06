/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_puppet_controller_4_cg_info.c
 *
 * Code generation for function 'puppet_controller_4_cg'
 *
 */

/* Include files */
#include "_coder_puppet_controller_4_cg_info.h"
#include "emlrt.h"
#include "tmwtypes.h"

/* Function Declarations */
static const mxArray *emlrtMexFcnResolvedFunctionsInfo(void);

/* Function Definitions */
static const mxArray *emlrtMexFcnResolvedFunctionsInfo(void)
{
  const mxArray *nameCaptureInfo;
  const char_T *data[8] = {
      "789ced5acd73d240140f16b59df1838bfe05bd5ab07cf971d086d2522bb4a52df5639c74"
      "49b61048b26b12281dcf9ebdf6c8c5194fdaabff8e5767bc79752c5d"
      "024966762005b780fb2e8fc78fdddfcb7bec6f1e094268231f1204e18e40ec4b8cf8dbdd"
      "38d2f5d704aff9f150d72ff862c7ae0b61cf3a07ffd4f532326cd8b2",
      "4960001df6562a48570d60d87b27180a26b490d684ca0572a46a704fd5e1ae3b2874227d"
      "cd05f5820ed4799da942b9bedbd005b36af533d4dc41af1e6794eb0d"
      "0f598f0d4a3d223efc6df6dda20e6c0d944d84ecc56845b51f9810232b4ade8dae165f46"
      "71c3aae206c6d0b6a2c44b9dba9948d3a0292524b9b2a4fbf23f1c31",
      "ff9bd4fc0992cf5cf08e8def06958f200a6a9435d8bfbef6887c4fa87c5e3c707fba8571"
      "1a32a83e7787ccd7effb9f9fbff0e9ef9b9025dfd7df9fe758f23976"
      "557c2dca7ec37edfee51f8223e1c255ab8965044514cc673c957a054da3a79edca637b00"
      "cfa03c044acc6aff3665fdb49ddb61f30dfbe27ebe04b1eaf0789c7c",
      "4175f47444be3495cf8b07ee47a730e7cd6075de1f31d6cf5fa7e5344b3ec7a6553fef53"
      "f8223e3c5959cec64c5038da2f5bab7833bdd2d40aeb22d7cf69d5cf"
      "795fdccf972040a94932d0e4699d439f52f9bc78e07e3885e9366456e7d06f7f7ef03954"
      "18bf8eeac50c2ce60b39f5cdfbecb251db2995f24a699debe8a49d5b",
      "3e8712e373e8e5f8f81c4a8ccfa1c1f66f53d64fea1c8a47ccd77fffda9faf835b18d82a"
      "d024d94496255c9d9eb647e47b4ee5f3e2c1f5d45da0259debea78f8"
      "1c9b755dade2ad15d4c4e28e9ede2fc41f272bb87870cc7575e2ce2d9f4b89f1b9f4727c"
      "5c3f89f1b934d8fe6dcafa599d4b6f0dc8d7c1b73319a90664545681",
      "e1e63f1c919ff55cfa8ccae7c503f7c75d2086bacaef97fe5b3e56ba2a2770bdb66154eb"
      "0d242aa2b9be9f6a957239aeab93766ef97327629372ff9aebe8707c"
      "8ecdba8ecefa73a733cafaffe57fa573d4fc09f24232c7cac7faf77e8acae7c503f7e7bc"
      "309d66b03aff3f3f20a67a9a7af87181259f63b3aea799444e5f2b64",
      "93b526c8ca56397610d78b07d9e9d7d3bf022ba516",
      ""};
  nameCaptureInfo = NULL;
  emlrtNameCaptureMxArrayR2016a(&data[0], 12480U, &nameCaptureInfo);
  return nameCaptureInfo;
}

mxArray *emlrtMexFcnProperties(void)
{
  mxArray *xEntryPoints;
  mxArray *xInputs;
  mxArray *xResult;
  const char_T *propFieldName[7] = {
      "Version",      "ResolvedFunctions", "Checksum",    "EntryPoints",
      "CoverageInfo", "IsPolymorphic",     "PropertyList"};
  const char_T *epFieldName[6] = {
      "Name",           "NumberOfInputs", "NumberOfOutputs",
      "ConstantInputs", "FullPath",       "TimeStamp"};
  xEntryPoints =
      emlrtCreateStructMatrix(1, 1, 6, (const char_T **)&epFieldName[0]);
  xInputs = emlrtCreateLogicalMatrix(1, 30);
  emlrtSetField(xEntryPoints, 0, "Name",
                emlrtMxCreateString("puppet_controller_4_cg"));
  emlrtSetField(xEntryPoints, 0, "NumberOfInputs",
                emlrtMxCreateDoubleScalar(30.0));
  emlrtSetField(xEntryPoints, 0, "NumberOfOutputs",
                emlrtMxCreateDoubleScalar(4.0));
  emlrtSetField(xEntryPoints, 0, "ConstantInputs", xInputs);
  emlrtSetField(xEntryPoints, 0, "FullPath",
                emlrtMxCreateString("/home/zach/git-repos/matlab/DRL/"
                                    "pushpuppets/puppet_controller_4_cg.m"));
  emlrtSetField(xEntryPoints, 0, "TimeStamp",
                emlrtMxCreateDoubleScalar(739233.72953703708));
  xResult =
      emlrtCreateStructMatrix(1, 1, 7, (const char_T **)&propFieldName[0]);
  emlrtSetField(xResult, 0, "Version",
                emlrtMxCreateString("9.14.0.2239454 (R2023a) Update 1"));
  emlrtSetField(xResult, 0, "ResolvedFunctions",
                (mxArray *)emlrtMexFcnResolvedFunctionsInfo());
  emlrtSetField(xResult, 0, "Checksum",
                emlrtMxCreateString("BgVyBzYNXmuHsNLRYrPLKE"));
  emlrtSetField(xResult, 0, "EntryPoints", xEntryPoints);
  return xResult;
}

/* End of code generation (_coder_puppet_controller_4_cg_info.c) */
