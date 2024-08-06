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
  const char_T *data[7] = {
      "789ced5add8ed240142e065d93c5951b7d0037f1c6b835bbe0ea95a180405cd80516fc8b"
      "e90eed2c2db49db12d2cebc5be82afe0a52f60e233181fc047d007f0"
      "cac4b86c3bd036998094ccb2d0737338f9daf9ce9cc37c39b470b15239c671dc06e7d88f"
      "078ebfe5c649d75fe3fc16c463aebf1988895de7e2befb08fed1f512",
      "326c38b09dc0003a1cdd29235d3580611f9e62c899d0425a1fca17c8b1aac1435587756f"
      "501946fa730f340a86d0f073568152b7ded33953b1c6196ade60548f"
      "af94fdc6a7ac4795528f64007f9b7fc72b4887fc0720297c0e493d1d1ab6c5b755fba109"
      "31b2781dd81a68f1b9da1e8f7b96827b18c3f30b1c2f0eeb67224d83",
      "a69812a5f6961ed8c751c87dac51f7e120e5ec05efdcf86e50f91c4446bd9606c7fbfb1c"
      "924fa0f2f9f199fbe416883466529d6e4f9977d08faf774e6222f745"
      "66c9772ff36b9d251fb1cbe21b50d69bf67b7787c2970ce02835c09d942c08427aa7987e"
      "059acdfdd3d79e3c0e26f04cca83a3c4acd65fb6f33b6ddef1403cce",
      "db41ac2e3c9927dfffeaeaa7907ccfa87c7e7ce6be0c0b74de1456e7ff77f2bb34f4aba2"
      "6facf9c2eae95d0a5f3280a7dbdbf94726a81c375a560ebfd8cdf4b5"
      "4a4188f4f4aaeb29edf747d24580dc1125a04957754ecd52f9fcf8cc7d2105721bc34a07"
      "cec4bf4c7575ef6c638d251fb165d755bd9685b572a5a8be799fdf36",
      "3ad566b32c370b91ae2eeaf98de654c7a239351c1fb155e18be6d4f9ac7f55e7541c32ef"
      "e073f060de04b730b055a08992892c8bbb3c7d0ddba72295cf8fcfae"
      "afde426de9ac74a1beb9cef4f9ea939f7f3659f2115b769d55f07e06f5b150d5771b959d"
      "a7e936aebd3c89747661cf6f34b73a16cdade1f888ad0a5f34b7ce67",
      "fd559d5b1313f226f841362b7680845a2a30bcfc4721f959cfad052a9f1f9fb94fde4231"
      "d459d6ff0bf856ba9f60c9476cd975564ae16ea76428dd1e1264c12c"
      "341e0f9ac562a4b38b7a7ea3f7588e2dda73f0e83dd6747cc4965d5797f53dd63f0dc686"
      "4c",
      ""};
  nameCaptureInfo = NULL;
  emlrtNameCaptureMxArrayR2016a(&data[0], 11248U, &nameCaptureInfo);
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
  xInputs = emlrtCreateLogicalMatrix(1, 24);
  emlrtSetField(xEntryPoints, 0, "Name",
                emlrtMxCreateString("puppet_controller_4_cg"));
  emlrtSetField(xEntryPoints, 0, "NumberOfInputs",
                emlrtMxCreateDoubleScalar(24.0));
  emlrtSetField(xEntryPoints, 0, "NumberOfOutputs",
                emlrtMxCreateDoubleScalar(1.0));
  emlrtSetField(xEntryPoints, 0, "ConstantInputs", xInputs);
  emlrtSetField(xEntryPoints, 0, "FullPath",
                emlrtMxCreateString("/home/zach/Documents/git-repos/matlab/DRL/"
                                    "pushpuppets/puppet_controller_4_cg.m"));
  emlrtSetField(xEntryPoints, 0, "TimeStamp",
                emlrtMxCreateDoubleScalar(739103.96042824071));
  xResult =
      emlrtCreateStructMatrix(1, 1, 7, (const char_T **)&propFieldName[0]);
  emlrtSetField(xResult, 0, "Version",
                emlrtMxCreateString("9.14.0.2239454 (R2023a) Update 1"));
  emlrtSetField(xResult, 0, "ResolvedFunctions",
                (mxArray *)emlrtMexFcnResolvedFunctionsInfo());
  emlrtSetField(xResult, 0, "Checksum",
                emlrtMxCreateString("Q18i2em6QLBaWYtFu0geLD"));
  emlrtSetField(xResult, 0, "EntryPoints", xEntryPoints);
  return xResult;
}

/* End of code generation (_coder_puppet_controller_4_cg_info.c) */
