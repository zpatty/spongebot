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
      "789ced594f73d240145f14ff1cfcc3453f416f9d16a705517bd086223015dad212ab8e93"
      "2ec9160249764d02a59fc26b8f5cbcaa57bf8e1fc09b5747e9269064"
      "6607d2600accbecbe3f163f7f7f21efb9b970424ca950400e001a076bc4afd7d274e39fe"
      "06f05b104f38fe6e2076ed1648fad6b9f867c7cbd8b051dfa6810175",
      "345aa9605d35a0611f9d13044c6461ad87944be454d5d091aaa3436f501d46fa6b0f340a"
      "86d0f073be85e4ce61570766cb1a67a87983513dbe33ae3739653dca"
      "8c7aa402f887c2c7151dda1a6c9818db2be9a66aaf9988602b4dbf4defd4dea449d76a91"
      "2e21c8b6d2d44bc3ba9958d390296524b9b9ae07f23f8998ff1d66fe",
      "14a9e42f7967c6779bc9471105771b1a1a5fdf2022df0b269f1f0fdd1fa7306e4326d5e7"
      "e194f906fdf8f7f4e4e57eeca238f9befefe72334e3ed7ae8bafcfd8"
      "6fdaffdb23065f2a80e34c9fb4338a2008d9cd52f6188ae2def93b4f1efb137826e50118"
      "715cfb0f18eb17eddc4e9b6f32108ff3a588d54167b3e40baba31711",
      "f9724c3e3f1eba1fc3c2fc6b465ce7fd59ccfaf9eba2918b93cfb545d5cfc70cbe5400cf"
      "36370a4f4c583dad37ac1db29bdbee69d5a2c0f57351f593755f9172"
      "10a8b425196af2a2cea15b4c3e3f1eba1f6e619c862ceb1cfaedcf4f3e8782d9eba85ecb"
      "a35aa55a52df7f2a6c18ed0351ac286291ebe8bc9d5b3e8752e373e8",
      "d5f8f81c4a8dcfa1e1f61f30d6cfeb1c4a22e61b7c7e1dccd7c52d026d156a926c62cb02"
      "d7a7a783887caf987c7e3cbc9e7a0bb4ae735d9d0d9f6bcbaeab2db2"
      "b78d7b4438d073f5eae6f36c93d4de9e715d9dbb73cbe7526a7c2ebd1a1fd74f6a7c2e0d"
      "b7ff80b17e59e7d27b13f275f1fd7c5e6a431937546878f94f22f2c7",
      "3d97be64f2f9f1d0fdf11628465de5cf4bff2f5f5cba2a6748a75d365a9d2e1614c12cd6"
      "9ff6c55289ebeabc9d5bfede89dabc3cbfe63a3a1d9f6bcbaea3cbfa"
      "dee92fb6f45d56",
      ""};
  nameCaptureInfo = NULL;
  emlrtNameCaptureMxArrayR2016a(&data[0], 11120U, &nameCaptureInfo);
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
                emlrtMxCreateDoubleScalar(2.0));
  emlrtSetField(xEntryPoints, 0, "ConstantInputs", xInputs);
  emlrtSetField(xEntryPoints, 0, "FullPath",
                emlrtMxCreateString("/home/zach/git-repos/matlab/DRL/"
                                    "pushpuppets/puppet_controller_4_cg.m"));
  emlrtSetField(xEntryPoints, 0, "TimeStamp",
                emlrtMxCreateDoubleScalar(739207.67298611114));
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
