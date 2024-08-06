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
      "789ced594f73d240140f0e5a6754e4a2570f3d3a364e0b564f9d06103a165aa0e0bf71c2"
      "926c4920c9ae49a0b407be8207fd001efd081efd007e00ef5efc029e"
      "9c712c4d0249667640c26c29ecbb3cdefcc2fedebec7fee6b1e1627bc518c77109ceb11f"
      "0f1d7fc78d93aebfc6052d8cc75c7f33147b769d8b07bee7e11f5c2f",
      "21c3867ddb090ca0c3d13765a4ab0630eca3530c39135a48eb41f90239563578a4eab0ea"
      "0f4ac3487fee8346c1101a7ece2850ea54bb3a672ad63843cd1f8cea"
      "f195b0dff894f52813ea910ce16f73ef7805e9903f0392c26791d4d5a1615b7c4bb51f99"
      "10238bd781ad81269fadecf3b86b29b88b313c7fc0f1e2b07e26d234",
      "688a29516a6de8a17d3422ee638db80f0729662e78e7c67783c8e72032ea363538dedf97"
      "887c02912f88cfdc27b7405e6326d5e9ee947987fdf879e724e63b9f"
      "649a7c3f1b67eb34f93cbb2cbe3e61bd697f77f7087cc9108e527ddc4ec98220a4b70ae9"
      "57a05e3f387dedcbe37002cfa43c38424c6bfd653bbfd3e61d0fc5e3",
      "bc1dc4eac09379f2fdafae7e8ec8b743e40be233f76558a0f3a6d03affbf93dfa5a15f15"
      "7da3cd17554fef13f892213cdddacc3d3641e9b8d6b4b2f8c5f66e4f"
      "2be505a6a7575d4f49ff3f922e02e4b628014dbaaa736a86c817c467ee8b5720b731b474"
      "6020fea5aaabfb83c41a4d3ecf965d57f54a06568aa582fae67d6ed3",
      "6897ebf5a25ccf335d5dd4f3cbe654c7d89c1a8dcfb355e16373ea7cd6bfaa732a8e9877"
      "f81e3c9cb7875b18d82ad044c94496c55d9ebe46ed5381c817c467d7"
      "577fa136745aba505dbf45f57ef5e9af3fec7e959bbfce2af86017f5b050d6b76ba5ad67"
      "e916aebc3c613abbb0e797cdad8eb1b9351a9f67abc2c7e6d6f9acbf",
      "aa73ebed09797bf8612623b681849a2a30fcfc8d88fcb4e7d63c912f88cfdc277fa128ea"
      "6c62f091eadc3a687d7b4093cfb365d75929853bed3d43e97491200b"
      "66bef6a45f2f1498ce2eeaf965efb11c5bb47b70f61e6b3a3ecf965d5797f53dd63ffb3c"
      "87af",
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
                emlrtMxCreateDoubleScalar(2.0));
  emlrtSetField(xEntryPoints, 0, "ConstantInputs", xInputs);
  emlrtSetField(xEntryPoints, 0, "FullPath",
                emlrtMxCreateString("/home/zach/Documents/git-repos/matlab/DRL/"
                                    "pushpuppets/puppet_controller_4_cg.m"));
  emlrtSetField(xEntryPoints, 0, "TimeStamp",
                emlrtMxCreateDoubleScalar(739065.43212962965));
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
