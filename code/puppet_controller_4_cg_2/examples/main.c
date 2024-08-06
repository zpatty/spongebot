/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * main.c
 *
 * Code generation for function 'main'
 *
 */

/*************************************************************************/
/* This automatically generated example C main file shows how to call    */
/* entry-point functions that MATLAB Coder generated. You must customize */
/* this file for your application. Do not modify this file directly.     */
/* Instead, make a copy of this file, modify it, and integrate it into   */
/* your development environment.                                         */
/*                                                                       */
/* This file initializes entry-point function arguments to a default     */
/* size and value before calling the entry-point functions. It does      */
/* not store or use any values returned from the entry-point functions.  */
/* If necessary, it does pre-allocate memory for returned values.        */
/* You can use this file as a starting point for a main function that    */
/* you can deploy in your application.                                   */
/*                                                                       */
/* After you copy the file, and before you deploy it, you must make the  */
/* following changes:                                                    */
/* * For variable-size function arguments, change the example sizes to   */
/* the sizes that your application requires.                             */
/* * Change the example values of function arguments to the values that  */
/* your application requires.                                            */
/* * If the entry-point functions return values, store these values or   */
/* otherwise use them as required by your application.                   */
/*                                                                       */
/*************************************************************************/

/* Include files */
#include "main.h"
#include "puppet_controller_4_cg.h"
#include "puppet_controller_4_cg_terminate.h"
#include "rt_nonfinite.h"

/* Function Declarations */
static void argInit_16x16_real_T(double result[256]);

static void argInit_16x1_real_T(double result[16]);

static double argInit_real_T(void);

/* Function Definitions */
static void argInit_16x16_real_T(double result[256])
{
  int i;
  /* Loop over the array to initialize each element. */
  for (i = 0; i < 256; i++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[i] = argInit_real_T();
  }
}

static void argInit_16x1_real_T(double result[16])
{
  int idx0;
  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 16; idx0++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[idx0] = argInit_real_T();
  }
}

static double argInit_real_T(void)
{
  return 0.0;
}

int main(int argc, char **argv)
{
  (void)argc;
  (void)argv;
  /* The initialize function is being called automatically from your entry-point
   * function. So, a call to initialize is not included here. */
  /* Invoke the entry-point functions.
You can call entry-point functions multiple times. */
  main_puppet_controller_4_cg();
  /* Terminate the application.
You do not need to do this more than one time. */
  puppet_controller_4_cg_terminate();
  return 0;
}

void main_puppet_controller_4_cg(void)
{
  double Kp_tmp[256];
  double q_tmp[16];
  double tau[16];
  double cvec[4];
  double d_tmp;
  /* Initialize function 'puppet_controller_4_cg' input arguments. */
  /* Initialize function input argument 'q'. */
  argInit_16x1_real_T(q_tmp);
  /* Initialize function input argument 'dq'. */
  /* Initialize function input argument 'qd'. */
  /* Initialize function input argument 'dqd'. */
  /* Initialize function input argument 'ddqd'. */
  d_tmp = argInit_real_T();
  /* Initialize function input argument 'Kp'. */
  argInit_16x16_real_T(Kp_tmp);
  /* Initialize function input argument 'KD'. */
  /* Call the entry-point 'puppet_controller_4_cg'. */
  puppet_controller_4_cg(q_tmp, q_tmp, q_tmp, q_tmp, q_tmp, d_tmp, d_tmp, d_tmp,
                         d_tmp, d_tmp, d_tmp, d_tmp, d_tmp, d_tmp, d_tmp, d_tmp,
                         Kp_tmp, Kp_tmp, d_tmp, d_tmp, d_tmp, d_tmp, d_tmp,
                         d_tmp, tau, cvec);
}

/* End of code generation (main.c) */
