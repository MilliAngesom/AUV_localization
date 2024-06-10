//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// MStep.cpp
//
// Code generation for function 'MStep'
//

// Include files
#include "MStep.h"
#include "rt_nonfinite.h"
#include "sum.h"
#include "coder_array.h"

// Function Definitions
void binary_expand_op(const coder::array<double, 2U> &in3,
                      const coder::array<double, 2U> &in4, double in1[2])
{
  coder::array<double, 2U> b_in3;
  coder::array<double, 2U> r;
  int aux_0_1;
  int aux_1_1;
  int loop_ub;
  int stride_0_1;
  int stride_1_1;
  if (in4.size(1) == 1) {
    loop_ub = in3.size(1);
  } else {
    loop_ub = in4.size(1);
  }
  b_in3.set_size(2, loop_ub);
  stride_0_1 = (in3.size(1) != 1);
  stride_1_1 = (in4.size(1) != 1);
  aux_0_1 = 0;
  aux_1_1 = 0;
  for (int i{0}; i < loop_ub; i++) {
    b_in3[2 * i] = in3[2 * aux_0_1] - in4[2 * aux_1_1];
    b_in3[2 * i + 1] = in3[2 * aux_0_1 + 1] - in4[2 * aux_1_1 + 1];
    aux_1_1 += stride_1_1;
    aux_0_1 += stride_0_1;
  }
  r.set_size(2, b_in3.size(1));
  loop_ub = b_in3.size(1) << 1;
  for (int i{0}; i < loop_ub; i++) {
    double varargin_1;
    varargin_1 = b_in3[i];
    r[i] = varargin_1 * varargin_1;
  }
  coder::sum(r, in1);
}

// End of code generation (MStep.cpp)
