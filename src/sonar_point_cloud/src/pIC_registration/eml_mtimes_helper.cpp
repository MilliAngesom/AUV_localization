//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// eml_mtimes_helper.cpp
//
// Code generation for function 'eml_mtimes_helper'
//

// Include files
#include "eml_mtimes_helper.h"
#include "rt_nonfinite.h"
#include "coder_array.h"

// Function Definitions
void binary_expand_op_4(coder::array<creal_T, 1U> &in1,
                        const coder::array<creal_T, 2U> &in2,
                        const coder::array<double, 2U> &in3,
                        const coder::array<double, 2U> &in4)
{
  coder::array<creal_T, 1U> c_in3;
  coder::array<double, 2U> b_in3;
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
  stride_0_1 = in3.size(1) << 1;
  c_in3.set_size(stride_0_1);
  for (int i{0}; i < stride_0_1; i++) {
    c_in3[i].re = b_in3[i];
    c_in3[i].im = 0.0;
  }
  in1.set_size(in2.size(0));
  loop_ub = in2.size(0);
  for (int i{0}; i < loop_ub; i++) {
    in1[i].re = 0.0;
    in1[i].im = 0.0;
    stride_0_1 = in2.size(1);
    for (stride_1_1 = 0; stride_1_1 < stride_0_1; stride_1_1++) {
      double b_in2_re_tmp;
      double c_in2_re_tmp;
      double d_in2_re_tmp;
      double in2_re_tmp;
      in2_re_tmp = in2[i + in2.size(0) * stride_1_1].re;
      b_in2_re_tmp = c_in3[stride_1_1].im;
      c_in2_re_tmp = in2[i + in2.size(0) * stride_1_1].im;
      d_in2_re_tmp = c_in3[stride_1_1].re;
      in1[i].re =
          in1[i].re + (in2_re_tmp * d_in2_re_tmp - c_in2_re_tmp * b_in2_re_tmp);
      in1[i].im =
          in1[i].im + (in2_re_tmp * b_in2_re_tmp + c_in2_re_tmp * d_in2_re_tmp);
    }
  }
}

// End of code generation (eml_mtimes_helper.cpp)
