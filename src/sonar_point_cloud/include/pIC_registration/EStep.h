//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// EStep.h
//
// Code generation for function 'EStep'
//

#ifndef ESTEP_H
#define ESTEP_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
void EStep(
    const coder::array<double, 2U> &r_cart, const coder::array<double, 3U> &r_P,
    const coder::array<double, 2U> &n_cart, const coder::array<double, 3U> &n_P,
    const double motion_q[3], const double motion_Pq[9],
    coder::array<double, 2U> &assoc_n, coder::array<double, 3U> &assoc_Pn,
    coder::array<double, 2U> &assoc_c, coder::array<double, 2U> &assoc_a,
    coder::array<double, 3U> &assoc_Pa, coder::array<double, 3U> &assoc_Pe,
    coder::array<double, 3U> &assoc_Pc, coder::array<double, 3U> &assoc_Jq,
    coder::array<double, 3U> &reg_uncertainty,
    coder::array<double, 2U> &P_r_index);

#endif
// End of code generation (EStep.h)
