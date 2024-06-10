//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// pIC_cov_weight_T_test.h
//
// Code generation for function 'pIC_cov_weight_T_test'
//

#ifndef PIC_COV_WEIGHT_T_TEST_H
#define PIC_COV_WEIGHT_T_TEST_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
void pIC_cov_weight_T_test(const double theta[3],
                           const coder::array<double, 2U> &Sr,
                           const coder::array<double, 2U> &assoc_n,
                           const coder::array<double, 3U> &assoc_Pn,
                           const coder::array<double, 2U> &assoc_a,
                           const coder::array<double, 3U> &assoc_Pa,
                           creal_T cov[9]);

#endif
// End of code generation (pIC_cov_weight_T_test.h)
