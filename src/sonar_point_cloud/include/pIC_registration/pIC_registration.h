//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// pIC_registration.h
//
// Code generation for function 'pIC_registration'
//

#ifndef PIC_REGISTRATION_H
#define PIC_REGISTRATION_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
extern void pIC_registration(const coder::array<double, 2U> &sourcePC,
                             const coder::array<double, 2U> &targetPC,
                             double sonar_noise,
                             const double initial_estimate[3],
                             const double initial_estimate_cov[3],
                             double source_size, double target_size,
                             double *out, double q[3], double *num_it,
                             double registration_uncertainty[9]);

#endif
// End of code generation (pIC_registration.h)
