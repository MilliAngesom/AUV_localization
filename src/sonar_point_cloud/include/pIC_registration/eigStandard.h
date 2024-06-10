//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// eigStandard.h
//
// Code generation for function 'eigStandard'
//

#ifndef EIGSTANDARD_H
#define EIGSTANDARD_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
void eigStandard(const array<double, 2U> &A, array<creal_T, 2U> &V,
                 array<creal_T, 2U> &D);

}

#endif
// End of code generation (eigStandard.h)
