//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// inv.h
//
// Code generation for function 'inv'
//

#ifndef INV_H
#define INV_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
void inv(const double x[9], double y[9]);

void inv(const array<double, 2U> &x, array<double, 2U> &y);

void inv(const creal_T x[9], creal_T y[9]);

} // namespace coder

#endif
// End of code generation (inv.h)
