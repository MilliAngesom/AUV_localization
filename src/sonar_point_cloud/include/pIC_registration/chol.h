//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// chol.h
//
// Code generation for function 'chol'
//

#ifndef CHOL_H
#define CHOL_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace internal {
int chol(array<double, 2U> &A, int &jmax);

}
} // namespace coder

#endif
// End of code generation (chol.h)
