//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// xdlahqr.h
//
// Code generation for function 'xdlahqr'
//

#ifndef XDLAHQR_H
#define XDLAHQR_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace internal {
namespace reflapack {
int xdlahqr(int ilo, int ihi, array<double, 2U> &h, int iloz, int ihiz,
            array<double, 2U> &z, array<double, 1U> &wr, array<double, 1U> &wi);

int xdlahqr(int ihi, array<double, 2U> &h, int ihiz, array<double, 2U> &z,
            array<double, 1U> &wr, array<double, 1U> &wi);

} // namespace reflapack
} // namespace internal
} // namespace coder

#endif
// End of code generation (xdlahqr.h)
