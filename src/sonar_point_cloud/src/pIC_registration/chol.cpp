//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// chol.cpp
//
// Code generation for function 'chol'
//

// Include files
#include "chol.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <cmath>

// Function Definitions
namespace coder {
namespace internal {
int chol(array<double, 2U> &A, int &jmax)
{
  int flag;
  int idxAjj;
  int mrows;
  int n;
  mrows = A.size(0);
  idxAjj = A.size(0);
  n = A.size(1);
  if (idxAjj <= n) {
    n = idxAjj;
  }
  jmax = 0;
  flag = 0;
  if (n != 0) {
    int i;
    int info;
    int j;
    bool exitg1;
    info = -1;
    j = 0;
    exitg1 = false;
    while ((!exitg1) && (j <= n - 1)) {
      double c;
      double ssq;
      int idxA1j;
      int k;
      idxA1j = j * mrows;
      idxAjj = idxA1j + j;
      ssq = 0.0;
      if (j >= 1) {
        for (k = 0; k < j; k++) {
          c = A[idxA1j + k];
          ssq += c * c;
        }
      }
      ssq = A[idxAjj] - ssq;
      if (ssq > 0.0) {
        ssq = std::sqrt(ssq);
        A[idxAjj] = ssq;
        if (j + 1 < n) {
          int ia0;
          int idxAjjp1;
          int nmj;
          nmj = (n - j) - 2;
          ia0 = (idxA1j + mrows) + 1;
          idxAjjp1 = idxAjj + mrows;
          if ((j != 0) && (nmj + 1 != 0)) {
            idxAjj = idxAjjp1;
            i = ia0 + mrows * nmj;
            for (int iac{ia0}; mrows < 0 ? iac >= i : iac <= i; iac += mrows) {
              c = 0.0;
              k = (iac + j) - 1;
              for (int ia{iac}; ia <= k; ia++) {
                c += A[ia - 1] * A[(idxA1j + ia) - iac];
              }
              A[idxAjj] = A[idxAjj] - c;
              idxAjj += mrows;
            }
          }
          ssq = 1.0 / ssq;
          if (mrows >= 1) {
            i = (idxAjjp1 + mrows * nmj) + 1;
            for (k = idxAjjp1 + 1; mrows < 0 ? k >= i : k <= i; k += mrows) {
              A[k - 1] = ssq * A[k - 1];
            }
          }
        }
        j++;
      } else {
        A[idxAjj] = ssq;
        info = j;
        exitg1 = true;
      }
    }
    flag = info + 1;
    if (info + 1 == 0) {
      jmax = n;
    } else {
      jmax = info;
    }
    for (j = 0; j <= jmax - 2; j++) {
      i = j + 2;
      for (idxAjj = i; idxAjj <= jmax; idxAjj++) {
        A[(idxAjj + A.size(0) * j) - 1] = 0.0;
      }
    }
  }
  return flag;
}

} // namespace internal
} // namespace coder

// End of code generation (chol.cpp)
