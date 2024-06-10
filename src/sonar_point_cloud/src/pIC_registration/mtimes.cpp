//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// mtimes.cpp
//
// Code generation for function 'mtimes'
//

// Include files
#include "mtimes.h"
#include "rt_nonfinite.h"
#include "coder_array.h"

// Function Definitions
namespace coder {
namespace internal {
namespace blas {
void mtimes(const array<creal_T, 2U> &A, const array<creal_T, 2U> &B,
            array<creal_T, 2U> &C)
{
  int inner;
  int m;
  int n;
  m = A.size(0);
  inner = A.size(1);
  n = B.size(0);
  C.set_size(A.size(0), B.size(0));
  for (int j{0}; j < n; j++) {
    int coffset;
    coffset = j * m;
    for (int i{0}; i < m; i++) {
      double s_im;
      double s_re;
      int k;
      s_re = 0.0;
      s_im = 0.0;
      for (k = 0; k < inner; k++) {
        double A_re_tmp;
        double B_im;
        double B_re;
        double b_A_re_tmp;
        B_re = B[k * B.size(0) + j].re;
        B_im = -B[k * B.size(0) + j].im;
        A_re_tmp = A[k * A.size(0) + i].re;
        b_A_re_tmp = A[k * A.size(0) + i].im;
        s_re += A_re_tmp * B_re - b_A_re_tmp * B_im;
        s_im += A_re_tmp * B_im + b_A_re_tmp * B_re;
      }
      k = coffset + i;
      C[k].re = s_re;
      C[k].im = s_im;
    }
  }
}

void mtimes(const array<double, 2U> &A, const array<double, 2U> &B,
            array<double, 2U> &C)
{
  int inner;
  int nc;
  inner = A.size(0);
  nc = B.size(1);
  C.set_size(3, B.size(1));
  for (int j{0}; j < nc; j++) {
    int boffset;
    int coffset;
    coffset = j * 3;
    boffset = j * B.size(0);
    C[coffset] = 0.0;
    C[coffset + 1] = 0.0;
    C[coffset + 2] = 0.0;
    for (int k{0}; k < inner; k++) {
      double bkj;
      bkj = B[boffset + k];
      C[coffset] = C[coffset] + A[k] * bkj;
      C[coffset + 1] = C[coffset + 1] + A[A.size(0) + k] * bkj;
      C[coffset + 2] = C[coffset + 2] + A[(A.size(0) << 1) + k] * bkj;
    }
  }
}

void mtimes(const creal_T A[9], const array<creal_T, 2U> &B,
            array<creal_T, 2U> &C)
{
  int n;
  n = B.size(1);
  C.set_size(3, B.size(1));
  for (int j{0}; j < n; j++) {
    double A_re_tmp;
    double b_A_re_tmp;
    double c_A_re_tmp;
    double d_A_re_tmp;
    double e_A_re_tmp;
    double f_A_re_tmp;
    int coffset_tmp;
    coffset_tmp = j * 3;
    A_re_tmp = B[coffset_tmp].im;
    b_A_re_tmp = B[coffset_tmp].re;
    c_A_re_tmp = B[coffset_tmp + 1].im;
    d_A_re_tmp = B[coffset_tmp + 1].re;
    e_A_re_tmp = B[coffset_tmp + 2].im;
    f_A_re_tmp = B[coffset_tmp + 2].re;
    for (int i{0}; i < 3; i++) {
      double g_A_re_tmp;
      double h_A_re_tmp;
      double i_A_re_tmp;
      double j_A_re_tmp;
      double k_A_re_tmp;
      double l_A_re_tmp;
      int b_i;
      g_A_re_tmp = A[i].re;
      h_A_re_tmp = A[i].im;
      i_A_re_tmp = A[i + 3].re;
      j_A_re_tmp = A[i + 3].im;
      k_A_re_tmp = A[i + 6].re;
      l_A_re_tmp = A[i + 6].im;
      b_i = coffset_tmp + i;
      C[b_i].re = ((g_A_re_tmp * b_A_re_tmp - h_A_re_tmp * A_re_tmp) +
                   (i_A_re_tmp * d_A_re_tmp - j_A_re_tmp * c_A_re_tmp)) +
                  (k_A_re_tmp * f_A_re_tmp - l_A_re_tmp * e_A_re_tmp);
      C[b_i].im = ((g_A_re_tmp * A_re_tmp + h_A_re_tmp * b_A_re_tmp) +
                   (i_A_re_tmp * c_A_re_tmp + j_A_re_tmp * d_A_re_tmp)) +
                  (k_A_re_tmp * e_A_re_tmp + l_A_re_tmp * f_A_re_tmp);
    }
  }
}

} // namespace blas
} // namespace internal
} // namespace coder

// End of code generation (mtimes.cpp)
