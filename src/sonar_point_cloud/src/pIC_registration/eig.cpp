//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// eig.cpp
//
// Code generation for function 'eig'
//

// Include files
#include "eig.h"
#include "eigHermitianStandard.h"
#include "eigStandard.h"
#include "pIC_registration_data.h"
#include "rt_nonfinite.h"
#include "xdlahqr.h"
#include "xzlarf.h"
#include "xzlarfg.h"
#include "xzungqr.h"
#include "coder_array.h"
#include <cmath>

// Function Definitions
namespace coder {
void eig(const array<double, 2U> &A, array<creal_T, 2U> &V,
         array<creal_T, 2U> &D)
{
  array<double, 2U> Q;
  array<double, 2U> a;
  array<double, 1U> tau;
  array<double, 1U> work;
  double alpha1;
  int n;
  n = A.size(0);
  V.set_size(A.size(0), A.size(0));
  D.set_size(A.size(0), A.size(0));
  if ((A.size(0) != 0) && (A.size(1) != 0)) {
    int iajm1;
    int ntau;
    bool p;
    ntau = A.size(0) * A.size(1);
    p = true;
    for (iajm1 = 0; iajm1 < ntau; iajm1++) {
      if ((!p) || (std::isinf(A[iajm1]) || std::isnan(A[iajm1]))) {
        p = false;
      }
    }
    if (!p) {
      V.set_size(A.size(0), A.size(0));
      ntau = A.size(0) * A.size(0);
      for (int i{0}; i < ntau; i++) {
        V[i].re = rtNaN;
        V[i].im = 0.0;
      }
      D.set_size(A.size(0), A.size(0));
      for (int i{0}; i < ntau; i++) {
        D[i].re = 0.0;
        D[i].im = 0.0;
      }
      for (iajm1 = 0; iajm1 < n; iajm1++) {
        D[iajm1 + D.size(0) * iajm1].re = rtNaN;
        D[iajm1 + D.size(0) * iajm1].im = 0.0;
      }
    } else {
      int b_i;
      int exitg1;
      int in;
      bool exitg2;
      p = (A.size(0) == A.size(1));
      if (p) {
        in = 0;
        exitg2 = false;
        while ((!exitg2) && (in <= A.size(1) - 1)) {
          b_i = 0;
          do {
            exitg1 = 0;
            if (b_i <= in) {
              if (!(A[b_i + A.size(0) * in] == A[in + A.size(0) * b_i])) {
                p = false;
                exitg1 = 1;
              } else {
                b_i++;
              }
            } else {
              in++;
              exitg1 = 2;
            }
          } while (exitg1 == 0);
          if (exitg1 == 1) {
            exitg2 = true;
          }
        }
      }
      if (p) {
        eigHermitianStandard(A, V, D);
      } else {
        p = (A.size(0) == A.size(1));
        if (p) {
          in = 0;
          exitg2 = false;
          while ((!exitg2) && (in <= A.size(1) - 1)) {
            b_i = 0;
            do {
              exitg1 = 0;
              if (b_i <= in) {
                if (!(A[b_i + A.size(0) * in] == -A[in + A.size(0) * b_i])) {
                  p = false;
                  exitg1 = 1;
                } else {
                  b_i++;
                }
              } else {
                in++;
                exitg1 = 2;
              }
            } while (exitg1 == 0);
            if (exitg1 == 1) {
              exitg2 = true;
            }
          }
        }
        if (p) {
          double ai;
          int b_n;
          int i;
          n = A.size(0);
          a.set_size(A.size(0), A.size(1));
          for (i = 0; i < ntau; i++) {
            a[i] = A[i];
          }
          b_n = A.size(0);
          ntau = A.size(0) - 1;
          tau.set_size(A.size(0) - 1);
          if (A.size(0) > 1) {
            for (b_i = n; b_i <= ntau; b_i++) {
              tau[b_i - 1] = 0.0;
            }
            work.set_size(A.size(0));
            ntau = A.size(0);
            for (i = 0; i < ntau; i++) {
              work[i] = 0.0;
            }
            i = A.size(0);
            for (b_i = 0; b_i <= i - 2; b_i++) {
              int i1;
              ntau = b_i * b_n;
              in = (b_i + 1) * b_n;
              alpha1 = a[(b_i + a.size(0) * b_i) + 1];
              i1 = (n - b_i) - 1;
              iajm1 = b_i + 3;
              if (iajm1 > b_n) {
                iajm1 = b_n;
              }
              ai = internal::reflapack::xzlarfg(i1, &alpha1, a, iajm1 + ntau);
              tau[b_i] = ai;
              a[(b_i + a.size(0) * b_i) + 1] = 1.0;
              ntau = (b_i + ntau) + 2;
              internal::reflapack::b_xzlarf(n, i1, ntau, ai, a, in + 1, b_n,
                                            work);
              internal::reflapack::xzlarf(i1, i1, ntau, ai, a, (b_i + in) + 2,
                                          b_n, work);
              a[(b_i + a.size(0) * b_i) + 1] = alpha1;
            }
          }
          Q.set_size(a.size(0), a.size(1));
          ntau = a.size(0) * a.size(1);
          for (i = 0; i < ntau; i++) {
            Q[i] = a[i];
          }
          for (in = n; in >= 2; in--) {
            ntau = (in - 1) * n - 1;
            for (b_i = 0; b_i <= in - 2; b_i++) {
              Q[(ntau + b_i) + 1] = 0.0;
            }
            iajm1 = ntau - n;
            i = in + 1;
            for (b_i = i; b_i <= n; b_i++) {
              Q[ntau + b_i] = Q[iajm1 + b_i];
            }
            i = n + 1;
            for (b_i = i; b_i <= n; b_i++) {
              Q[ntau + b_i] = 0.0;
            }
          }
          for (b_i = 0; b_i < n; b_i++) {
            Q[b_i] = 0.0;
          }
          Q[0] = 1.0;
          i = A.size(0) + 1;
          for (in = i; in <= n; in++) {
            ntau = (in - 1) * n;
            for (b_i = 0; b_i < n; b_i++) {
              Q[ntau + b_i] = 0.0;
            }
            Q[(ntau + in) - 1] = 1.0;
          }
          internal::reflapack::xzungqr(A.size(0) - 1, A.size(0) - 1,
                                       A.size(0) - 1, Q, A.size(0) + 2,
                                       A.size(0), tau);
          iajm1 = internal::reflapack::xdlahqr(A.size(0), a, A.size(0), Q, tau,
                                               work);
          D.set_size(A.size(0), A.size(0));
          ntau = A.size(0) * A.size(0);
          for (i = 0; i < ntau; i++) {
            D[i].re = 0.0;
            D[i].im = 0.0;
          }
          for (b_i = 0; b_i < iajm1; b_i++) {
            D[b_i + D.size(0) * b_i].re = rtNaN;
            D[b_i + D.size(0) * b_i].im = 0.0;
          }
          i = iajm1 + 1;
          for (b_i = i; b_i <= n; b_i++) {
            D[(b_i + D.size(0) * (b_i - 1)) - 1].re = 0.0;
            D[(b_i + D.size(0) * (b_i - 1)) - 1].im = work[b_i - 1];
          }
          if (iajm1 == 0) {
            V.set_size(Q.size(0), Q.size(1));
            ntau = Q.size(0) * Q.size(1);
            for (i = 0; i < ntau; i++) {
              V[i].re = Q[i];
              V[i].im = 0.0;
            }
            in = 1;
            n = a.size(0);
            do {
              exitg1 = 0;
              if (in <= n) {
                if (in != n) {
                  ai = a[in + a.size(0) * (in - 1)];
                  if (ai != 0.0) {
                    if (ai < 0.0) {
                      ntau = 1;
                    } else {
                      ntau = -1;
                    }
                    for (b_i = 0; b_i < n; b_i++) {
                      alpha1 = V[b_i + V.size(0) * (in - 1)].re;
                      ai = static_cast<double>(ntau) *
                           V[b_i + V.size(0) * in].re;
                      if (ai == 0.0) {
                        V[b_i + V.size(0) * (in - 1)].re =
                            alpha1 / 1.4142135623730951;
                        V[b_i + V.size(0) * (in - 1)].im = 0.0;
                      } else if (alpha1 == 0.0) {
                        V[b_i + V.size(0) * (in - 1)].re = 0.0;
                        V[b_i + V.size(0) * (in - 1)].im =
                            ai / 1.4142135623730951;
                      } else {
                        V[b_i + V.size(0) * (in - 1)].re =
                            alpha1 / 1.4142135623730951;
                        V[b_i + V.size(0) * (in - 1)].im =
                            ai / 1.4142135623730951;
                      }
                      V[b_i + V.size(0) * in].re =
                          V[b_i + V.size(0) * (in - 1)].re;
                      V[b_i + V.size(0) * in].im =
                          -V[b_i + V.size(0) * (in - 1)].im;
                    }
                    in += 2;
                  } else {
                    in++;
                  }
                } else {
                  in++;
                }
              } else {
                exitg1 = 1;
              }
            } while (exitg1 == 0);
          } else {
            V.set_size(a.size(0), a.size(1));
            ntau = a.size(0) * a.size(1);
            for (i = 0; i < ntau; i++) {
              V[i].re = rtNaN;
              V[i].im = 0.0;
            }
          }
        } else {
          eigStandard(A, V, D);
        }
      }
    }
  }
}

} // namespace coder

// End of code generation (eig.cpp)
