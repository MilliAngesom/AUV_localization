//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// eigStandard.cpp
//
// Code generation for function 'eigStandard'
//

// Include files
#include "eigStandard.h"
#include "pIC_registration_rtwutil.h"
#include "rt_nonfinite.h"
#include "xdlahqr.h"
#include "xdtrevc3.h"
#include "xnrm2.h"
#include "xzgebal.h"
#include "xzlarf.h"
#include "xzlarfg.h"
#include "xzlascl.h"
#include "xzunghr.h"
#include "coder_array.h"
#include <cmath>

// Function Definitions
namespace coder {
void eigStandard(const array<double, 2U> &A, array<creal_T, 2U> &V,
                 array<creal_T, 2U> &D)
{
  array<creal_T, 1U> W;
  array<double, 2U> b_A;
  array<double, 2U> vr;
  array<double, 1U> scale;
  array<double, 1U> work;
  array<double, 1U> wr;
  double absxk;
  int b_n;
  int i;
  int ihi;
  int k;
  int n;
  int ntau;
  n = A.size(0);
  b_A.set_size(A.size(0), A.size(1));
  ntau = A.size(0) * A.size(1);
  for (i = 0; i < ntau; i++) {
    b_A[i] = A[i];
  }
  b_n = A.size(0);
  W.set_size(A.size(0));
  V.set_size(A.size(0), A.size(0));
  if (A.size(0) != 0) {
    double anrm;
    bool scalea;
    anrm = 0.0;
    scalea = (A.size(1) == 0);
    if (!scalea) {
      bool exitg1;
      k = 0;
      exitg1 = false;
      while ((!exitg1) && (k <= ntau - 1)) {
        absxk = std::abs(A[k]);
        if (std::isnan(absxk)) {
          anrm = rtNaN;
          exitg1 = true;
        } else {
          if (absxk > anrm) {
            anrm = absxk;
          }
          k++;
        }
      }
    }
    if (std::isinf(anrm) || std::isnan(anrm)) {
      W.set_size(A.size(0));
      ntau = A.size(0);
      for (i = 0; i < ntau; i++) {
        W[i].re = rtNaN;
        W[i].im = 0.0;
      }
      V.set_size(A.size(0), A.size(0));
      ntau = A.size(0) * A.size(0);
      for (i = 0; i < ntau; i++) {
        V[i].re = rtNaN;
        V[i].im = 0.0;
      }
    } else {
      double cscale;
      double f1;
      int c_n;
      int ilo;
      int in;
      int info;
      int scl_tmp;
      int u0;
      cscale = anrm;
      scalea = false;
      if ((anrm > 0.0) && (anrm < 6.7178761075670888E-139)) {
        scalea = true;
        cscale = 6.7178761075670888E-139;
        internal::reflapack::xzlascl(anrm, cscale, A.size(0), A.size(0), b_A,
                                     A.size(0));
      } else if (anrm > 1.4885657073574029E+138) {
        scalea = true;
        cscale = 1.4885657073574029E+138;
        internal::reflapack::xzlascl(anrm, cscale, A.size(0), A.size(0), b_A,
                                     A.size(0));
      }
      ilo = internal::reflapack::xzgebal(b_A, scale, ihi);
      c_n = b_A.size(0);
      if (b_A.size(0) < 1) {
        ntau = 0;
      } else {
        ntau = b_A.size(0) - 1;
      }
      wr.set_size(ntau);
      if ((ihi - ilo) + 1 > 1) {
        for (int b_i{0}; b_i <= ilo - 2; b_i++) {
          wr[b_i] = 0.0;
        }
        for (int b_i{ihi}; b_i <= ntau; b_i++) {
          wr[b_i - 1] = 0.0;
        }
        work.set_size(b_A.size(0));
        ntau = b_A.size(0);
        for (i = 0; i < ntau; i++) {
          work[i] = 0.0;
        }
        i = ihi - 1;
        for (int b_i{ilo}; b_i <= i; b_i++) {
          ntau = (b_i - 1) * c_n;
          in = b_i * c_n + 1;
          absxk = b_A[b_i + b_A.size(0) * (b_i - 1)];
          scl_tmp = ihi - b_i;
          u0 = b_i + 2;
          if (u0 > c_n) {
            u0 = c_n;
          }
          f1 = internal::reflapack::xzlarfg(scl_tmp, &absxk, b_A, u0 + ntau);
          wr[b_i - 1] = f1;
          b_A[b_i + b_A.size(0) * (b_i - 1)] = 1.0;
          ntau = (b_i + ntau) + 1;
          internal::reflapack::b_xzlarf(ihi, scl_tmp, ntau, f1, b_A, in, c_n,
                                        work);
          internal::reflapack::xzlarf(scl_tmp, c_n - b_i, ntau, f1, b_A,
                                      b_i + in, c_n, work);
          b_A[b_i + b_A.size(0) * (b_i - 1)] = absxk;
        }
      }
      vr.set_size(b_A.size(0), b_A.size(1));
      ntau = b_A.size(0) * b_A.size(1);
      for (i = 0; i < ntau; i++) {
        vr[i] = b_A[i];
      }
      internal::reflapack::xzunghr(A.size(0), ilo, ihi, vr, A.size(0), wr);
      info =
          internal::reflapack::xdlahqr(ilo, ihi, b_A, ilo, ihi, vr, wr, work);
      if (info == 0) {
        internal::reflapack::xdtrevc3(b_A, vr);
        c_n = vr.size(0);
        if ((vr.size(0) != 0) && (vr.size(1) != 0)) {
          if (ilo != ihi) {
            for (int b_i{ilo}; b_i <= ihi; b_i++) {
              if (c_n >= 1) {
                i = b_i + c_n * (c_n - 1);
                for (k = b_i; c_n < 0 ? k >= i : k <= i; k += c_n) {
                  vr[k - 1] = scale[b_i - 1] * vr[k - 1];
                }
              }
            }
          }
          i = ilo - 1;
          for (int b_i{i}; b_i >= 1; b_i--) {
            f1 = scale[b_i - 1];
            if (static_cast<int>(f1) != b_i) {
              for (k = 0; k < c_n; k++) {
                ntau = k * c_n;
                u0 = (b_i + ntau) - 1;
                absxk = vr[u0];
                scl_tmp = (static_cast<int>(f1) + ntau) - 1;
                vr[u0] = vr[scl_tmp];
                vr[scl_tmp] = absxk;
              }
            }
          }
          i = ihi + 1;
          for (int b_i{i}; b_i <= c_n; b_i++) {
            f1 = scale[b_i - 1];
            if (static_cast<int>(f1) != b_i) {
              for (k = 0; k < c_n; k++) {
                ntau = k * c_n;
                u0 = (b_i + ntau) - 1;
                absxk = vr[u0];
                scl_tmp = (static_cast<int>(f1) + ntau) - 1;
                vr[u0] = vr[scl_tmp];
                vr[scl_tmp] = absxk;
              }
            }
          }
        }
        for (int b_i{0}; b_i < b_n; b_i++) {
          if (!(work[b_i] < 0.0)) {
            if ((b_i + 1 != b_n) && (work[b_i] > 0.0)) {
              double c;
              double f;
              double g;
              double s;
              in = b_i * b_n;
              scl_tmp = (b_i + 1) * b_n;
              absxk = 1.0 / rt_hypotd_snf(
                                internal::blas::xnrm2(b_n, vr, in + 1),
                                internal::blas::xnrm2(b_n, vr, scl_tmp + 1));
              i = in + b_n;
              for (k = in + 1; k <= i; k++) {
                vr[k - 1] = absxk * vr[k - 1];
              }
              i = scl_tmp + b_n;
              for (k = scl_tmp + 1; k <= i; k++) {
                vr[k - 1] = absxk * vr[k - 1];
              }
              for (ntau = 0; ntau < b_n; ntau++) {
                absxk = vr[ntau + vr.size(0) * b_i];
                f1 = vr[ntau + vr.size(0) * (b_i + 1)];
                scale[ntau] = absxk * absxk + f1 * f1;
              }
              k = 0;
              if (b_n > 1) {
                absxk = std::abs(scale[0]);
                for (ihi = 2; ihi <= b_n; ihi++) {
                  s = std::abs(scale[ihi - 1]);
                  if (s > absxk) {
                    k = ihi - 1;
                    absxk = s;
                  }
                }
              }
              f = vr[k + vr.size(0) * b_i];
              g = vr[k + vr.size(0) * (b_i + 1)];
              f1 = std::abs(f);
              absxk = std::abs(g);
              if (g == 0.0) {
                c = 1.0;
                s = 0.0;
              } else if (f == 0.0) {
                c = 0.0;
                if (g >= 0.0) {
                  s = 1.0;
                } else {
                  s = -1.0;
                }
              } else if ((f1 > 1.4916681462400413E-154) &&
                         (f1 < 4.7403759540545887E+153) &&
                         (absxk > 1.4916681462400413E-154) &&
                         (absxk < 4.7403759540545887E+153)) {
                double d;
                d = std::sqrt(f * f + g * g);
                c = f1 / d;
                if (!(f >= 0.0)) {
                  d = -d;
                }
                s = g / d;
              } else {
                double d;
                absxk = std::fmin(
                    4.49423283715579E+307,
                    std::fmax(2.2250738585072014E-308, std::fmax(f1, absxk)));
                s = f / absxk;
                absxk = g / absxk;
                d = std::sqrt(s * s + absxk * absxk);
                c = std::abs(s) / d;
                if (!(f >= 0.0)) {
                  d = -d;
                }
                s = absxk / d;
              }
              for (ihi = 0; ihi < b_n; ihi++) {
                ntau = scl_tmp + ihi;
                absxk = vr[ntau];
                u0 = in + ihi;
                f1 = vr[u0];
                vr[ntau] = c * absxk - s * f1;
                vr[u0] = c * f1 + s * absxk;
              }
              vr[k + vr.size(0) * (b_i + 1)] = 0.0;
            } else {
              ntau = b_i * b_n;
              absxk = 1.0 / internal::blas::xnrm2(b_n, vr, ntau + 1);
              i = ntau + b_n;
              for (k = ntau + 1; k <= i; k++) {
                vr[k - 1] = absxk * vr[k - 1];
              }
            }
          }
        }
        V.set_size(vr.size(0), vr.size(1));
        ntau = vr.size(0) * vr.size(1);
        for (i = 0; i < ntau; i++) {
          V[i].re = vr[i];
          V[i].im = 0.0;
        }
        for (ntau = 2; ntau <= b_n; ntau++) {
          if ((work[ntau - 2] > 0.0) && (work[ntau - 1] < 0.0)) {
            for (int b_i{0}; b_i < b_n; b_i++) {
              absxk = V[b_i + V.size(0) * (ntau - 2)].re;
              f1 = V[b_i + V.size(0) * (ntau - 1)].re;
              V[b_i + V.size(0) * (ntau - 2)].im = f1;
              V[b_i + V.size(0) * (ntau - 1)].re = absxk;
              V[b_i + V.size(0) * (ntau - 1)].im = -f1;
            }
          }
        }
      } else {
        V.set_size(A.size(0), A.size(0));
        ntau = A.size(0) * A.size(0);
        for (i = 0; i < ntau; i++) {
          V[i].re = rtNaN;
          V[i].im = 0.0;
        }
      }
      if (scalea) {
        i = A.size(0) - info;
        internal::reflapack::xzlascl(cscale, anrm, i, wr, info + 1);
        internal::reflapack::xzlascl(cscale, anrm, i, work, info + 1);
        if (info != 0) {
          internal::reflapack::xzlascl(cscale, anrm, ilo - 1, wr, 1);
          internal::reflapack::xzlascl(cscale, anrm, ilo - 1, work, 1);
        }
      }
      if (info != 0) {
        for (int b_i{ilo}; b_i <= info; b_i++) {
          wr[b_i - 1] = rtNaN;
          work[b_i - 1] = 0.0;
        }
      }
      W.set_size(wr.size(0));
      ntau = wr.size(0);
      for (i = 0; i < ntau; i++) {
        W[i].re = wr[i];
        W[i].im = work[i];
      }
    }
  }
  D.set_size(A.size(0), A.size(0));
  ntau = A.size(0) * A.size(0);
  for (i = 0; i < ntau; i++) {
    D[i].re = 0.0;
    D[i].im = 0.0;
  }
  for (k = 0; k < n; k++) {
    D[k + D.size(0) * k] = W[k];
  }
}

} // namespace coder

// End of code generation (eigStandard.cpp)
