//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// chi2inv.cpp
//
// Code generation for function 'chi2inv'
//

// Include files
#include "chi2inv.h"
#include "eml_gammainc.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Declarations
static double rt_powd_snf(double u0, double u1);

// Function Definitions
static double rt_powd_snf(double u0, double u1)
{
  double y;
  if (std::isnan(u0) || std::isnan(u1)) {
    y = rtNaN;
  } else {
    double d;
    double d1;
    d = std::abs(u0);
    d1 = std::abs(u1);
    if (std::isinf(u1)) {
      if (d == 1.0) {
        y = 1.0;
      } else if (d > 1.0) {
        if (u1 > 0.0) {
          y = rtInf;
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = rtInf;
      }
    } else if (d1 == 0.0) {
      y = 1.0;
    } else if (d1 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = std::sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > std::floor(u1))) {
      y = rtNaN;
    } else {
      y = std::pow(u0, u1);
    }
  }
  return y;
}

namespace coder {
double chi2inv(double p)
{
  double x;
  if ((p >= 0.0) && (p <= 1.0)) {
    if ((p > 0.0) && (p < 1.0)) {
      double b_p;
      double log1mpLower;
      double pLower;
      double r;
      double z;
      double zhi;
      double zlo;
      int i;
      int sgn;
      bool exitg1;
      bool upper;
      b_p = p;
      upper = false;
      if (p > 0.5) {
        b_p = 1.0 - p;
        upper = true;
        pLower = 1.0 - (1.0 - p);
        log1mpLower = std::log(1.0 - p);
      } else {
        pLower = p;
        if (1.0 - p != 1.0) {
          log1mpLower = std::log(1.0 - p) * (-p / ((1.0 - p) - 1.0));
        } else {
          log1mpLower = -p;
        }
      }
      r = std::log(pLower);
      if (-1.24 * r > 2.0) {
        r = rt_powd_snf(pLower * 1.9999999999999993, 1.0);
        if (r < 2.2250738585072014E-306) {
          r = 2.2250738585072014E-306;
        }
      } else {
        if (std::abs(pLower - 0.5) <= 0.425) {
          r = 0.180625 - (pLower - 0.5) * (pLower - 0.5);
          z = (pLower - 0.5) *
              (((((((2509.0809287301227 * r + 33430.575583588128) * r +
                    67265.7709270087) *
                       r +
                   45921.95393154987) *
                      r +
                  13731.693765509461) *
                     r +
                 1971.5909503065513) *
                    r +
                133.14166789178438) *
                   r +
               3.3871328727963665) /
              (((((((5226.4952788528544 * r + 28729.085735721943) * r +
                    39307.895800092709) *
                       r +
                   21213.794301586597) *
                      r +
                  5394.1960214247511) *
                     r +
                 687.18700749205789) *
                    r +
                42.313330701600911) *
                   r +
               1.0);
        } else {
          if (pLower - 0.5 < 0.0) {
            r = std::sqrt(-r);
          } else {
            r = std::sqrt(-std::log(1.0 - pLower));
          }
          if (r <= 5.0) {
            r -= 1.6;
            z = (((((((0.00077454501427834139 * r + 0.022723844989269184) * r +
                      0.24178072517745061) *
                         r +
                     1.2704582524523684) *
                        r +
                    3.6478483247632045) *
                       r +
                   5.769497221460691) *
                      r +
                  4.6303378461565456) *
                     r +
                 1.4234371107496835) /
                (((((((1.0507500716444169E-9 * r + 0.00054759380849953455) * r +
                      0.015198666563616457) *
                         r +
                     0.14810397642748008) *
                        r +
                    0.6897673349851) *
                       r +
                   1.6763848301838038) *
                      r +
                  2.053191626637759) *
                     r +
                 1.0);
          } else {
            r -= 5.0;
            z = (((((((2.0103343992922881E-7 * r + 2.7115555687434876E-5) * r +
                      0.0012426609473880784) *
                         r +
                     0.026532189526576124) *
                        r +
                    0.29656057182850487) *
                       r +
                   1.7848265399172913) *
                      r +
                  5.4637849111641144) *
                     r +
                 6.6579046435011033) /
                (((((((2.0442631033899397E-15 * r + 1.4215117583164459E-7) * r +
                      1.8463183175100548E-5) *
                         r +
                     0.00078686913114561329) *
                        r +
                    0.014875361290850615) *
                       r +
                   0.13692988092273581) *
                      r +
                  0.599832206555888) *
                     r +
                 1.0);
          }
          if (pLower - 0.5 < 0.0) {
            z = -z;
          }
        }
        r = (z * 0.333333166666625 + 1.0) - 0.111111;
        r *= 2.0 * r * r;
        if (r > 10.4) {
          r = -2.0 * (log1mpLower - 0.0 * std::log(0.5 * r));
        }
      }
      z = 0.5 * r;
      r = rtInf;
      pLower = rtInf;
      if (b_p > 1.0021E-294) {
        log1mpLower = 2.2204460492503131E-14 * b_p;
      } else {
        log1mpLower = 2.2251089859537388E-308;
      }
      if (upper) {
        sgn = -1;
      } else {
        sgn = 1;
      }
      zlo = 0.0;
      zhi = 1.7976931348623157E+308;
      i = 0;
      exitg1 = false;
      while ((!exitg1) && (i < 1000)) {
        double f;
        f = static_cast<double>(sgn) * (eml_gammainc(z, upper) - b_p);
        if ((f * r < 0.0) && (std::abs(r) <= std::abs(f))) {
          z = 0.5 * z + 0.5 * pLower;
          f = static_cast<double>(sgn) * (eml_gammainc(z, upper) - b_p);
        }
        if (f > 0.0) {
          zhi = z;
        } else {
          zlo = z;
        }
        if ((std::abs(f) < log1mpLower) ||
            (std::abs(z - pLower) <
             2.2204460492503131E-16 * z + 2.2250738585072014E-308)) {
          exitg1 = true;
        } else {
          bool guard1;
          pLower = z;
          r = f;
          guard1 = false;
          if (i < 500) {
            z *= 1.0 - f / (z * std::exp(0.0 * std::log(z) - z) +
                            f * ((z + 1.0) - 1.0) / 2.0);
            if (z <= zlo) {
              if (zlo == 0.0) {
                if (std::abs(static_cast<double>(upper) - b_p) <
                    std::abs(eml_gammainc(2.2250738585072014E-308, upper) -
                             b_p)) {
                  z = 0.0;
                  exitg1 = true;
                } else {
                  zlo = 2.2250738585072014E-308;
                  guard1 = true;
                }
              } else {
                guard1 = true;
              }
            } else {
              if (z >= zhi) {
                z = 0.01 * zlo + 0.99 * zhi;
              }
              i++;
            }
          } else {
            if (1.0E+8 * zlo < zhi) {
              pLower = 1.0E+8 * zlo;
              r = static_cast<double>(sgn) *
                  (eml_gammainc(pLower, upper) - b_p);
              if (r > 0.0) {
                zhi = pLower;
              } else {
                zlo = pLower;
              }
            }
            z = 0.5 * zlo + 0.5 * zhi;
            i++;
          }
          if (guard1) {
            z = 0.99 * zlo + 0.01 * zhi;
            i++;
          }
        }
      }
      x = z * 2.0;
    } else if (p == 0.0) {
      x = 0.0;
    } else {
      x = rtInf;
    }
  } else {
    x = rtNaN;
  }
  return x;
}

} // namespace coder

// End of code generation (chi2inv.cpp)
