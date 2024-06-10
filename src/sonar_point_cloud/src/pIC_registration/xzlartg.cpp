//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// xzlartg.cpp
//
// Code generation for function 'xzlartg'
//

// Include files
#include "xzlartg.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions
namespace coder {
namespace internal {
namespace reflapack {
double xzlartg(double f, double g, double &sn, double &r)
{
  double cs;
  double f1;
  f1 = std::abs(f);
  r = std::abs(g);
  if (g == 0.0) {
    cs = 1.0;
    sn = 0.0;
    r = f;
  } else if (f == 0.0) {
    cs = 0.0;
    if (g >= 0.0) {
      sn = 1.0;
    } else {
      sn = -1.0;
    }
  } else if ((f1 > 1.4916681462400413E-154) && (f1 < 4.7403759540545887E+153) &&
             (r > 1.4916681462400413E-154) && (r < 4.7403759540545887E+153)) {
    r = std::sqrt(f * f + g * g);
    cs = f1 / r;
    if (!(f >= 0.0)) {
      r = -r;
    }
    sn = g / r;
  } else {
    double gs;
    double u;
    u = std::fmin(4.49423283715579E+307,
                  std::fmax(2.2250738585072014E-308, std::fmax(f1, r)));
    f1 = f / u;
    gs = g / u;
    r = std::sqrt(f1 * f1 + gs * gs);
    cs = std::abs(f1) / r;
    if (!(f >= 0.0)) {
      r = -r;
    }
    sn = gs / r;
    r *= u;
  }
  return cs;
}

} // namespace reflapack
} // namespace internal
} // namespace coder

// End of code generation (xzlartg.cpp)
