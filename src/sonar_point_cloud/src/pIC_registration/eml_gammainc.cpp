//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// eml_gammainc.cpp
//
// Code generation for function 'eml_gammainc'
//

// Include files
#include "eml_gammainc.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions
namespace coder {
double eml_gammainc(double x, bool upper)
{
  double rval;
  if (!(x > 0.0)) {
    if (x == 0.0) {
      rval = upper;
    } else {
      rval = rtNaN;
    }
  } else if (std::isinf(x)) {
    rval = 1.0 - static_cast<double>(upper);
  } else {
    double dj;
    double old;
    double v;
    double vsq;
    double xD0;
    int exitg1;
    if (std::abs(1.0 - x) > 0.1 * (x + 1.0)) {
      if (2.2250738585072014E-308 * x > 1.0) {
        xD0 = x;
      } else if ((x < 1.0) && (1.7976931348623157E+308 * x < 1.0)) {
        xD0 = -std::log(x) - 1.0;
      } else {
        xD0 = (std::log(1.0 / x) + x) - 1.0;
      }
    } else {
      v = (1.0 - x) / (x + 1.0);
      vsq = v * v;
      xD0 = (1.0 - x) * v;
      old = xD0;
      v *= 2.0;
      dj = 3.0;
      do {
        exitg1 = 0;
        v *= vsq;
        xD0 += v / dj;
        if (xD0 == old) {
          exitg1 = 1;
        } else {
          old = xD0;
          dj += 2.0;
        }
      } while (exitg1 == 0);
    }
    if (x > 1.0E+6) {
      double sqrtx;
      double t;
      double tsq;
      sqrtx = std::sqrt(x);
      t = std::abs((1.0 - x) - 1.0) / sqrtx;
      tsq = t * t;
      if (t < 15.0) {
        v = 0.70710678118654746 * t;
        dj = 3.97886080735226 / (v + 3.97886080735226);
        dj = 0.5 *
             ((((((((((((((((((((((0.0012710976495261409 * (dj - 0.5) +
                                   0.00011931402283834095) *
                                      (dj - 0.5) -
                                  0.0039638509736051354) *
                                     (dj - 0.5) -
                                 0.00087077963531729586) *
                                    (dj - 0.5) +
                                0.0077367252831352668) *
                                   (dj - 0.5) +
                               0.0038333512626488732) *
                                  (dj - 0.5) -
                              0.012722381378212275) *
                                 (dj - 0.5) -
                             0.013382364453346007) *
                                (dj - 0.5) +
                            0.016131532973325226) *
                               (dj - 0.5) +
                           0.039097684558848406) *
                              (dj - 0.5) +
                          0.0024936720005350331) *
                             (dj - 0.5) -
                         0.0838864557023002) *
                            (dj - 0.5) -
                        0.11946395996432542) *
                           (dj - 0.5) +
                       0.016620792496936737) *
                          (dj - 0.5) +
                      0.35752427444953105) *
                         (dj - 0.5) +
                     0.80527640875291062) *
                        (dj - 0.5) +
                    1.1890298290927332) *
                       (dj - 0.5) +
                   1.3704021768233816) *
                      (dj - 0.5) +
                  1.313146538310231) *
                     (dj - 0.5) +
                 1.0792551515585667) *
                    (dj - 0.5) +
                0.77436819911953858) *
                   (dj - 0.5) +
               0.49016508058531844) *
                  (dj - 0.5) +
              0.27537474159737679) *
             dj * std::exp(-v * v) * 2.5066282746310002 * std::exp(0.5 * tsq);
        vsq = (dj * ((tsq - 3.0) * t) - (tsq - 4.0)) / 6.0;
        old = (dj * ((tsq * tsq - 9.0) * tsq + 6.0) -
               ((tsq - 1.0) * tsq - 6.0) * t) /
              72.0;
        v = (dj * (((((5.0 * tsq + 45.0) * tsq - 81.0) * tsq - 315.0) * tsq +
                    270.0) *
                   t) -
             ((((5.0 * tsq + 40.0) * tsq - 111.0) * tsq - 174.0) * tsq +
              192.0)) /
            6480.0;
      } else {
        dj = (((3.0 - 15.0 / tsq) / tsq - 1.0) / tsq + 1.0) / t;
        vsq = (((25.0 - 210.0 / tsq) / tsq - 4.0) / tsq + 1.0) / tsq;
        old = (((130.0 - 1750.0 / tsq) / tsq - 11.0) / tsq + 1.0) / (tsq * t);
        v = (((546.0 - 11368.0 / tsq) / tsq - 26.0) / tsq + 1.0) / (tsq * tsq);
      }
      v = ((dj / sqrtx + vsq / x) + old / (x * sqrtx)) + v / (x * x);
      if (-0.99999999999999989 - xD0 < 709.782712893384) {
        rval = std::exp(-0.99999999999999989 - xD0) * v;
      } else {
        rval = std::exp((-0.99999999999999989 - xD0) + std::log(v));
      }
      if (!upper) {
        rval = 1.0 - rval;
      }
    } else if (x < 1.0) {
      int i;
      dj = 1.0;
      if (x > 2.2204460492503131E-16) {
        v = x;
        dj = 2.0;
        do {
          exitg1 = 0;
          v = x * v / ((dj - 1.0) + 1.0);
          if (v < 2.2204460492503131E-16) {
            exitg1 = 1;
          } else {
            dj++;
          }
        } while (exitg1 == 0);
      }
      v = 0.0;
      i = static_cast<int>(-((-1.0 - (dj - 1.0)) + 1.0));
      for (int b_i{0}; b_i < i; b_i++) {
        v = x * (v + 1.0) / (((dj - 1.0) - static_cast<double>(b_i)) + 1.0);
      }
      v++;
      if (-0.99999999999999989 - xD0 < 709.782712893384) {
        rval = std::exp(-0.99999999999999989 - xD0) * v;
      } else {
        rval = std::exp((-0.99999999999999989 - xD0) + std::log(v));
      }
      if (rval > 1.0) {
        rval = 1.0;
      }
      if (upper) {
        rval = 1.0 - rval;
      }
    } else {
      int i;
      v = 1.0;
      dj = 1.0;
      do {
        exitg1 = 0;
        i = static_cast<int>(std::floor(x + 1.0));
        if (static_cast<int>(dj) <= i) {
          v = (1.0 - dj) * v / x;
          if (std::abs(v) < 2.2204460492503131E-16) {
            exitg1 = 1;
          } else {
            dj++;
          }
        } else {
          exitg1 = 1;
        }
      } while (exitg1 == 0);
      if (static_cast<int>(dj) <= i) {
        v = 1.0;
      } else {
        v = 1.0;
        dj = 1.0;
      }
      i = static_cast<int>(dj);
      for (int b_i{0}; b_i <= i - 2; b_i++) {
        v = (1.0 - ((dj - 1.0) - static_cast<double>(b_i))) * v / x + 1.0;
      }
      v /= x;
      if (-0.99999999999999989 - xD0 < 709.782712893384) {
        rval = std::exp(-0.99999999999999989 - xD0) * v;
      } else {
        rval = std::exp((-0.99999999999999989 - xD0) + std::log(v));
      }
      if (rval > 1.0) {
        rval = 1.0;
      }
      if (!upper) {
        rval = 1.0 - rval;
      }
    }
  }
  return rval;
}

} // namespace coder

// End of code generation (eml_gammainc.cpp)
