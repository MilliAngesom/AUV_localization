//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// inv.cpp
//
// Code generation for function 'inv'
//

// Include files
#include "inv.h"
#include "pIC_registration_data.h"
#include "pIC_registration_rtwutil.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <algorithm>
#include <cmath>

// Function Definitions
namespace coder {
void inv(const double x[9], double y[9])
{
  double b_x[9];
  double absx11;
  double absx21;
  double absx31;
  int p1;
  int p2;
  int p3;
  std::copy(&x[0], &x[9], &b_x[0]);
  p1 = 0;
  p2 = 3;
  p3 = 6;
  absx11 = std::abs(x[0]);
  absx21 = std::abs(x[1]);
  absx31 = std::abs(x[2]);
  if ((absx21 > absx11) && (absx21 > absx31)) {
    p1 = 3;
    p2 = 0;
    b_x[0] = x[1];
    b_x[1] = x[0];
    b_x[3] = x[4];
    b_x[4] = x[3];
    b_x[6] = x[7];
    b_x[7] = x[6];
  } else if (absx31 > absx11) {
    p1 = 6;
    p3 = 0;
    b_x[0] = x[2];
    b_x[2] = x[0];
    b_x[3] = x[5];
    b_x[5] = x[3];
    b_x[6] = x[8];
    b_x[8] = x[6];
  }
  b_x[1] /= b_x[0];
  b_x[2] /= b_x[0];
  b_x[4] -= b_x[1] * b_x[3];
  b_x[5] -= b_x[2] * b_x[3];
  b_x[7] -= b_x[1] * b_x[6];
  b_x[8] -= b_x[2] * b_x[6];
  if (std::abs(b_x[5]) > std::abs(b_x[4])) {
    int itmp;
    itmp = p2;
    p2 = p3;
    p3 = itmp;
    absx11 = b_x[1];
    b_x[1] = b_x[2];
    b_x[2] = absx11;
    absx11 = b_x[4];
    b_x[4] = b_x[5];
    b_x[5] = absx11;
    absx11 = b_x[7];
    b_x[7] = b_x[8];
    b_x[8] = absx11;
  }
  b_x[5] /= b_x[4];
  b_x[8] -= b_x[5] * b_x[7];
  absx11 = (b_x[1] * b_x[5] - b_x[2]) / b_x[8];
  absx21 = -(b_x[1] + b_x[7] * absx11) / b_x[4];
  y[p1] = ((1.0 - b_x[3] * absx21) - b_x[6] * absx11) / b_x[0];
  y[p1 + 1] = absx21;
  y[p1 + 2] = absx11;
  absx11 = -b_x[5] / b_x[8];
  absx21 = (1.0 - b_x[7] * absx11) / b_x[4];
  y[p2] = -(b_x[3] * absx21 + b_x[6] * absx11) / b_x[0];
  y[p2 + 1] = absx21;
  y[p2 + 2] = absx11;
  absx11 = 1.0 / b_x[8];
  absx21 = -b_x[7] * absx11 / b_x[4];
  y[p3] = -(b_x[3] * absx21 + b_x[6] * absx11) / b_x[0];
  y[p3 + 1] = absx21;
  y[p3 + 2] = absx11;
}

void inv(const array<double, 2U> &x, array<double, 2U> &y)
{
  array<double, 2U> b_x;
  array<int, 2U> ipiv;
  array<int, 2U> p;
  if ((x.size(0) == 0) || (x.size(1) == 0)) {
    int b_n;
    y.set_size(x.size(0), x.size(1));
    b_n = x.size(0) * x.size(1);
    for (int i{0}; i < b_n; i++) {
      y[i] = x[i];
    }
  } else {
    double smax;
    int b_n;
    int i;
    int i1;
    int n;
    int temp_tmp;
    int u1;
    int yk;
    n = x.size(0);
    y.set_size(x.size(0), x.size(1));
    b_n = x.size(0) * x.size(1);
    for (i = 0; i < b_n; i++) {
      y[i] = 0.0;
    }
    b_x.set_size(x.size(0), x.size(1));
    for (i = 0; i < b_n; i++) {
      b_x[i] = x[i];
    }
    b_n = x.size(0);
    ipiv.set_size(1, x.size(0));
    ipiv[0] = 1;
    yk = 1;
    for (int k{2}; k <= b_n; k++) {
      yk++;
      ipiv[k - 1] = yk;
    }
    b_n = x.size(0) - 1;
    u1 = x.size(0);
    if (b_n <= u1) {
      u1 = b_n;
    }
    for (int j{0}; j < u1; j++) {
      int b_tmp;
      int jp1j;
      int mmj_tmp;
      mmj_tmp = n - j;
      b_tmp = j * (n + 1);
      jp1j = b_tmp + 2;
      if (mmj_tmp < 1) {
        b_n = -1;
      } else {
        b_n = 0;
        if (mmj_tmp > 1) {
          smax = std::abs(b_x[b_tmp]);
          for (int k{2}; k <= mmj_tmp; k++) {
            double s;
            s = std::abs(b_x[(b_tmp + k) - 1]);
            if (s > smax) {
              b_n = k - 1;
              smax = s;
            }
          }
        }
      }
      if (b_x[b_tmp + b_n] != 0.0) {
        if (b_n != 0) {
          yk = j + b_n;
          ipiv[j] = yk + 1;
          for (int k{0}; k < n; k++) {
            b_n = k * n;
            temp_tmp = j + b_n;
            smax = b_x[temp_tmp];
            i = yk + b_n;
            b_x[temp_tmp] = b_x[i];
            b_x[i] = smax;
          }
        }
        i = b_tmp + mmj_tmp;
        for (temp_tmp = jp1j; temp_tmp <= i; temp_tmp++) {
          b_x[temp_tmp - 1] = b_x[temp_tmp - 1] / b_x[b_tmp];
        }
      }
      b_n = b_tmp + n;
      yk = b_n;
      for (temp_tmp = 0; temp_tmp <= mmj_tmp - 2; temp_tmp++) {
        smax = b_x[b_n + temp_tmp * n];
        if (smax != 0.0) {
          i = yk + 2;
          i1 = mmj_tmp + yk;
          for (jp1j = i; jp1j <= i1; jp1j++) {
            b_x[jp1j - 1] =
                b_x[jp1j - 1] + b_x[((b_tmp + jp1j) - yk) - 1] * -smax;
          }
        }
        yk += n;
      }
    }
    b_n = x.size(0);
    p.set_size(1, x.size(0));
    p[0] = 1;
    yk = 1;
    for (int k{2}; k <= b_n; k++) {
      yk++;
      p[k - 1] = yk;
    }
    i = ipiv.size(1);
    for (int k{0}; k < i; k++) {
      i1 = ipiv[k];
      if (i1 > k + 1) {
        b_n = p[i1 - 1];
        p[i1 - 1] = p[k];
        p[k] = b_n;
      }
    }
    for (int k{0}; k < n; k++) {
      i = p[k];
      y[k + y.size(0) * (i - 1)] = 1.0;
      for (int j{k + 1}; j <= n; j++) {
        if (y[(j + y.size(0) * (i - 1)) - 1] != 0.0) {
          i1 = j + 1;
          for (temp_tmp = i1; temp_tmp <= n; temp_tmp++) {
            y[(temp_tmp + y.size(0) * (i - 1)) - 1] =
                y[(temp_tmp + y.size(0) * (i - 1)) - 1] -
                y[(j + y.size(0) * (i - 1)) - 1] *
                    b_x[(temp_tmp + b_x.size(0) * (j - 1)) - 1];
          }
        }
      }
    }
    for (int j{0}; j < n; j++) {
      b_n = n * j - 1;
      for (int k{n}; k >= 1; k--) {
        yk = n * (k - 1) - 1;
        i = k + b_n;
        smax = y[i];
        if (smax != 0.0) {
          y[i] = smax / b_x[k + yk];
          for (temp_tmp = 0; temp_tmp <= k - 2; temp_tmp++) {
            i1 = (temp_tmp + b_n) + 1;
            y[i1] = y[i1] - y[i] * b_x[(temp_tmp + yk) + 1];
          }
        }
      }
    }
  }
}

void inv(const creal_T x[9], creal_T y[9])
{
  creal_T b_x[9];
  double absx11;
  double absx21;
  double absx31;
  double brm;
  double d;
  double im;
  double re;
  double t1_im;
  double t1_re;
  double t2_im;
  double t2_re;
  int p1;
  int p2;
  int p3;
  std::copy(&x[0], &x[9], &b_x[0]);
  p1 = 0;
  p2 = 3;
  p3 = 6;
  absx11 = rt_hypotd_snf(x[0].re, x[0].im);
  absx21 = rt_hypotd_snf(x[1].re, x[1].im);
  absx31 = rt_hypotd_snf(x[2].re, x[2].im);
  if ((absx21 > absx11) && (absx21 > absx31)) {
    p1 = 3;
    p2 = 0;
    b_x[0] = x[1];
    b_x[1] = x[0];
    b_x[3] = x[4];
    b_x[4] = x[3];
    b_x[6] = x[7];
    b_x[7] = x[6];
  } else if (absx31 > absx11) {
    p1 = 6;
    p3 = 0;
    b_x[0] = x[2];
    b_x[2] = x[0];
    b_x[3] = x[5];
    b_x[5] = x[3];
    b_x[6] = x[8];
    b_x[8] = x[6];
  }
  if (b_x[0].im == 0.0) {
    if (b_x[1].im == 0.0) {
      re = b_x[1].re / b_x[0].re;
      im = 0.0;
    } else if (b_x[1].re == 0.0) {
      re = 0.0;
      im = b_x[1].im / b_x[0].re;
    } else {
      re = b_x[1].re / b_x[0].re;
      im = b_x[1].im / b_x[0].re;
    }
  } else if (b_x[0].re == 0.0) {
    if (b_x[1].re == 0.0) {
      re = b_x[1].im / b_x[0].im;
      im = 0.0;
    } else if (b_x[1].im == 0.0) {
      re = 0.0;
      im = -(b_x[1].re / b_x[0].im);
    } else {
      re = b_x[1].im / b_x[0].im;
      im = -(b_x[1].re / b_x[0].im);
    }
  } else {
    brm = std::abs(b_x[0].re);
    absx31 = std::abs(b_x[0].im);
    if (brm > absx31) {
      absx31 = b_x[0].im / b_x[0].re;
      d = b_x[0].re + absx31 * b_x[0].im;
      re = (b_x[1].re + absx31 * b_x[1].im) / d;
      im = (b_x[1].im - absx31 * b_x[1].re) / d;
    } else if (absx31 == brm) {
      if (b_x[0].re > 0.0) {
        absx31 = 0.5;
      } else {
        absx31 = -0.5;
      }
      if (b_x[0].im > 0.0) {
        d = 0.5;
      } else {
        d = -0.5;
      }
      re = (b_x[1].re * absx31 + b_x[1].im * d) / brm;
      im = (b_x[1].im * absx31 - b_x[1].re * d) / brm;
    } else {
      absx31 = b_x[0].re / b_x[0].im;
      d = b_x[0].im + absx31 * b_x[0].re;
      re = (absx31 * b_x[1].re + b_x[1].im) / d;
      im = (absx31 * b_x[1].im - b_x[1].re) / d;
    }
  }
  b_x[1].re = re;
  b_x[1].im = im;
  if (b_x[0].im == 0.0) {
    if (b_x[2].im == 0.0) {
      absx21 = b_x[2].re / b_x[0].re;
      absx11 = 0.0;
    } else if (b_x[2].re == 0.0) {
      absx21 = 0.0;
      absx11 = b_x[2].im / b_x[0].re;
    } else {
      absx21 = b_x[2].re / b_x[0].re;
      absx11 = b_x[2].im / b_x[0].re;
    }
  } else if (b_x[0].re == 0.0) {
    if (b_x[2].re == 0.0) {
      absx21 = b_x[2].im / b_x[0].im;
      absx11 = 0.0;
    } else if (b_x[2].im == 0.0) {
      absx21 = 0.0;
      absx11 = -(b_x[2].re / b_x[0].im);
    } else {
      absx21 = b_x[2].im / b_x[0].im;
      absx11 = -(b_x[2].re / b_x[0].im);
    }
  } else {
    brm = std::abs(b_x[0].re);
    absx31 = std::abs(b_x[0].im);
    if (brm > absx31) {
      absx31 = b_x[0].im / b_x[0].re;
      d = b_x[0].re + absx31 * b_x[0].im;
      absx21 = (b_x[2].re + absx31 * b_x[2].im) / d;
      absx11 = (b_x[2].im - absx31 * b_x[2].re) / d;
    } else if (absx31 == brm) {
      if (b_x[0].re > 0.0) {
        absx31 = 0.5;
      } else {
        absx31 = -0.5;
      }
      if (b_x[0].im > 0.0) {
        d = 0.5;
      } else {
        d = -0.5;
      }
      absx21 = (b_x[2].re * absx31 + b_x[2].im * d) / brm;
      absx11 = (b_x[2].im * absx31 - b_x[2].re * d) / brm;
    } else {
      absx31 = b_x[0].re / b_x[0].im;
      d = b_x[0].im + absx31 * b_x[0].re;
      absx21 = (absx31 * b_x[2].re + b_x[2].im) / d;
      absx11 = (absx31 * b_x[2].im - b_x[2].re) / d;
    }
  }
  b_x[2].re = absx21;
  b_x[2].im = absx11;
  b_x[4].re -= re * b_x[3].re - im * b_x[3].im;
  b_x[4].im -= re * b_x[3].im + im * b_x[3].re;
  b_x[5].re -= absx21 * b_x[3].re - absx11 * b_x[3].im;
  b_x[5].im -= absx21 * b_x[3].im + absx11 * b_x[3].re;
  b_x[7].re -= re * b_x[6].re - im * b_x[6].im;
  b_x[7].im -= re * b_x[6].im + im * b_x[6].re;
  b_x[8].re -= absx21 * b_x[6].re - absx11 * b_x[6].im;
  b_x[8].im -= absx21 * b_x[6].im + absx11 * b_x[6].re;
  if (rt_hypotd_snf(b_x[5].re, b_x[5].im) >
      rt_hypotd_snf(b_x[4].re, b_x[4].im)) {
    int itmp;
    itmp = p2;
    p2 = p3;
    p3 = itmp;
    b_x[1] = b_x[2];
    b_x[2].re = re;
    b_x[2].im = im;
    t1_re = b_x[4].re;
    t1_im = b_x[4].im;
    b_x[4] = b_x[5];
    b_x[5].re = t1_re;
    b_x[5].im = t1_im;
    t1_re = b_x[7].re;
    t1_im = b_x[7].im;
    b_x[7] = b_x[8];
    b_x[8].re = t1_re;
    b_x[8].im = t1_im;
  }
  if (b_x[4].im == 0.0) {
    if (b_x[5].im == 0.0) {
      re = b_x[5].re / b_x[4].re;
      im = 0.0;
    } else if (b_x[5].re == 0.0) {
      re = 0.0;
      im = b_x[5].im / b_x[4].re;
    } else {
      re = b_x[5].re / b_x[4].re;
      im = b_x[5].im / b_x[4].re;
    }
  } else if (b_x[4].re == 0.0) {
    if (b_x[5].re == 0.0) {
      re = b_x[5].im / b_x[4].im;
      im = 0.0;
    } else if (b_x[5].im == 0.0) {
      re = 0.0;
      im = -(b_x[5].re / b_x[4].im);
    } else {
      re = b_x[5].im / b_x[4].im;
      im = -(b_x[5].re / b_x[4].im);
    }
  } else {
    brm = std::abs(b_x[4].re);
    absx31 = std::abs(b_x[4].im);
    if (brm > absx31) {
      absx31 = b_x[4].im / b_x[4].re;
      d = b_x[4].re + absx31 * b_x[4].im;
      re = (b_x[5].re + absx31 * b_x[5].im) / d;
      im = (b_x[5].im - absx31 * b_x[5].re) / d;
    } else if (absx31 == brm) {
      if (b_x[4].re > 0.0) {
        absx31 = 0.5;
      } else {
        absx31 = -0.5;
      }
      if (b_x[4].im > 0.0) {
        d = 0.5;
      } else {
        d = -0.5;
      }
      re = (b_x[5].re * absx31 + b_x[5].im * d) / brm;
      im = (b_x[5].im * absx31 - b_x[5].re * d) / brm;
    } else {
      absx31 = b_x[4].re / b_x[4].im;
      d = b_x[4].im + absx31 * b_x[4].re;
      re = (absx31 * b_x[5].re + b_x[5].im) / d;
      im = (absx31 * b_x[5].im - b_x[5].re) / d;
    }
  }
  b_x[8].re -= re * b_x[7].re - im * b_x[7].im;
  b_x[8].im -= re * b_x[7].im + im * b_x[7].re;
  absx11 = (b_x[1].re * re - b_x[1].im * im) - b_x[2].re;
  absx21 = (b_x[1].re * im + b_x[1].im * re) - b_x[2].im;
  if (b_x[8].im == 0.0) {
    if (absx21 == 0.0) {
      t1_re = absx11 / b_x[8].re;
      t1_im = 0.0;
    } else if (absx11 == 0.0) {
      t1_re = 0.0;
      t1_im = absx21 / b_x[8].re;
    } else {
      t1_re = absx11 / b_x[8].re;
      t1_im = absx21 / b_x[8].re;
    }
  } else if (b_x[8].re == 0.0) {
    if (absx11 == 0.0) {
      t1_re = absx21 / b_x[8].im;
      t1_im = 0.0;
    } else if (absx21 == 0.0) {
      t1_re = 0.0;
      t1_im = -(absx11 / b_x[8].im);
    } else {
      t1_re = absx21 / b_x[8].im;
      t1_im = -(absx11 / b_x[8].im);
    }
  } else {
    brm = std::abs(b_x[8].re);
    absx31 = std::abs(b_x[8].im);
    if (brm > absx31) {
      absx31 = b_x[8].im / b_x[8].re;
      d = b_x[8].re + absx31 * b_x[8].im;
      t1_re = (absx11 + absx31 * absx21) / d;
      t1_im = (absx21 - absx31 * absx11) / d;
    } else if (absx31 == brm) {
      if (b_x[8].re > 0.0) {
        absx31 = 0.5;
      } else {
        absx31 = -0.5;
      }
      if (b_x[8].im > 0.0) {
        d = 0.5;
      } else {
        d = -0.5;
      }
      t1_re = (absx11 * absx31 + absx21 * d) / brm;
      t1_im = (absx21 * absx31 - absx11 * d) / brm;
    } else {
      absx31 = b_x[8].re / b_x[8].im;
      d = b_x[8].im + absx31 * b_x[8].re;
      t1_re = (absx31 * absx11 + absx21) / d;
      t1_im = (absx31 * absx21 - absx11) / d;
    }
  }
  absx11 = -(b_x[1].re + (b_x[7].re * t1_re - b_x[7].im * t1_im));
  absx21 = -(b_x[1].im + (b_x[7].re * t1_im + b_x[7].im * t1_re));
  if (b_x[4].im == 0.0) {
    if (absx21 == 0.0) {
      t2_re = absx11 / b_x[4].re;
      t2_im = 0.0;
    } else if (absx11 == 0.0) {
      t2_re = 0.0;
      t2_im = absx21 / b_x[4].re;
    } else {
      t2_re = absx11 / b_x[4].re;
      t2_im = absx21 / b_x[4].re;
    }
  } else if (b_x[4].re == 0.0) {
    if (absx11 == 0.0) {
      t2_re = absx21 / b_x[4].im;
      t2_im = 0.0;
    } else if (absx21 == 0.0) {
      t2_re = 0.0;
      t2_im = -(absx11 / b_x[4].im);
    } else {
      t2_re = absx21 / b_x[4].im;
      t2_im = -(absx11 / b_x[4].im);
    }
  } else {
    brm = std::abs(b_x[4].re);
    absx31 = std::abs(b_x[4].im);
    if (brm > absx31) {
      absx31 = b_x[4].im / b_x[4].re;
      d = b_x[4].re + absx31 * b_x[4].im;
      t2_re = (absx11 + absx31 * absx21) / d;
      t2_im = (absx21 - absx31 * absx11) / d;
    } else if (absx31 == brm) {
      if (b_x[4].re > 0.0) {
        absx31 = 0.5;
      } else {
        absx31 = -0.5;
      }
      if (b_x[4].im > 0.0) {
        d = 0.5;
      } else {
        d = -0.5;
      }
      t2_re = (absx11 * absx31 + absx21 * d) / brm;
      t2_im = (absx21 * absx31 - absx11 * d) / brm;
    } else {
      absx31 = b_x[4].re / b_x[4].im;
      d = b_x[4].im + absx31 * b_x[4].re;
      t2_re = (absx31 * absx11 + absx21) / d;
      t2_im = (absx31 * absx21 - absx11) / d;
    }
  }
  absx11 = (1.0 - (b_x[3].re * t2_re - b_x[3].im * t2_im)) -
           (b_x[6].re * t1_re - b_x[6].im * t1_im);
  absx21 = (0.0 - (b_x[3].re * t2_im + b_x[3].im * t2_re)) -
           (b_x[6].re * t1_im + b_x[6].im * t1_re);
  if (b_x[0].im == 0.0) {
    if (absx21 == 0.0) {
      y[p1].re = absx11 / b_x[0].re;
      y[p1].im = 0.0;
    } else if (absx11 == 0.0) {
      y[p1].re = 0.0;
      y[p1].im = absx21 / b_x[0].re;
    } else {
      y[p1].re = absx11 / b_x[0].re;
      y[p1].im = absx21 / b_x[0].re;
    }
  } else if (b_x[0].re == 0.0) {
    if (absx11 == 0.0) {
      y[p1].re = absx21 / b_x[0].im;
      y[p1].im = 0.0;
    } else if (absx21 == 0.0) {
      y[p1].re = 0.0;
      y[p1].im = -(absx11 / b_x[0].im);
    } else {
      y[p1].re = absx21 / b_x[0].im;
      y[p1].im = -(absx11 / b_x[0].im);
    }
  } else {
    brm = std::abs(b_x[0].re);
    absx31 = std::abs(b_x[0].im);
    if (brm > absx31) {
      absx31 = b_x[0].im / b_x[0].re;
      d = b_x[0].re + absx31 * b_x[0].im;
      y[p1].re = (absx11 + absx31 * absx21) / d;
      y[p1].im = (absx21 - absx31 * absx11) / d;
    } else if (absx31 == brm) {
      if (b_x[0].re > 0.0) {
        absx31 = 0.5;
      } else {
        absx31 = -0.5;
      }
      if (b_x[0].im > 0.0) {
        d = 0.5;
      } else {
        d = -0.5;
      }
      y[p1].re = (absx11 * absx31 + absx21 * d) / brm;
      y[p1].im = (absx21 * absx31 - absx11 * d) / brm;
    } else {
      absx31 = b_x[0].re / b_x[0].im;
      d = b_x[0].im + absx31 * b_x[0].re;
      y[p1].re = (absx31 * absx11 + absx21) / d;
      y[p1].im = (absx31 * absx21 - absx11) / d;
    }
  }
  y[p1 + 1].re = t2_re;
  y[p1 + 1].im = t2_im;
  y[p1 + 2].re = t1_re;
  y[p1 + 2].im = t1_im;
  if (b_x[8].im == 0.0) {
    if (-im == 0.0) {
      t1_re = -re / b_x[8].re;
      t1_im = 0.0;
    } else if (-re == 0.0) {
      t1_re = 0.0;
      t1_im = -im / b_x[8].re;
    } else {
      t1_re = -re / b_x[8].re;
      t1_im = -im / b_x[8].re;
    }
  } else if (b_x[8].re == 0.0) {
    if (-re == 0.0) {
      t1_re = -im / b_x[8].im;
      t1_im = 0.0;
    } else if (-im == 0.0) {
      t1_re = 0.0;
      t1_im = -(-re / b_x[8].im);
    } else {
      t1_re = -im / b_x[8].im;
      t1_im = -(-re / b_x[8].im);
    }
  } else {
    brm = std::abs(b_x[8].re);
    absx31 = std::abs(b_x[8].im);
    if (brm > absx31) {
      absx31 = b_x[8].im / b_x[8].re;
      d = b_x[8].re + absx31 * b_x[8].im;
      t1_re = (-re + absx31 * -im) / d;
      t1_im = (-im - absx31 * -re) / d;
    } else if (absx31 == brm) {
      if (b_x[8].re > 0.0) {
        absx31 = 0.5;
      } else {
        absx31 = -0.5;
      }
      if (b_x[8].im > 0.0) {
        d = 0.5;
      } else {
        d = -0.5;
      }
      t1_re = (-re * absx31 + -im * d) / brm;
      t1_im = (-im * absx31 - -re * d) / brm;
    } else {
      absx31 = b_x[8].re / b_x[8].im;
      d = b_x[8].im + absx31 * b_x[8].re;
      t1_re = (absx31 * -re - im) / d;
      t1_im = (absx31 * -im - (-re)) / d;
    }
  }
  absx11 = b_x[7].re * t1_re - b_x[7].im * t1_im;
  absx21 = b_x[7].re * t1_im + b_x[7].im * t1_re;
  if (b_x[4].im == 0.0) {
    if (0.0 - absx21 == 0.0) {
      t2_re = (1.0 - absx11) / b_x[4].re;
      t2_im = 0.0;
    } else if (1.0 - absx11 == 0.0) {
      t2_re = 0.0;
      t2_im = (0.0 - absx21) / b_x[4].re;
    } else {
      t2_re = (1.0 - absx11) / b_x[4].re;
      t2_im = (0.0 - absx21) / b_x[4].re;
    }
  } else if (b_x[4].re == 0.0) {
    if (1.0 - absx11 == 0.0) {
      t2_re = (0.0 - absx21) / b_x[4].im;
      t2_im = 0.0;
    } else if (0.0 - absx21 == 0.0) {
      t2_re = 0.0;
      t2_im = -((1.0 - absx11) / b_x[4].im);
    } else {
      t2_re = (0.0 - absx21) / b_x[4].im;
      t2_im = -((1.0 - absx11) / b_x[4].im);
    }
  } else {
    brm = std::abs(b_x[4].re);
    absx31 = std::abs(b_x[4].im);
    if (brm > absx31) {
      absx31 = b_x[4].im / b_x[4].re;
      d = b_x[4].re + absx31 * b_x[4].im;
      t2_re = ((1.0 - absx11) + absx31 * (0.0 - absx21)) / d;
      t2_im = ((0.0 - absx21) - absx31 * (1.0 - absx11)) / d;
    } else if (absx31 == brm) {
      if (b_x[4].re > 0.0) {
        absx31 = 0.5;
      } else {
        absx31 = -0.5;
      }
      if (b_x[4].im > 0.0) {
        d = 0.5;
      } else {
        d = -0.5;
      }
      t2_re = ((1.0 - absx11) * absx31 + (0.0 - absx21) * d) / brm;
      t2_im = ((0.0 - absx21) * absx31 - (1.0 - absx11) * d) / brm;
    } else {
      absx31 = b_x[4].re / b_x[4].im;
      d = b_x[4].im + absx31 * b_x[4].re;
      t2_re = (absx31 * (1.0 - absx11) + (0.0 - absx21)) / d;
      t2_im = (absx31 * (0.0 - absx21) - (1.0 - absx11)) / d;
    }
  }
  absx11 = -((b_x[3].re * t2_re - b_x[3].im * t2_im) +
             (b_x[6].re * t1_re - b_x[6].im * t1_im));
  absx21 = -((b_x[3].re * t2_im + b_x[3].im * t2_re) +
             (b_x[6].re * t1_im + b_x[6].im * t1_re));
  if (b_x[0].im == 0.0) {
    if (absx21 == 0.0) {
      y[p2].re = absx11 / b_x[0].re;
      y[p2].im = 0.0;
    } else if (absx11 == 0.0) {
      y[p2].re = 0.0;
      y[p2].im = absx21 / b_x[0].re;
    } else {
      y[p2].re = absx11 / b_x[0].re;
      y[p2].im = absx21 / b_x[0].re;
    }
  } else if (b_x[0].re == 0.0) {
    if (absx11 == 0.0) {
      y[p2].re = absx21 / b_x[0].im;
      y[p2].im = 0.0;
    } else if (absx21 == 0.0) {
      y[p2].re = 0.0;
      y[p2].im = -(absx11 / b_x[0].im);
    } else {
      y[p2].re = absx21 / b_x[0].im;
      y[p2].im = -(absx11 / b_x[0].im);
    }
  } else {
    brm = std::abs(b_x[0].re);
    absx31 = std::abs(b_x[0].im);
    if (brm > absx31) {
      absx31 = b_x[0].im / b_x[0].re;
      d = b_x[0].re + absx31 * b_x[0].im;
      y[p2].re = (absx11 + absx31 * absx21) / d;
      y[p2].im = (absx21 - absx31 * absx11) / d;
    } else if (absx31 == brm) {
      if (b_x[0].re > 0.0) {
        absx31 = 0.5;
      } else {
        absx31 = -0.5;
      }
      if (b_x[0].im > 0.0) {
        d = 0.5;
      } else {
        d = -0.5;
      }
      y[p2].re = (absx11 * absx31 + absx21 * d) / brm;
      y[p2].im = (absx21 * absx31 - absx11 * d) / brm;
    } else {
      absx31 = b_x[0].re / b_x[0].im;
      d = b_x[0].im + absx31 * b_x[0].re;
      y[p2].re = (absx31 * absx11 + absx21) / d;
      y[p2].im = (absx31 * absx21 - absx11) / d;
    }
  }
  y[p2 + 1].re = t2_re;
  y[p2 + 1].im = t2_im;
  y[p2 + 2].re = t1_re;
  y[p2 + 2].im = t1_im;
  if (b_x[8].im == 0.0) {
    t1_re = 1.0 / b_x[8].re;
    t1_im = 0.0;
  } else if (b_x[8].re == 0.0) {
    t1_re = 0.0;
    t1_im = -(1.0 / b_x[8].im);
  } else {
    brm = std::abs(b_x[8].re);
    absx31 = std::abs(b_x[8].im);
    if (brm > absx31) {
      absx31 = b_x[8].im / b_x[8].re;
      d = b_x[8].re + absx31 * b_x[8].im;
      t1_re = (absx31 * 0.0 + 1.0) / d;
      t1_im = (0.0 - absx31) / d;
    } else if (absx31 == brm) {
      if (b_x[8].re > 0.0) {
        absx31 = 0.5;
      } else {
        absx31 = -0.5;
      }
      if (b_x[8].im > 0.0) {
        d = 0.5;
      } else {
        d = -0.5;
      }
      t1_re = (absx31 + 0.0 * d) / brm;
      t1_im = (0.0 * absx31 - d) / brm;
    } else {
      absx31 = b_x[8].re / b_x[8].im;
      d = b_x[8].im + absx31 * b_x[8].re;
      t1_re = absx31 / d;
      t1_im = (absx31 * 0.0 - 1.0) / d;
    }
  }
  absx11 = -b_x[7].re * t1_re - -b_x[7].im * t1_im;
  absx21 = -b_x[7].re * t1_im + -b_x[7].im * t1_re;
  if (b_x[4].im == 0.0) {
    if (absx21 == 0.0) {
      t2_re = absx11 / b_x[4].re;
      t2_im = 0.0;
    } else if (absx11 == 0.0) {
      t2_re = 0.0;
      t2_im = absx21 / b_x[4].re;
    } else {
      t2_re = absx11 / b_x[4].re;
      t2_im = absx21 / b_x[4].re;
    }
  } else if (b_x[4].re == 0.0) {
    if (absx11 == 0.0) {
      t2_re = absx21 / b_x[4].im;
      t2_im = 0.0;
    } else if (absx21 == 0.0) {
      t2_re = 0.0;
      t2_im = -(absx11 / b_x[4].im);
    } else {
      t2_re = absx21 / b_x[4].im;
      t2_im = -(absx11 / b_x[4].im);
    }
  } else {
    brm = std::abs(b_x[4].re);
    absx31 = std::abs(b_x[4].im);
    if (brm > absx31) {
      absx31 = b_x[4].im / b_x[4].re;
      d = b_x[4].re + absx31 * b_x[4].im;
      t2_re = (absx11 + absx31 * absx21) / d;
      t2_im = (absx21 - absx31 * absx11) / d;
    } else if (absx31 == brm) {
      if (b_x[4].re > 0.0) {
        absx31 = 0.5;
      } else {
        absx31 = -0.5;
      }
      if (b_x[4].im > 0.0) {
        d = 0.5;
      } else {
        d = -0.5;
      }
      t2_re = (absx11 * absx31 + absx21 * d) / brm;
      t2_im = (absx21 * absx31 - absx11 * d) / brm;
    } else {
      absx31 = b_x[4].re / b_x[4].im;
      d = b_x[4].im + absx31 * b_x[4].re;
      t2_re = (absx31 * absx11 + absx21) / d;
      t2_im = (absx31 * absx21 - absx11) / d;
    }
  }
  absx11 = -((b_x[3].re * t2_re - b_x[3].im * t2_im) +
             (b_x[6].re * t1_re - b_x[6].im * t1_im));
  absx21 = -((b_x[3].re * t2_im + b_x[3].im * t2_re) +
             (b_x[6].re * t1_im + b_x[6].im * t1_re));
  if (b_x[0].im == 0.0) {
    if (absx21 == 0.0) {
      y[p3].re = absx11 / b_x[0].re;
      y[p3].im = 0.0;
    } else if (absx11 == 0.0) {
      y[p3].re = 0.0;
      y[p3].im = absx21 / b_x[0].re;
    } else {
      y[p3].re = absx11 / b_x[0].re;
      y[p3].im = absx21 / b_x[0].re;
    }
  } else if (b_x[0].re == 0.0) {
    if (absx11 == 0.0) {
      y[p3].re = absx21 / b_x[0].im;
      y[p3].im = 0.0;
    } else if (absx21 == 0.0) {
      y[p3].re = 0.0;
      y[p3].im = -(absx11 / b_x[0].im);
    } else {
      y[p3].re = absx21 / b_x[0].im;
      y[p3].im = -(absx11 / b_x[0].im);
    }
  } else {
    brm = std::abs(b_x[0].re);
    absx31 = std::abs(b_x[0].im);
    if (brm > absx31) {
      absx31 = b_x[0].im / b_x[0].re;
      d = b_x[0].re + absx31 * b_x[0].im;
      y[p3].re = (absx11 + absx31 * absx21) / d;
      y[p3].im = (absx21 - absx31 * absx11) / d;
    } else if (absx31 == brm) {
      if (b_x[0].re > 0.0) {
        absx31 = 0.5;
      } else {
        absx31 = -0.5;
      }
      if (b_x[0].im > 0.0) {
        d = 0.5;
      } else {
        d = -0.5;
      }
      y[p3].re = (absx11 * absx31 + absx21 * d) / brm;
      y[p3].im = (absx21 * absx31 - absx11 * d) / brm;
    } else {
      absx31 = b_x[0].re / b_x[0].im;
      d = b_x[0].im + absx31 * b_x[0].re;
      y[p3].re = (absx31 * absx11 + absx21) / d;
      y[p3].im = (absx31 * absx21 - absx11) / d;
    }
  }
  y[p3 + 1].re = t2_re;
  y[p3 + 1].im = t2_im;
  y[p3 + 2].re = t1_re;
  y[p3 + 2].im = t1_im;
}

} // namespace coder

// End of code generation (inv.cpp)
