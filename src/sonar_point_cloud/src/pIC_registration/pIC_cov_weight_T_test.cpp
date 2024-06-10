//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// pIC_cov_weight_T_test.cpp
//
// Code generation for function 'pIC_cov_weight_T_test'
//

// Include files
#include "pIC_cov_weight_T_test.h"
#include "chol.h"
#include "eig.h"
#include "eml_mtimes_helper.h"
#include "inv.h"
#include "mtimes.h"
#include "pIC_registration_data.h"
#include "pIC_registration_rtwutil.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <cmath>

// Function Declarations
static void binary_expand_op_2(coder::array<creal_T, 2U> &in1,
                               const coder::array<double, 2U> &in2,
                               const coder::array<creal_T, 2U> &in3,
                               const coder::array<double, 2U> &in4);

static void binary_expand_op_3(
    creal_T in1[9], const double in2[9], const coder::array<creal_T, 2U> &in3,
    int in4, const signed char in5[2], const signed char in6[2], int in7,
    const coder::array<double, 2U> &in8, const signed char in9[2],
    const signed char in10[2], const coder::array<double, 2U> &in11,
    const signed char in12[2], const signed char in13[2]);

// Function Definitions
static void binary_expand_op_2(coder::array<creal_T, 2U> &in1,
                               const coder::array<double, 2U> &in2,
                               const coder::array<creal_T, 2U> &in3,
                               const coder::array<double, 2U> &in4)
{
  coder::array<creal_T, 2U> b_in4;
  coder::array<creal_T, 2U> r;
  int aux_0_1;
  int aux_1_1;
  int b_loop_ub;
  int loop_ub;
  int stride_1_1;
  b_in4.set_size(in4.size(0), in4.size(1));
  loop_ub = in4.size(0) * in4.size(1);
  for (int i{0}; i < loop_ub; i++) {
    b_in4[i].re = in4[i];
    b_in4[i].im = 0.0;
  }
  in1.set_size(3, b_in4.size(1));
  loop_ub = b_in4.size(1);
  for (int i{0}; i < 3; i++) {
    for (stride_1_1 = 0; stride_1_1 < loop_ub; stride_1_1++) {
      in1[i + 3 * stride_1_1].re = 0.0;
      in1[i + 3 * stride_1_1].im = 0.0;
      b_loop_ub = in3.size(1);
      for (aux_0_1 = 0; aux_0_1 < b_loop_ub; aux_0_1++) {
        double b_in3_re_tmp;
        double c_in3_re_tmp;
        double d_in3_re_tmp;
        double in3_re_tmp;
        in3_re_tmp = in3[i + 3 * aux_0_1].re;
        b_in3_re_tmp = b_in4[aux_0_1 + b_in4.size(0) * stride_1_1].im;
        c_in3_re_tmp = in3[i + 3 * aux_0_1].im;
        d_in3_re_tmp = b_in4[aux_0_1 + b_in4.size(0) * stride_1_1].re;
        in1[i + 3 * stride_1_1].re =
            in1[i + 3 * stride_1_1].re +
            (in3_re_tmp * d_in3_re_tmp - c_in3_re_tmp * b_in3_re_tmp);
        in1[i + 3 * stride_1_1].im =
            in1[i + 3 * stride_1_1].im +
            (in3_re_tmp * b_in3_re_tmp + c_in3_re_tmp * d_in3_re_tmp);
      }
    }
  }
  if (in1.size(1) == 1) {
    loop_ub = in2.size(1);
  } else {
    loop_ub = in1.size(1);
  }
  r.set_size(3, loop_ub);
  b_loop_ub = (in2.size(1) != 1);
  stride_1_1 = (in1.size(1) != 1);
  aux_0_1 = 0;
  aux_1_1 = 0;
  for (int i{0}; i < loop_ub; i++) {
    r[3 * i].re = 2.0 * in2[3 * aux_0_1] + 2.0 * in1[3 * aux_1_1].re;
    r[3 * i].im = 2.0 * in1[3 * aux_1_1].im;
    r[3 * i + 1].re =
        2.0 * in2[3 * aux_0_1 + 1] + 2.0 * in1[3 * aux_1_1 + 1].re;
    r[3 * i + 1].im = 2.0 * in1[3 * aux_1_1 + 1].im;
    r[3 * i + 2].re =
        2.0 * in2[3 * aux_0_1 + 2] + 2.0 * in1[3 * aux_1_1 + 2].re;
    r[3 * i + 2].im = 2.0 * in1[3 * aux_1_1 + 2].im;
    aux_1_1 += stride_1_1;
    aux_0_1 += b_loop_ub;
  }
  in1.set_size(3, r.size(1));
  loop_ub = r.size(1);
  for (int i{0}; i < loop_ub; i++) {
    in1[3 * i] = r[3 * i];
    in1[3 * i + 1] = r[3 * i + 1];
    in1[3 * i + 2] = r[3 * i + 2];
  }
}

static void binary_expand_op_3(
    creal_T in1[9], const double in2[9], const coder::array<creal_T, 2U> &in3,
    int in4, const signed char in5[2], const signed char in6[2], int in7,
    const coder::array<double, 2U> &in8, const signed char in9[2],
    const signed char in10[2], const coder::array<double, 2U> &in11,
    const signed char in12[2], const signed char in13[2])
{
  coder::array<creal_T, 2U> b_in3;
  coder::array<creal_T, 2U> r1;
  coder::array<double, 2U> r;
  int i1;
  int in10_idx_0;
  int in12_idx_0;
  int in4_idx_1;
  int in6_idx_0;
  int in9_idx_0;
  in4_idx_1 = in5[1];
  in6_idx_0 = in6[0];
  in9_idx_0 = in9[0];
  in10_idx_0 = in10[0];
  in12_idx_0 = in12[0];
  r.set_size(((in6_idx_0 + in9_idx_0) + in10_idx_0) + in12_idx_0, in7);
  for (int i{0}; i < in7; i++) {
    for (i1 = 0; i1 < in6_idx_0; i1++) {
      r[i1 + r.size(0) * i] = 0.0;
    }
    for (i1 = 0; i1 < in9_idx_0; i1++) {
      r[(i1 + in6_idx_0) + r.size(0) * i] = in8[i1 + in9_idx_0 * i];
    }
    for (i1 = 0; i1 < in10_idx_0; i1++) {
      r[((i1 + in6_idx_0) + in9_idx_0) + r.size(0) * i] = 0.0;
    }
    for (i1 = 0; i1 < in12_idx_0; i1++) {
      r[(((i1 + in6_idx_0) + in9_idx_0) + in10_idx_0) + r.size(0) * i] =
          in11[i1 + in12_idx_0 * i];
    }
  }
  in6_idx_0 = in13[1];
  r1.set_size(in4, in4_idx_1 + in6_idx_0);
  for (int i{0}; i < in4_idx_1; i++) {
    for (i1 = 0; i1 < in4; i1++) {
      r1[i1 + r1.size(0) * i].re = 0.0;
      r1[i1 + r1.size(0) * i].im = 0.0;
    }
  }
  for (int i{0}; i < in6_idx_0; i++) {
    for (i1 = 0; i1 < in4; i1++) {
      in10_idx_0 = i + in4_idx_1;
      r1[i1 + r1.size(0) * in10_idx_0].re = r[i1 + in4 * i];
      r1[i1 + r1.size(0) * in10_idx_0].im = 0.0;
    }
  }
  b_in3.set_size(3, r1.size(1));
  in9_idx_0 = r1.size(1);
  for (int i{0}; i < 3; i++) {
    for (i1 = 0; i1 < in9_idx_0; i1++) {
      b_in3[i + 3 * i1].re = 0.0;
      b_in3[i + 3 * i1].im = 0.0;
      in6_idx_0 = in3.size(1);
      for (in10_idx_0 = 0; in10_idx_0 < in6_idx_0; in10_idx_0++) {
        double b_in3_re_tmp;
        double c_in3_re_tmp;
        double d_in3_re_tmp;
        double in3_re_tmp;
        in3_re_tmp = in3[i + 3 * in10_idx_0].re;
        b_in3_re_tmp = r1[in10_idx_0 + r1.size(0) * i1].im;
        c_in3_re_tmp = in3[i + 3 * in10_idx_0].im;
        d_in3_re_tmp = r1[in10_idx_0 + r1.size(0) * i1].re;
        b_in3[i + 3 * i1].re =
            b_in3[i + 3 * i1].re +
            (in3_re_tmp * d_in3_re_tmp - c_in3_re_tmp * b_in3_re_tmp);
        b_in3[i + 3 * i1].im =
            b_in3[i + 3 * i1].im +
            (in3_re_tmp * b_in3_re_tmp + c_in3_re_tmp * d_in3_re_tmp);
      }
    }
  }
  in6_idx_0 = (b_in3.size(1) != 1);
  in9_idx_0 = 0;
  for (int i{0}; i < 3; i++) {
    in1[3 * i].re = 2.0 * in2[3 * i] + 2.0 * b_in3[3 * in9_idx_0].re;
    in1[3 * i].im = 2.0 * b_in3[3 * in9_idx_0].im;
    i1 = 3 * i + 1;
    in1[i1].re = 2.0 * in2[i1] + 2.0 * b_in3[3 * in9_idx_0 + 1].re;
    in1[i1].im = 2.0 * b_in3[3 * in9_idx_0 + 1].im;
    i1 = 3 * i + 2;
    in1[i1].re = 2.0 * in2[i1] + 2.0 * b_in3[3 * in9_idx_0 + 2].re;
    in1[i1].im = 2.0 * b_in3[3 * in9_idx_0 + 2].im;
    in9_idx_0 += in6_idx_0;
  }
}

void pIC_cov_weight_T_test(const double theta[3],
                           const coder::array<double, 2U> &Sr,
                           const coder::array<double, 2U> &assoc_n,
                           const coder::array<double, 3U> &assoc_Pn,
                           const coder::array<double, 2U> &assoc_a,
                           const coder::array<double, 3U> &assoc_Pa,
                           creal_T cov[9])
{
  static const signed char B[9]{1, 0, 0, 0, 1, 0, 0, 0, 1};
  coder::array<creal_T, 2U> A;
  coder::array<creal_T, 2U> E;
  coder::array<creal_T, 2U> K;
  coder::array<creal_T, 2U> L;
  coder::array<creal_T, 2U> Lambda;
  coder::array<creal_T, 2U> b_K;
  coder::array<creal_T, 2U> dg_dX;
  coder::array<creal_T, 1U> R_hat;
  coder::array<creal_T, 1U> c_assoc_a;
  coder::array<double, 2U> JHx;
  coder::array<double, 2U> J_X;
  coder::array<double, 2U> J_theta;
  coder::array<double, 2U> b_assoc_a;
  coder::array<double, 2U> ci;
  coder::array<double, 2U> col33;
  coder::array<double, 2U> col63;
  coder::array<double, 2U> r;
  coder::array<double, 2U> r1;
  creal_T b_dg_dtheta[9];
  creal_T dg_dtheta[9];
  double C[9];
  double absxi;
  double absxr;
  double cosy_tmp;
  double siny_tmp;
  double w1_re;
  double w3_re;
  double w4_re;
  double xi;
  double xr;
  int b_i;
  int coffset;
  int i;
  int kidx;
  int loop_ub_tmp;
  int nx;
  int result;
  int u0;
  unsigned int w1_re_tmp_tmp;
  signed char b_input_sizes[2];
  signed char b_sizes[2];
  signed char c_input_sizes[2];
  signed char d_input_sizes[2];
  signed char input_sizes[2];
  signed char sizes[2];
  bool empty_non_axis_sizes;
  coder::inv(Sr, J_X);
  kidx = coder::internal::chol(J_X, nx);
  if (nx < 1) {
    loop_ub_tmp = 0;
  } else {
    loop_ub_tmp = nx;
  }
  L.set_size(loop_ub_tmp, loop_ub_tmp);
  for (i = 0; i < loop_ub_tmp; i++) {
    for (int j{0}; j < loop_ub_tmp; j++) {
      L[j + L.size(0) * i].re = J_X[j + J_X.size(0) * i];
      L[j + L.size(0) * i].im = 0.0;
    }
  }
  if (kidx != 0) {
    coder::eig(Sr, E, Lambda);
    // error('Sigma must be positive semi-definite.')
    nx = Lambda.size(0) * Lambda.size(1);
    for (loop_ub_tmp = 0; loop_ub_tmp < nx; loop_ub_tmp++) {
      xr = Lambda[loop_ub_tmp].re;
      xi = Lambda[loop_ub_tmp].im;
      if (xi == 0.0) {
        if (xr < 0.0) {
          absxr = 0.0;
          absxi = std::sqrt(-xr);
        } else {
          absxr = std::sqrt(xr);
          absxi = 0.0;
        }
      } else if (xr == 0.0) {
        if (xi < 0.0) {
          absxr = std::sqrt(-xi / 2.0);
          absxi = -absxr;
        } else {
          absxr = std::sqrt(xi / 2.0);
          absxi = absxr;
        }
      } else if (std::isnan(xr)) {
        absxr = rtNaN;
        absxi = rtNaN;
      } else if (std::isnan(xi)) {
        absxr = rtNaN;
        absxi = rtNaN;
      } else if (std::isinf(xi)) {
        absxr = std::abs(xi);
        absxi = xi;
      } else if (std::isinf(xr)) {
        if (xr < 0.0) {
          absxr = 0.0;
          absxi = xi * -xr;
        } else {
          absxr = xr;
          absxi = 0.0;
        }
      } else {
        absxr = std::abs(xr);
        absxi = std::abs(xi);
        if ((absxr > 4.4942328371557893E+307) ||
            (absxi > 4.4942328371557893E+307)) {
          absxr *= 0.5;
          absxi = rt_hypotd_snf(absxr, absxi * 0.5);
          if (absxi > absxr) {
            absxr = std::sqrt(absxi) * std::sqrt(absxr / absxi + 1.0);
          } else {
            absxr = std::sqrt(absxi) * 1.4142135623730951;
          }
        } else {
          absxr = std::sqrt((rt_hypotd_snf(absxr, absxi) + absxr) * 0.5);
        }
        if (xr > 0.0) {
          absxi = 0.5 * (xi / absxr);
        } else {
          if (xi < 0.0) {
            absxi = -absxr;
          } else {
            absxi = absxr;
          }
          absxr = 0.5 * (xi / absxi);
        }
      }
      Lambda[loop_ub_tmp].re = absxr;
      Lambda[loop_ub_tmp].im = absxi;
    }
    coder::internal::blas::mtimes(Lambda, E, L);
  }
  //  points and weights
  //  a_x = assoc.a(1,:);
  //  a_y = assoc.a(2,:);
  //  n_x = assoc.n(1,:);
  //  n_y = assoc.n(2,:);
  //  w1 = L(2*i-1,2*i-1);
  //  w2 = L(2*i-1,2*i);
  //  w3 = L(2*i,2*i-1);
  //  w4 = L(2*i,2*i);
  //  residuals
  cosy_tmp = std::cos(theta[2]);
  siny_tmp = std::sin(theta[2]);
  // x,y
  ci.set_size(2, assoc_n.size(1));
  kidx = assoc_n.size(1) << 1;
  for (i = 0; i < kidx; i++) {
    ci[i] = 0.0;
  }
  i = assoc_n.size(1);
  for (b_i = 0; b_i < i; b_i++) {
    absxi = assoc_n[2 * b_i];
    absxr = assoc_n[2 * b_i + 1];
    ci[2 * b_i] = (absxi * cosy_tmp - absxr * siny_tmp) + theta[0];
    ci[2 * b_i + 1] = (absxi * siny_tmp + absxr * cosy_tmp) + theta[1];
  }
  if (assoc_a.size(1) == ci.size(1)) {
    b_assoc_a.set_size(2, assoc_a.size(1));
    loop_ub_tmp = assoc_a.size(1) << 1;
    for (i = 0; i < loop_ub_tmp; i++) {
      b_assoc_a[i] = assoc_a[i] - ci[i];
    }
    c_assoc_a.set_size(loop_ub_tmp);
    for (i = 0; i < loop_ub_tmp; i++) {
      c_assoc_a[i].re = b_assoc_a[i];
      c_assoc_a[i].im = 0.0;
    }
    R_hat.set_size(L.size(0));
    kidx = L.size(0);
    for (i = 0; i < kidx; i++) {
      R_hat[i].re = 0.0;
      R_hat[i].im = 0.0;
      nx = L.size(1);
      for (int j{0}; j < nx; j++) {
        absxi = L[i + L.size(0) * j].re;
        absxr = c_assoc_a[j].im;
        xr = L[i + L.size(0) * j].im;
        xi = c_assoc_a[j].re;
        R_hat[i].re = R_hat[i].re + (absxi * xi - xr * absxr);
        R_hat[i].im = R_hat[i].im + (absxi * absxr + xr * xi);
      }
    }
  } else {
    binary_expand_op_4(R_hat, L, assoc_a, ci);
  }
  //  L = Sr;
  //  R_hat = R;
  J_theta.set_size(2 * assoc_a.size(1), 3);
  loop_ub_tmp = 2 * assoc_a.size(1) * 3;
  for (i = 0; i < loop_ub_tmp; i++) {
    J_theta[i] = 0.0;
  }
  //  Preallocate memory for efficiency
  i = assoc_a.size(1);
  for (b_i = 0; b_i < i; b_i++) {
    // Jacobian d_Ri/d_Theta
    //  J_theta =
    //
    //  [ -w1, -w2, w1*(n_y*cos(q_th) + n_x*sin(q_th)) - w2*(n_x*cos(q_th) -
    //  n_y*sin(q_th))] [ -w3, -w4, w3*(n_y*cos(q_th) + n_x*sin(q_th)) -
    //  w4*(n_x*cos(q_th) - n_y*sin(q_th))]
    // constants
    w1_re_tmp_tmp = static_cast<unsigned int>(b_i + 1) << 1;
    w1_re = L[(static_cast<int>(w1_re_tmp_tmp) +
               L.size(0) * (static_cast<int>(w1_re_tmp_tmp) - 2)) -
              2]
                .re;
    xi = L[(static_cast<int>(w1_re_tmp_tmp) +
            L.size(0) * (static_cast<int>(w1_re_tmp_tmp) - 1)) -
           2]
             .re;
    w3_re = L[(static_cast<int>(w1_re_tmp_tmp) +
               L.size(0) * (static_cast<int>(w1_re_tmp_tmp) - 2)) -
              1]
                .re;
    w4_re = L[(static_cast<int>(w1_re_tmp_tmp) +
               L.size(0) * (static_cast<int>(w1_re_tmp_tmp) - 1)) -
              1]
                .re;
    absxi = assoc_n[2 * b_i];
    xr = assoc_n[2 * b_i + 1];
    // Jacobian d_Ri/d_Theta
    J_theta[static_cast<int>(w1_re_tmp_tmp) - 2] = -w1_re;
    J_theta[(static_cast<int>(w1_re_tmp_tmp) + J_theta.size(0)) - 2] = -xi;
    absxr = xr * cosy_tmp + absxi * siny_tmp;
    absxi = absxi * cosy_tmp - xr * siny_tmp;
    J_theta[(static_cast<int>(w1_re_tmp_tmp) + J_theta.size(0) * 2) - 2] =
        w1_re * absxr - xi * absxi;
    J_theta[static_cast<int>(w1_re_tmp_tmp - 1U)] = -w3_re;
    J_theta[(static_cast<int>(w1_re_tmp_tmp) + J_theta.size(0)) - 1] = -w4_re;
    J_theta[(static_cast<int>(w1_re_tmp_tmp) + J_theta.size(0) * 2) - 1] =
        w3_re * absxr - w4_re * absxi;
  }
  // Jacobian d_Ri/d_X
  //  Xi = [assoc.a; assoc.n];
  //  X = reshape(Xi,4*Nmp,1);
  //  J_X = jacobian(Ri, Ui);
  //  J_X =
  //  [ w1, w2, - w1*cos(q_th) - w2*sin(q_th), w1*sin(q_th) - w2*cos(q_th)]
  //  [ w3, w4, - w3*cos(q_th) - w4*sin(q_th), w3*sin(q_th) - w4*cos(q_th)]
  J_X.set_size(2 * assoc_a.size(1), 4 * assoc_a.size(1));
  kidx = 2 * assoc_a.size(1) * (4 * assoc_a.size(1));
  for (i = 0; i < kidx; i++) {
    J_X[i] = 0.0;
  }
  i = assoc_a.size(1);
  for (b_i = 0; b_i < i; b_i++) {
    // constants (to check)
    nx = static_cast<int>(static_cast<unsigned int>(b_i + 1) << 1);
    w1_re = L[(nx + L.size(0) * (nx - 2)) - 2].re;
    xi = L[(nx + L.size(0) * (nx - 1)) - 2].re;
    w3_re = L[(nx + L.size(0) * (nx - 2)) - 1].re;
    w4_re = L[(nx + L.size(0) * (nx - 1)) - 1].re;
    w1_re_tmp_tmp = (static_cast<unsigned int>(b_i) << 1) + 1U;
    absxi = static_cast<double>(b_i) * 4.0 + 1.0;
    J_X[(static_cast<int>(w1_re_tmp_tmp) +
         J_X.size(0) * (static_cast<int>(absxi) - 1)) -
        1] = w1_re;
    J_X[(static_cast<int>(w1_re_tmp_tmp) +
         J_X.size(0) * (static_cast<int>(absxi + 1.0) - 1)) -
        1] = xi;
    J_X[(static_cast<int>(w1_re_tmp_tmp) +
         J_X.size(0) * (static_cast<int>(absxi + 2.0) - 1)) -
        1] = -w1_re * cosy_tmp - xi * siny_tmp;
    J_X[(static_cast<int>(w1_re_tmp_tmp) +
         J_X.size(0) * (static_cast<int>(absxi + 3.0) - 1)) -
        1] = w1_re * siny_tmp - xi * cosy_tmp;
    J_X[static_cast<int>(w1_re_tmp_tmp) +
        J_X.size(0) * (static_cast<int>(absxi) - 1)] = w3_re;
    J_X[static_cast<int>(w1_re_tmp_tmp) +
        J_X.size(0) * (static_cast<int>(absxi + 1.0) - 1)] = w4_re;
    J_X[static_cast<int>(w1_re_tmp_tmp) +
        J_X.size(0) * (static_cast<int>(absxi + 2.0) - 1)] =
        -w3_re * cosy_tmp - w4_re * siny_tmp;
    J_X[static_cast<int>(w1_re_tmp_tmp) +
        J_X.size(0) * (static_cast<int>(absxi + 3.0) - 1)] =
        w3_re * siny_tmp - w4_re * cosy_tmp;
    //      J_X(i*2+1:i*2+2,i*4+1:i*4+4) = [L(2*(i+1)-1,2*(i+1)-1),
    //      L(2*(i+1)-1,2*(i+1)), - L(2*(i+1)-1,2*(i+1)-1)*cos(q_th) -
    //      L(2*(i+1)-1,2*(i+1))*sin(q_th), L(2*(i+1)-1,2*(i+1)-1)*sin(q_th) -
    //      L(2*(i+1)-1,2*(i+1))*cos(q_th);
    //                                      L(2*(i+1),2*(i+1)-1),
    //                                      L(2*(i+1),2*(i+1)), -
    //                                      L(2*(i+1),2*(i+1)-1)*cos(q_th) -
    //                                      L(2*(i+1),2*(i+1))*sin(q_th),
    //                                      L(2*(i+1),2*(i+1)-1)*sin(q_th) -
    //                                      L(2*(i+1),2*(i+1))*cos(q_th)];
  }
  //  Hessian Calculation from Jacobian
  //   with respect to theta%
  //  the jacobial from symbolics
  //  jacobian(J_theta,theta) =
  //  [ 0, 0, 0] [ 0, 0, w1*(n_x*cos(q_th) - n_y*sin(q_th)) + w2*(n_y*cos(q_th)
  //  + n_x*sin(q_th))] [ 0, 0, 0] [ 0, 0, w3*(n_x*cos(q_th) - n_y*sin(q_th)) +
  //  w4*(n_y*cos(q_th) + n_x*sin(q_th))]
  //
  col33.set_size(1, assoc_a.size(1));
  //  col33 = complex(1,Nmp);
  col63.set_size(1, assoc_a.size(1));
  //  col63 = complex(1,Nmp);
  i = assoc_a.size(1);
  for (b_i = 0; b_i < i; b_i++) {
    // constants
    absxi = assoc_n[2 * b_i];
    xr = assoc_n[2 * b_i + 1];
    absxr = absxi * cosy_tmp - xr * siny_tmp;
    absxi = xr * cosy_tmp + absxi * siny_tmp;
    nx = static_cast<int>(static_cast<unsigned int>(b_i + 1) << 1);
    col33[b_i] = L[(nx + L.size(0) * (nx - 2)) - 2].re * absxr +
                 L[(nx + L.size(0) * (nx - 1)) - 2].re * absxi;
    col63[b_i] = L[(nx + L.size(0) * (nx - 2)) - 1].re * absxr +
                 L[(nx + L.size(0) * (nx - 1)) - 1].re * absxi;
  }
  if (assoc_a.size(1) != 0) {
    result = assoc_a.size(1);
  } else if (col33.size(1) != 0) {
    result = col33.size(1);
  } else if (col63.size(1) != 0) {
    result = col63.size(1);
  } else {
    result = 0;
  }
  empty_non_axis_sizes = (result == 0);
  if (empty_non_axis_sizes || (assoc_a.size(1) != 0)) {
    input_sizes[0] = 2;
  } else {
    input_sizes[0] = 0;
  }
  if (empty_non_axis_sizes || (col33.size(1) != 0)) {
    b_input_sizes[0] = 1;
  } else {
    b_input_sizes[0] = 0;
  }
  if (empty_non_axis_sizes || (assoc_a.size(1) != 0)) {
    c_input_sizes[0] = 2;
  } else {
    c_input_sizes[0] = 0;
  }
  if (empty_non_axis_sizes || (col63.size(1) != 0)) {
    sizes[0] = 1;
  } else {
    sizes[0] = 0;
  }
  absxr = 6.0 * static_cast<double>(assoc_a.size(1));
  if (6 * assoc_a.size(1) != 0) {
    u0 = 6 * assoc_a.size(1);
  } else if (static_cast<int>(absxr) != 0) {
    u0 = static_cast<int>(absxr);
  } else {
    u0 = 6 * assoc_a.size(1);
    if (u0 < 0) {
      u0 = 0;
    }
    if (static_cast<int>(absxr) > u0) {
      u0 = static_cast<int>(absxr);
    }
  }
  empty_non_axis_sizes = (u0 == 0);
  if (empty_non_axis_sizes || (6 * assoc_a.size(1) != 0)) {
    d_input_sizes[1] = 2;
  } else {
    d_input_sizes[1] = 0;
  }
  if (empty_non_axis_sizes || (static_cast<int>(absxr) != 0)) {
    b_sizes[1] = 1;
  } else {
    b_sizes[1] = 0;
  }
  //  the Kronecker tensor product
  //  And the Hessian d(g)/d(theta)
  kidx = J_theta.size(0);
  for (int j{0}; j < 3; j++) {
    coffset = j * 3;
    nx = j * J_theta.size(0);
    C[coffset] = 0.0;
    C[coffset + 1] = 0.0;
    C[coffset + 2] = 0.0;
    for (loop_ub_tmp = 0; loop_ub_tmp < kidx; loop_ub_tmp++) {
      absxi = J_theta[nx + loop_ub_tmp];
      C[coffset] += J_theta[loop_ub_tmp] * absxi;
      C[coffset + 1] += J_theta[J_theta.size(0) + loop_ub_tmp] * absxi;
      C[coffset + 2] += J_theta[(J_theta.size(0) << 1) + loop_ub_tmp] * absxi;
    }
  }
  A.set_size(1, R_hat.size(0));
  kidx = R_hat.size(0);
  for (i = 0; i < kidx; i++) {
    A[i].re = R_hat[i].re;
    A[i].im = -R_hat[i].im;
  }
  nx = A.size(1);
  K.set_size(3, A.size(1) * 3);
  kidx = -1;
  for (coffset = 0; coffset < nx; coffset++) {
    absxi = A[coffset].re;
    absxr = A[coffset].im;
    for (b_i = 0; b_i < 3; b_i++) {
      i = B[3 * b_i];
      K[kidx + 1].re = static_cast<double>(i) * absxi;
      K[kidx + 1].im = static_cast<double>(i) * absxr;
      i = B[3 * b_i + 1];
      K[kidx + 2].re = static_cast<double>(i) * absxi;
      K[kidx + 2].im = static_cast<double>(i) * absxr;
      i = B[3 * b_i + 2];
      K[kidx + 3].re = static_cast<double>(i) * absxi;
      K[kidx + 3].im = static_cast<double>(i) * absxr;
      kidx += 3;
    }
  }
  if (d_input_sizes[1] + b_sizes[1] == 3) {
    nx = input_sizes[0];
    kidx = b_input_sizes[0];
    loop_ub_tmp = c_input_sizes[0];
    coffset = sizes[0];
    i = (input_sizes[0] + b_input_sizes[0]) + c_input_sizes[0];
    r.set_size(i + sizes[0], result);
    for (int j{0}; j < result; j++) {
      for (b_i = 0; b_i < nx; b_i++) {
        r[b_i + r.size(0) * j] = 0.0;
      }
      for (b_i = 0; b_i < kidx; b_i++) {
        r[nx + r.size(0) * j] = col33[kidx * j];
      }
      for (b_i = 0; b_i < loop_ub_tmp; b_i++) {
        r[((b_i + nx) + kidx) + r.size(0) * j] = 0.0;
      }
      for (b_i = 0; b_i < coffset; b_i++) {
        r[i + r.size(0) * j] = col63[coffset * j];
      }
    }
    E.set_size(u0, 3);
    kidx = d_input_sizes[1];
    for (i = 0; i < kidx; i++) {
      for (int j{0}; j < u0; j++) {
        E[j + E.size(0) * i].re = 0.0;
        E[j + E.size(0) * i].im = 0.0;
      }
    }
    kidx = b_sizes[1];
    for (i = 0; i < kidx; i++) {
      for (int j{0}; j < u0; j++) {
        E[j + E.size(0) * d_input_sizes[1]].re = r[j];
        E[j + E.size(0) * d_input_sizes[1]].im = 0.0;
      }
    }
    b_K.set_size(3, E.size(1));
    kidx = E.size(1);
    for (i = 0; i < 3; i++) {
      for (int j{0}; j < kidx; j++) {
        b_K[i + 3 * j].re = 0.0;
        b_K[i + 3 * j].im = 0.0;
        nx = K.size(1);
        for (b_i = 0; b_i < nx; b_i++) {
          xi = K[i + 3 * b_i].re;
          w3_re = E[b_i + E.size(0) * j].im;
          absxi = K[i + 3 * b_i].im;
          absxr = E[b_i + E.size(0) * j].re;
          b_K[i + 3 * j].re = b_K[i + 3 * j].re + (xi * absxr - absxi * w3_re);
          b_K[i + 3 * j].im = b_K[i + 3 * j].im + (xi * w3_re + absxi * absxr);
        }
      }
    }
    for (i = 0; i < 9; i++) {
      dg_dtheta[i].re = 2.0 * C[i] + 2.0 * b_K[i].re;
      dg_dtheta[i].im = 2.0 * b_K[i].im;
    }
  } else {
    binary_expand_op_3(dg_dtheta, C, K, u0, d_input_sizes, input_sizes, result,
                       col33, b_input_sizes, c_input_sizes, col63, sizes,
                       b_sizes);
  }
  //  Hessian Calculation from Jacobian
  //   with respect to X (point and matches)
  //   s2 = jacobian(J_theta,Ui) =
  //
  //  [ 0, 0,                           0,                           0]
  //  [ 0, 0, w1*sin(q_th) - w2*cos(q_th), w1*cos(q_th) + w2*sin(q_th)]
  //  [ 0, 0,                           0,                           0]
  //  [ 0, 0, w3*sin(q_th) - w4*cos(q_th), w3*cos(q_th) + w4*sin(q_th)]
  JHx.set_size(6 * assoc_a.size(1), 4 * assoc_a.size(1));
  loop_ub_tmp = 6 * assoc_a.size(1) * (4 * assoc_a.size(1));
  for (i = 0; i < loop_ub_tmp; i++) {
    JHx[i] = 0.0;
  }
  i = assoc_a.size(1);
  for (b_i = 0; b_i < i; b_i++) {
    // constants (to check)
    nx = static_cast<int>(static_cast<unsigned int>(b_i + 1) << 1);
    w1_re = L[(nx + L.size(0) * (nx - 2)) - 2].re;
    xi = L[(nx + L.size(0) * (nx - 1)) - 2].re;
    w3_re = L[(nx + L.size(0) * (nx - 2)) - 1].re;
    w4_re = L[(nx + L.size(0) * (nx - 1)) - 1].re;
    absxi = static_cast<double>(b_i) * 6.0 + 1.0;
    absxr = static_cast<double>(b_i) * 4.0 + 1.0;
    JHx[(static_cast<int>(absxi) +
         JHx.size(0) * (static_cast<int>(absxr) - 1)) -
        1] = 0.0;
    JHx[(static_cast<int>(absxi) +
         JHx.size(0) * (static_cast<int>(absxr + 1.0) - 1)) -
        1] = 0.0;
    JHx[(static_cast<int>(absxi) +
         JHx.size(0) * (static_cast<int>(absxr + 2.0) - 1)) -
        1] = 0.0;
    JHx[(static_cast<int>(absxi) +
         JHx.size(0) * (static_cast<int>(absxr + 3.0) - 1)) -
        1] = 0.0;
    JHx[(static_cast<int>(absxi + 1.0) +
         JHx.size(0) * (static_cast<int>(absxr) - 1)) -
        1] = 0.0;
    JHx[(static_cast<int>(absxi + 1.0) +
         JHx.size(0) * (static_cast<int>(absxr + 1.0) - 1)) -
        1] = 0.0;
    JHx[(static_cast<int>(absxi + 1.0) +
         JHx.size(0) * (static_cast<int>(absxr + 2.0) - 1)) -
        1] = 0.0;
    JHx[(static_cast<int>(absxi + 1.0) +
         JHx.size(0) * (static_cast<int>(absxr + 3.0) - 1)) -
        1] = 0.0;
    JHx[(static_cast<int>(absxi + 2.0) +
         JHx.size(0) * (static_cast<int>(absxr) - 1)) -
        1] = 0.0;
    JHx[(static_cast<int>(absxi + 2.0) +
         JHx.size(0) * (static_cast<int>(absxr + 1.0) - 1)) -
        1] = 0.0;
    JHx[(static_cast<int>(absxi + 2.0) +
         JHx.size(0) * (static_cast<int>(absxr + 2.0) - 1)) -
        1] = w1_re * siny_tmp - xi * cosy_tmp;
    JHx[(static_cast<int>(absxi + 2.0) +
         JHx.size(0) * (static_cast<int>(absxr + 3.0) - 1)) -
        1] = w1_re * cosy_tmp + xi * siny_tmp;
    JHx[(static_cast<int>(absxi + 3.0) +
         JHx.size(0) * (static_cast<int>(absxr) - 1)) -
        1] = 0.0;
    JHx[(static_cast<int>(absxi + 3.0) +
         JHx.size(0) * (static_cast<int>(absxr + 1.0) - 1)) -
        1] = 0.0;
    JHx[(static_cast<int>(absxi + 3.0) +
         JHx.size(0) * (static_cast<int>(absxr + 2.0) - 1)) -
        1] = 0.0;
    JHx[(static_cast<int>(absxi + 3.0) +
         JHx.size(0) * (static_cast<int>(absxr + 3.0) - 1)) -
        1] = 0.0;
    JHx[(static_cast<int>(absxi + 4.0) +
         JHx.size(0) * (static_cast<int>(absxr) - 1)) -
        1] = 0.0;
    JHx[(static_cast<int>(absxi + 4.0) +
         JHx.size(0) * (static_cast<int>(absxr + 1.0) - 1)) -
        1] = 0.0;
    JHx[(static_cast<int>(absxi + 4.0) +
         JHx.size(0) * (static_cast<int>(absxr + 2.0) - 1)) -
        1] = 0.0;
    JHx[(static_cast<int>(absxi + 4.0) +
         JHx.size(0) * (static_cast<int>(absxr + 3.0) - 1)) -
        1] = 0.0;
    JHx[(static_cast<int>(absxi + 5.0) +
         JHx.size(0) * (static_cast<int>(absxr) - 1)) -
        1] = 0.0;
    JHx[(static_cast<int>(absxi + 5.0) +
         JHx.size(0) * (static_cast<int>(absxr + 1.0) - 1)) -
        1] = 0.0;
    JHx[(static_cast<int>(absxi + 5.0) +
         JHx.size(0) * (static_cast<int>(absxr + 2.0) - 1)) -
        1] = w3_re * siny_tmp - w4_re * cosy_tmp;
    JHx[(static_cast<int>(absxi + 5.0) +
         JHx.size(0) * (static_cast<int>(absxr + 3.0) - 1)) -
        1] = w3_re * cosy_tmp + w4_re * siny_tmp;
  }
  A.set_size(1, R_hat.size(0));
  kidx = R_hat.size(0);
  for (i = 0; i < kidx; i++) {
    A[i].re = R_hat[i].re;
    A[i].im = -R_hat[i].im;
  }
  nx = A.size(1);
  K.set_size(3, A.size(1) * 3);
  kidx = -1;
  for (coffset = 0; coffset < nx; coffset++) {
    absxi = A[coffset].re;
    absxr = A[coffset].im;
    for (b_i = 0; b_i < 3; b_i++) {
      i = B[3 * b_i];
      K[kidx + 1].re = static_cast<double>(i) * absxi;
      K[kidx + 1].im = static_cast<double>(i) * absxr;
      i = B[3 * b_i + 1];
      K[kidx + 2].re = static_cast<double>(i) * absxi;
      K[kidx + 2].im = static_cast<double>(i) * absxr;
      i = B[3 * b_i + 2];
      K[kidx + 3].re = static_cast<double>(i) * absxi;
      K[kidx + 3].im = static_cast<double>(i) * absxr;
      kidx += 3;
    }
  }
  coder::internal::blas::mtimes(J_theta, J_X, r1);
  if (r1.size(1) == JHx.size(1)) {
    E.set_size(JHx.size(0), JHx.size(1));
    for (i = 0; i < loop_ub_tmp; i++) {
      E[i].re = JHx[i];
      E[i].im = 0.0;
    }
    b_K.set_size(3, E.size(1));
    kidx = E.size(1);
    for (i = 0; i < 3; i++) {
      for (int j{0}; j < kidx; j++) {
        b_K[i + 3 * j].re = 0.0;
        b_K[i + 3 * j].im = 0.0;
        nx = K.size(1);
        for (b_i = 0; b_i < nx; b_i++) {
          xi = K[i + 3 * b_i].re;
          w3_re = E[b_i + E.size(0) * j].im;
          absxi = K[i + 3 * b_i].im;
          absxr = E[b_i + E.size(0) * j].re;
          b_K[i + 3 * j].re = b_K[i + 3 * j].re + (xi * absxr - absxi * w3_re);
          b_K[i + 3 * j].im = b_K[i + 3 * j].im + (xi * w3_re + absxi * absxr);
        }
      }
    }
    dg_dX.set_size(3, r1.size(1));
    kidx = 3 * r1.size(1);
    for (i = 0; i < kidx; i++) {
      dg_dX[i].re = 2.0 * r1[i] + 2.0 * b_K[i].re;
      dg_dX[i].im = 2.0 * b_K[i].im;
    }
  } else {
    binary_expand_op_2(dg_dX, r1, K, JHx);
  }
  //  force dg_dtheta to be symmetric
  for (i = 0; i < 3; i++) {
    absxi = dg_dtheta[3 * i].re + dg_dtheta[i].re;
    absxr = dg_dtheta[3 * i].im - dg_dtheta[i].im;
    if (absxr == 0.0) {
      b_dg_dtheta[3 * i].re = absxi / 2.0;
      b_dg_dtheta[3 * i].im = 0.0;
    } else if (absxi == 0.0) {
      b_dg_dtheta[3 * i].re = 0.0;
      b_dg_dtheta[3 * i].im = absxr / 2.0;
    } else {
      b_dg_dtheta[3 * i].re = absxi / 2.0;
      b_dg_dtheta[3 * i].im = absxr / 2.0;
    }
    nx = 3 * i + 1;
    absxi = dg_dtheta[nx].re + dg_dtheta[i + 3].re;
    absxr = dg_dtheta[nx].im - dg_dtheta[i + 3].im;
    if (absxr == 0.0) {
      b_dg_dtheta[nx].re = absxi / 2.0;
      b_dg_dtheta[nx].im = 0.0;
    } else if (absxi == 0.0) {
      b_dg_dtheta[nx].re = 0.0;
      b_dg_dtheta[nx].im = absxr / 2.0;
    } else {
      b_dg_dtheta[nx].re = absxi / 2.0;
      b_dg_dtheta[nx].im = absxr / 2.0;
    }
    nx = 3 * i + 2;
    absxi = dg_dtheta[nx].re + dg_dtheta[i + 6].re;
    absxr = dg_dtheta[nx].im - dg_dtheta[i + 6].im;
    if (absxr == 0.0) {
      b_dg_dtheta[nx].re = absxi / 2.0;
      b_dg_dtheta[nx].im = 0.0;
    } else if (absxi == 0.0) {
      b_dg_dtheta[nx].re = 0.0;
      b_dg_dtheta[nx].im = absxr / 2.0;
    } else {
      b_dg_dtheta[nx].re = absxi / 2.0;
      b_dg_dtheta[nx].im = absxr / 2.0;
    }
  }
  coder::inv(b_dg_dtheta, dg_dtheta);
  J_X.set_size(4 * assoc_a.size(1), 4 * assoc_a.size(1));
  loop_ub_tmp = 4 * assoc_a.size(1) * (4 * assoc_a.size(1));
  for (i = 0; i < loop_ub_tmp; i++) {
    J_X[i] = 0.0;
  }
  i = assoc_a.size(1);
  for (b_i = 0; b_i < i; b_i++) {
    absxi = static_cast<double>(b_i) * 4.0;
    J_X[(static_cast<int>(absxi + 1.0) +
         J_X.size(0) * (static_cast<int>(absxi + 1.0) - 1)) -
        1] = assoc_Pa[4 * b_i];
    J_X[(static_cast<int>((absxi + 1.0) + 1.0) +
         J_X.size(0) * (static_cast<int>(absxi + 1.0) - 1)) -
        1] = assoc_Pa[4 * b_i + 1];
    J_X[(static_cast<int>(absxi + 1.0) +
         J_X.size(0) * (static_cast<int>((absxi + 1.0) + 1.0) - 1)) -
        1] = assoc_Pa[4 * b_i + 2];
    J_X[(static_cast<int>((absxi + 1.0) + 1.0) +
         J_X.size(0) * (static_cast<int>((absxi + 1.0) + 1.0) - 1)) -
        1] = assoc_Pa[4 * b_i + 3];
    J_X[(static_cast<int>(absxi + 3.0) +
         J_X.size(0) * (static_cast<int>(absxi + 3.0) - 1)) -
        1] = assoc_Pn[4 * b_i];
    J_X[(static_cast<int>((absxi + 3.0) + 1.0) +
         J_X.size(0) * (static_cast<int>(absxi + 3.0) - 1)) -
        1] = assoc_Pn[4 * b_i + 1];
    J_X[(static_cast<int>(absxi + 3.0) +
         J_X.size(0) * (static_cast<int>((absxi + 3.0) + 1.0) - 1)) -
        1] = assoc_Pn[4 * b_i + 2];
    J_X[(static_cast<int>((absxi + 3.0) + 1.0) +
         J_X.size(0) * (static_cast<int>((absxi + 3.0) + 1.0) - 1)) -
        1] = assoc_Pn[4 * b_i + 3];
  }
  //  A = dg_dtheta \ dg_dX;
  //  B = dg_dX' / dg_dtheta;
  //  cov = A * Sx * B;
  coder::internal::blas::mtimes(dg_dtheta, dg_dX, K);
  E.set_size(J_X.size(0), J_X.size(1));
  for (i = 0; i < loop_ub_tmp; i++) {
    E[i].re = J_X[i];
    E[i].im = 0.0;
  }
  b_K.set_size(3, E.size(1));
  kidx = E.size(1);
  for (i = 0; i < 3; i++) {
    for (int j{0}; j < kidx; j++) {
      b_K[i + 3 * j].re = 0.0;
      b_K[i + 3 * j].im = 0.0;
      nx = K.size(1);
      for (b_i = 0; b_i < nx; b_i++) {
        xi = K[i + 3 * b_i].re;
        w3_re = E[b_i + E.size(0) * j].im;
        absxi = K[i + 3 * b_i].im;
        absxr = E[b_i + E.size(0) * j].re;
        b_K[i + 3 * j].re = b_K[i + 3 * j].re + (xi * absxr - absxi * w3_re);
        b_K[i + 3 * j].im = b_K[i + 3 * j].im + (xi * w3_re + absxi * absxr);
      }
    }
  }
  kidx = b_K.size(1);
  for (int j{0}; j < 3; j++) {
    coffset = j * 3;
    for (b_i = 0; b_i < 3; b_i++) {
      w1_re = 0.0;
      absxi = 0.0;
      for (loop_ub_tmp = 0; loop_ub_tmp < kidx; loop_ub_tmp++) {
        nx = loop_ub_tmp * 3 + j;
        absxr = dg_dX[nx].re;
        xr = -dg_dX[nx].im;
        nx = loop_ub_tmp * 3 + b_i;
        xi = b_K[nx].re;
        w3_re = b_K[nx].im;
        w1_re += xi * absxr - w3_re * xr;
        absxi += xi * xr + w3_re * absxr;
      }
      i = coffset + b_i;
      b_dg_dtheta[i].re = w1_re;
      b_dg_dtheta[i].im = absxi;
    }
  }
  for (i = 0; i < 3; i++) {
    absxi = b_dg_dtheta[i].re;
    absxr = b_dg_dtheta[i].im;
    xr = b_dg_dtheta[i + 3].re;
    xi = b_dg_dtheta[i + 3].im;
    w3_re = b_dg_dtheta[i + 6].re;
    w4_re = b_dg_dtheta[i + 6].im;
    for (int j{0}; j < 3; j++) {
      double b_dg_dtheta_re_tmp;
      double c_dg_dtheta_re_tmp;
      double dg_dtheta_re_tmp;
      w1_re = dg_dtheta[3 * j].im;
      cosy_tmp = dg_dtheta[3 * j].re;
      nx = 3 * j + 1;
      siny_tmp = dg_dtheta[nx].im;
      dg_dtheta_re_tmp = dg_dtheta[nx].re;
      nx = 3 * j + 2;
      b_dg_dtheta_re_tmp = dg_dtheta[nx].im;
      c_dg_dtheta_re_tmp = dg_dtheta[nx].re;
      b_i = i + 3 * j;
      cov[b_i].re = ((absxi * cosy_tmp - absxr * w1_re) +
                     (xr * dg_dtheta_re_tmp - xi * siny_tmp)) +
                    (w3_re * c_dg_dtheta_re_tmp - w4_re * b_dg_dtheta_re_tmp);
      cov[b_i].im = ((absxi * w1_re + absxr * cosy_tmp) +
                     (xr * siny_tmp + xi * dg_dtheta_re_tmp)) +
                    (w3_re * b_dg_dtheta_re_tmp + w4_re * c_dg_dtheta_re_tmp);
    }
  }
}

// End of code generation (pIC_cov_weight_T_test.cpp)
