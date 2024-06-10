//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// EStep.cpp
//
// Code generation for function 'EStep'
//

// Include files
#include "EStep.h"
#include "pIC_registration_data.h"
#include "pIC_registration_types.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <cmath>

// Function Declarations
static void binary_expand_op_1(coder::array<double, 2U> &in1,
                               const coder::array<double, 2U> &in3,
                               const coder::array<unsigned int, 1U> &in4,
                               const coder::array<double, 3U> &in5,
                               const int in6[2], const int in7[2],
                               const int in8[2]);

// Function Definitions
static void binary_expand_op_1(coder::array<double, 2U> &in1,
                               const coder::array<double, 2U> &in3,
                               const coder::array<unsigned int, 1U> &in4,
                               const coder::array<double, 3U> &in5,
                               const int in6[2], const int in7[2],
                               const int in8[2])
{
  int i;
  int i1;
  int in6_idx_0;
  int in7_idx_0;
  int in8_idx_0;
  int loop_ub;
  int stride_0_1_tmp;
  int stride_1_1;
  in6_idx_0 = in6[0];
  in7_idx_0 = in7[0];
  in8_idx_0 = in8[0];
  in1.set_size(1, in1.size(1));
  if (in7_idx_0 == 1) {
    i = in4.size(0);
  } else {
    i = in7_idx_0;
  }
  if (in8_idx_0 == 1) {
    i1 = in4.size(0);
  } else {
    i1 = in8_idx_0;
  }
  if (i1 == 1) {
    if (i == 1) {
      if (in6_idx_0 == 1) {
        loop_ub = in4.size(0);
      } else {
        loop_ub = in6_idx_0;
      }
    } else {
      loop_ub = i;
    }
  } else {
    loop_ub = i1;
  }
  in1.set_size(in1.size(0), loop_ub);
  stride_0_1_tmp = (in4.size(0) != 1);
  stride_1_1 = (in6_idx_0 != 1);
  in7_idx_0 = (in7_idx_0 != 1);
  in6_idx_0 = (in8_idx_0 != 1);
  for (i = 0; i < loop_ub; i++) {
    double d;
    double d1;
    i1 = static_cast<int>(in4[i * stride_0_1_tmp]) - 1;
    d = in3[2 * i1];
    d1 = in3[2 * i1 + 1];
    in1[i] = (d * d * in5[4 * (static_cast<int>(in4[i * stride_1_1]) - 1)] +
              2.0 * d * d1 *
                  in5[4 * (static_cast<int>(in4[i * in7_idx_0]) - 1) + 2]) +
             d1 * d1 * in5[4 * (static_cast<int>(in4[i * in6_idx_0]) - 1) + 3];
  }
}

void EStep(
    const coder::array<double, 2U> &r_cart, const coder::array<double, 3U> &r_P,
    const coder::array<double, 2U> &n_cart, const coder::array<double, 3U> &n_P,
    const double motion_q[3], const double motion_Pq[9],
    coder::array<double, 2U> &assoc_n, coder::array<double, 3U> &assoc_Pn,
    coder::array<double, 2U> &assoc_c, coder::array<double, 2U> &assoc_a,
    coder::array<double, 3U> &assoc_Pa, coder::array<double, 3U> &assoc_Pe,
    coder::array<double, 3U> &assoc_Pc, coder::array<double, 3U> &assoc_Jq,
    coder::array<double, 3U> &reg_uncertainty,
    coder::array<double, 2U> &P_r_index)
{
  coder::array<double, 3U> Jq;
  coder::array<double, 3U> Pc_j;
  coder::array<double, 3U> Pe;
  coder::array<double, 3U> Pe_ij;
  coder::array<double, 3U> det_Pe_ij;
  coder::array<double, 3U> inv_Pe_ij;
  coder::array<double, 2U> a;
  coder::array<double, 2U> c;
  coder::array<double, 2U> d;
  coder::array<double, 2U> j;
  coder::array<int, 2U> c_i;
  coder::array<unsigned int, 1U> a_tmp;
  coder::array<signed char, 2U> indexBuffer;
  coder::array<bool, 2U> x;
  double Jp[4];
  double Jq_tmp;
  double b_Jq_tmp;
  double b_Pc_j;
  double c_Pc_j;
  double cosy_tmp;
  double siny_tmp;
  int b_i;
  int i;
  int ibtile;
  int loop_ub_tmp;
  int ntilecols;
  bool exitg1;
  //  For debug purposes
  // function [assoc,lines,P_r_index,P_n_index] = EStep(r,n,motion)
  P_r_index.set_size(1, r_cart.size(1));
  ibtile = r_cart.size(1);
  for (i = 0; i < ibtile; i++) {
    P_r_index[i] = 0.0;
  }
  // function [assoc] = EStep(r,n,motion)
  //  Function EStep
  //
  //  Computes correspondences between two scans
  //  Input:
  //    r: reference points
  //    n: new points
  //    motion: struct with q =[x y yaw] and Pq[3x3]
  //
  //  Output:
  //    A: associations
  // global handle_assoc_scanK;
  siny_tmp = std::sin(motion_q[2]);
  cosy_tmp = std::cos(motion_q[2]);
  //  Jp = [-cosy siny; -siny -cosy];
  Jp[0] = cosy_tmp;
  Jp[2] = -siny_tmp;
  Jp[1] = siny_tmp;
  Jp[3] = cosy_tmp;
  //  c_j n point to r
  // x,y
  c.set_size(2, n_cart.size(1));
  loop_ub_tmp = n_cart.size(1) << 1;
  for (i = 0; i < loop_ub_tmp; i++) {
    c[i] = 0.0;
  }
  i = n_cart.size(1);
  for (b_i = 0; b_i < i; b_i++) {
    Jq_tmp = n_cart[2 * b_i];
    b_Jq_tmp = n_cart[2 * b_i + 1];
    c[2 * b_i] = (Jq_tmp * cosy_tmp - b_Jq_tmp * siny_tmp) + motion_q[0];
    c[2 * b_i + 1] = (Jq_tmp * siny_tmp + b_Jq_tmp * cosy_tmp) + motion_q[1];
  }
  //  c =  n.cart;
  //  Pq
  if (n_cart.size(1) < 1) {
    j.set_size(1, 0);
  } else {
    j.set_size(1, n_cart.size(1));
    ibtile = n_cart.size(1) - 1;
    for (i = 0; i <= ibtile; i++) {
      j[i] = static_cast<double>(i) + 1.0;
    }
  }
  a_tmp.set_size(j.size(1));
  ibtile = j.size(1);
  for (i = 0; i < ibtile; i++) {
    a_tmp[i] = static_cast<unsigned int>(j[i]);
  }
  Pc_j.set_size(2, 2, n_cart.size(1));
  ntilecols = n_cart.size(1);
  for (b_i = 0; b_i < ntilecols; b_i++) {
    ibtile = (b_i << 2) - 1;
    Pc_j[ibtile + 1] = 1.0;
    Pc_j[ibtile + 2] = 0.0;
    Pc_j[ibtile + 3] = 0.0;
    Pc_j[ibtile + 4] = 1.0;
  }
  Jq.set_size(2, 3, Pc_j.size(2));
  ibtile = Pc_j.size(2);
  for (i = 0; i < ibtile; i++) {
    Jq[6 * i] = Pc_j[4 * i];
    Jq[6 * i + 1] = Pc_j[4 * i + 1];
    Jq[6 * i + 2] = Pc_j[4 * i + 2];
    Jq[6 * i + 3] = Pc_j[4 * i + 3];
  }
  ibtile = a_tmp.size(0);
  for (i = 0; i < ibtile; i++) {
    b_i = static_cast<int>(a_tmp[i]) - 1;
    Jq_tmp = n_cart[2 * b_i];
    b_Jq_tmp = n_cart[2 * b_i + 1];
    Jq[6 * i + 4] = -Jq_tmp * siny_tmp - b_Jq_tmp * cosy_tmp;
    Jq[6 * i + 5] = Jq_tmp * cosy_tmp - b_Jq_tmp * siny_tmp;
  }
  //  Buffers initialization
  a.set_size(2, n_cart.size(1));
  for (i = 0; i < loop_ub_tmp; i++) {
    a[i] = 0.0;
  }
  //  a buffer
  Pe.set_size(2, 2, n_cart.size(1));
  loop_ub_tmp = n_cart.size(1) << 2;
  for (i = 0; i < loop_ub_tmp; i++) {
    Pe[i] = 0.0;
  }
  //  Pe buffer
  indexBuffer.set_size(1, n_cart.size(1));
  ibtile = n_cart.size(1);
  for (i = 0; i < ibtile; i++) {
    indexBuffer[i] = 0;
  }
  Pc_j.set_size(2, 2, n_cart.size(1));
  for (i = 0; i < loop_ub_tmp; i++) {
    Pc_j[i] = 0.0;
  }
  // Pc buffer
  i = n_cart.size(1);
  for (int b_j{0}; b_j < i; b_j++) {
    double b_Jq[6];
    double b_Jp[4];
    double c_Jp[4];
    double c_Jq[4];
    double d_Pc_j;
    double e_Pc_j;
    for (loop_ub_tmp = 0; loop_ub_tmp < 2; loop_ub_tmp++) {
      for (ibtile = 0; ibtile < 3; ibtile++) {
        b_Jq[loop_ub_tmp + (ibtile << 1)] =
            (Jq[loop_ub_tmp + 6 * b_j] * motion_Pq[3 * ibtile] +
             Jq[(loop_ub_tmp + 6 * b_j) + 2] * motion_Pq[3 * ibtile + 1]) +
            Jq[(loop_ub_tmp + 6 * b_j) + 4] * motion_Pq[3 * ibtile + 2];
      }
      Jq_tmp = Jp[loop_ub_tmp];
      b_Jq_tmp = Jp[loop_ub_tmp + 2];
      b_Pc_j = b_Jq[loop_ub_tmp];
      c_Pc_j = b_Jq[loop_ub_tmp + 2];
      d_Pc_j = b_Jq[loop_ub_tmp + 4];
      for (ibtile = 0; ibtile < 2; ibtile++) {
        ntilecols = loop_ub_tmp + (ibtile << 1);
        b_Jp[ntilecols] = Jq_tmp * n_P[2 * ibtile + 4 * b_j] +
                          b_Jq_tmp * n_P[(2 * ibtile + 4 * b_j) + 1];
        c_Jq[ntilecols] = (b_Pc_j * Jq[ibtile + 6 * b_j] +
                           c_Pc_j * Jq[(ibtile + 6 * b_j) + 2]) +
                          d_Pc_j * Jq[(ibtile + 6 * b_j) + 4];
      }
      Jq_tmp = b_Jp[loop_ub_tmp + 2];
      b_Jq_tmp = b_Jp[loop_ub_tmp];
      c_Jp[loop_ub_tmp] = b_Jq_tmp * cosy_tmp + Jq_tmp * -siny_tmp;
      c_Jp[loop_ub_tmp + 2] = b_Jq_tmp * siny_tmp + Jq_tmp * cosy_tmp;
    }
    Pc_j[4 * b_j] = c_Jq[0] + c_Jp[0];
    Pc_j[4 * b_j + 1] = c_Jq[1] + c_Jp[1];
    Pc_j[4 * b_j + 2] = c_Jq[2] + c_Jp[2];
    Pc_j[4 * b_j + 3] = c_Jq[3] + c_Jp[3];
    if (r_cart.size(1) < 1) {
      j.set_size(1, 0);
    } else {
      j.set_size(1, r_cart.size(1));
      ibtile = r_cart.size(1) - 1;
      for (loop_ub_tmp = 0; loop_ub_tmp <= ibtile; loop_ub_tmp++) {
        j[loop_ub_tmp] = static_cast<double>(loop_ub_tmp) + 1.0;
      }
    }
    a_tmp.set_size(j.size(1));
    ibtile = j.size(1);
    for (loop_ub_tmp = 0; loop_ub_tmp < ibtile; loop_ub_tmp++) {
      a_tmp[loop_ub_tmp] = static_cast<unsigned int>(j[loop_ub_tmp]);
    }
    Jq_tmp = c[2 * b_j];
    b_Jq_tmp = c[2 * b_j + 1];
    d.set_size(2, a_tmp.size(0));
    ibtile = a_tmp.size(0);
    //  Covariances of r and q-p composition
    b_Pc_j = Pc_j[4 * b_j];
    c_Pc_j = Pc_j[4 * b_j + 2];
    d_Pc_j = Pc_j[4 * b_j + 1];
    e_Pc_j = Pc_j[4 * b_j + 3];
    Pe_ij.set_size(2, 2, a_tmp.size(0));
    for (loop_ub_tmp = 0; loop_ub_tmp < ibtile; loop_ub_tmp++) {
      ntilecols = static_cast<int>(a_tmp[loop_ub_tmp]) - 1;
      d[2 * loop_ub_tmp] = r_cart[2 * ntilecols] - Jq_tmp;
      d[2 * loop_ub_tmp + 1] = r_cart[2 * ntilecols + 1] - b_Jq_tmp;
      Pe_ij[4 * loop_ub_tmp] = r_P[4 * ntilecols] + b_Pc_j;
      Pe_ij[4 * loop_ub_tmp + 2] = r_P[4 * ntilecols + 2] + c_Pc_j;
      Pe_ij[4 * loop_ub_tmp + 1] = r_P[4 * ntilecols + 1] + d_Pc_j;
      Pe_ij[4 * loop_ub_tmp + 3] = r_P[4 * ntilecols + 3] + e_Pc_j;
    }
    //  inv(Pe_ij) for a 2x2xn matrix. Each matrix must be [a b;b d] form
    det_Pe_ij.set_size(1, 1, a_tmp.size(0));
    ibtile = a_tmp.size(0);
    for (loop_ub_tmp = 0; loop_ub_tmp < ibtile; loop_ub_tmp++) {
      ntilecols = static_cast<int>(a_tmp[loop_ub_tmp]) - 1;
      Jq_tmp = Pe_ij[4 * ntilecols + 2];
      det_Pe_ij[loop_ub_tmp] =
          Pe_ij[4 * ntilecols] * Pe_ij[4 * ntilecols + 3] - Jq_tmp * Jq_tmp;
    }
    inv_Pe_ij.set_size(2, 2, a_tmp.size(0));
    ibtile = a_tmp.size(0);
    for (loop_ub_tmp = 0; loop_ub_tmp < ibtile; loop_ub_tmp++) {
      ntilecols = static_cast<int>(a_tmp[loop_ub_tmp]) - 1;
      Jq_tmp = det_Pe_ij[ntilecols];
      inv_Pe_ij[4 * loop_ub_tmp] = Pe_ij[4 * ntilecols + 3] / Jq_tmp;
      b_Jq_tmp = -Pe_ij[4 * ntilecols + 2] / Jq_tmp;
      inv_Pe_ij[4 * loop_ub_tmp + 2] = b_Jq_tmp;
      inv_Pe_ij[4 * loop_ub_tmp + 1] = b_Jq_tmp;
      inv_Pe_ij[4 * loop_ub_tmp + 3] = Pe_ij[4 * ntilecols] / Jq_tmp;
    }
    //  Mahalanobis distance
    if (PARAM.sm.estep_method == 0.0) {
      int b_szb[2];
      int c_szb[2];
      int szb[2];
      //  ICNN
      // [d_value,d_index] = min(dist);
      szb[0] = 1;
      b_szb[0] = 1;
      if (a_tmp.size(0) != 1) {
        szb[0] = a_tmp.size(0);
      }
      c_szb[0] = 1;
      if (a_tmp.size(0) != 1) {
        b_szb[0] = a_tmp.size(0);
        if (a_tmp.size(0) != 1) {
          c_szb[0] = a_tmp.size(0);
        }
      }
      if (a_tmp.size(0) == 1) {
        loop_ub_tmp = szb[0];
      } else {
        loop_ub_tmp = a_tmp.size(0);
      }
      if (a_tmp.size(0) == 1) {
        ibtile = b_szb[0];
      } else {
        ibtile = a_tmp.size(0);
      }
      if (loop_ub_tmp == 1) {
        ntilecols = ibtile;
      } else {
        ntilecols = loop_ub_tmp;
      }
      if (a_tmp.size(0) == 1) {
        b_i = c_szb[0];
      } else {
        b_i = a_tmp.size(0);
      }
      if ((a_tmp.size(0) == szb[0]) && (a_tmp.size(0) == b_szb[0]) &&
          (loop_ub_tmp == ibtile) && (a_tmp.size(0) == c_szb[0]) &&
          (ntilecols == b_i)) {
        j.set_size(1, a_tmp.size(0));
        ibtile = a_tmp.size(0);
        for (loop_ub_tmp = 0; loop_ub_tmp < ibtile; loop_ub_tmp++) {
          ntilecols = static_cast<int>(a_tmp[loop_ub_tmp]) - 1;
          Jq_tmp = d[2 * ntilecols];
          b_Jq_tmp = d[2 * ntilecols + 1];
          j[loop_ub_tmp] =
              (Jq_tmp * Jq_tmp * inv_Pe_ij[4 * ntilecols] +
               2.0 * Jq_tmp * b_Jq_tmp * inv_Pe_ij[4 * ntilecols + 2]) +
              b_Jq_tmp * b_Jq_tmp * inv_Pe_ij[4 * ntilecols + 3];
        }
      } else {
        binary_expand_op_1(j, d, a_tmp, inv_Pe_ij, szb, b_szb, c_szb);
      }
      ntilecols = j.size(1);
      ibtile = 1;
      Jq_tmp = j[0];
      for (loop_ub_tmp = 2; loop_ub_tmp <= ntilecols; loop_ub_tmp++) {
        b_Jq_tmp = j[loop_ub_tmp - 1];
        for (b_i = 0; b_i < 1; b_i++) {
          bool p;
          if (std::isnan(b_Jq_tmp)) {
            p = false;
          } else if (std::isnan(Jq_tmp)) {
            p = true;
          } else {
            p = (Jq_tmp > b_Jq_tmp);
          }
          if (p) {
            Jq_tmp = b_Jq_tmp;
            ibtile = loop_ub_tmp;
          }
        }
      }
      //  adjusted to tell the min function to operate along the second
      //  dimension
      if (Jq_tmp <= chi2value) {
        //  + Pc_j(:,:,j);
        //  save a_j & Pa_j
        Pe[4 * b_j] = Pe_ij[4 * (ibtile - 1)];
        Pe[4 * b_j + 1] = Pe_ij[4 * (ibtile - 1) + 1];
        a[2 * b_j] = r_cart[2 * (ibtile - 1)];
        Pe[4 * b_j + 2] = Pe_ij[4 * (ibtile - 1) + 2];
        Pe[4 * b_j + 3] = Pe_ij[4 * (ibtile - 1) + 3];
        a[2 * b_j + 1] = r_cart[2 * (ibtile - 1) + 1];
        //              Pe(:,:,j) = Pa_j + Pc_j(:,:,j);
        indexBuffer[b_j] = 1;
      }
      //  Virtual point
      //  elseif PARAM.sm.estep_method == 1
    }
  }
  //  Keep only the associaton necessary values
  x.set_size(1, indexBuffer.size(1));
  ibtile = indexBuffer.size(1);
  for (i = 0; i < ibtile; i++) {
    x[i] = (indexBuffer[i] == 1);
  }
  ntilecols = x.size(1);
  ibtile = 0;
  c_i.set_size(1, x.size(1));
  b_i = 0;
  exitg1 = false;
  while ((!exitg1) && (b_i <= ntilecols - 1)) {
    if (x[b_i]) {
      ibtile++;
      c_i[ibtile - 1] = b_i + 1;
      if (ibtile >= ntilecols) {
        exitg1 = true;
      } else {
        b_i++;
      }
    } else {
      b_i++;
    }
  }
  if (x.size(1) == 1) {
    if (ibtile == 0) {
      c_i.set_size(1, 0);
    }
  } else {
    if (ibtile < 1) {
      ibtile = 0;
    }
    c_i.set_size(c_i.size(0), ibtile);
  }
  a_tmp.set_size(c_i.size(1));
  ibtile = c_i.size(1);
  for (i = 0; i < ibtile; i++) {
    a_tmp[i] = static_cast<unsigned int>(c_i[i]);
  }
  assoc_n.set_size(2, a_tmp.size(0));
  ibtile = a_tmp.size(0);
  assoc_Pn.set_size(2, 2, a_tmp.size(0));
  assoc_c.set_size(2, a_tmp.size(0));
  assoc_a.set_size(2, a_tmp.size(0));
  assoc_Pa.set_size(2, 2, a_tmp.size(0));
  assoc_Pe.set_size(2, 2, a_tmp.size(0));
  assoc_Pc.set_size(2, 2, a_tmp.size(0));
  assoc_Jq.set_size(2, 3, a_tmp.size(0));
  //  assoc.Jq_ = Jq_(:,:,index);
  reg_uncertainty.set_size(2, 2, a_tmp.size(0));
  for (i = 0; i < ibtile; i++) {
    ntilecols = static_cast<int>(a_tmp[i]) - 1;
    assoc_n[2 * i] = n_cart[2 * ntilecols];
    assoc_c[2 * i] = c[2 * ntilecols];
    assoc_a[2 * i] = a[2 * ntilecols];
    Jq_tmp = n_P[4 * ntilecols];
    assoc_Pn[4 * i] = Jq_tmp;
    assoc_Pa[4 * i] = Jq_tmp;
    b_Jq_tmp = Pe[4 * ntilecols];
    assoc_Pe[4 * i] = b_Jq_tmp;
    assoc_Pc[4 * i] = Pc_j[4 * ntilecols];
    Jq_tmp = n_P[4 * ntilecols + 1];
    assoc_Pn[4 * i + 1] = Jq_tmp;
    assoc_Pa[4 * i + 1] = Jq_tmp;
    b_Pc_j = Pe[4 * ntilecols + 1];
    assoc_Pe[4 * i + 1] = b_Pc_j;
    assoc_Pc[4 * i + 1] = Pc_j[4 * ntilecols + 1];
    assoc_n[2 * i + 1] = n_cart[2 * ntilecols + 1];
    assoc_c[2 * i + 1] = c[2 * ntilecols + 1];
    assoc_a[2 * i + 1] = a[2 * ntilecols + 1];
    Jq_tmp = n_P[4 * ntilecols + 2];
    assoc_Pn[4 * i + 2] = Jq_tmp;
    assoc_Pa[4 * i + 2] = Jq_tmp;
    c_Pc_j = Pe[4 * ntilecols + 2];
    assoc_Pe[4 * i + 2] = c_Pc_j;
    assoc_Pc[4 * i + 2] = Pc_j[4 * ntilecols + 2];
    Jq_tmp = n_P[4 * ntilecols + 3];
    assoc_Pn[4 * i + 3] = Jq_tmp;
    assoc_Pa[4 * i + 3] = Jq_tmp;
    Jq_tmp = Pe[4 * ntilecols + 3];
    assoc_Pe[4 * i + 3] = Jq_tmp;
    assoc_Pc[4 * i + 3] = Pc_j[4 * ntilecols + 3];
    for (loop_ub_tmp = 0; loop_ub_tmp < 3; loop_ub_tmp++) {
      assoc_Jq[2 * loop_ub_tmp + 6 * i] = Jq[2 * loop_ub_tmp + 6 * ntilecols];
      assoc_Jq[(2 * loop_ub_tmp + 6 * i) + 1] =
          Jq[(2 * loop_ub_tmp + 6 * ntilecols) + 1];
    }
    reg_uncertainty[4 * i] = b_Jq_tmp;
    reg_uncertainty[4 * i + 1] = b_Pc_j;
    reg_uncertainty[4 * i + 2] = c_Pc_j;
    reg_uncertainty[4 * i + 3] = Jq_tmp;
  }
}

// End of code generation (EStep.cpp)
