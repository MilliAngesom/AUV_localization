//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// pIC_registration.cpp
//
// Code generation for function 'pIC_registration'
//

// Include files
#include "pIC_registration.h"
#include "EStep.h"
#include "MStep.h"
#include "chi2inv.h"
#include "inv.h"
#include "mtimes.h"
#include "pIC_cov_weight_T_test.h"
#include "pIC_registration_data.h"
#include "pIC_registration_initialize.h"
#include "pIC_registration_types.h"
#include "rt_nonfinite.h"
#include "sum.h"
#include "coder_array.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>

// Function Definitions
void pIC_registration(const coder::array<double, 2U> &sourcePC,
                      const coder::array<double, 2U> &targetPC,
                      double sonar_noise, const double initial_estimate[3],
                      const double initial_estimate_cov[3], double, double,
                      double *out, double q[3], double *num_it,
                      double registration_uncertainty[9])
{
  static const double dv[6]{0.33, 0.0, 0.26, 0.0, 0.0, 3.1415926535897931};
  coder::array<double, 3U> assoc_Jq;
  coder::array<double, 3U> assoc_Pa;
  coder::array<double, 3U> assoc_Pc;
  coder::array<double, 3U> assoc_Pe;
  coder::array<double, 3U> assoc_Pn;
  coder::array<double, 3U> reg_uncertainty;
  coder::array<double, 3U> source_scan_P;
  coder::array<double, 3U> target_scan_P;
  coder::array<double, 2U> A;
  coder::array<double, 2U> C;
  coder::array<double, 2U> H;
  coder::array<double, 2U> a__2;
  coder::array<double, 2U> assoc_a;
  coder::array<double, 2U> assoc_n;
  coder::array<double, 2U> c_C;
  coder::array<double, 2U> e;
  coder::array<double, 2U> invC;
  coder::array<double, 2U> r;
  coder::array<double, 2U> source_scan_cart;
  coder::array<double, 2U> target_scan_cart;
  coder::array<double, 1U> E;
  double motion_Pq[9];
  double bkj;
  double errorK1;
  double num_converged;
  int aoffset;
  int b_num_it;
  int b_out;
  int i;
  int ibtile;
  if (!isInitialized_pIC_registration) {
    pIC_registration_initialize();
  }
  //  function [out, q, num_it, registration_uncertainty] =
  //  pIC_registration(sourcePC_x,sourcePC_y, targetPC_x, targetPC_y, ...
  //      sonar_noise, initial_estimate, initial_estimate_cov, source_size,
  //      target_size)
  //  INPUTS
  //  sourcePC_x, sourcePC_y, targetPC_x and targetPC_y are the source and
  //  target PCs. each of them are vectors eg targetPC_x is a vector that
  //  contains the x coordinates of the point clouds
  //  sonar_noise is a scalar value that represents the std of noise of the
  //  sensor.
  //  initial_estimate [x; y; theta] ---- the input should be 3 by 1
  //  initial_estimate_cov: this the uncertainity of the initial estimaate
  //  (could be deadreckoning covariance) it should be could be 1 by 3 or 3 by
  //  1 [cov_x, cov_y, cov_theta]
  //  source_size and target_size are scalars that contains the number of
  //                         points in source and target PC respectively.
  //  OUTPUTS
  //  out: is a flag that signals if the registration process was succesful or
  //       not. 1 is success whereas 0 is failure
  //  q: is the estimated [x,y,theta]. It is 3 by 1 vector.
  //  num_it: a scalar value that holds the number of iteration the program ran
  //  registration_uncertainty: this is the scan-matching uncertainty
  //  PARAMETERS===================================================================
  // Global
  PARAM.chi2 = 9.2103403719761818;
  // chi2 at a 95% confidence level
  // Vehicle
  for (i = 0; i < 6; i++) {
    PARAM.Tdvl2is[i] = dv[i];
  }
  // PARAM.Tdvl2is      =  [0.5 0 -0.2 0 0 pi]';
  // Imaging sonar
  //  IS_setup(1:2)       =  [48 0.096];
  //  PARAM.IS_range      =  IS_setup(1);     %maximum beam distance (m)
  //  PARAM.IS_range_res  =  IS_setup(2);     %beam resolution
  //  PARAM.IS_angle_res  =  IS_setup(3);     %resolucio% angular------[ME]--
  // [#_of_beams, #_of_bins]
  //  PARAM.IS_size       =  [round(2*pi/IS_setup(3))...
  //                          round(PARAM.IS_range/PARAM.IS_range_res)];--[ME]
  // Scan Matching Parameters
  PARAM.sm.estep_method = 0.0;
  // 0 nearest neighbour; 1 virtual point
  PARAM.sm.step = 1.0;
  PARAM.sm.confidence = 0.7;
  PARAM.sm.maxIterations = 250.0;
  PARAM.sm.errorRatio = 0.0001;
  PARAM.sm.nIterationSmoothConvergence = 2.0;
  //  Motion -----------------------------------------------------------------
  //  motion std deviation [x(m) y(m) yaw(rad)]
  //  Sensor -----------------------------------------------------------------
  // x,y,yaw.Position respect to the robot's zero (z = 0)
  PARAM.sm.error[0] = 0.0001;
  PARAM.sm.uncertainty[0] = 1.5;
  PARAM.motion.deviation[0] = 0.1;
  PARAM.sensor.position[0] = 0.4;
  PARAM.sm.error[1] = 0.0001;
  PARAM.sm.uncertainty[1] = 1.5;
  PARAM.motion.deviation[1] = 0.1;
  PARAM.sensor.position[1] = 0.0;
  PARAM.sm.error[2] = 0.0001;
  PARAM.sm.uncertainty[2] = 0.05;
  PARAM.motion.deviation[2] = 0.17453292519943295;
  PARAM.sensor.position[2] = 3.1415926535897931;
  //  std deviation [angular(rad) range(m)] [+/-5cm +/-1.5deg]
  PARAM.sensor.deviation[0] = 0.1;
  PARAM.sensor.deviation[1] = 0.05235987755982989;
  PARAM.sensor.range = 50.0;
  PARAM.sensor.resolution = 0.1;
  PARAM.sensor.sector = 6.2831853071795862;
  PARAM.sensor.thetaStep = 0.031415926535897934;
  //  Segmentation -----------------------------------------------------------
  PARAM.segmentation.threshold = 100.0;
  PARAM.segmentation.initRange = 1.5;
  PARAM.segmentation.maxRange = 50.0;
  PARAM.segmentation.maxDistance = 50.0;
  PARAM.segmentation.maxRangesPerAngle = 1.0;
  //  Configuration -----------------------------------------------------------
  PARAM.debug = 0.0;
  PARAM.plotFrom = 0.0;
  chi2value = coder::chi2inv(0.7);
  // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  // DVL
  PARAM.SDub = 0.3;
  // m/s    velocity x bottom
  PARAM.SDvb = 0.3;
  // m/s    velocity y bottom
  PARAM.SDwb = 0.15;
  // m/s    velocity z bottom
  PARAM.SDuw = 0.6;
  // m/s    velocity x water
  PARAM.SDvw = 0.6;
  // m/s    velocity y water
  PARAM.SDww = 0.3;
  // m/s    velocity z water
  PARAM.SDyaw = 0.2;
  // rad    yaw
  PARAM.SDzp = 0.02;
  // m      depth (pressure)
  // MTI
  PARAM.SDmtiyaw = 0.2;
  // rad    yaw
  PARAM.SDmtivy = 0.2;
  // rad/s  yaw
  // Model
  PARAM.SDur = 0.1;
  // m/s^2   velocity x robot
  PARAM.SDvr = 0.05;
  // m/s^2   velocity y robot
  PARAM.SDwr = 0.1;
  // m/s^2   velocity z robot
  PARAM.SDvywr = 0.25;
  // rad/s^2  ang. velocity yaw robot
  //  CONFIGURATION================================================================
  // Slam
  CONFIG.plot_slam = 1.0;
  // If 1 plot slam online, 2 for offline
  CONFIG.plot_local_traj = 0.0;
  // If 1 plot local odometry
  CONFIG.plot_sonar = 0.0;
  // If 1 plot sonar beam
  CONFIG.plot_robot = 0.0;
  // If 1 plot robot icon
  CONFIG.plot_ellipse_slam = 1.0;
  // If 1 plot slam uncertainty
  CONFIG.plot_ellipse_odom = 0.0;
  // If 1 plot odometry uncertainty
  CONFIG.plot_ellipse_sm = 1.0;
  // If 1 plot scan matching uncertainty
  CONFIG.plot_scans = 4.0;
  // if 1 plot at the end, 2 plot last scan, 3 plot all scan points, 4 replot
  // online
  CONFIG.plot_grid = 1.0;
  // if 1 plot grid connections
  CONFIG.beamwidth = 0.052359877559829883;
  // Horizontal beamwidth of the IS
  CONFIG.display = 1.0;
  // if 1 display few debug messages, 2 for more, 3 for plot scan#
  CONFIG.debug = 0.0;
  // if 1 store motion vectors, 2 display motion vectors
  CONFIG.video = 0.0;
  // if 1 store image frames for video
  //  CONFIG.min_range            = 0;  %minimum range for doing SLAM
  CONFIG.max_range = 10.0;
  // maximum range for doing SLAM
  CONFIG.state_dist = 10.0;
  // maximum state vector distance for doing SLAM
  //  CONFIG.state_dist_min       = 0;  %minimum previous state poses distance
  //  for doing SLAM CONFIG.sm_K                 = 1;  %sm uncertainty
  //  coinficiency factor
  PARAM.debug_scanforming = 0.0;
  //  Store points in the .cart attribute
  source_scan_cart.set_size(2, sourcePC.size(1));
  ibtile = sourcePC.size(1);
  for (aoffset = 0; aoffset < ibtile; aoffset++) {
    source_scan_cart[2 * aoffset] = sourcePC[2 * aoffset];
    source_scan_cart[2 * aoffset + 1] = sourcePC[2 * aoffset + 1];
  }
  //  Create a 2x2 diagonal matrix for each point with diagonal elements
  bkj = sonar_noise * 0.0;
  if (source_scan_cart.size(1) > 2) {
    aoffset = source_scan_cart.size(1);
  } else {
    aoffset = 2;
  }
  source_scan_P.set_size(2, 2, aoffset);
  for (i = 0; i < aoffset; i++) {
    ibtile = (i << 2) - 1;
    source_scan_P[ibtile + 1] = sonar_noise;
    source_scan_P[ibtile + 2] = bkj;
    source_scan_P[ibtile + 3] = bkj;
    source_scan_P[ibtile + 4] = sonar_noise;
  }
  target_scan_cart.set_size(2, targetPC.size(1));
  ibtile = targetPC.size(1);
  for (aoffset = 0; aoffset < ibtile; aoffset++) {
    target_scan_cart[2 * aoffset] = targetPC[2 * aoffset];
    target_scan_cart[2 * aoffset + 1] = targetPC[2 * aoffset + 1];
  }
  //  Fill the .P field of the current_scan with a 2 by 2 diagonal matrix
  //  where the diagonal element value is equal to sensor noise level for each
  //  point
  if (target_scan_cart.size(1) > 2) {
    aoffset = target_scan_cart.size(1);
  } else {
    aoffset = 2;
  }
  target_scan_P.set_size(2, 2, aoffset);
  for (i = 0; i < aoffset; i++) {
    ibtile = (i << 2) - 1;
    target_scan_P[ibtile + 1] = sonar_noise;
    target_scan_P[ibtile + 2] = bkj;
    target_scan_P[ibtile + 3] = bkj;
    target_scan_P[ibtile + 4] = sonar_noise;
  }
  //  this is the initial estimate given to the scan matching algorithm
  q[0] = initial_estimate[0];
  q[1] = initial_estimate[1];
  q[2] = initial_estimate[2];
  //  it should be 3 by 1
  //  uncertainty in the initial estimate
  std::memset(&motion_Pq[0], 0, 9U * sizeof(double));
  motion_Pq[0] = initial_estimate_cov[0];
  motion_Pq[4] = initial_estimate_cov[1];
  motion_Pq[8] = initial_estimate_cov[2];
  //  PIC SCAN MATCHING
  // function [out,q,num_it,assoc] = pIC(scanK,scanK1,motion)
  //  Debug mode 3
  // global handle_assoc_scanK;
  //  frame=1;
  errorK1 = 1.0E+6;
  num_converged = 0.0;
  b_num_it = 0;
  //  % Remove unnecessary points
  //  indexDelete = isnan(scanK.cart(1,:));
  //  scanK.cart(:,indexDelete) = [];
  //  scanK.P(:,:,indexDelete) = [];
  //
  //  indexDelete = isnan(scanK1.cart(1,:));
  //  scanK1.cart(:,indexDelete) = [];
  //  scanK1.P(:,:,indexDelete) = [];
  //  if (PARAM.debug >= 1) && (NUM_SM >= PARAM.plotFrom)
  b_out = 0;
  while ((b_num_it < 250) && (b_out == 0)) {
    double qmin[3];
    // E_STEP
    // [assoc,assoc_lines,scanK_P_index,scanK1_Pindex] = EStep(scanK, scanK1,
    // motion);
    EStep(target_scan_cart, target_scan_P, source_scan_cart, source_scan_P, q,
          motion_Pq, assoc_n, assoc_Pn, e, assoc_a, assoc_Pa, assoc_Pe,
          assoc_Pc, assoc_Jq, reg_uncertainty, a__2);
    // Debug
    //  if PARAM.debug >= 2 && (NUM_SM >= PARAM.plotFrom)
    // M_STEP
    if (assoc_a.size(1) == 0) {
      //      error('No associations avaiable.');
      ibtile = -1;
      qmin[0] = q[0];
      qmin[1] = q[1];
      qmin[2] = q[2];
      //  PARAM.sm.uncertainty4 = zeros(3);
    } else {
      double b_C[9];
      double invA[9];
      double varargin_1[2];
      double d;
      double d1;
      double siny;
      int coffset;
      int nc;
      unsigned int u;
      E.set_size(2 * assoc_a.size(1));
      ibtile = 2 * assoc_a.size(1);
      for (aoffset = 0; aoffset < ibtile; aoffset++) {
        E[aoffset] = 0.0;
      }
      H.set_size(2 * assoc_a.size(1), 3);
      ibtile = 2 * assoc_a.size(1) * 3;
      for (aoffset = 0; aoffset < ibtile; aoffset++) {
        H[aoffset] = 0.0;
      }
      u = static_cast<unsigned int>(assoc_a.size(1)) << 1;
      C.set_size(static_cast<int>(u), static_cast<int>(u));
      ibtile = static_cast<int>(u) * static_cast<int>(u);
      for (aoffset = 0; aoffset < ibtile; aoffset++) {
        C[aoffset] = 0.0;
      }
      aoffset = assoc_a.size(1);
      d = q[0];
      d1 = q[1];
      bkj = q[2];
      for (i = 0; i < aoffset; i++) {
        double d2;
        u = static_cast<unsigned int>(2.0 * (static_cast<double>(i) + 1.0) -
                                      1.0);
        for (ibtile = 0; ibtile < 2; ibtile++) {
          siny = static_cast<double>(u) + static_cast<double>(ibtile);
          varargin_1[ibtile] = siny;
          E[static_cast<int>(siny) - 1] =
              (assoc_a[ibtile + 2 * i] - e[ibtile + 2 * i]) +
              ((assoc_Jq[ibtile + 6 * i] * d +
                assoc_Jq[(ibtile + 6 * i) + 2] * d1) +
               assoc_Jq[(ibtile + 6 * i) + 4] * bkj);
        }
        siny = varargin_1[0];
        d2 = varargin_1[1];
        for (ibtile = 0; ibtile < 3; ibtile++) {
          H[(static_cast<int>(siny) + H.size(0) * ibtile) - 1] =
              assoc_Jq[2 * ibtile + 6 * i];
          H[(static_cast<int>(d2) + H.size(0) * ibtile) - 1] =
              assoc_Jq[(2 * ibtile + 6 * i) + 1];
        }
        C[(static_cast<int>(varargin_1[0]) +
           C.size(0) * (static_cast<int>(varargin_1[0]) - 1)) -
          1] = assoc_Pe[4 * i];
        C[(static_cast<int>(varargin_1[1]) +
           C.size(0) * (static_cast<int>(varargin_1[0]) - 1)) -
          1] = assoc_Pe[4 * i + 1];
        C[(static_cast<int>(varargin_1[0]) +
           C.size(0) * (static_cast<int>(varargin_1[1]) - 1)) -
          1] = assoc_Pe[4 * i + 2];
        C[(static_cast<int>(varargin_1[1]) +
           C.size(0) * (static_cast<int>(varargin_1[1]) - 1)) -
          1] = assoc_Pe[4 * i + 3];
      }
      coder::inv(C, invC);
      coder::internal::blas::mtimes(H, invC, A);
      ibtile = A.size(1);
      for (int j{0}; j < 3; j++) {
        coffset = j * 3;
        i = j * H.size(0);
        b_C[coffset] = 0.0;
        b_C[coffset + 1] = 0.0;
        b_C[coffset + 2] = 0.0;
        for (int k{0}; k < ibtile; k++) {
          aoffset = k * 3;
          bkj = H[i + k];
          b_C[coffset] += A[aoffset] * bkj;
          b_C[coffset + 1] += A[aoffset + 1] * bkj;
          b_C[coffset + 2] += A[aoffset + 2] * bkj;
        }
      }
      coder::inv(b_C, invA);
      ibtile = H.size(0);
      c_C.set_size(3, H.size(0));
      for (int j{0}; j < ibtile; j++) {
        coffset = j * 3;
        for (i = 0; i < 3; i++) {
          c_C[coffset + i] = (invA[i] * H[j] + invA[i + 3] * H[H.size(0) + j]) +
                             invA[i + 6] * H[(H.size(0) << 1) + j];
        }
      }
      ibtile = c_C.size(1);
      nc = invC.size(1);
      A.set_size(3, invC.size(1));
      for (int j{0}; j < nc; j++) {
        coffset = j * 3;
        i = j * invC.size(0);
        A[coffset] = 0.0;
        A[coffset + 1] = 0.0;
        A[coffset + 2] = 0.0;
        for (int k{0}; k < ibtile; k++) {
          aoffset = k * 3;
          bkj = invC[i + k];
          A[coffset] = A[coffset] + c_C[aoffset] * bkj;
          A[coffset + 1] = A[coffset + 1] + c_C[aoffset + 1] * bkj;
          A[coffset + 2] = A[coffset + 2] + c_C[aoffset + 2] * bkj;
        }
      }
      ibtile = A.size(1);
      qmin[0] = 0.0;
      qmin[1] = 0.0;
      qmin[2] = 0.0;
      for (int k{0}; k < ibtile; k++) {
        aoffset = k * 3;
        qmin[0] += A[aoffset] * E[k];
        qmin[1] += A[aoffset + 1] * E[k];
        qmin[2] += A[aoffset + 2] * E[k];
      }
      //  Compute error
      bkj = std::cos(qmin[2]);
      siny = std::sin(qmin[2]);
      // x,y
      e.set_size(2, assoc_n.size(1));
      ibtile = assoc_n.size(1) << 1;
      for (aoffset = 0; aoffset < ibtile; aoffset++) {
        e[aoffset] = 0.0;
      }
      aoffset = assoc_n.size(1);
      for (i = 0; i < aoffset; i++) {
        d = assoc_n[2 * i];
        d1 = assoc_n[2 * i + 1];
        e[2 * i] = (d * bkj - d1 * siny) + qmin[0];
        e[2 * i + 1] = (d * siny + d1 * bkj) + qmin[1];
      }
      if (e.size(1) == assoc_a.size(1)) {
        r.set_size(2, e.size(1));
        for (aoffset = 0; aoffset < ibtile; aoffset++) {
          bkj = e[aoffset] - assoc_a[aoffset];
          r[aoffset] = bkj * bkj;
        }
        coder::sum(r, varargin_1);
      } else {
        binary_expand_op(e, assoc_a, varargin_1);
      }
      bkj = varargin_1[0] + varargin_1[1];
      //  Convergence criteria
      //  diff = abs(PARAM.realq' - qmin');
      if ((std::abs(1.0 - bkj / errorK1) <= 0.0001) ||
          ((std::abs(q[0] - qmin[0]) < 0.0001) &&
           (std::abs(q[1] - qmin[1]) < 0.0001) &&
           (std::abs(q[2] - qmin[2]) < 0.0001))) {
        num_converged++;
      } else {
        num_converged = 0.0;
      }
      //  Build solution
      errorK1 = bkj;
      //  Smooth convergence criterion
      if (num_converged > 2.0) {
        creal_T dcv[9];
        //      PARAM.sm.uncertainty = inv(H'*invC*H); %displacement uncertainty
        //      PARAM.sm.uncertainty1 = pIC_cov(qmin,C,assoc);
        //      PARAM.sm.uncertainty3 = pIC_cov(qmin,C,assoc);
        //      lala=eye(size(C));
        //      PARAM.sm.Censi_num = icp_covariance_agg(qmin,C,assoc);
        //      PARAM.sm.unc_2LS2 = inv(H'*inv(2*(C^2))*H);
        //      PARAM.sm.unc_2LS = inv(H'*inv(2*(C))*H);
        pIC_cov_weight_T_test(qmin, C, assoc_n, assoc_Pn, assoc_a, assoc_Pa,
                              dcv);
        //      PARAM.sm.unc_LS = 2*(err/(i-3))*invA;
        for (aoffset = 0; aoffset < 9; aoffset++) {
          PARAM.sm.unc_Haralick[aoffset] = dcv[aoffset].re;
          PARAM.sm.unc_LS[aoffset] = invA[aoffset] / 2.0;
        }
        //      PARAM.sm.unc_LS = inv(H'*inv(2*(C)^2)*H);
        //      PARAM.sm.uncertainty = invA;
        //      PARAM.sm.unc_LS = [invA(2,2),invA(1,2),invA(2,3);
        //      invA(2,1),invA(1,1),invA(1,3); invA(3,2),invA(3,1),invA(3,3)];
        ibtile = 1;
      } else {
        ibtile = 0;
      }
    }
    // Debug
    //  if PARAM.debug >= 2 && (NUM_SM >= PARAM.plotFrom)
    q[0] = qmin[0];
    q[1] = qmin[1];
    q[2] = qmin[2];
    if (ibtile == 1) {
      b_out = 1;
    } else if (ibtile == -1) {
      b_out = -1;
    } else {
      b_num_it++;
    }
  }
  if (b_num_it == 250) {
    b_out = 2;
  }
  //  if PARAM.debug >= 1 && NUM_SM >= PARAM.plotFrom
  *out = b_out;
  *num_it = b_num_it;
  std::copy(&PARAM.sm.unc_Haralick[0], &PARAM.sm.unc_Haralick[9],
            &registration_uncertainty[0]);
}

// End of code generation (pIC_registration.cpp)
