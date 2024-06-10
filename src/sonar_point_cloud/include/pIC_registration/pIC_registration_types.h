//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// pIC_registration_types.h
//
// Code generation for function 'pIC_registration'
//

#ifndef PIC_REGISTRATION_TYPES_H
#define PIC_REGISTRATION_TYPES_H

// Include files
#include "rtwtypes.h"

// Type Definitions
struct struct_T {
  double threshold;
  double initRange;
  double maxRange;
  double maxDistance;
  double maxRangesPerAngle;
};

struct b_struct_T {
  double plot_slam;
  double plot_local_traj;
  double plot_sonar;
  double plot_robot;
  double plot_ellipse_slam;
  double plot_ellipse_odom;
  double plot_ellipse_sm;
  double plot_scans;
  double plot_grid;
  double beamwidth;
  double display;
  double debug;
  double video;
  double max_range;
  double state_dist;
};

struct c_struct_T {
  double estep_method;
  double step;
  double confidence;
  double maxIterations;
  double errorRatio;
  double error[3];
  double nIterationSmoothConvergence;
  double uncertainty[3];
  double unc_Haralick[9];
  double unc_LS[9];
};

struct d_struct_T {
  double deviation[3];
};

struct e_struct_T {
  double position[3];
  double deviation[2];
  double range;
  double resolution;
  double sector;
  double thetaStep;
};

struct f_struct_T {
  double chi2;
  double Tdvl2is[6];
  double IS_range;
  double IS_range_res;
  c_struct_T sm;
  d_struct_T motion;
  e_struct_T sensor;
  struct_T segmentation;
  double debug;
  double plotFrom;
  double SDub;
  double SDvb;
  double SDwb;
  double SDuw;
  double SDvw;
  double SDww;
  double SDyaw;
  double SDzp;
  double SDmtiyaw;
  double SDmtivy;
  double SDur;
  double SDvr;
  double SDwr;
  double SDvywr;
  double debug_scanforming;
};

#endif
// End of code generation (pIC_registration_types.h)
