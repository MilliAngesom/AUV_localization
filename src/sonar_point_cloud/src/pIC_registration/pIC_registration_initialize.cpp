//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// pIC_registration_initialize.cpp
//
// Code generation for function 'pIC_registration_initialize'
//

// Include files
#include "pIC_registration_initialize.h"
#include "pIC_registration_data.h"
#include "pIC_registration_types.h"
#include "rt_nonfinite.h"

// Function Definitions
void pIC_registration_initialize()
{
  static const f_struct_T r{
      9.2103403719761818,                              // chi2
      {0.33, 0.0, 0.26, 0.0, 0.0, 3.1415926535897931}, // Tdvl2is
      48.0,                                            // IS_range
      0.096,                                           // IS_range_res
      {
          0.0,                      // estep_method
          1.0,                      // step
          0.7,                      // confidence
          250.0,                    // maxIterations
          0.0001,                   // errorRatio
          {0.0001, 0.0001, 0.0001}, // error
          2.0,                      // nIterationSmoothConvergence
          {1.5, 1.5, 0.05},         // uncertainty
          {0.0019766794915243944, -5.136045778392475E-5, 1.8304456870163777E-19,
           -5.1360457783924981E-5, 0.0021647088662344028,
           -7.3194925022043582E-21, 1.2270092522020462E-19,
           2.5102692138205316E-20, 0.00014471081874334693}, // unc_Haralick
          {0.0017381934515522935, -6.9505989859528186E-5,
           -7.7387916981564818E-21, -6.95059898595282E-5, 0.0019926531735989651,
           -4.210579544118884E-21, -8.2124440426272382E-21,
           9.3684637771792822E-21, 0.00031968775376601036} // unc_LS
      },                                                   // sm
      {
          {0.1, 0.1, 0.17453292519943295} // deviation
      },                                  // motion
      {
          {0.4, 0.0, 3.1415926535897931}, // position
          {0.1, 0.05235987755982989},     // deviation
          50.0,                           // range
          0.1,                            // resolution
          6.2831853071795862,             // sector
          0.031415926535897934            // thetaStep
      },                                  // sensor
      {
          100.0, // threshold
          1.5,   // initRange
          50.0,  // maxRange
          50.0,  // maxDistance
          1.0    // maxRangesPerAngle
      },         // segmentation
      0.0,       // debug
      0.0,       // plotFrom
      0.3,       // SDub
      0.3,       // SDvb
      0.15,      // SDwb
      0.6,       // SDuw
      0.6,       // SDvw
      0.3,       // SDww
      0.2,       // SDyaw
      0.02,      // SDzp
      0.2,       // SDmtiyaw
      0.2,       // SDmtivy
      0.1,       // SDur
      0.05,      // SDvr
      0.1,       // SDwr
      0.25,      // SDvywr
      0.0        // debug_scanforming
  };
  CONFIG.plot_slam = 1.0;
  CONFIG.plot_local_traj = 0.0;
  CONFIG.plot_sonar = 0.0;
  CONFIG.plot_robot = 0.0;
  CONFIG.plot_ellipse_slam = 1.0;
  CONFIG.plot_ellipse_odom = 0.0;
  CONFIG.plot_ellipse_sm = 1.0;
  CONFIG.plot_scans = 4.0;
  CONFIG.plot_grid = 1.0;
  CONFIG.beamwidth = 0.052359877559829883;
  CONFIG.display = 1.0;
  CONFIG.debug = 0.0;
  CONFIG.video = 0.0;
  CONFIG.max_range = 10.0;
  CONFIG.state_dist = 10.0;
  PARAM = r;
  chi2value = 2.4079456086518722;
  isInitialized_pIC_registration = true;
}

// End of code generation (pIC_registration_initialize.cpp)
