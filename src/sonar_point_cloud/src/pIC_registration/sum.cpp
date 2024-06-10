//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// sum.cpp
//
// Code generation for function 'sum'
//

// Include files
#include "sum.h"
#include "rt_nonfinite.h"
#include "coder_array.h"

// Function Definitions
namespace coder {
void sum(const array<double, 2U> &x, double y[2])
{
  if (x.size(1) == 0) {
    y[0] = 0.0;
    y[1] = 0.0;
  } else {
    int firstBlockLength;
    int lastBlockLength;
    int nblocks;
    int xoffset;
    if (x.size(1) <= 1024) {
      firstBlockLength = x.size(1);
      lastBlockLength = 0;
      nblocks = 1;
    } else {
      firstBlockLength = 1024;
      nblocks = static_cast<int>(static_cast<unsigned int>(x.size(1)) >> 10);
      lastBlockLength = x.size(1) - (nblocks << 10);
      if (lastBlockLength > 0) {
        nblocks++;
      } else {
        lastBlockLength = 1024;
      }
    }
    y[0] = x[0];
    y[1] = x[1];
    for (int k{2}; k <= firstBlockLength; k++) {
      xoffset = (k - 1) << 1;
      y[0] += x[xoffset];
      y[1] += x[xoffset + 1];
    }
    for (int ib{2}; ib <= nblocks; ib++) {
      double bsum_idx_0;
      double bsum_idx_1;
      int hi;
      firstBlockLength = (ib - 1) << 11;
      bsum_idx_0 = x[firstBlockLength];
      bsum_idx_1 = x[firstBlockLength + 1];
      if (ib == nblocks) {
        hi = lastBlockLength;
      } else {
        hi = 1024;
      }
      for (int k{2}; k <= hi; k++) {
        xoffset = firstBlockLength + ((k - 1) << 1);
        bsum_idx_0 += x[xoffset];
        bsum_idx_1 += x[xoffset + 1];
      }
      y[0] += bsum_idx_0;
      y[1] += bsum_idx_1;
    }
  }
}

} // namespace coder

// End of code generation (sum.cpp)
