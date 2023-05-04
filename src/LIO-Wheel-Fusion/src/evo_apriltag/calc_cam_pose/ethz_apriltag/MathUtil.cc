#include "calc_cam_pose/ethz_apriltag/MathUtil.h"

namespace AprilTags {

// Output operator for std::pair<float,float>, useful for debugging
std::ostream &operator<<(std::ostream &os, const std::pair<float, float> &pt) {
  os << pt.first << "," << pt.second;
  return os;
}

}  // namespace
