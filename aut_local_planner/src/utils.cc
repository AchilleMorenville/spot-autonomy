#include "aut_local_planner/utils.h"

#include <utility>

namespace aut_local_planner {

Motion::Motion() : d_x(0.0f), d_y(0.0f), d_theta(0.0f) {}

Motion::Motion(const float d_x, const float d_y, const float d_theta) : d_x(d_x), d_y(d_y), d_theta(d_theta) {}

Motion::Motion(const Motion& m) : d_x(m.d_x), d_y(m.d_y), d_theta(m.d_theta) {}

Motion& Motion::operator=(Motion m) {
  std::swap(d_x, m.d_x);
  std::swap(d_y, m.d_y);
  std::swap(d_theta, m.d_theta);
  return *this;
}

Motion::~Motion() {}

bool Motion::IsStatic() {
  return d_x == 0.0f && d_y == 0.0f && d_theta == 0.0f;
}

}  // namespace aut_local_planner