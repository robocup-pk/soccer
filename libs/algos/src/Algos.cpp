#include "Algos.h"
#include <vector>
#include <queue>
#include <unordered_map>
#include <queue>
#include <cmath>
#include <algorithm>
#include <set>
#include <functional>

namespace algos {
state::Waypoint FindDirectionVector(const state::Waypoint& start, const state::Waypoint& goal) {
  state::Waypoint direction_vector = goal - start;
  return direction_vector.Normalize();
}

}  // namespace algos