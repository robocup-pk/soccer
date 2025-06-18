#include "Algos.h"

state::Waypoint FindDirectionVector(const state::Waypoint& start, const state::Waypoint& goal) {
  state::Waypoint direction_vector = goal - start;
  return direction_vector.Normalize();
}