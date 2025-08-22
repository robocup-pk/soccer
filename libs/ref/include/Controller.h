#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <vector>

#include "SoccerObject.h"
#include "AutoRef.h"
#include "Game.h"

namespace ref {

class Controller {
 public:
  void DoFouls(std::vector<state::SoccerObject>& soccer_objects, Game& g, ref::AutoRef& referee);
};

};  // namespace ref

#endif  // CONTROLLER_H