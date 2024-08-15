/**
 * bipe
 *  Copyright (C) 2021  Lagniez Jean-Marie
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Affero General Public License as published
 *  by the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Affero General Public License for more details.
 *
 *  You should have received a copy of the GNU Affero General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#pragma once

#include "src/bipartition/methods/Method.hpp"
#include "src/bipartition/option/OptionBackbone.hpp"
#include "src/utils/Gate.hpp"
#include "src/utils/Problem.hpp"
#include "src/utils/ProblemTypes.hpp"

namespace bipe {
namespace bipartition {
class Backbone : public Method {
 private:
  class WrapperSolver *m_solver = nullptr;

 public:
  ~Backbone();
  Backbone();

  void interrupt();

  bool run(Problem &p, std::vector<Gate> &backbone,
           std::vector<std::vector<bool>> &setOfModels, OptionBackbone option,
           std::ostream &out);
};
}  // namespace bipartition
}  // namespace bipe