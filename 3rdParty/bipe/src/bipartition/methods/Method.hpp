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

#include <chrono>

#include "src/bipartition/option/Option.hpp"
#include "src/bipartition/option/OptionBackbone.hpp"
#include "src/bipartition/option/OptionBipartition.hpp"
#include "src/bipartition/option/OptionDac.hpp"
#include "src/utils/Gate.hpp"
#include "src/utils/Problem.hpp"

namespace bipe {

class WrapperSolver;

namespace bipartition {
class Backbone;

class Method {
 protected:
  unsigned m_timeLimit = 0;
  std::chrono::time_point<std::chrono::system_clock> m_start, m_end;

  bool m_isInterrupted = false;
  class WrapperSolver *m_solver = nullptr;
  class Backbone *m_backboneMethod = nullptr;
  class DACircuit *m_dacMethod = nullptr;

 public:
  virtual ~Method();

  void constructInputFromUnits(Problem &p, std::vector<Lit> &units,
                               std::vector<Var> &input);

  Problem *simplifyOneCall(Problem &p, const std::string &solverName,
                           const unsigned nbConflict, std::vector<Gate> &units,
                           std::ostream &out,
                           std::vector<std::vector<bool>> &setOfModels);

  Problem *simplifyBackbone(Problem &p, const OptionBackbone &optBackbone,
                            std::vector<Gate> &units, std::ostream &out,
                            std::vector<std::vector<bool>> &setOfModels);

  Problem *simplifyDac(Problem &p, const OptionDac &optionDac,
                       std::vector<Gate> &units, std::ostream &out,
                       std::vector<std::vector<bool>> &setOfModels);

  /**
   * @brief Ask if we have still time of computing for the bi-partition.
   *
   * @return true
   * @return false
   */
  inline bool isInterrupt() {
    if (!m_timeLimit) return false;
    m_end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = m_end - m_start;
    return (elapsed_seconds.count() > m_timeLimit);
  }

  virtual void interrupt();
};
}  // namespace bipartition
}  // namespace bipe