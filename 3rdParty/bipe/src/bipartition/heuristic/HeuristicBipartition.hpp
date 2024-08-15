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

#include "src/utils/Problem.hpp"
#include "src/utils/ProblemTypes.hpp"

namespace bipe {
namespace bipartition {
class HeuristicBipartition {
 protected:
  std::vector<Lit> m_assumptions;

 public:
  virtual ~HeuristicBipartition() {}

  static HeuristicBipartition *makeHeuristicBipartition(
      Problem &p, const std::vector<Lit> &selectors, const std::string &method,
      std::ostream &out);
  inline std::vector<Lit> &remainingAssumptions() { return m_assumptions; }
  inline bool isEmpty() { return !m_assumptions.size(); }

  inline void popVarFromAssumption(Var v) {
    unsigned j = 0;
    for (unsigned i = 0; i < m_assumptions.size(); i++)
      if (m_assumptions[i].var() != v) m_assumptions[j++] = m_assumptions[i];
    m_assumptions.resize(j);
  }

  inline void setAssumption(std::vector<Lit> &assumptions) {
    m_assumptions = assumptions;
  }
  virtual Lit nextAssumption();
};
}  // namespace bipartition
}  // namespace bipe