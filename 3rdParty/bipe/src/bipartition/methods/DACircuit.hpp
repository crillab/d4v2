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

#include <vector>

#include "src/bipartition/methods/Method.hpp"
#include "src/bipartition/option/Option.hpp"
#include "src/solver/WrapperSolver.hpp"
#include "src/utils/Gate.hpp"
#include "src/utils/Problem.hpp"

namespace bipe {
namespace bipartition {

class DACircuit : public Method {
 private:
  const unsigned MAX_SIZE_XOR = 5;

  bool m_interrupted = false;
  WrapperSolver *m_solver = nullptr;

  std::vector<bool> m_markedProtected;
  std::vector<bool> m_markedAsOutput;
  std::vector<bool> m_markedAsProjected;
  std::vector<bool> m_marker;
  std::vector<bool> m_markerDescendant;

  std::vector<Var> m_mustUnmark;
  std::vector<Var> m_mustUnmarkDescendant;

  std::vector<std::vector<Var>> m_edgeIn;
  std::vector<std::vector<Var>> m_edgeOut;
  std::vector<std::vector<Lit>> m_impliedList;

  void constructImpliedList(Problem &p, WrapperSolver *solver,
                            std::vector<std::vector<Lit>> &impliedList,
                            std::vector<Lit> &units);

  void extractEquivClass(Problem &p, std::vector<Gate> &gates,
                         std::vector<std::vector<Lit>> &equivClassList);

  unsigned getScore(Var v);

  void identifyEquiv(Problem &p, std::vector<Gate> &listOfGates,
                     std::vector<Lit> &units, std::ostream &out);

  void identifyAndGate(Problem &p, std::vector<Gate> &listOfGates,
                       std::ostream &out);

  void identifyXorGate(Problem &p, std::vector<Gate> &listOfGates,
                       std::ostream &out);

  void init(Problem &p, std::vector<Lit> &units);

  void addRelation(std::vector<Var> &in, std::vector<Var> &out);

  bool canWeBuildXor(std::vector<std::vector<Lit>> &clauses,
                     std::vector<int> &indices, std::vector<Lit> &xorClause);

  void markDescendant(Var v);
  void unmarkDescendant();

 public:
  void interrupt();
  bool run(Problem &p, std::vector<Gate> &listOfGates,
           std::vector<std::vector<bool>> &setOfModels, const OptionDac &optDac,
           std::ostream &out);
};
}  // namespace bipartition
}  // namespace bipe