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

#include "src/utils/ProblemTypes.hpp"

namespace bipe {
class Problem {
 private:
  std::vector<std::vector<Lit>> m_clauses;
  std::vector<double> m_weightLit;
  std::vector<Var> m_projected;
  std::vector<Var> m_protected;
  std::vector<Var> m_input;
  std::vector<Var> m_output;
  std::vector<Var> m_heapVariable;
  unsigned m_nbVar;

 public:
  Problem();
  Problem(int nbVar, std::vector<double> &weightLit, std::vector<Var> &selected,
          std::vector<Var> &protect);
  Problem(Problem *problem);
  Problem(Problem &problem);
  ~Problem();

  inline unsigned getNbVar() { return m_nbVar; }
  inline std::vector<Var> &getProjectedVar() { return m_projected; }
  inline std::vector<Var> &getProtectedVar() { return m_protected; }
  inline std::vector<double> &getWeightLit() { return m_weightLit; }
  inline std::vector<std::vector<Lit>> &getClauses() { return m_clauses; }
  inline std::vector<Var> &getInput() { return m_input; }
  inline std::vector<Var> &getOutput() { return m_output; }
  inline std::vector<Var> &getHeapVariable() { return m_heapVariable; }

  inline void setNbVar(unsigned nbVar) { m_nbVar = nbVar; }
  inline void setClauses(std::vector<std::vector<Lit>> &clauses) {
    m_clauses = clauses;
  }

  Problem *getUnsatProblem();
  Problem *getConditionedFormula(std::vector<Lit> &units);

  void display(std::ostream &out);
  void displayStat(std::ostream &out, std::string startLine);
};
}  // namespace bipe
