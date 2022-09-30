/*
 * d4
 * Copyright (C) 2020  Univ. Artois & CNRS
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include "../ProblemManager.hpp"
#include "../ProblemTypes.hpp"
#include "src/solvers/WrapperSolver.hpp"

namespace d4 {
class ProblemManagerCnf : public ProblemManager {
 private:
  std::vector<std::vector<Lit>> m_clauses;

 public:
  ProblemManagerCnf();
  ProblemManagerCnf(int nbVar, std::vector<double> &weightLit,
                    std::vector<double> &weightVar, std::vector<Var> &selected);
  ProblemManagerCnf(ProblemManager *problem);

  ProblemManagerCnf(std::string &nameFile);
  ~ProblemManagerCnf();
  void display(std::ostream &out) override;
  std::vector<std::vector<Lit>> &getClauses() { return m_clauses; }
  void setClauses(std::vector<std::vector<Lit>> &clauses) {
    m_clauses = clauses;
  }
  void displayStat(std::ostream &out, std::string startLine) override;
  ProblemManager *getUnsatProblem() override;
  ProblemManager *getConditionedFormula(std::vector<Lit> &units) override;
};
}  // namespace d4
