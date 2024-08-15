/*
 * d4
 * Copyright (C) 2020  Univ. Artois & CNRS
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
 */
#pragma once

#include "../CnfMatrix.hpp"
#include "../ProblemManager.hpp"
#include "../ProblemTypes.hpp"
#include "src/solvers/WrapperSolver.hpp"

namespace d4 {
class ProblemManagerCnf : public ProblemManager, public CnfMatrix {
 public:
  ProblemManagerCnf();

  ProblemManagerCnf(int nbVar, std::vector<mpz::mpf_float> &weightLit,
                    std::vector<mpz::mpf_float> &weightVar,
                    std::vector<Var> &selected);

  ProblemManagerCnf(int nbVar, std::vector<mpz::mpf_float> &weightLit,
                    std::vector<mpz::mpf_float> &weightVar,
                    std::vector<Var> &selected, std::vector<Var> &maxVar,
                    std::vector<Var> &indVar);

  ProblemManagerCnf(ProblemManager *problem);

  ProblemManagerCnf(const std::string &nameFile);
  ProblemManagerCnf(const int fd, bool keepOpen = false);

  ~ProblemManagerCnf();
  void display(std::ostream &out) override;
  void displayStat(std::ostream &out, std::string startLine) override;
  ProblemManager *getUnsatProblem() override;
  ProblemManager *getConditionedFormula(std::vector<Lit> &units) override;

  inline ProblemInputType getProblemType() override { return PB_CNF; }
};
}  // namespace d4
