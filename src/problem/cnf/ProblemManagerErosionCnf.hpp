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

#include "../ProblemManager.hpp"
#include "../ProblemTypes.hpp"
#include "src/solvers/WrapperSolver.hpp"

namespace d4 {
class ProblemManagerErosionCnf : public ProblemManager {
 private:
  std::vector<std::vector<Lit>> m_softClauses;
  std::vector<std::vector<Lit>> m_hardClauses;

 public:
  ProblemManagerErosionCnf();
  ProblemManagerErosionCnf(int nbVar, std::vector<mpz::mpf_float> &weightLit,
                           std::vector<mpz::mpf_float> &weightVar,
                           std::vector<Var> &selected);
  ProblemManagerErosionCnf(int nbVar, std::vector<mpz::mpf_float> &weightLit,
                           std::vector<mpz::mpf_float> &weightVar,
                           std::vector<Var> &selected, std::vector<Var> &maxVar,
                           std::vector<Var> &indVar);
  ProblemManagerErosionCnf(ProblemManager *problem);

  ProblemManagerErosionCnf(const std::string &nameFile);
  ~ProblemManagerErosionCnf();
  void display(std::ostream &out) override;

  std::vector<std::vector<Lit>> &getSoftClauses() { return m_softClauses; }
  std::vector<std::vector<Lit>> &getHardClauses() { return m_hardClauses; }

  inline void setSoftClauses(std::vector<std::vector<Lit>> &clauses) {
    m_softClauses = clauses;
  }

  inline void setHardClauses(std::vector<std::vector<Lit>> &clauses) {
    m_hardClauses = clauses;
  }

  void displayStat(std::ostream &out, std::string startLine) override;
  ProblemManager *getUnsatProblem() override;
  ProblemManager *getConditionedFormula(std::vector<Lit> &units) override;

  inline ProblemInputType getProblemType() override { return PB_CNF; }
};
}  // namespace d4
