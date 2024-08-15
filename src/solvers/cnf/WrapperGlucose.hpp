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

#include "../WrapperSolver.hpp"
#include "3rdParty/glucose-3.0/core/Solver.h"
#include "src/problem/ProblemManager.hpp"
#include "src/problem/ProblemTypes.hpp"

namespace d4 {
class WrapperGlucose : public WrapperSolver {
 private:
  Glucose::Solver s;
  Glucose::vec<Glucose::Var> m_setOfVar_m;

  std::vector<Lit> m_assumption;
  std::vector<lbool> m_model;
  bool m_activeModel;
  bool m_needModel;

 protected:
  using WrapperSolver::m_isInAssumption;

 public:
  void initSolver(ProblemManager &p) override;
  bool solve(std::vector<Var> &setOfVar) override;
  bool solve() override;
  void uncheckedEnqueue(Lit l) override;
  bool varIsAssigned(Var v) override;
  bool getPolarity(Var v) override;
  bool decideAndComputeUnit(Lit l, std::vector<Lit> &units) override;
  void whichAreUnits(std::vector<Var> &component,
                     std::vector<Lit> &units) override;
  void restart() override;
  void setAssumption(std::vector<Lit> &assums) override;
  std::vector<Lit> &getAssumption() override;
  void pushAssumption(Lit l) override;
  void popAssumption(unsigned count) override;
  void displayAssumption(std::ostream &out) override;
  void setNeedModel(bool b) override;
  void showTrail() override;
  std::vector<lbool> &getModel() override;
  lbool getModelVar(Var v) override;
  void getUnits(std::vector<Lit> &units) override;
  bool propagateAssumption() override;

  double getActivity(Var v) override;
  double getCountConflict(Var v) override;
  void setCountConflict(Var v, double count) override;
  unsigned getNbConflict() override;
  void setReversePolarity(bool value) override;
  void decayCountConflict() override;
  bool isUnsat() override;

  inline void getCore() { assert(0); }
  inline void getLastIUP(Lit l) { assert(0); }
};
}  // namespace d4