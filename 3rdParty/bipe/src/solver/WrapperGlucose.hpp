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

#include "3rdParty/glucose-3.0/core/Solver.h"
#include "core/SolverTypes.h"
#include "src/solver/WrapperSolver.hpp"
#include "src/utils/Problem.hpp"
#include "src/utils/ProblemTypes.hpp"

namespace bipe {
class WrapperGlucose : public WrapperSolver {
 private:
  Glucose::Solver s;
  Glucose::vec<Glucose::Var> m_setOfVar_m;

  std::vector<Lit> m_assumption;
  std::vector<bool> m_model;
  bool m_activeModel;
  bool m_needModel;

  inline int litToInt(Glucose::Lit l) {
    return Glucose::sign(l) ? -Glucose::var(l) : Glucose::var(l);
  }

 protected:
  using WrapperSolver::m_isInAssumption;

 public:
  ~WrapperGlucose() override;

  void initSolver(Problem &p) override;
  Status solveLimited(int nbConflict) override;
  bool solve() override;
  void uncheckedEnqueue(Lit l) override;
  bool varIsAssigned(Var v) override;

  bool decideAndComputeUnit(Lit l, std::vector<Lit> &units) override;
  bool decideAndTest(std::vector<Lit> &lits, std::vector<Lit> &core) override;
  void restart() override;
  void setAssumption(std::vector<Lit> &assums) override;
  std::vector<Lit> &getAssumption() override;
  void pushAssumption(Lit l) override;
  void popAssumption(unsigned count) override;
  void cleanAssumption() override;
  void displayAssumption(std::ostream &out) override;
  void setNeedModel(bool b) override;
  void showTrail() override;
  std::vector<bool> &getModel() override;
  void getUnits(std::vector<Lit> &units) override;
  void getCore(std::vector<Lit> &core) override;
  void addClauseInit(std::vector<Lit> &cl) override;
  void setLastIndexAssumption(int lastIndex) override;
  void oncePriorityVar(std::vector<Var> &priority) override;
  void setHeapVariable(std::vector<Var> &heapVar) override;

  void setReversePolarity(bool value) override;
  void displayToCnf(std::ostream &out) override;
  void interrupt() override;

  inline void setIncrementalMode(const bool m) override {
    if (m) s.setIncrementalMode();
  }
};
}  // namespace bipe