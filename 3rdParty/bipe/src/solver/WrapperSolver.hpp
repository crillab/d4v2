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

#include <ostream>
#include <vector>

#include "src/utils/Problem.hpp"
#include "src/utils/ProblemTypes.hpp"

namespace bipe {

enum Status { UNS, SAT, UND };

class WrapperSolver {
 private:
 protected:
  std::vector<char> m_isInAssumption;

 public:
  static WrapperSolver *makeWrapperSolver(const std::string solverName,
                                          std::ostream &out);

  virtual ~WrapperSolver() {}
  virtual void initSolver(Problem &p) = 0;
  virtual Status solveLimited(int nbConflict) = 0;
  virtual bool solve() = 0;
  virtual void uncheckedEnqueue(Lit l) = 0;
  virtual void restart() = 0;
  virtual void setAssumption(std::vector<Lit> &assums) = 0;
  virtual std::vector<Lit> &getAssumption() = 0;
  virtual void pushAssumption(Lit l) = 0;
  virtual void popAssumption(unsigned count = 1) = 0;
  virtual void cleanAssumption() = 0;
  virtual void displayAssumption(std::ostream &out) = 0;
  virtual bool varIsAssigned(Var v) = 0;
  virtual void setNeedModel(bool b) = 0;
  virtual void showTrail() = 0;
  virtual std::vector<bool> &getModel() = 0;
  virtual void getUnits(std::vector<Lit> &units) = 0;
  virtual void setReversePolarity(bool value) = 0;
  virtual void displayToCnf(std::ostream &out) = 0;
  virtual void interrupt() = 0;
  virtual void setIncrementalMode(const bool m) = 0;
  virtual void getCore(std::vector<Lit> &core) = 0;
  virtual void addClauseInit(std::vector<Lit> &cl) = 0;
  virtual void setLastIndexAssumption(int lastIndex) = 0;
  virtual void oncePriorityVar(std::vector<Var> &priority) = 0;
  virtual void setHeapVariable(std::vector<Var> &heapVar) = 0;

  // this function returns false if the propagation gives a conflict.
  virtual bool decideAndComputeUnit(Lit l, std::vector<Lit> &units) = 0;
  virtual bool decideAndTest(std::vector<Lit> &lits,
                             std::vector<Lit> &core) = 0;

  unsigned sizeAssumption() { return getAssumption().size(); }

  /**
   Check out if a variable is already in the assumption.

   @param[in] v, the variable we want to know if it is already in the
   assumption list.

   \return true if v is in the assumption list, false otherwise.
*/
  inline bool isInAssumption(Var v) {
    return m_isInAssumption[v];
  }  // isInassumption

  /**
   * @brief Pop all the element of the assumption.
   *
   */
  inline void resetAssumption() { popAssumption(getAssumption().size()); }
};
}  // namespace bipe
