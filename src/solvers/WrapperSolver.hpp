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

#include <boost/program_options.hpp>

#include "ActivityManager.hpp"
#include "PolarityManager.hpp"
#include "src/problem/ProblemManager.hpp"
#include "src/problem/ProblemTypes.hpp"

namespace d4 {
namespace po = boost::program_options;
class WrapperSolver : public ActivityManager, public PolarityManager {
 private:
 protected:
  std::vector<char> m_isInAssumption;

 public:
  static WrapperSolver *makeWrapperSolver(po::variables_map &vm,
                                          std::ostream &out);
  static WrapperSolver *makeWrapperSolverPreproc(po::variables_map &vm,
                                                 std::ostream &out);

  virtual ~WrapperSolver() {}
  virtual void initSolver(ProblemManager &p) = 0;
  virtual bool solve(std::vector<Var> &setOfVar) = 0;
  virtual bool solve() = 0;
  virtual void uncheckedEnqueue(Lit l) = 0;
  virtual void restart() = 0;
  virtual void setAssumption(std::vector<Lit> &assums) = 0;
  virtual std::vector<Lit> &getAssumption() = 0;
  virtual void pushAssumption(Lit l) = 0;
  virtual void popAssumption(unsigned count = 1) = 0;
  virtual void displayAssumption(std::ostream &out) = 0;
  virtual bool varIsAssigned(Var v) = 0;
  virtual void setNeedModel(bool b) = 0;
  virtual void showTrail() = 0;
  virtual std::vector<lbool> &getModel() = 0;
  virtual lbool getModelVar(Var v) = 0;
  virtual void getUnits(std::vector<Lit> &units) = 0;
  virtual unsigned getNbConflict() = 0;
  virtual void setReversePolarity(bool value) = 0;
  virtual bool propagateAssumption() = 0;

  // this function returns false if the propagation gives a conflict.
  virtual bool decideAndComputeUnit(Lit l, std::vector<Lit> &units) = 0;

  virtual void whichAreUnits(std::vector<Var> &component,
                             std::vector<Lit> &units) = 0;

  unsigned sizeAssumption() { return getAssumption().size(); }

  bool warmStart(int iteration, int sizeQuery, std::vector<Var> &setOfVar,
                 std::ostream &out);

  /**
     Check out if a variable is already in the assumption.

     @param[in] l, the literal we want to know if it is already in the
     assumption list.

     \return true if l is in the assumption list, false otherwise.
  */
  inline bool isInAssumption(Lit l) {
    return m_isInAssumption[l.var()] == 1 + l.sign();
  }  // isInassumption

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
}  // namespace d4
