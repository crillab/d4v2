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

#include <boost/math/special_functions/math_fwd.hpp>
#include <boost/multiprecision/gmp.hpp>
#include <boost/program_options.hpp>

#include "src/problem/ProblemTypes.hpp"

namespace d4 {
namespace po = boost::program_options;
class ProblemManager {
 protected:
  unsigned m_nbVar;
  std::vector<double> m_weightLit;
  std::vector<double> m_weightVar;
  std::vector<Var> m_selected;
  std::vector<Var> m_maxVar;
  std::vector<Var> m_indVar;
  bool m_isUnsat = false;

 public:
  static ProblemManager *makeProblemManager(po::variables_map &vm,
                                            std::ostream &out);

  virtual ~ProblemManager() { ; }
  unsigned getNbVar() { return m_nbVar; }
  void setNbVar(int n) { m_nbVar = n; }

  virtual void display(std::ostream &out) = 0;
  virtual void displayStat(std::ostream &out, std::string startLine) = 0;
  virtual ProblemManager *getUnsatProblem() = 0;
  virtual ProblemManager *getConditionedFormula(std::vector<Lit> &units) = 0;

  inline std::vector<Var> &getSelectedVar() { return m_selected; }
  inline std::vector<Var> &getMaxVar() { return m_maxVar; }
  inline std::vector<Var> &getIndVar() { return m_indVar; }
  inline std::vector<double> &getWeightLit() { return m_weightLit; }
  inline std::vector<double> &getWeightVar() { return m_weightVar; }

  inline double getWeightLit(Lit l) { return m_weightLit[l.intern()]; }
  inline double getWeightVar(Var v) { return m_weightVar[v]; }

  inline unsigned getNbSelectedVar() { return m_selected.size(); }
  inline bool isUnsat() { return m_isUnsat; }
  inline void isUnsat(bool b) { m_isUnsat = b; }

  /**
     Get the weight for a variable.
   */
  template <typename T>
  inline T getWeightVar(Var v) {
    return T(m_weightVar[v]);
  }  // getWeightLar

  /**
     Get the weight for a literal.
   */
  template <typename T>
  inline T getWeightLit(Lit l) {
    return T(m_weightLit[l.intern()]);
  }  // getWeightLit

  /**
     Compute the value for free and unit variables.

     @param[in] units, the units variables
     @param[in] frees, the free variables

     \return the right value
  */
  template <typename T>
  inline T computeWeightUnitFree(std::vector<Lit> &units,
                                 std::vector<Var> &frees) {
    T tmp = 1;
    for (auto &l : units) {
      assert(l.intern() < m_weightLit.size());
      tmp *= T(m_weightLit[l.intern()]);
    }
    for (auto &v : frees) {
      assert(v < (int)m_weightVar.size());
      tmp *= T(m_weightVar[v]);
    }

    return tmp;
  }  // computeWeightUnitFree
};
}  // namespace d4
