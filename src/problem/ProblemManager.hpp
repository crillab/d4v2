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

#include <boost/math/special_functions/math_fwd.hpp>
#include <boost/multiprecision/gmp.hpp>

#include "src/exceptions/FactoryException.hpp"
#include "src/methods/DataBranch.hpp"
#include "src/problem/ProblemTypes.hpp"

namespace d4 {
namespace mpz = boost::multiprecision;

enum ProblemInputType { PB_CNF, PB_TCNF, PB_CIRC, PB_QBF, PB_NONE };

class ProblemInputTypeManager {
 public:
  static std::string getInputType(const ProblemInputType &m) {
    if (m == PB_CNF) return "cnf";
    if (m == PB_CIRC) return "circuit";
    if (m == PB_TCNF) return "cnf+theory";
    if (m == PB_QBF) return "qbf";

    throw(FactoryException("Operator Type unknown", __FILE__, __LINE__));
  }  // getOperatorType

  static ProblemInputType getInputType(const std::string &m) {
    if (m == "cnf") return PB_CNF;
    if (m == "circuit") return PB_CIRC;
    if (m == "tcnf") return PB_TCNF;
    if (m == "qbf") return PB_QBF;

    throw(FactoryException("Operator Type unknown", __FILE__, __LINE__));
  }  // getOperatorType
};

class ProblemManager {
 protected:
  unsigned m_nbVar;
  std::vector<mpz::mpf_float> m_weightLit;
  std::vector<mpz::mpf_float> m_weightVar;
  std::vector<Var> m_selected;
  std::vector<Var> m_maxVar;
  std::vector<Var> m_indVar;
  bool m_isUnsat = false;

 public:
  static ProblemManager *makeProblemManager(const std::string &in,
                                            ProblemInputType pbType,
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
  inline std::vector<mpz::mpf_float> &getWeightLit() { return m_weightLit; }
  inline std::vector<mpz::mpf_float> &getWeightVar() { return m_weightVar; }

  inline mpz::mpf_float getWeightLit(Lit l) { return m_weightLit[l.intern()]; }
  inline mpz::mpf_float getWeightVar(Var v) { return m_weightVar[v]; }

  inline unsigned getNbSelectedVar() { return m_selected.size(); }
  inline bool isUnsat() { return m_isUnsat; }
  inline void isUnsat(bool b) { m_isUnsat = b; }

  inline bool isFloat() {
    for (unsigned i = 0; i < getNbVar(); i++) {
      Lit l = Lit::makeLitTrue(i);
      if (getWeightLit(l) != 1 || getWeightLit(~l) != 1) return true;
    }

    return false;
  }  // isFloat

  virtual ProblemInputType getProblemType() { return PB_NONE; }

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

  /**
   * @brief Compute the value for a branch regarding its unit literals and the
   * free variables.
   *
   * @param[in] b, the branch we want to compute the weight.
   * \return the right value
   */
  template <typename T>
  inline T computeWeightUnitFree(DataBranch<T> &b) {
    return computeWeightUnitFree<T>(b.unitLits, b.freeVars);
  }  // computeWeightUnitFree
};
}  // namespace d4
