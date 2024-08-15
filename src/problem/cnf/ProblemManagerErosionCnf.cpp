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

#include "ProblemManagerErosionCnf.hpp"

#include "ParserErosionDimacs.hpp"
#include "src/problem/ProblemManager.hpp"

namespace d4 {
/**
   Constructor.

   @param[in] nameFile, parse the instance from a file
 */
ProblemManagerErosionCnf::ProblemManagerErosionCnf(
    const std::string &nameFile) {
  ParserErosionDimacs parser;
  m_nbVar = parser.parse_erosion_DIMACS(nameFile, this);

  m_weightVar.resize(m_nbVar + 1, 0);
  for (unsigned i = 0; i <= m_nbVar; i++)
    m_weightVar[i] = m_weightLit[i << 1] + m_weightLit[(i << 1) + 1];
}  // constructor

/**
   Constructor.
   Construct an empty formula.
 */
ProblemManagerErosionCnf::ProblemManagerErosionCnf() {
  m_nbVar = 0;
}  // constructor

/**
 * @brief Construct a new Problem Manager Cnf:: Problem Manager Cnf object
 *
 * @param problem, a problem manager object.
 */
ProblemManagerErosionCnf::ProblemManagerErosionCnf(ProblemManager *problem) {
  m_nbVar = problem->getNbVar();
  m_weightLit = problem->getWeightLit();
  m_weightVar = problem->getWeightVar();
  m_selected = problem->getSelectedVar();
  m_maxVar = problem->getMaxVar();
  m_indVar = problem->getIndVar();
  m_isUnsat = false;

  m_softClauses =
      static_cast<ProblemManagerErosionCnf *>(problem)->getSoftClauses();
  m_hardClauses =
      static_cast<ProblemManagerErosionCnf *>(problem)->getHardClauses();
}  // constructor

/**
 * @brief Construct a new Problem Manager Cnf:: Problem Manager Cnf object
 *
 * @param nbVar, the number of variables.
 * @param weightLit, the weights associate with the literals.
 * @param weightVar, the weights associate with the variables (sum of weight
   of the lit)
 * @param selected, the projected variables.
 * @param maxVar is the set of existential variables.
 * @param indVar is the set of randomized variables.
 */
ProblemManagerErosionCnf::ProblemManagerErosionCnf(
    int nbVar, std::vector<mpz::mpf_float> &weightLit,
    std::vector<mpz::mpf_float> &weightVar, std::vector<Var> &selected,
    std::vector<Var> &maxVar, std::vector<Var> &indVar) {
  m_nbVar = nbVar;
  m_weightLit = weightLit;
  m_weightVar = weightVar;
  m_selected = selected;
  m_maxVar = maxVar;
  m_indVar = indVar;
  m_isUnsat = false;
}  // constructor

/**
 * @brief Construct a new Problem Manager Cnf:: Problem Manager Cnf object
 *
 * @param nbVar, the number of variables.
 * @param weightLit, the weights associate with the literals.
 * @param weightVar, the weights associate with the variables (sum of weight
   of the lit)
 * @param selected, the projected variables.
 */
ProblemManagerErosionCnf::ProblemManagerErosionCnf(
    int nbVar, std::vector<mpz::mpf_float> &weightLit,
    std::vector<mpz::mpf_float> &weightVar, std::vector<Var> &selected) {
  m_nbVar = nbVar;
  m_weightLit = weightLit;
  m_weightVar = weightVar;
  m_selected = selected;
  m_isUnsat = false;
}  // constructor

/**
   Destructor.
 */
ProblemManagerErosionCnf::~ProblemManagerErosionCnf() {
  m_softClauses.clear();
  m_hardClauses.clear();
  m_nbVar = 0;
}  // destructor

/**
 * @brief Get the Unsat ProblemManager object.
 *
 * @return an unsatisfiable problem.
 */
ProblemManager *ProblemManagerErosionCnf::getUnsatProblem() {
  ProblemManagerErosionCnf *ret = new ProblemManagerErosionCnf(this);
  ret->m_isUnsat = true;

  std::vector<Lit> cl;
  Lit l = Lit::makeLit(1, false);

  cl.push_back(l);
  ret->getHardClauses().push_back(cl);

  cl[0] = l.neg();
  ret->getHardClauses().push_back(cl);

  return ret;
}  // getUnsatProblem

/**
 * @brief Simplify the formula by unit propagation and return the resulting
 * CNF formula.
 *
 * @param units is the set of unit literals we want to condition with.
 * @return the simplified formula.
 */
ProblemManager *ProblemManagerErosionCnf::getConditionedFormula(
    std::vector<Lit> &units) {
  ProblemManagerErosionCnf *ret = new ProblemManagerErosionCnf(this);

  std::vector<char> value(m_nbVar + 1, 0);
  for (auto l : units) {
    value[l.var()] = l.sign() + 1;
    ret->getSoftClauses().push_back({l});
  }

  for (auto cl : m_softClauses) {
    // get the simplified clause.
    std::vector<Lit> scl;
    bool isSAT = false;
    for (auto l : cl) {
      if (!value[l.var()]) scl.push_back(l);

      isSAT = l.sign() + 1 == value[l.var()];
      if (isSAT) break;
    }

    // add the simplified clause if needed.
    if (!isSAT) ret->getSoftClauses().push_back(scl);
  }

  for (auto cl : m_hardClauses) {
    // get the simplified clause.
    std::vector<Lit> scl;
    bool isSAT = false;
    for (auto l : cl) {
      if (!value[l.var()]) scl.push_back(l);

      isSAT = l.sign() + 1 == value[l.var()];
      if (isSAT) break;
    }

    // add the simplified clause if needed.
    if (!isSAT) ret->getHardClauses().push_back(scl);
  }

  return ret;
}  // getConditionedFormula

/**
   Display the problem.

   @param[out] out, the stream where the messages are redirected.
 */
void ProblemManagerErosionCnf::display(std::ostream &out) {
  out << "weight list: ";
  for (unsigned i = 1; i <= m_nbVar; i++) {
    Lit l = Lit::makeLit(i, false);
    out << i << "[" << m_weightVar[i] << "] ";
    out << l << "(" << m_weightLit[l.intern()] << ") ";
    out << ~l << "(" << m_weightLit[(~l).intern()] << ") ";
  }
  out << "\n";

  out << "selected var: ";
  for (auto v : getSelectedVar()) out << v << " ";
  out << "\n";

  out << "p cnf " << m_nbVar << " "
      << m_softClauses.size() + m_hardClauses.size() << "\n";
  for (auto cl : m_softClauses) {
    for (auto &l : cl) out << l << " ";
    out << "0\n";
  }

  for (auto cl : m_hardClauses) {
    out << "t ";
    for (auto &l : cl) out << l << " ";
    out << "0\n";
  }
}  // diplay

/**
   Print out some statistic about the problem. Each line will start with the
   string startLine given in parameter.

   @param[in] out, the stream where the messages are redirected.
   @param[in] startLine, each line will start with this string.
 */
void ProblemManagerErosionCnf::displayStat(std::ostream &out,
                                           std::string startLine) {
  unsigned nbSoftLits = 0, nbSoftBin = 0, nbSoftTer = 0, nbSoftMoreThree = 0;
  unsigned nbHardLits = 0, nbHardBin = 0, nbHardTer = 0, nbHardMoreThree = 0;

  for (auto &c : m_softClauses) {
    nbSoftLits += c.size();
    if (c.size() == 2) nbSoftBin++;
    if (c.size() == 3) nbSoftTer++;
    if (c.size() > 3) nbSoftMoreThree++;
  }

  for (auto &c : m_hardClauses) {
    nbHardLits += c.size();
    if (c.size() == 2) nbHardBin++;
    if (c.size() == 3) nbHardTer++;
    if (c.size() > 3) nbHardMoreThree++;
  }

  out << startLine << "Number of variables: " << m_nbVar << "\n";
  out << startLine << "Number of soft clauses: " << m_softClauses.size()
      << "\n";
  out << startLine << "Number of binary soft clauses: " << nbSoftBin << "\n";
  out << startLine << "Number of ternary soft clauses: " << nbSoftTer << "\n";
  out << startLine
      << "Number of soft clauses larger than 3: " << nbSoftMoreThree << "\n";
  out << startLine << "Number of literals in soft clauses: " << nbSoftLits
      << "\n";

  out << startLine << "Number of hard clauses: " << m_hardClauses.size()
      << "\n";
  out << startLine << "Number of binary hard clauses: " << nbHardBin << "\n";
  out << startLine << "Number of ternary hard clauses: " << nbHardTer << "\n";
  out << startLine
      << "Number of hard clauses larger than 3: " << nbHardMoreThree << "\n";
  out << startLine << "Number of literals in hard clauses: " << nbHardLits
      << "\n";
}  // displaystat

}  // namespace d4
