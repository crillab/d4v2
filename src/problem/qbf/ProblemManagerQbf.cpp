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

#include "ProblemManagerQbf.hpp"

#include "ParserQDimacs.hpp"
#include "src/problem/ProblemManager.hpp"

namespace d4 {
/**
   Constructor.

   @param[in] nameFile, parse the instance from a file
 */
ProblemManagerQbf::ProblemManagerQbf(const std::string &nameFile) {
  ParserQDimacs parser;
  m_nbVar = parser.parse_QDIMACS(nameFile, this);

  m_weightLit.resize((m_nbVar + 1) << 1, 1);
  m_weightVar.resize(m_nbVar + 1, 2);
}  // constructor

/**
   Constructor.

   @param[in] fd, parse the instance from the file descriptor.
 */
ProblemManagerQbf::ProblemManagerQbf(const int fd, bool keepOpen) {
  ParserQDimacs parser;
  m_nbVar = parser.parse_QDIMACS(fd, this, keepOpen);

  m_weightVar.resize(m_nbVar + 1, 0);
  for (unsigned i = 0; i <= m_nbVar; i++)
    m_weightVar[i] = m_weightLit[i << 1] + m_weightLit[(i << 1) + 1];
}  // constructor

/**
   Constructor.
   Construct an empty formula.
 */
ProblemManagerQbf::ProblemManagerQbf() { m_nbVar = 0; }  // constructor

/**
   Destructor.
 */
ProblemManagerQbf::~ProblemManagerQbf() {
  m_clauses.clear();
  m_nbVar = 0;
}  // destructor

/**
 * @brief Get the Unsat ProblemManager object.
 *
 * @return an unsatisfiable problem.
 */
ProblemManager *ProblemManagerQbf::getUnsatProblem() {
  ProblemManagerQbf *ret = new ProblemManagerQbf();
  ret->m_isUnsat = true;
  ret->setNbVar(m_nbVar);
  ret->setBlocks(m_qblocks);

  std::vector<Lit> cl;
  Lit l = Lit::makeLit(1, false);

  cl.push_back(l);
  ret->getClauses().push_back(cl);

  cl[0] = l.neg();
  ret->getClauses().push_back(cl);

  return ret;
}  // getUnsatProblem

/**
 * @brief Simplify the formula by unit propagation and return the resulting CNF
 * formula.
 *
 * @param units is the set of unit literals we want to condition with.
 * @return the simplified formula.
 */
ProblemManager *ProblemManagerQbf::getConditionedFormula(
    std::vector<Lit> &units) {
  ProblemManagerQbf *ret = new ProblemManagerQbf();
  ret->setNbVar(m_nbVar);
  ret->setBlocks(m_qblocks);

  std::vector<char> value(m_nbVar + 1, 0);
  for (auto l : units) {
    value[l.var()] = l.sign() + 1;
    ret->getClauses().push_back({l});
  }

  for (auto cl : m_clauses) {
    // get the simplified clause.
    std::vector<Lit> scl;
    bool isSAT = false;
    for (auto l : cl) {
      if (!value[l.var()]) scl.push_back(l);

      isSAT = l.sign() + 1 == value[l.var()];
      if (isSAT) break;
    }

    // add the simplified clause if needed.
    if (!isSAT) ret->getClauses().push_back(scl);
  }

  return ret;
}  // getConditionedFormula

/**
   Display the problem.

   @param[out] out, the stream where the messages are redirected.
 */
void ProblemManagerQbf::display(std::ostream &out) {
  out << "p cnf " << m_nbVar << " " << m_clauses.size() << "\n";

  for (auto &b : m_qblocks) {
    out << (b.isUniversal ? "a " : "e ");
    for (auto &v : b.variables) out << v << ' ';
    out << "0\n";
  }

  for (auto cl : m_clauses) {
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
void ProblemManagerQbf::displayStat(std::ostream &out, std::string startLine) {
  unsigned nbLits = 0;
  unsigned nbBin = 0;
  unsigned nbTer = 0;
  unsigned nbMoreThree = 0;

  for (auto &c : m_clauses) {
    nbLits += c.size();
    if (c.size() == 2) nbBin++;
    if (c.size() == 3) nbTer++;
    if (c.size() > 3) nbMoreThree++;
  }

  out << startLine << "Number of variables: " << m_nbVar << "\n";
  out << startLine << "Number of clauses: " << m_clauses.size() << "\n";
  out << startLine << "Number of binary clauses: " << nbBin << "\n";
  out << startLine << "Number of ternary clauses: " << nbTer << "\n";
  out << startLine << "Number of clauses larger than 3: " << nbMoreThree
      << "\n";
  out << startLine << "Number of literals: " << nbLits << "\n";
}  // displaystat

}  // namespace d4
