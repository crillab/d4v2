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

#include "ProblemManagerCircuit.hpp"

#include "ParserCircuit.hpp"
#include "src/problem/ProblemManager.hpp"

namespace d4 {

/**
 * @brief Construct a new Problem Manager Circuit:: Problem Manager Circuit
 * object
 *
 * @param nameFile
 */
ProblemManagerCircuit::ProblemManagerCircuit(const std::string &nameFile) {
  ParserCircuit parser;
  m_nbVar = parser.parse_Circuit(nameFile, this);

  m_weightVar.resize(m_nbVar + 1, 0);
  for (unsigned i = 0; i <= m_nbVar; i++)
    m_weightVar[i] = m_weightLit[i << 1] + m_weightLit[(i << 1) + 1];
}

/**
 * @brief Construct a new Problem Manager Circuit:: Problem Manager Circuit
 * object
 *
 */
ProblemManagerCircuit::ProblemManagerCircuit() { m_nbVar = 0; }

/**
 * @brief Construct a new Problem Manager Circuit:: Problem Manager Circuit
 * object
 *
 * @param problem
 */
ProblemManagerCircuit::ProblemManagerCircuit(ProblemManager *problem) {
  m_nbVar = problem->getNbVar();
  m_weightLit = problem->getWeightLit();
  m_weightVar = problem->getWeightVar();
  m_selected = problem->getSelectedVar();
  m_maxVar = problem->getMaxVar();
  m_indVar = problem->getIndVar();
  m_isUnsat = false;
}  // constructor

/**
 * @brief Destroy the Problem Manager Circuit:: Problem Manager Circuit object
 *
 */
ProblemManagerCircuit::~ProblemManagerCircuit() { m_nbVar = 0; }

/**
 * @brief
 *
 * @param out
 */
void ProblemManagerCircuit::display(std::ostream &out) {
  out << "p noncnf " << m_nbVar << "\n";
  assert(gates.size() == wires.size() && gates.size() == m_nbVar);

  for (unsigned i = 0; i < gates.size(); i++) {
    if (!gates[i]) continue;
    out << gates[i] << " -1 ";
    out << i + 1 << " ";

    for (auto &v : wires[i]) out << v << " ";
    out << "0\n";
  }
}  // display

/**
 * @brief
 *
 * @param out
 * @param startLine
 */
void ProblemManagerCircuit::displayStat(std::ostream &out,
                                        std::string startLine) {
  out << "c problemManagerCircuit displayStat() -- not implemented yet\n";
  // TODO
  // Afficher le nombre de portes
  // Afficher le nomnbre de variables
  // Stat sur le circuit
}  // displayStat

ProblemManager *ProblemManagerCircuit::getUnsatProblem() {
  std::cerr << "Method not corectly implemented yet, just used for compilation "
               "purposes, do not use !";
  ProblemManagerCircuit *ret = new ProblemManagerCircuit();
  return ret;
}

/**
 * @brief
 *
 * @param units
 * @return ProblemManager*
 */
ProblemManager *ProblemManagerCircuit::getConditionedFormula(
    std::vector<Lit> &units) {
  std::cerr << "Method not corectly implemented yet, just used for compilation "
               "purposes, do not use !";
  assert(0);
  ProblemManagerCircuit *ret = new ProblemManagerCircuit();
  return ret;
}

}  // namespace d4
