/**
 * eliminator
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

#include <iostream>
#include <vector>

#include "EliminatorGates.hpp"
#include "EliminatorResolution.hpp"
#include "src/utils/ProblemTypes.hpp"

namespace bipe {
namespace eliminator {

class Eliminator {
 private:
  bool m_isInterrupted = false;
  std::vector<bool> m_marked;
  std::vector<bool> m_isUnit;

  EliminatorGates m_elimGates;
  EliminatorResolution m_elimResolution;

 public:
  /**
   * @brief Construct a new Eliminator object
   *
   */
  Eliminator();

  /**
   * @brief Try to eliminate from the given formula some variables regarding a
   * given directed acyclic circuit and by using resolution. We suppose that the
   * given problem is satisfiable.
   *
   * @param nbVar is the number of variables for the given formula.
   * @param[out] clauses is the formula we want to simplify.
   * @param input is the set of input variables.
   * @param dac is a directed acyclic circuit.
   * @param[out] eliminated is the literal we eliminated.
   * @param verbose is set to true if logs are printed out, false otherwise.
   * @param limitNbClause gives the maximum number of clauses we can have.
   */
  void eliminate(unsigned nbVar, std::vector<std::vector<Lit>> &clauses,
                 std::vector<Var> &input, std::vector<Gate> &dac,
                 std::vector<Lit> &eliminated, bool verbose,
                 unsigned limitNbClauses);

  /**
   * @brief Set the interrupted flag to true.
   *
   */
  inline void interrupt() {
    m_isInterrupted = true;
    m_elimGates.interrupt();
    m_elimResolution.interrupt();
  }
};
}  // namespace eliminator
}  // namespace bipe