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

#include <vector>

#include "src/utils/ProblemTypes.hpp"

namespace bipe {
namespace eliminator {
class EliminatorResolution {
 private:
  std::vector<bool> m_marked;
  std::vector<bool> m_isUnit;
  bool m_isInterrupt = false;
  unsigned m_largerClauses;

  /**
   * @brief Select a variable to be forget.
   *
   * @param vars is the set of variables we search for a candidate.
   * @param occClauses is the occurrence list of clauses.
   * @return a variable from vars, var_Undef if vars is empty.
   */
  Var selectVarAndPop(std::vector<Var> &vars,
                      std::vector<std::vector<unsigned>> &occClauses);

  /**
   * @brief
   *
   * @param v
   * @param clauses
   * @param occClauses
   * @param result
   */
  bool generateAllResolution(Var v, std::vector<std::vector<Lit>> &clauses,
                             std::vector<std::vector<unsigned>> &occClauses,
                             std::vector<std::vector<Lit>> &result);

  /**
   * @brief
   *
   * @param occ
   * @param idx
   */
  inline void removeOcc(std::vector<unsigned> &occ, unsigned idx) {
    unsigned pos = 0;
    while (pos < occ.size() && occ[pos] != idx) pos++;
    assert(pos < occ.size());
    occ[pos] = occ.back();
    occ.pop_back();
  }  // removeOcc

 public:
  /**
   * @brief Try to eliminate some output variable.
   *
   * @param nbVar is the number of variables for the input formula.
   * @param clauses is the CNF.
   * @param input are the input variables.
   * @param[out] eliminated are the elminated variables.
   * @param verbose is set to true if we want to print out logs.
   * @param limitNbClauses is the maximum number of clauses allowed.
   */
  void eliminate(unsigned nbVar, std::vector<std::vector<Lit>> &clauses,
                 std::vector<Var> &input, std::vector<Lit> &eliminated,
                 bool verbose, unsigned limitNbClauses);

  /**
   * @brief Ask to interrupt the process.
   *
   */
  inline void interrupt() { m_isInterrupt = true; }
};
}  // namespace eliminator
}  // namespace bipe