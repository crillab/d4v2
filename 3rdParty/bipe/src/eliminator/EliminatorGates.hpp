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

#include "src/utils/Gate.hpp"
#include "src/utils/ProblemTypes.hpp"

namespace bipe {
namespace eliminator {

class EliminatorGates {
 private:
  std::vector<bool> m_marked;
  std::vector<bool> m_markedCanBeUsed;
  std::vector<bool> m_isUnit;
  bool m_isInterrupted;

  static const unsigned LIMIT_XOR_SIZE = 5;
  Lit m_memory_xor[1 << LIMIT_XOR_SIZE][5];
  unsigned m_lookupTable[1 << LIMIT_XOR_SIZE];

  /**
   * @brief Consider a set of unit literals and apply BCP on the clauses and the
   * DAC.
   *
   * @param units are the units.
   * @param clauses is the set of clauses.
   * @param occClauses is the occurrence list for the clauses.
   * @param freePlace stores the index of clauses we removed.
   * @param dac is the DAC.
   * @param occDac is the occurrence list for the DAC.
   * @param parentCounter gives for each literal the number of times their are
   * parents.
   * @param eliminated is the set of literal we eliminated.
   *
   * \return false if the problem is proven unsat, true otherwise.
   */
  bool removeUnit(std::vector<Lit> &units,
                  std::vector<std::vector<Lit>> &clauses,
                  std::vector<std::vector<unsigned>> &occClauses,
                  std::vector<unsigned> &freePlace, std::vector<Gate> &dac,
                  std::vector<std::vector<unsigned>> &occDac,
                  std::vector<unsigned> &parentCounter,
                  std::vector<Lit> &eliminated);

  /**
   * @brief Eliminate one equivalence given in parameter.
   *
   * @param units are the units.
   * @param clauses is the set of clauses.
   * @param occClauses is the occurrence list for the clauses.
   * @param freePlace stores the index of clauses we removed.
   * @param g is the gate we use to eliminate the variable.
   * @param idxGate is the index of the gates.
   * @param occDac is the occurrence list for the DAC.
   * @param parentCounter gives for each literal the number of times their are
   * parents.
   * @param eliminated is the set of literal we eliminated.
   *
   */
  void eliminateOneEquiv(std::vector<Lit> &units,
                         std::vector<std::vector<Lit>> &clauses,
                         std::vector<std::vector<unsigned>> &occClauses,
                         std::vector<unsigned> &freePlace, Gate &g,
                         unsigned idxGate,
                         std::vector<std::vector<unsigned>> &occDac,
                         std::vector<unsigned> &parentCounter,
                         std::vector<Lit> &eliminated);

  /**
   * @brief Remove one OR gate given as a parameter.
   *
   * @param units are the units.
   * @param clauses is the set of clauses.
   * @param occClauses is the occurrence list for the clauses.
   * @param freePlace stores the index of clauses we removed.
   * @param g is the gate we use to eliminate the variable.
   * @param idxGate is the index of the gates.
   * @param occDac is the occurrence list for the DAC.
   * @param parentCounter gives for each literal the number of times their are
   * parents.
   * @param eliminated is the set of literal we eliminated.
   */
  void eliminateOneOr(std::vector<Lit> &units,
                      std::vector<std::vector<Lit>> &clauses,
                      std::vector<std::vector<unsigned>> &occClauses,
                      std::vector<unsigned> &freePlace, Gate &g,
                      unsigned idxGate,
                      std::vector<std::vector<unsigned>> &occDac,
                      std::vector<unsigned> &parentCounter,
                      std::vector<Lit> &eliminated);

  /**
   * @brief Remove one XOR gate given as a parameter.
   *
   * @param units are the units.
   * @param clauses is the set of clauses.
   * @param occClauses is the occurrence list for the clauses.
   * @param freePlace stores the index of clauses we removed.
   * @param g is the gate we use to eliminate the variable.
   * @param idxGate is the index of the gates.
   * @param occDac is the occurrence list for the DAC.
   * @param parentCounter gives for each literal the number of times their are
   * parents.
   * @param eliminated is the set of literal we eliminated.
   */
  void eliminateOneXor(std::vector<Lit> &units,
                       std::vector<std::vector<Lit>> &clauses,
                       std::vector<std::vector<unsigned>> &occClauses,
                       std::vector<unsigned> &freePlace, Gate &g,
                       unsigned idxGate,
                       std::vector<std::vector<unsigned>> &occDac,
                       std::vector<unsigned> &parentCounter,
                       std::vector<Lit> &eliminated);

  /**
   * @brief Try to eliminate one variable using one gate.
   *
   * @param units is the set of unit literal (empty at the beginning).
   * @param clauses is the set of clauses.
   * @param occClauses is the occurrence list of clauses.
   * @param freePlace are free place where we can store clauses.
   * @param dac is the DAC.
   * @param occDac is the occurrence list of the DAC.
   * @param parentCounter count the number of times a variable is parent.
   * @param eliminated give the set of eliminated variables.
   * @param limitNbClause gives the maximum number of clauses we can have.
   * @return true if one variables hase been eliminated, false otherwise.
   */
  bool eliminateOneGate(std::vector<Lit> &units,
                        std::vector<std::vector<Lit>> &clauses,
                        std::vector<std::vector<unsigned>> &occClauses,
                        std::vector<unsigned> &freePlace,
                        std::vector<Gate> &dac,
                        std::vector<std::vector<unsigned>> &occDac,
                        std::vector<unsigned> &parentCounter,
                        std::vector<Lit> &eliminated, unsigned limitNbClauses);

  /**
   * @brief Search in occ the idx and remove it.
   *
   * @param occ is the list we want to remove an element.
   * @param idx is the element we want to remove.
   */
  template <class T>
  inline void removeOcc(std::vector<T> &occ, T idx) {
    unsigned pos = 0;
    while (pos < occ.size() && occ[pos] != idx) pos++;
    assert(pos < occ.size());
    occ[pos] = occ.back();
    occ.pop_back();
  }  // removeOcc

  /**
   * @brief Decide if a gate can be used to eliminate a variable.
   *
   * @param g is the gate we test.
   * @param clauses is the set of clauses.
   * @param freePlace is the number of free room.
   * @param occClauses is the occurrence list of clauses.
   * @param limitNbClauses gives the maximum number of clauses we can have after
   * replacing the variable.
   * @return true if we can remove the variable, false otherwise.
   */
  bool canBeUsed(Gate &g, std::vector<std::vector<Lit>> &clauses,
                 std::vector<unsigned> &freePlace,
                 std::vector<std::vector<unsigned>> &occClauses,
                 unsigned limitNbClauses);

 public:
  /**
   * @brief Construct a new Eliminator object.
   * It is deeply link to another eliminator.
   *
   */
  EliminatorGates();

  /**
   * @brief Try to eliminate from the given formula some variables regarding a
   * given directed acyclic circuit. We suppose that the given problem is
   * satisfiable.
   *
   * @param nbVar is the number of variables for the given formula.
   * @param[out] clauses is the formula we want to simplify.
   * @param dac is a directed acyclic circuit.
   * @param[out] eliminated is the literal we eliminated.
   * @param verbose is set to true if logs are printed out, false otherwise.
   * @param limitNbClause gives the maximum number of clauses we can have.
   */
  void eliminate(unsigned nbVar, std::vector<std::vector<Lit>> &clauses,
                 std::vector<Gate> &dac, std::vector<Lit> &eliminated,
                 bool verbose, unsigned limitNbClauses);

  /**
   * @brief Set the interrupted flag to true.
   *
   */
  inline void interrupt() { m_isInterrupted = true; }
};

}  // namespace eliminator
}  // namespace bipe