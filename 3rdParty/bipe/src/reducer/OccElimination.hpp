/**
 * reducer
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

#include "Method.hpp"
#include "Propagator.hpp"

namespace bipe {
namespace reducer {
class OccElimination : public Method {
 private:
  std::ostream &m_out;

  /**
   * @brief Try to unit literals from l and the binary clause ~l is in.
   *
   * @param propagator is the class that manage the BCP process.
   * @param l is the central literal.
   */
  void occRemoveBin(Propagator &propagator, Lit l);

  /**
   * @brief Generate the occurrence list regarding the non binary clause.
   *
   * @param propagator gives the set of clauses.
   * @param[out] occurrence is the occurrence list we want to compute.
   */
  void generateOccurrenceLit(Propagator &propagator,
                             std::vector<std::vector<CRef>> &occurrence);

  /**
   * @brief Generate the sorted list of literals.
   *
   * @param propagator gives the set of clauses.
   * @param occurrence is the occurrence list.
   * @param[out] litList is the list of literal we get.
   */
  void generateLitList(Propagator &propagator,
                       std::vector<std::vector<CRef>> &occurrence,
                       std::vector<Lit> &litList);

  /**
   * @brief Apply the lit elimination process on large clauses.
   *
   * @param propagator gives the set of clauses.
   * @param occurrence gives the occurrence list for each literal.
   */
  void removeLitFromLargeClauses(Propagator &propagator,
                                 std::vector<std::vector<CRef>> &occurrence);

 public:
  /**
   * @brief Construct a new OccElimination object
   *
   * @param out is stream where will be printed out the log.
   */
  OccElimination(std::ostream &out);

  /**
   * @brief Run the OccElimination process.
   *
   * @param problem is the CNF we want to process.
   * @param nbIteration is the number of iteration we want to realize (if -1
   * then we run until we reach a fix point).
   * @param verbose is true if we want to print out information about the
   * process, false otherwise.
   * @param[out] result is the CNF obtained after OccElimination.
   */
  void run(unsigned nbVar, std::vector<std::vector<Lit>> &clauses,
           int nbIteration, bool verbose,
           std::vector<std::vector<Lit>> &result);

  /**
   * @brief Run the OccElimination process.
   *
   * @param problem is the CNF we want to process.
   * @param nbIteration is the number of iteration we want to realize (if -1
   * then we run until we reach a fix point).
   * @param verbose is true if we want to print out information about the
   * process, false otherwise.
   */
  void run(unsigned nbVar, std::vector<std::vector<Lit>> &clauses,
           int nbIteration, bool verbose);

  /**
   * @brief Run the OccElimination process.
   *
   * @param propagator is used to manage BCP.
   * @param nbIteration is the number of iteration we want to realize (if -1
   * then we run until we reach a fix point).
   * @param verbose is true if we want to print out information about the
   * process, false otherwise.
   */
  void run(Propagator &propagator, int nbIteration, bool verbose);

  /**
   * @brief Print information.
   */
  void displayInfo();
};
}  // namespace reducer
}  // namespace bipe