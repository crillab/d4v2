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

#include <map>

#include "Propagator.hpp"

namespace bipe {
namespace reducer {
class Method {
 protected:
  bool m_isInterrupted = false;
  unsigned m_nbRemoveLit;

 public:
  virtual ~Method();

  /**
   * @brief Ask to spot the method.
   *
   */
  virtual void interrupt() { m_isInterrupted = true; }

  /**
   * @brief Method factory.
   *
   * @param meth is the method we want to create.
   * @param out is the stream where are printed out the logs.
   * @return the asked method.
   */
  static Method *makeMethod(const std::string &meth, std::ostream &out);

  /**
   * @brief Run the vivification process.
   *
   * @param problem is the CNF we want to process.
   * @param nbIteration is the number of iteration we want to realize (if -1
   * then we run until we reach a fix point).
   * @param verbose is true if we want to print out information about the
   * process, false otherwise.
   * @param[out] result is the CNF obtained after vivification.
   */
  virtual void run(unsigned nbVar, std::vector<std::vector<Lit>> &clauses,
                   int nbIteration, bool verbose,
                   std::vector<std::vector<Lit>> &result) = 0;

  /**
   * @brief Run the vivification process.
   *
   * @param problem is the CNF we want to process.
   * @param nbIteration is the number of iteration we want to realize (if -1
   * then we run until we reach a fix point).
   * @param verbose is true if we want to print out information about the
   * process, false otherwise.
   */
  virtual void run(unsigned nbVar, std::vector<std::vector<Lit>> &clauses,
                   int nbIteration, bool verbose) = 0;

  /**
   * @brief Run the vivification process.
   *
   * @param propagator is used to manage BCP.
   * @param nbIteration is the number of iteration we want to realize (if -1
   * then we run until we reach a fix point).
   * @param verbose is true if we want to print out information about the
   * process, false otherwise.
   */
  virtual void run(Propagator &propagator, int nbIteration, bool verbose) = 0;

  /**
   * @brief Get the Nb Remove Lit object
   */
  inline unsigned getNbRemoveLit() { return m_nbRemoveLit; }
};
}  // namespace reducer
}  // namespace bipe