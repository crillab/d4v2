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

#include <vector>

#include "../PreprocManager.hpp"
#include "3rdParty/bipe/src/bipartition/methods/Method.hpp"
#include "3rdParty/bipe/src/eliminator/Eliminator.hpp"
#include "3rdParty/bipe/src/reducer/Method.hpp"
#include "src/problem/ProblemTypes.hpp"
#include "src/problem/cnf/ProblemManagerCnf.hpp"
#include "src/solvers/WrapperSolver.hpp"

namespace d4 {
class PreprocSharpEquiv : public PreprocManager {
 private:
  int m_nbIteration;
  bool m_isInterrupted = false;

  /**
   * @brief Compute the bipartition.
   *
   * @param pcnf is the CNF we want to compute the bipartition (and some
   * gates).
   * @param[out] units stores the unit literals.
   * @param[out] input stores the input variables.
   * @param[out] output is the set of output variables.
   * @param[out] gates stores the extracted gates.
   * @param timeout is the timeout for computing the bipartition.
   */
  void computeBipartition(ProblemManagerCnf &pcnf, std::vector<Lit> &units,
                          std::vector<bipe::Var> &input,
                          std::vector<bipe::Var> &output,
                          std::vector<bipe::Gate> &gates, unsigned timeout);

 public:
  PreprocSharpEquiv(int nbIteration, std::ostream &out);
  ~PreprocSharpEquiv();
  virtual ProblemManager *run(ProblemManager *pin, unsigned timeout) override;

  /**
   * @brief Stop.
   *
   */
  inline void interrupt() { m_isInterrupted = true; }  // interrupt
};
}  // namespace d4
