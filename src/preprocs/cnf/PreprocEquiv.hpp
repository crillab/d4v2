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
#include "3rdParty/bipe/src/reducer/Method.hpp"
#include "src/problem/ProblemTypes.hpp"
#include "src/solvers/WrapperSolver.hpp"

namespace d4 {
class PreprocEquiv : public PreprocManager {
 private:
  int m_nbIteration;
  bool m_isInterrupted = false;
  bipe::reducer::Method *m_isRunningReducer = NULL;
  bipe::bipartition::Method *m_isRunningBackbone = NULL;

 public:
  PreprocEquiv(int nbIteration, std::ostream &out);
  ~PreprocEquiv();
  virtual ProblemManager *run(ProblemManager *pin, unsigned timeout) override;

  /**
   * @brief Stop.
   *
   */
  inline void interrupt() {
    m_isInterrupted = true;
    if (m_isRunningReducer) {
      std::cout << "c [EQUIV PREPROC] Stop the reducer because timeout\n";
      m_isRunningReducer->interrupt();
    }

    if (m_isRunningBackbone) {
      std::cout
          << "c [EQUIV PREPROC] Stop the backbone extractor because timeout\n";
      m_isRunningBackbone->interrupt();
    }
  }  // interrupt
};
}  // namespace d4
