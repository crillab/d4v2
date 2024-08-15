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
#include "3rdParty/bipe/src/reducer/Method.hpp"
#include "src/problem/ProblemTypes.hpp"
#include "src/solvers/WrapperSolver.hpp"

namespace d4 {
class PreprocReducer : public PreprocManager {
 private:
  std::string m_method;
  int m_nbIteration;
  bipe::reducer::Method *m_isRunning = NULL;

 public:
  PreprocReducer(const std::string &method, int nbIteration, std::ostream &out);
  ~PreprocReducer();
  virtual ProblemManager *run(ProblemManager *pin, unsigned timeout) override;

  /**
   * @brief Stop.
   *
   */
  inline void interrupt() {
    if (m_isRunning) {
      std::cout << "c [REDUCE PREPROC] Stop because timeout\n";
      m_isRunning->interrupt();
    }
  }  // interrupt
};
}  // namespace d4
