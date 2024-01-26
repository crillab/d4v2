/*
 * d4
 * Copyright (C) 2020  Univ. Artois & CNRS
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include "src/preprocs/PreprocManager.hpp"
#include "src/problem/ProblemManager.hpp"
#include "src/config/Config.hpp"

namespace d4 {
class MethodManager {
 protected:
  std::clock_t currentTime;

 public:
  virtual ~MethodManager() {}

  static MethodManager *makeMethodManager(Config &config,
                                          std::ostream &out);

  static MethodManager *makeMethodManager(Config &config,
                                          ProblemManager *problem,
                                          std::string meth, int precision,
                                          bool isFloat, std::ostream &out);

  static void displayInfoVariables(ProblemManager *problem, std::ostream &out);

  static ProblemManager *runPreproc(Config &config,
                                    ProblemManager *initProblem,
                                    std::ostream &out,
                                    LastBreathPreproc &lastBreath);

  virtual void run(Config &config) = 0;
  virtual void interrupt() {}

  inline void initTimer() { currentTime = clock(); }
  inline float getTimer() {
    return (float)(clock() - currentTime) / CLOCKS_PER_SEC;
  }
};
}  // namespace d4
