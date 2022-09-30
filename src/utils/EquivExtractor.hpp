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

#include <vector>

#include "src/problem/ProblemTypes.hpp"
#include "src/solvers/WrapperSolver.hpp"

namespace d4 {
class EquivExtractor {
 private:
  std::vector<bool> m_markedVar;
  std::vector<bool> m_markedVarInter;
  std::vector<bool> m_flagVar;

 public:
  EquivExtractor() { ; }  // empty constructor
  EquivExtractor(int nbVar);
  void initEquivExtractor(int nbVar);
  bool interCollectUnit(WrapperSolver &s, Var v, std::vector<Var> &listVarPU,
                        std::vector<bool> &flagVar);

  void searchEquiv(WrapperSolver &s, std::vector<Var> &v,
                   std::vector<std::vector<Var> > &equivVar);
};
}  // namespace d4
