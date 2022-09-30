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
#include <cassert>
#include <src/problem/ProblemManager.hpp>
#include <src/problem/ProblemTypes.hpp>
#include <vector>

#include "SpecManagerCnf.hpp"

namespace d4 {
class SpecManagerCnfDyn : public SpecManagerCnf {
 private:
  std::vector<int> m_reviewWatcher;
  void initClauses(std::vector<std::vector<Lit>> &clauses);

 public:
  SpecManagerCnfDyn(ProblemManager &p);

  void preUpdate(std::vector<Lit> &lits);
  void postUpdate(std::vector<Lit> &lits);

  // we cannot use this function here
  inline void initialize(std::vector<Var> &setOfVar, std::vector<Lit> &units) {
    assert(0);
  }
};
}  // namespace d4
