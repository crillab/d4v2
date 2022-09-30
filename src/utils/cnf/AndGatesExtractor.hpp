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
#include "src/specs/cnf/SpecManagerCnf.hpp"

namespace d4 {
struct AndGate {
  Lit output;
  std::vector<Lit> input;

  void display() {
    std::cout << output << " <-> ";
    for (auto &l : input) std::cout << l << " ";
    std::cout << "\n";
  }
};

class AndGatesExtractor {
 private:
  std::vector<bool> m_markedVar;
  std::vector<u_int8_t> m_flagVar;

 public:
  AndGatesExtractor() { ; }  // empty constructor
  AndGatesExtractor(int nbVar);
  void init(int nbVar);

  void searchAndGates(SpecManagerCnf *om, std::vector<Var> &v,
                      std::vector<AndGate> &gates);
};
}  // namespace d4
