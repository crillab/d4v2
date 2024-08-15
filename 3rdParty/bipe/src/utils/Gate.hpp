/**
 * bipe
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

#include <vector>

#include "src/utils/Problem.hpp"

namespace bipe {

enum TypeGate { UNIT, EQUIV, OR, AND, XOR, RM };

struct Gate {
  TypeGate type;
  Lit output;
  std::vector<Lit> input;

  inline void display() {
    std::cout << output << " <-> ";

    switch (type) {
      case UNIT:
        std::cout << "(T";
        break;
      case EQUIV:
        std::cout << "(";
        break;
      case OR:
        std::cout << "OR(";
        break;
      case AND:
        std::cout << "AND(";
        break;
      case XOR:
        std::cout << "(XOR";
        break;
      default:
        break;
    }

    for (auto l : input) std::cout << l << " ";
    std::cout << ")\n";
  }
};
}  // namespace bipe