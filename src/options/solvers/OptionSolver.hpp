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

#include <string>

#include "src/exceptions/FactoryException.hpp"

namespace d4 {
enum SolverName { GLUCOSE_CNF, MINISAT_CNF };

class SolverNameManager {
 public:
  static std::string getSolverName(const SolverName& m) {
    if (m == GLUCOSE_CNF) return "glucose cnf";
    if (m == MINISAT_CNF) return "minisat cnf";

    throw(FactoryException("Solver Name unknown", __FILE__, __LINE__));
  }  // getSolverName

  static SolverName getSolverName(const std::string& m) {
    if (m == "glucose") return GLUCOSE_CNF;
    if (m == "minisat") return MINISAT_CNF;

    throw(FactoryException("Solver Name unknown", __FILE__, __LINE__));
  }  // getSolverName
};

class OptionSolver {
 public:
  SolverName solverName;

  friend std::ostream& operator<<(std::ostream& out, const OptionSolver& dt) {
    out << " Option Solver:"
        << " solver name(" << SolverNameManager::getSolverName(dt.solverName)
        << ")";

    return out;
  }  // <<
};

}  // namespace d4