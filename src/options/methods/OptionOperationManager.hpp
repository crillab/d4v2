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
#include "src/methods/MethodManager.hpp"

namespace d4 {

enum OperationType { OP_COUNTING, OP_CIRC };

class OperationTypeManager {
 public:
  static std::string getOperatorType(const OperationType& m) {
    if (m == OP_COUNTING) return "counting";
    if (m == OP_CIRC) return "ddnnf-compiler";

    throw(FactoryException("Operator Type unknown", __FILE__, __LINE__));
  }  // getOperatorType

  static OperationType getOperatorType(const std::string& m) {
    if (m == "counting" || m == "counting-global-cache") return OP_COUNTING;
    if (m == "ddnnf-compiler") return OP_CIRC;

    throw(FactoryException("Operator Type unknown", __FILE__, __LINE__));
  }  // getOperatorType

  static OperationType getOperatorType(const MethodName& m) {
    if (m == METH_COUNTING || m == METH_COUNTING_GLOBAL_CACHE)
      return OP_COUNTING;
    if (m == METH_DDNNF) return OP_CIRC;

    throw(FactoryException("Operator Type unknown", __FILE__, __LINE__));
  }  // getOperatorType
};

class OptionOperationManager {
 public:
  OperationType operatorType;

  friend std::ostream& operator<<(std::ostream& out,
                                  const OptionOperationManager& dt) {
    out << " Operator option:"
        << " operator type("
        << OperationTypeManager::getOperatorType(dt.operatorType) << ")";
    return out;
  }  // <<
};

}  // namespace d4