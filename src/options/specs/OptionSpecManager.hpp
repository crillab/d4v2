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
enum SpecUpdateType {
  SPEC_DYNAMIC,
  SPEC_DYNAMIC_BLOCKED_SIMP,
  SPEC_DYNAMIC_PURE_SIMP
};

class SpecUpdateManager {
 public:
  static std::string getSpecUpdate(const SpecUpdateType& m) {
    if (m == SPEC_DYNAMIC) return "dynamic";
    if (m == SPEC_DYNAMIC_BLOCKED_SIMP) return "dynamicBlockedSimp";

    throw(FactoryException("Spec Update unknown", __FILE__, __LINE__));
  }  // getOperatorType

  static SpecUpdateType getSpecUpdate(const std::string& m) {
    if (m == "dynamic") return SPEC_DYNAMIC;
    if (m == "dynamicBlockedSimp") return SPEC_DYNAMIC_BLOCKED_SIMP;

    throw(FactoryException("Operator Type unknown", __FILE__, __LINE__));
  }  // getSpectUpdate
};

class OptionSpecManager {
 public:
  SpecUpdateType specUpdateType;

  friend std::ostream& operator<<(std::ostream& out,
                                  const OptionSpecManager& dt) {
    out << " Option Spec Manager:"
        << " update mode("
        << SpecUpdateManager::getSpecUpdate(dt.specUpdateType) << ")";
    return out;
  }  // <<
};
}  // namespace d4