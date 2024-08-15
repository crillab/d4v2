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

#include "ConfigurationPreproc.hpp"
#include "src/methods/MethodManager.hpp"

namespace d4 {

class Configuration {
 public:
  MethodName methodName;
  int precision;
  bool isFloat;
  std::string inputName;
  ProblemInputType problemInputType;

  ConfigurationPeproc configurationPreproc;

  friend std::ostream& operator<<(std::ostream& out, const Configuration& dt) {
    out << "c Configuration:\n"
        << "c Method used:" << MethodNameManager::getMethodName(dt.methodName)
        << "\n";

    return out;
  }  // <<
};
}  // namespace d4