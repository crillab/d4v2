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

#include <iostream>
#include <string>

#include "src/configurations/ConfigurationProjMcMethod.hpp"
#include "src/options/cache/OptionCacheManager.hpp"
#include "src/options/methods/OptionDpllStyleMethod.hpp"
#include "src/options/solvers/OptionSolver.hpp"

namespace d4 {
class OptionProjMcMethod {
 public:
  bool refinement;
  OptionCacheManager optionCache;
  OptionSolver optionSolver;
  OptionSpecManager optionSpecs;
  OptionDpllStyleMethod optionCounter;

  /**
   * @brief Construct a new Option Proj Mc Method object regarding a given
   * configuration.
   *
   * @param config is the configuration we use for setting the option.
   */
  OptionProjMcMethod(const ConfigurationProjMcMethod& config);

  friend std::ostream& operator<<(std::ostream& out,
                                  const OptionProjMcMethod& dt) {
    out << " Option ProjMC Method:"
        << " refinement(" << dt.refinement << ")"
        << " cache(" << dt.optionCache << ")"
        << " solver(" << dt.optionSolver << ")"
        << " spec(" << dt.optionSpecs << ")"
        << " counter(" << dt.optionCounter << ")";
    return out;
  }  // <<
};
}  // namespace d4