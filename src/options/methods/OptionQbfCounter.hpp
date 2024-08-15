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

#include "src/configurations/ConfigurationQbfCounter.hpp"
#include "src/exceptions/FactoryException.hpp"
#include "src/options/branchingHeuristic/OptionBranchingHeuristic.hpp"
#include "src/options/branchingHeuristic/OptionPartitioningHeuristic.hpp"
#include "src/options/cache/OptionCacheManager.hpp"

namespace d4 {
class OptionQbfCounter {
 public:
  OptionCacheManager optionCacheManager;
  OptionSolver optionSolver;
  OptionSpecManager optionSpecManager;
  OptionBranchingHeuristic optionBranchingHeuristic;
  OptionPartitioningHeuristic optionPartitioningHeuristic;

  /**
   * @brief Construct a new object with the default parameter.
   */
  OptionQbfCounter() : OptionQbfCounter(ConfigurationQbfCounter()) {}

  /**
   * @brief Construct a new Option Dpll Style Method object.
   *
   * @param config gives the method configuration.
   */
  OptionQbfCounter(const ConfigurationQbfCounter& config);

  friend std::ostream& operator<<(std::ostream& out,
                                  const OptionQbfCounter& dt) {
    out << " Option QBF Counter: ";
    return out;
  }  // <<
};

}  // namespace d4