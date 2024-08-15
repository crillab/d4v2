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

#include "src/configurations/ConfigurationEREMethod.hpp"
#include "src/options/branchingHeuristic/OptionBranchingHeuristic.hpp"
#include "src/options/cache/OptionCacheManager.hpp"
#include "src/options/solvers/OptionSolver.hpp"
#include "src/options/specs/OptionSpecManager.hpp"

namespace d4 {
class OptionEREMethod {
 public:
  bool greedyInitActivated;
  bool digOnAnd;
  double threshold;
  OptionSolver optionSolver;
  OptionSpecManager optionSpecManager;

  bool cutExist;
  bool phaseHeuristicBestExist;
  unsigned randomPhaseHeuristicExist;
  OptionBranchingHeuristic optionBranchingHeuristicExist;
  OptionCacheManager optionCacheManagerExist;

  bool computeComponentOnRandom;
  OptionBranchingHeuristic optionBranchingHeuristicRandom;
  OptionCacheManager optionCacheManagerRandom;

  /**
   * @brief Construct a new object with the default parameters.
   */
  OptionEREMethod() : OptionEREMethod(ConfigurationEREMethod()) {}

  /**
   * @brief Construct an OptionEREMethd from a configuration.
   *
   * @param config gives the method configuration.
   */
  OptionEREMethod(const ConfigurationEREMethod& config);

  friend std::ostream& operator<<(std::ostream& out,
                                  const OptionEREMethod& dt) {
    out << " Option ERE Method:";
    return out;
  }  // <<
};
}  // namespace d4