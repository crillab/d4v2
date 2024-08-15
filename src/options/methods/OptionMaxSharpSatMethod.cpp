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

#include "OptionMaxSharpSatMethod.hpp"

namespace d4 {

/**
 * @brief OptionMaxSharpSat::OptionMaxSharpSat implementation.
 */
OptionMaxSharpSatMethod::OptionMaxSharpSatMethod(
    const ConfigurationMaxSharpSatMathod& config) {
  // general options.
  greedyInitActivated = config.greedyInitActivated;
  digOnAnd = config.digOnAnd;
  threshold = config.threshold;
  optionSolver = {config.solver.solverName};
  optionSpecManager = {config.specManager.specUpdateType};

  // options about the max level.
  randomPhaseHeuristicMax = config.randomPhaseHeuristicMax;
  phaseHeuristicMax = config.phaseHeuristicMax;
  optionBranchingHeuristicMax =
      OptionBranchingHeuristic(config.branchingHeuristicMax);
  optionCacheManagerMax = OptionCacheManager(config.cacheManagerMax);

  // options about the ind level.
  optionBranchingHeuristicInd =
      OptionBranchingHeuristic(config.branchingHeuristicInd);
  optionCacheManagerInd = OptionCacheManager(config.cacheManagerInd);
}

}  // namespace d4