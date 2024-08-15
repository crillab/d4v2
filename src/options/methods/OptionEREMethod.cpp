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

#include "OptionEREMethod.hpp"

namespace d4 {

/**
 * @brief OptionEREMethod::OptionEREMethod implementation.
 *
 * @param config
 */
OptionEREMethod::OptionEREMethod(const ConfigurationEREMethod& config) {
  // general options.
  greedyInitActivated = config.greedyInitActivated;
  digOnAnd = config.digOnAnd;
  threshold = config.threshold;
  optionSolver = {config.solver.solverName};
  optionSpecManager = {config.specManager.specUpdateType};

  // options about the exist level.
  cutExist = config.cutExist;
  phaseHeuristicBestExist = config.phaseHeuristicBestExist;
  randomPhaseHeuristicExist = config.randomPhaseHeuristicExist;
  optionBranchingHeuristicExist =
      OptionBranchingHeuristic(config.branchingHeuristicExist);
  optionCacheManagerExist = OptionCacheManager(config.cacheManagerExist);

  // options about the random level.
  computeComponentOnRandom = config.computeComponentOnRandom;
  optionBranchingHeuristicRandom =
      OptionBranchingHeuristic(config.branchingHeuristicRandom);
  optionCacheManagerRandom = OptionCacheManager(config.cacheManagerRandom);
}  // constructor

}  // namespace d4