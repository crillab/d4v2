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

#include "src/configurations/ConfigurationBranchingHeuristic.hpp"

namespace d4 {

/**
 * @brief Construct a new Option Branching Heuristic object with the default
 * configuration.
 *
 */
OptionBranchingHeuristic::OptionBranchingHeuristic()
    : OptionBranchingHeuristic(ConfigurationBranchingHeuristic()) {
}  // constructor

/**
 * @brief Construct a new Option Branching Heuristic object with a given
 * configuration.
 *
 * @param config is the configuration used to fill the option structure.
 */
OptionBranchingHeuristic::OptionBranchingHeuristic(
    const ConfigurationBranchingHeuristic& config) {
  scoringMethodType = config.scoringMethodType;
  phaseHeuristicType = config.phaseHeuristicType;
  branchingHeuristicType = config.branchingHeuristicType;
  reversePhase = config.reversePhase;
  freqDecay = config.freqDecay;
  limitSizeClause = config.limitSizeClause;
}  // constructor
}  // namespace d4