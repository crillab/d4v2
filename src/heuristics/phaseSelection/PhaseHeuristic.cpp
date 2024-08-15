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
#include "PhaseHeuristic.hpp"

#include "PhaseHeuristicFalse.hpp"
#include "PhaseHeuristicOccurrence.hpp"
#include "PhaseHeuristicPolarity.hpp"
#include "PhaseHeuristicTrue.hpp"
#include "src/exceptions/FactoryException.hpp"

namespace d4 {

/**
   Create a phase heuristic object able to select the value of a given variable.

   @param[in] vm, the set of options.
 */
PhaseHeuristic *PhaseHeuristic::makePhaseHeuristic(
    const OptionBranchingHeuristic &options, SpecManager &s, PolarityManager &p,
    std::ostream &out) {
  if (options.phaseHeuristicType == PHASE_FALSE)
    return new PhaseHeuristicFalse(options.reversePhase);
  if (options.phaseHeuristicType == PHASE_TRUE)
    return new PhaseHeuristicTrue(options.reversePhase);
  if (options.phaseHeuristicType == PHASE_POLARITY)
    return new PhaseHeuristicPolarity(p, options.reversePhase);
  if (options.phaseHeuristicType == PHASE_OCCURRENCE)
    return new PhaseHeuristicOccurrence(s, options.reversePhase);

  throw(FactoryException("Cannot create a PhaseHeuristic", __FILE__, __LINE__));
}  // makePhaseHeuristic

}  // namespace d4
