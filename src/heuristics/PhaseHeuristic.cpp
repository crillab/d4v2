/*
 * d4
 * Copyright (C) 2020  Univ. Artois & CNRS
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
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

   @param[in] config, the configuration.
 */
PhaseHeuristic *PhaseHeuristic::makePhaseHeuristic(Config &config,
                                                   SpecManager &s,
                                                   PolarityManager &p,
                                                   std::ostream &out) {
  out << "c [CONSTRUCTOR] Phase heuristic: " << config.phase_heuristic
      << ((config.phase_heuristic_reversed) ? "(reversed)" : "") << "\n";

  if (config.phase_heuristic == "false") return new PhaseHeuristicFalse(config.phase_heuristic_reversed);
  if (config.phase_heuristic == "true") return new PhaseHeuristicTrue(config.phase_heuristic_reversed);
  if (config.phase_heuristic == "polarity") return new PhaseHeuristicPolarity(p, config.phase_heuristic_reversed);
  if (config.phase_heuristic == "occurrence") return new PhaseHeuristicOccurrence(s, config.phase_heuristic_reversed);

  throw(FactoryException("Cannot create a PhaseHeuristic", __FILE__, __LINE__));
}  // makePhaseHeuristic

}  // namespace d4
