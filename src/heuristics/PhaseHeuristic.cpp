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

   @param[in] vm, the set of options.
 */
PhaseHeuristic *PhaseHeuristic::makePhaseHeuristic(po::variables_map &vm,
                                                   SpecManager &s,
                                                   PolarityManager &p,
                                                   std::ostream &out) {
  std::string meth = vm["phase-heuristic"].as<std::string>();
  bool rev = vm["phase-heuristic-reversed"].as<bool>();

  out << "c [CONSTRUCTOR] Phase heuristic: " << meth
      << ((rev) ? "(reversed)" : "") << "\n";

  if (meth == "false") return new PhaseHeuristicFalse(rev);
  if (meth == "true") return new PhaseHeuristicTrue(rev);
  if (meth == "polarity") return new PhaseHeuristicPolarity(p, rev);
  if (meth == "occurrence") return new PhaseHeuristicOccurrence(s, rev);

  throw(FactoryException("Cannot create a PhaseHeuristic", __FILE__, __LINE__));
}  // makePhaseHeuristic

}  // namespace d4
