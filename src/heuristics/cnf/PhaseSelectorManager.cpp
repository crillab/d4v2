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

#include "PhaseSelectorManager.hpp"

#include <ostream>

#include "PhaseSelectorDynamic.hpp"
#include "PhaseSelectorNone.hpp"
#include "PhaseSelectorStatic.hpp"

namespace d4 {

/**
   Constructor.

   @param[in] staticPartitioner, give the partitioner used.
 */
PhaseSelectorManager::PhaseSelectorManager(
    PartitioningHeuristicStaticSingle *staticPartitioner) {
  m_staticPartitioner = staticPartitioner;
}  // constructor

/**
   Create a selector manager regarding the given options.

   @param[in] limit, the limit number of variables before switching.
   @param[in] dynamicPhase, if we switch dynamically.
   @param[in] bucketNumber, the computed partition.

   \return a selector manager used to decide if we want to switch between the
   static decomposition to the dynamic one.
 */
PhaseSelectorManager *PhaseSelectorManager::makePhaseSelectorManager(
    Config &config, PartitioningHeuristicStaticSingle *staticPartitioner,
    std::ostream &out) {
  if (config.partitioning_heuristic_bipartite_phase == "none" || (config.partitioning_heuristic_bipartite_phase_static <= 0 && !config.partitioning_heuristic_bipartite_phase_dynamic))
    return new PhaseSelectorNone(staticPartitioner, out);

  if (!config.partitioning_heuristic_bipartite_phase_dynamic)
    return new PhaseSelectorStatic(staticPartitioner, config.partitioning_heuristic_bipartite_phase_static, out);

  return new PhaseSelectorDynamic(staticPartitioner, config.partitioning_heuristic_bipartite_phase_dynamic, out);
}  // makePhaseSelectorManager

}  // namespace d4
