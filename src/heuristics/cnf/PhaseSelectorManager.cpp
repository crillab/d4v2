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
    po::variables_map &vm, PartitioningHeuristicStaticSingle *staticPartitioner,
    std::ostream &out) {
  int limitPhase =
      vm["partitioning-heuristic-bipartite-phase-static"].as<int>();
  double dynamicPhase =
      vm["partitioning-heuristic-bipartite-phase-dynamic"].as<double>();
  std::string phase =
      vm["partitioning-heuristic-bipartite-phase"].as<std::string>();

  if (phase == "none" || (limitPhase <= 0 && !dynamicPhase))
    return new PhaseSelectorNone(staticPartitioner, out);

  if (!dynamicPhase)
    return new PhaseSelectorStatic(staticPartitioner, limitPhase, out);

  return new PhaseSelectorDynamic(staticPartitioner, dynamicPhase, out);
}  // makePhaseSelectorManager

}  // namespace d4
