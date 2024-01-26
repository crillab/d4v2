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
#include "PartitioningHeuristic.hpp"

#include <bitset>

#include "PartitioningHeuristicNone.hpp"
#include "cnf/PartitioningHeuristicBipartiteDual.hpp"
#include "cnf/PartitioningHeuristicBipartitePrimal.hpp"
#include "cnf/PartitioningHeuristicStaticMulti.hpp"
#include "cnf/PartitioningHeuristicStaticSingleDual.hpp"
#include "cnf/PartitioningHeuristicStaticSinglePrimal.hpp"
#include "src/exceptions/FactoryException.hpp"
#include "src/utils/AtMost1Extractor.hpp"

namespace d4 {

/**
   Simply returns a partitioner that does nothing.

   @param[in] out, the stream where is print out the log.
 */
PartitioningHeuristic *PartitioningHeuristic::makePartitioningHeuristicNone(
    std::ostream &out) {
  out << "c [CONSTRUCTOR] Partitioner manager: none\n";
  return new PartitioningHeuristicNone();
}  // makePartitioningHeuristicNone

/**
   Create a partitioner.

   @param[in] config, the configuration.
   @param[in] s, a view on the problem's structure.

   \return a partioner if the options are ocrrect, NULL otherwise.
 */
PartitioningHeuristic *PartitioningHeuristic::makePartitioningHeuristic(
    Config &config, SpecManager &s, WrapperSolver &ws,
    std::ostream &out) {
  if (config.partitioning_heuristic == "none") return makePartitioningHeuristicNone(out);

  out << "c [CONSTRUCTOR] Partitioner manager: " << config.partitioning_heuristic << " " << config.input_type << " "
      << "reduceFormula(" << config.partitioning_heuristic_simplification_hyperedge << ") "
      << "equivSimp(" << config.partitioning_heuristic_simplification_equivalence << ") "
      << "phase(" << config.partitioning_heuristic_bipartite_phase << ") "
      << "dynamicPhase(" << config.partitioning_heuristic_bipartite_phase_dynamic << ") "
      << "staticPhase(" << config.partitioning_heuristic_bipartite_phase_static << ")\n";

  if (config.input_type == "cnf" || config.input_type == "dimacs") {
    if (config.partitioning_heuristic == "bipartition-primal")
      return new PartitioningHeuristicBipartitePrimal(config, ws, s, out);
    if (config.partitioning_heuristic == "bipartition-dual")
      return new PartitioningHeuristicBipartiteDual(config, ws, s, out);
    if (config.partitioning_heuristic == "decomposition-static-dual") {
      PartitioningHeuristicStaticSingleDual *ret =
          new PartitioningHeuristicStaticSingleDual(config, ws, s, out);
      ret->init(out);
      return ret;
    }
    if (config.partitioning_heuristic == "decomposition-static-primal") {
      PartitioningHeuristicStaticSinglePrimal *ret =
          new PartitioningHeuristicStaticSinglePrimal(config, ws, s, out);
      ret->init(out);
      return ret;
    }
    if (config.partitioning_heuristic == "decomposition-static-multi") {
      PartitioningHeuristicStaticMulti *ret =
          new PartitioningHeuristicStaticMulti(config, ws, s, out);
      ret->init(out);
      return ret;
    }
  }

  throw(FactoryException("Cannot create a PartitioningHeuristic", __FILE__,
                         __LINE__));
}  // makePartitioningHeuristic

/**
   Associate for each variable in the component an equivalence class.

   @pararm[in] eqManager, the equivalence manager.
   @param[in] solver, the SAT solver used in the equivalence manager.
   @param[in] component, the set of variables of the component we want to cut.
   @param[out] unitEquiv, the set of unit literals we find out.
   @param[out] equiClass, the equivalence class we computed (we suppose that the
   verctor is large enough and then we do not allocate).
*/
void PartitioningHeuristic::computeEquivClass(
    EquivExtractor &eqManager, WrapperSolver &solver,
    std::vector<Var> &component, std::vector<Lit> &unitEquiv,
    std::vector<Var> &equivClass, std::vector<std::vector<Var>> &equivVar) {
  for (auto &v : component) {
    assert(equivClass.size() >= (unsigned)v);
    equivClass[v] = v;
  }

  eqManager.searchEquiv(solver, component, equivVar);
  solver.whichAreUnits(component, unitEquiv);

  for (auto &c : equivVar) {
    Var vi = c.back();
    for (auto &v : c) equivClass[v] = vi;
  }
}  // computeEquivclass

}  // namespace d4
