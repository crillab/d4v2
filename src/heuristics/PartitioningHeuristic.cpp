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

   @param[in] vm, the list of options.
   @param[in] s, a view on the problem's structure.

   \return a partioner if the options are ocrrect, NULL otherwise.
 */
PartitioningHeuristic *PartitioningHeuristic::makePartitioningHeuristic(
    po::variables_map &vm, SpecManager &s, WrapperSolver &ws,
    std::ostream &out) {
  std::string meth = vm["partitioning-heuristic"].as<std::string>();
  std::string inType = vm["input-type"].as<std::string>();

  if (meth == "none") return makePartitioningHeuristicNone(out);

  bool reduceFormula =
      vm["partitioning-heuristic-simplification-hyperedge"].as<bool>();
  bool equivSimp =
      vm["partitioning-heuristic-simplification-equivalence"].as<bool>();
  int staticPhase =
      vm["partitioning-heuristic-bipartite-phase-static"].as<int>();
  double dynamicPhase =
      vm["partitioning-heuristic-bipartite-phase-dynamic"].as<double>();
  std::string phase =
      vm["partitioning-heuristic-bipartite-phase"].as<std::string>();

  out << "c [CONSTRUCTOR] Partitioner manager: " << meth << " " << inType << " "
      << "reduceFormula(" << reduceFormula << ") "
      << "equivSimp(" << equivSimp << ") "
      << "phase(" << phase << ") "
      << "dynamicPhase(" << dynamicPhase << ") "
      << "staticPhase(" << staticPhase << ")\n";

  if (inType == "cnf" || inType == "dimacs") {
    if (meth == "bipartition-primal")
      return new PartitioningHeuristicBipartitePrimal(vm, ws, s, out);
    if (meth == "bipartition-dual")
      return new PartitioningHeuristicBipartiteDual(vm, ws, s, out);
    if (meth == "decomposition-static-dual") {
      PartitioningHeuristicStaticSingleDual *ret =
          new PartitioningHeuristicStaticSingleDual(vm, ws, s, out);
      ret->init(out);
      return ret;
    }
    if (meth == "decomposition-static-primal") {
      PartitioningHeuristicStaticSinglePrimal *ret =
          new PartitioningHeuristicStaticSinglePrimal(vm, ws, s, out);
      ret->init(out);
      return ret;
    }
    if (meth == "decomposition-static-multi") {
      PartitioningHeuristicStaticMulti *ret =
          new PartitioningHeuristicStaticMulti(vm, ws, s, out);
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
