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
#include "PartitioningHeuristicStatic.hpp"

#include "PartitioningHeuristicStaticMulti.hpp"
#include "PartitioningHeuristicStaticNone.hpp"
#include "PartitioningHeuristicStaticSingleDual.hpp"
#include "PartitioningHeuristicStaticSinglePrimal.hpp"
#include "src/exceptions/FactoryException.hpp"

namespace d4 {
/**
   Constructor.

   @param[in] vm, the option list.
   @param[in] s, a wrapper on a solver.
   @param[in] om, a structure manager.
*/
PartitioningHeuristicStatic::PartitioningHeuristicStatic(po::variables_map &vm,
                                                         WrapperSolver &s,
                                                         SpecManager &om,
                                                         std::ostream &out)
    : PartitioningHeuristicStatic(
          vm, s, om, dynamic_cast<SpecManagerCnf &>(om).getNbClause(),
          dynamic_cast<SpecManagerCnf &>(om).getNbVariable(),
          dynamic_cast<SpecManagerCnf &>(om).getSumSizeClauses(), out) {
}  // constructor

/**
   Constructor.

   @param[in] vm, the option list.
   @param[in] s, a wrapper on a solver.
   @param[in] om, a structure manager.
   @param[in] nbClause, the number of clauses.
   @param[in] nbVar, the number of variables.
   @param[in] sumSize, which give the number of literals.
*/
PartitioningHeuristicStatic::PartitioningHeuristicStatic(
    po::variables_map &vm, WrapperSolver &s, SpecManager &om, int nbClause,
    int nbVar, int sumSize, std::ostream &out)
    : m_s(s), m_om(dynamic_cast<SpecManagerCnf &>(om)) {
  m_nbVar = nbVar;
  m_nbClause = nbClause;

  m_em.initEquivExtractor(m_nbVar + 1);

  // get the options.
  m_reduceFormula =
      vm["partitioning-heuristic-simplification-hyperedge"].as<bool>();
  m_equivSimp =
      vm["partitioning-heuristic-simplification-equivalence"].as<bool>();

  m_isInitialized = false;
  m_pm = NULL;
}  // constructor

/**
   Destructor.
*/
PartitioningHeuristicStatic::~PartitioningHeuristicStatic() {
  if (m_pm) delete m_pm;
}  // destructor

/**
   Generate a static partitioner regarding the given option list.

   @param[in] vm, the option list.
   @param[in] s, a wrapper to a solver.
   @param[in] om, a formula manager.
   @param[in] nbClause, the number of clauses of the formula.
   @param[in] nbVar, the number of variables of the formula.
   @param[in] sumSize, the number of literals of the formula.
   @param[in] type, a string that gives the partioner type.

   \return a static partioner.
 */
PartitioningHeuristicStatic *
PartitioningHeuristicStatic::makePartitioningHeuristicStatic(
    po::variables_map &vm, WrapperSolver &s, SpecManager &om, int nbClause,
    int nbVar, int sumSize, const std::string &type, std::ostream &out) {
  std::string opt =
      vm["partitioning-heuristic-bipartite-phase"].as<std::string>();

  PartitioningHeuristicStatic *ret = NULL;

  if (opt == "none")
    ret = new PartitioningHeuristicStaticNone(vm, s, om, nbClause, nbVar,
                                              sumSize, out);
  else if (opt == "multi")
    ret = new PartitioningHeuristicStaticMulti(vm, s, om, nbClause, nbVar,
                                               sumSize, out);
  else if (opt == "dual" || (opt == "natural" && type == "dual"))
    ret = new PartitioningHeuristicStaticSingleDual(vm, s, om, nbClause, nbVar,
                                                    sumSize, out);
  else if (opt == "primal" || (opt == "natural" && type == "primal"))
    ret = new PartitioningHeuristicStaticSinglePrimal(vm, s, om, nbClause,
                                                      nbVar, sumSize, out);
  else
    throw(FactoryException("Cannot create a PartitioningHeuristic", __FILE__,
                           __LINE__));
  ret->init(out);
  return ret;
}  // makePartitioningHeuristicStatic

}  // namespace d4
