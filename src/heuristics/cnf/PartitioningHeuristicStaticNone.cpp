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
#include "PartitioningHeuristicStaticNone.hpp"

namespace d4 {

/**
   Constructor.

   @param[in] vm, the option list.
   @param[in] s, a wrapper on a solver.
   @param[in] om, a structure manager.
 */
PartitioningHeuristicStaticNone::PartitioningHeuristicStaticNone(
    po::variables_map &vm, WrapperSolver &s, SpecManager &om, std::ostream &out)
    : PartitioningHeuristicStaticNone(
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
PartitioningHeuristicStaticNone::PartitioningHeuristicStaticNone(
    po::variables_map &vm, WrapperSolver &s, SpecManager &om, int nbClause,
    int nbVar, int sumSize, std::ostream &out)
    : PartitioningHeuristicStatic(vm, s, om, nbClause, nbVar, sumSize, out) {
  out << "c [CONSTRUCTOR] Static partitioner: none\n";

  m_isInitialized = true;
}  // constructor

/**
   Destructor.
 */
PartitioningHeuristicStaticNone::~PartitioningHeuristicStaticNone() {

}  // destructor

/**
   In the case where we do not considere tree decomposition, then we return the
   all set.

   @param[in] component, the set of variables.
   @param[out] cutSet, the cut set we compute.
 */
void PartitioningHeuristicStaticNone::computeCutSet(std::vector<Var> &component,
                                                    std::vector<Var> &cutSet) {
  cutSet = component;
}  // computeCutSet

/**
   Put all the variables in the same bucket 1.

   @param[in] component, the set of varaibles the problem is constructed on.
   @param[in] equivClass, the equivalence class for each variable.
   @param[in] equivVar, the list of equivalences.
   @param[out] bucketNumber, the decomposition tree in term of index.
 */
void PartitioningHeuristicStaticNone::computeDecomposition(
    std::vector<Var> &component, std::vector<Var> &equivClass,
    std::vector<std::vector<Var>> &equivVar,
    std::vector<unsigned> &bucketNumber) {
  for (auto &v : component) bucketNumber[v] = 1;
}  // computeDecomposition

}  // namespace d4
