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
#pragma once

#include "PartitioningHeuristicStatic.hpp"

namespace d4 {
class PartitioningHeuristicStaticNone : public PartitioningHeuristicStatic {
 protected:
  void computeDecomposition(std::vector<Var> &component,
                            std::vector<Var> &equivClass,
                            std::vector<std::vector<Var>> &equivVar,
                            std::vector<unsigned> &bucketNumber);

 public:
  PartitioningHeuristicStaticNone(po::variables_map &vm, WrapperSolver &s,
                                  SpecManager &om, std::ostream &out);

  PartitioningHeuristicStaticNone(po::variables_map &vm, WrapperSolver &s,
                                  SpecManager &om, int nbClause, int nbVar,
                                  int sumSize, std::ostream &out);

  ~PartitioningHeuristicStaticNone();

  inline bool isInitialized() { return false; }
  inline bool isStillOk(std::vector<Var> &component) { return false; }
  inline void init(std::ostream &out) { m_isInitialized = true; }

  void computeCutSet(std::vector<Var> &component, std::vector<Var> &cutSet);
};

}  // namespace d4
