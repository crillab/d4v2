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
#pragma once
#include <ostream>

#include "PartitioningHeuristicStatic.hpp"
#include "PartitioningHeuristicStaticSingleDual.hpp"
#include "PartitioningHeuristicStaticSinglePrimal.hpp"

namespace d4 {
class PartitioningHeuristicStaticMulti : public PartitioningHeuristicStatic {
 private:
  PartitioningHeuristicStaticSingle *m_partitionStaticDual;
  PartitioningHeuristicStaticSinglePrimal *m_partitionStaticPrimal;
  PartitioningHeuristicStaticSingle *m_partitionStaticUsed;

  double m_ratio;
  std::vector<Var> m_equivClass;

 protected:
  void computeDecomposition(std::vector<Var> &component,
                            std::vector<Var> &equivClass,
                            std::vector<std::vector<Var>> &equivVar);

 public:
  PartitioningHeuristicStaticMulti(const OptionPartitioningHeuristic &options,
                                   WrapperSolver &s, SpecManager &om,
                                   std::ostream &out);

  PartitioningHeuristicStaticMulti(const OptionPartitioningHeuristic &options,
                                   WrapperSolver &s, SpecManager &om,
                                   int nbClause, int nbVar, int sumSize,
                                   std::ostream &out);

  virtual ~PartitioningHeuristicStaticMulti();

  void computeCutSet(std::vector<Var> &component, std::vector<Var> &cutSet);

  void init(std::ostream &out);
};
}  // namespace d4
