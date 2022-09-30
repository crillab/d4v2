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
#include <ostream>

#include "PartitioningHeuristicStatic.hpp"
#include "PartitioningHeuristicStaticSingleDual.hpp"
#include "PartitioningHeuristicStaticSinglePrimal.hpp"
#include "PhaseSelectorManager.hpp"

namespace d4 {
class PhaseSelectorManager;

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
  PartitioningHeuristicStaticMulti(po::variables_map &vm, WrapperSolver &s,
                                   SpecManager &om, std::ostream &out);

  PartitioningHeuristicStaticMulti(po::variables_map &vm, WrapperSolver &s,
                                   SpecManager &om, int nbClause, int nbVar,
                                   int sumSize, std::ostream &out);

  virtual ~PartitioningHeuristicStaticMulti();

  void computeCutSet(std::vector<Var> &component, std::vector<Var> &cutSet);

  bool isStillOk(std::vector<Var> &component);

  void init(std::ostream &out);
};
}  // namespace d4
