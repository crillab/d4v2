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

#include "PartitioningHeuristicStaticSingle.hpp"

namespace d4 {
class PartitioningHeuristicStaticSinglePrimal
    : public PartitioningHeuristicStaticSingle {
 private:
  std::vector<bool> m_markedVar;

 protected:
  void setBucketLevelFromEdges(std::vector<std::vector<unsigned>> &hypergraph,
                               std::vector<unsigned> &indices,
                               std::vector<int> &mapping, unsigned level);

  void setCutSetBucketLevelFromEdges(
      std::vector<std::vector<unsigned>> &hypergraph,
      std::vector<int> &partition, std::vector<unsigned> &indices,
      std::vector<int> &mapping, unsigned level);

 public:
  PartitioningHeuristicStaticSinglePrimal(
      const OptionPartitioningHeuristic &options, WrapperSolver &s,
      SpecManager &om, std::ostream &out);

  PartitioningHeuristicStaticSinglePrimal(
      const OptionPartitioningHeuristic &options, WrapperSolver &s,
      SpecManager &om, int nbClause, int nbVar, int sumSize, std::ostream &out);

  ~PartitioningHeuristicStaticSinglePrimal();
};
}  // namespace d4
