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
#include "PhaseSelectorManager.hpp"

namespace d4 {
class PhaseSelectorManager;

struct DistribSize {
  unsigned cutSize;
  unsigned leftTreeSize;
  unsigned rightTreeSize;
  unsigned level;

  void display() {
    std::cout << "cut size: " << cutSize << " "
              << "left size: " << leftTreeSize << " "
              << "right size: " << rightTreeSize << " "
              << "level: " << level << "\n";
  }

  double getRatio() {
    if (!leftTreeSize || !rightTreeSize) return 0;

    if (leftTreeSize > rightTreeSize)
      return (double)rightTreeSize / (double)leftTreeSize;
    return (double)leftTreeSize / (double)rightTreeSize;
  }
};

class PartitioningHeuristicStaticSingle : public PartitioningHeuristicStatic {
 protected:
  struct Strata {
    unsigned fatherId;
    std::vector<unsigned> part;
  };

  struct LevelInfo {
    unsigned separatorLevel;
    unsigned cutSize;
  };

  const unsigned LIMIT = 10;

  // to store the hypergraph, and then avoid reallocated memory.
  HyperGraph m_hypergraph;
  HyperGraphExtractor *m_hypergraphExtractor;
  PhaseSelectorManager *m_phaseSelector;

  std::vector<unsigned> m_bucketNumber;
  std::vector<bool> m_markedVar;
  std::vector<LevelInfo> m_levelInfo;
  std::vector<Var> m_equivClass;
  std::vector<unsigned> m_levelDistribution;

  void distributePartition(std::vector<std::vector<unsigned>> &hypergraph,
                           std::vector<int> &partition,
                           std::vector<unsigned> &mappingEdge,
                           std::vector<Var> &mappingVar,
                           std::vector<Strata> &stack, unsigned &level);

  void splitWrtPartition(HyperGraph &hypergraph, std::vector<int> &partition,
                         std::vector<unsigned> &mappingEdge,
                         std::vector<unsigned> &cutSet,
                         std::vector<unsigned> &indicesFirst,
                         std::vector<unsigned> &indicesSecond);

  void assignLevel(std::vector<std::vector<unsigned>> &hypergraph,
                   unsigned idFather, std::vector<unsigned> &indices,
                   std::vector<Var> &mappingVar, unsigned &level);

  void saveHyperGraph(std::vector<std::vector<unsigned>> &savedHyperGraph);

  void setHyperGraph(std::vector<std::vector<unsigned>> &savedHyperGraph,
                     std::vector<unsigned> &indices, HyperGraph &hypergraph);

  virtual void setBucketLevelFromEdges(
      std::vector<std::vector<unsigned>> &hypergraph,
      std::vector<unsigned> &indices, std::vector<int> &mapping,
      unsigned level) {}

  virtual void setCutSetBucketLevelFromEdges(
      std::vector<std::vector<unsigned>> &hypergraph,
      std::vector<int> &partition, std::vector<unsigned> &indices,
      std::vector<int> &mapping, unsigned level) {
    setBucketLevelFromEdges(hypergraph, indices, mapping, level);
  }

 public:
  PartitioningHeuristicStaticSingle(po::variables_map &vm, WrapperSolver &s,
                                    SpecManager &om, std::ostream &out);

  PartitioningHeuristicStaticSingle(po::variables_map &vm, WrapperSolver &s,
                                    SpecManager &om, int nbClause, int nbVar,
                                    int sumSize, std::ostream &out);

  virtual ~PartitioningHeuristicStaticSingle();

  void computeDecomposition(std::vector<Var> &component,
                            std::vector<Var> &equivClass,
                            std::vector<std::vector<Var>> &equivVar,
                            std::vector<unsigned> &bucketNumber);

  void computeCutSet(std::vector<Var> &component, std::vector<Var> &cutSet);

  bool isStillOk(std::vector<Var> &component);
  void init(std::ostream &out);
  DistribSize computeDistribSize(std::vector<Var> &component);

  inline void setIsInitialized(bool b) { m_isInitialized = b; }
  inline bool getIsInitialized() { return m_isInitialized; }
  inline std::vector<unsigned> &getBucketNumber() { return m_bucketNumber; }

  inline unsigned &getLimitSeparatorLevel(unsigned i) {
    return m_levelInfo[i].separatorLevel;
  }

  inline unsigned &getLimitCutSizeLevel(unsigned i) {
    return m_levelInfo[i].cutSize;
  }
};
}  // namespace d4
