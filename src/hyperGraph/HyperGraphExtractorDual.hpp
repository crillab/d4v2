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

#include <iostream>
#include <vector>

#include "HyperGraph.hpp"
#include "HyperGraphExtractor.hpp"
#include "src/problem/ProblemTypes.hpp"
#include "src/specs/cnf/SpecManagerCnf.hpp"

namespace d4 {
class HyperGraphExtractorDual : public HyperGraphExtractor {
 private:
  std::vector<bool> m_markedVar;
  std::vector<int> m_unmarkSet;

  std::vector<bool> m_markedClauses;
  std::vector<bool> m_keepClause;
  std::vector<unsigned> m_idxClauses;
  std::vector<unsigned> m_sizeClause;
  std::vector<unsigned> m_countClause;

  std::vector<unsigned *> m_mapVarEdge;

  void reduceHyperGraph(SpecManagerCnf &om, HyperGraph &hypergraph,
                        std::vector<Var> &considered,
                        std::vector<unsigned> &idxClauses,
                        std::vector<Var> &equivClass);

  void clashHyperEdgeIndex(HyperGraph &hypergraph, std::vector<int> &partition,
                           std::vector<unsigned> &indices);

 public:
  HyperGraphExtractorDual(unsigned nbVar, unsigned nbClause);

  void constructHyperGraph(SpecManagerCnf &om, std::vector<Var> &component,
                           std::vector<Var> &equivClass,
                           std::vector<std::vector<Var> > &equivVar,
                           bool reduceFormula, std::vector<Var> &considered,
                           HyperGraph &hypergraph);

  void extractCutFromHyperGraph(HyperGraph &hypergraph,
                                std::vector<Var> &considered,
                                std::vector<int> &partition,
                                std::vector<int> &cutSet);
};
}  // namespace d4
