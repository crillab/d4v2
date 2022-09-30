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

#include "HyperGraph.hpp"

namespace d4 {
/**
   Initialize the hyper graph to NULL.
*/
HyperGraph::HyperGraph()
    : m_hypergraph(NULL), m_hypergraphCapacity(0), m_hypergraphSize(0) {}

/**
   Initialize the hyper graph structure with the maximum capacity.

   @param[in] capacity, the max capacity.
*/
HyperGraph::HyperGraph(unsigned capacity) : m_hypergraphCapacity(capacity) {
  m_hypergraph = new unsigned[capacity];
  m_hypergraphSize = 0;
}  // constructor

/**
   Free the allocated memory.
 */
HyperGraph::~HyperGraph() {
  if (m_hypergraph) delete[] m_hypergraph;
}  // destructor

/**
   Print out the hyper graph.
*/
void HyperGraph::display() {
  for (auto it : *this) {
    for (auto e : it) std::cout << e << " ";
    std::cout << "\n";
  }
}  // displayHyperGraph

/**
   Initialize the data structure.

   @param[in] capacity, the max capacity.
 */
void HyperGraph::init(unsigned capacity) {
  if (m_hypergraph) delete[] m_hypergraph;
  m_hypergraphCapacity = capacity;
  m_hypergraph = new unsigned[capacity];
  m_hypergraphSize = 0;
}  // init

}  // namespace d4
