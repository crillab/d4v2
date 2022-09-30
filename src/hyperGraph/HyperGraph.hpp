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

#include <cassert>
#include <iostream>
#include <iterator>

#include "HyperEdge.hpp"

namespace d4 {
/**
   We use a raw representation of the hyper graph.
   -> [size1] [...elts1 ...] [size2] [... elts2...] .......
*/
class HyperGraph {
 private:
  unsigned *m_hypergraph;
  unsigned m_hypergraphCapacity;
  unsigned m_hypergraphSize;

 public:
  HyperGraph();
  HyperGraph(unsigned capacity);
  ~HyperGraph();

  inline void incSize() { m_hypergraphSize++; }
  inline void decSize() { m_hypergraphSize--; }
  inline void setSize(unsigned size) { m_hypergraphSize = size; }
  inline unsigned getSize() { return m_hypergraphSize; }
  inline unsigned *getEdges() { return m_hypergraph; }
  inline unsigned getCapicity() { return m_hypergraphCapacity; }

  inline unsigned operator[](unsigned i) const {
    assert(i < m_hypergraphCapacity);
    return m_hypergraph[i];
  }

  inline unsigned &operator[](unsigned i) {
    assert(i < m_hypergraphCapacity);
    return m_hypergraph[i];
  }

  class Iterator {
   private:
    HyperEdge m_hyperEdge;

   public:
    Iterator(unsigned *ptr, unsigned pos) : m_hyperEdge(pos, ptr) {}

    inline HyperEdge &operator*() { return m_hyperEdge; }
    inline HyperEdge operator->() { return m_hyperEdge; }

    Iterator &operator++() {
      m_hyperEdge.next();
      return *this;
    }
    Iterator operator++(int) {
      Iterator tmp = *this;
      ++(*this);
      return tmp;
    }

    friend bool operator==(const Iterator &a, const Iterator &b) {
      return a.m_hyperEdge.getId() == b.m_hyperEdge.getId();
    };
    friend bool operator!=(const Iterator &a, const Iterator &b) {
      return a.m_hyperEdge.getId() != b.m_hyperEdge.getId();
    };
  };

  Iterator begin() { return Iterator(m_hypergraph, 0); }
  Iterator end() { return Iterator(m_hypergraph, m_hypergraphSize); }

  void display();
  void init(unsigned capacity);
};
}  // namespace d4
