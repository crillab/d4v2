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

#include "Branch.hpp"
#include "Node.hpp"
#include "src/problem/ProblemManager.hpp"

namespace d4 {
template <class T, typename U>
class DecomposableAndNode : public Node<T> {
 public:
  using Node<T>::header;

  DecomposableAndNode() = delete;

  T nbModels;
  U size;
  Node<T> *sons[0];

  /**
     Init the two branches using the data coming from the solver.

     @param[in] left, the left branch.
     @param[in] right, the right branch.
   */
  DecomposableAndNode(unsigned _size, Node<T> **_sons) {
    nbModels = T(0);
    header.stamp = 0;
    header.typeNode = TypeNode::TypeDecAndNode;
    size = _size;
    for (unsigned i = 0; i < size; i++) sons[i] = _sons[i];
  }  // constructor

  /**
     Deallocate the memory.

     @param[in] node, is equivalent to this.
     @param[in] func, give for the type of node the deallocate function.
     @param[in] globalstamp, get the stamp number.
  */
  static void deallocate(Node<T> *node, void (**func)(), unsigned globalStamp) {
    if (node->header.stamp == globalStamp) return;
    node->header.stamp = globalStamp;
    reinterpret_cast<DecomposableAndNode *>(node)->nbModels.~T();

    for (unsigned i = 0;
         i < reinterpret_cast<DecomposableAndNode *>(node)->size; i++)
      reinterpret_cast<void (**)(Node<T> *, void (**func)(), unsigned)>(
          func)[(reinterpret_cast<DecomposableAndNode *>(node)->sons[i])
                    ->header.typeNode](
          reinterpret_cast<DecomposableAndNode *>(node)->sons[i], func,
          globalStamp);
  }  // destructor

  ~DecomposableAndNode() { nbModels.~T(); }  // destructor

  /**
     Ask for the number of models of the formula under an interpretation.

     @param[in] node, is equivalent to this.
     @param[in] func, give for the type of node the deallocate function.
     @param[in] fixedValue, the assigment we consider
     @param[in] problem, the problem we are solving (use to get information
     about weight).
     @param[in] globalStamp, use to stamp if we visit a not or not.

     \return the number of models.
   */
  static T computeNbModels(Node<T> *node, T (**func)(),
                           std::vector<ValueVar> &fixedValue,
                           ProblemManager &problem, unsigned globalStamp) {
    auto *p = reinterpret_cast<DecomposableAndNode *>(node);
    if (node->header.stamp == globalStamp) return p->nbModels;

    p->nbModels = T(1);
    for (unsigned i = 0; i < p->size; i++) {
      p->nbModels *= reinterpret_cast<T (**)(
          Node<T> *, T(**func)(), std::vector<ValueVar> &, ProblemManager &,
          unsigned)>(func)[p->sons[i]->header.typeNode](
          p->sons[i], func, fixedValue, problem, globalStamp);
      if (p->nbModels == 0) break;
    }

    node->header.stamp = globalStamp;
    return p->nbModels;
  }  // computeNbModels

  /**
     Ask if the formula is satisfiable under an interpretation.

     @param[in] node, is equivalent to this.
     @param[in] func, give for the type of node the deallocate function.
     @param[in] fixedValue, the assigment we consider
     @param[in] globalStamp, use to stamp if we visit a not or not.

     \return true if the problem is satisfiable, false otherwise.
   */
  static bool isSAT(Node<T> *node, bool (**func)(),
                    std::vector<ValueVar> &fixedValue, unsigned globalStamp) {
    auto *p = reinterpret_cast<DecomposableAndNode *>(node);
    if (node->header.stamp == globalStamp) return p->nbModels == 1;
    node->header.stamp = globalStamp;

    for (unsigned i = 0; i < p->size; i++) {
      p->nbModels = reinterpret_cast<bool (**)(
          Node<T> *, bool (**func)(), std::vector<ValueVar> &, unsigned)>(
          func)[p->sons[i]->header.typeNode](p->sons[i], func, fixedValue,
                                             globalStamp);
      if (p->nbModels == 0) return false;
    }

    return true;
  }  // isSAT

  /**
     Print out the NNF in a stream.

     @param[in] node, is equivalent to this.
     @param[in] func, give for the type of node the deallocate function.
     @param[in] out, the stream where we print out the formula.
     @param[in] idx, the next possible index.
     @param[in] globalStamp, use to stamp if we visit a not or not.

     \return the index of the node.
  */
  static unsigned printNNF(Node<T> *node, unsigned (**func)(),
                           std::ostream &out, unsigned &idx,
                           unsigned globalStamp) {
    auto *p = reinterpret_cast<DecomposableAndNode *>(node);
    if (p->header.stamp == globalStamp) return (unsigned)p->nbModels;
    p->nbModels = idx++;

    out << "a " << (unsigned)p->nbModels << " 0\n";

    for (unsigned i = 0; i < p->size; i++) {
      unsigned sidx =
          reinterpret_cast<unsigned (**)(Node<T> *, unsigned (**func)(),
                                         std::ostream &, unsigned &, unsigned)>(
              func)[p->sons[i]->header.typeNode](p->sons[i], func, out, idx,
                                                 globalStamp);
      out << (unsigned)p->nbModels << " " << sidx << " 0\n";
    }

    p->header.stamp = globalStamp;
    return (unsigned)p->nbModels;
  }  // printNNF
};
}  // namespace d4
