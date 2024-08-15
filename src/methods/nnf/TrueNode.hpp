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

#include <iostream>
#include <vector>

#include "Node.hpp"
#include "src/problem/ProblemManager.hpp"

namespace d4 {
template <class T>
class TrueNode : public Node<T> {
 public:
  T nbModels;  // is used to get an index to print out the problem.

  using Node<T>::header;

  /**
     Constructor.
   */
  TrueNode() {
    header.typeNode = TypeNode::TypeTrueNode;
    header.stamp = 0;
  }  // constructor.

  /**
     Deallocate the memory.

     @param[in] node, is equivalent to this.
     @param[in] func, give for the type of node the deallocate function.
     @param[in] globalstamp, get the stamp number.
   */
  static void deallocate(Node<T> *node, void (**func)(), unsigned globalStamp) {
    auto *p = reinterpret_cast<TrueNode *>(node);
    if (p->header.stamp == globalStamp) return;
    p->header.stamp = globalStamp;
    p->nbModels.~T();
  }  // destructor

  /**
     Ask for the number of models of the formula under an interpretation.
     Because it is a true node, the number of models is of course 1.

     @param[in] node, is equivalent to this.
     @param[in] func, give for the type of node the deallocate function.
     @param[in] fixedValue, the assigment we consider
     @param[in] problem, the problem we are solving (use to get information
     about weight).
     @param[in] globalStamp, use to stamp if we visit a not or not (not used
     here)

     \return the number of models.
   */
  static T computeNbModels(Node<T> *node, T (**func)(),
                           std::vector<ValueVar> &fixedValue,
                           ProblemManager &problem, unsigned globalStamp) {
    return T(1);
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
    auto *p = reinterpret_cast<TrueNode *>(node);
    if (p->header.stamp == globalStamp) return (unsigned)p->nbModels;
    p->nbModels = idx++;

    out << "t " << (unsigned)p->nbModels << " 0\n";

    p->header.stamp = globalStamp;
    return (unsigned)p->nbModels;
  }  // printNNF
};
}  // namespace d4
