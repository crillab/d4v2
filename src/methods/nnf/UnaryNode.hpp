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
class UnaryNode : public Node<T> {
 public:
  using Node<T>::header;
  T nbModels;
  Branch<T, U> b;
  // data[0 .. b.nbLit - 1] gives the unit literals for the left branch.
  // data[b.nbLit .. b.nbLit + b.nbFree - 1] gives the free variables for the
  // left branch.
  U data[0];

  UnaryNode() = delete;

  /**
     Init the two branches using the data coming from the solver.

     @param[in] left, the left branch.
     @param[in] right, the right branch.
   */
  UnaryNode(DataBranch<Node<T> *> &branch) {
    header.typeNode = TypeNode::TypeUnaryNode;
    header.stamp = 0;
    b.d = branch.d;
    nbModels = T(0);

    b.nbUnits = branch.unitLits.size();
    b.nbFree = branch.freeVars.size();

    unsigned pos = 0;
    for (auto &l : branch.unitLits) data[pos++] = l.intern();
    for (auto &v : branch.freeVars) data[pos++] = v;
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
    reinterpret_cast<UnaryNode *>(node)->nbModels.~T();
    reinterpret_cast<void (**)(Node<T> *, void (**func)(), unsigned)>(
        func)[(reinterpret_cast<UnaryNode *>(node)->b).d->header.typeNode](
        (reinterpret_cast<UnaryNode *>(node)->b).d, func, globalStamp);
  }  // destructor

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
    auto *p = reinterpret_cast<UnaryNode *>(node);

    if (node->header.stamp == globalStamp) return p->nbModels;
    p->nbModels =
        p->b.computeNbModels(func, p->data, fixedValue, problem, globalStamp);
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
    auto *p = reinterpret_cast<UnaryNode *>(node);

    if (node->header.stamp == globalStamp) return p->nbModels == 1;
    p->nbModels = p->b.isSAT(func, p->data, fixedValue, globalStamp);
    node->header.stamp = globalStamp;
    return p->nbModels == 1;
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
    auto *p = reinterpret_cast<UnaryNode *>(node);
    if (p->header.stamp == globalStamp) return (unsigned)p->nbModels;
    p->nbModels = idx++;

    out << "o " << (unsigned)p->nbModels << " 0\n";
    p->b.printNNF((unsigned)p->nbModels, p->data, func, out, idx, globalStamp);

    p->header.stamp = globalStamp;
    return (unsigned)p->nbModels;
  }  // printNNF
};
}  // namespace d4
