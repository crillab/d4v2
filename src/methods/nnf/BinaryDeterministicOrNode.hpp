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

#include "../DataBranch.hpp"
#include "Branch.hpp"
#include "Node.hpp"
#include "src/problem/ProblemManager.hpp"

namespace d4 {
template <class T, typename U>
class BinaryDeterministicOrNode : public Node<T> {
 public:
  using Node<T>::header;
  T nbModels;
  Branch<T, U> l, r;
  // data[0 .. l.nbLit - 1] gives the unit literals for the left branch.
  // data[l.nbLit .. l.nbLit + l.nbFree - 1] gives the free variables for the
  // left branch. data[l.nbLit + l.nbFree .. l.nbLit + l.nbFree + r.nbLit - 1]
  // gives the unit literals for the rightt branch.
  // data[l.nbLit + l.nbFree + r.nbLit .. l.nbLit + l.nbFree + r.nbLit +
  // r.nbFree - 1] gives the free variables for the right branch.
  U data[0];

  BinaryDeterministicOrNode() = delete;

  /**
     Deallocate the memory.

     @param[in] node, is equivalent to this.
     @param[in] func, give for the type of node the deallocate function.
     @param[in] globalstamp, get the stamp number.
  */
  static void deallocate(Node<T> *node, void (**func)(), unsigned globalStamp) {
    if (node->header.stamp == globalStamp) return;
    node->header.stamp = globalStamp;

    auto *p = reinterpret_cast<BinaryDeterministicOrNode *>(node);
    p->nbModels.~T();
    reinterpret_cast<void (**)(Node<T> *, void (**func)(), unsigned)>(
        func)[(p->l).d->header.typeNode]((p->l).d, func, globalStamp);
    reinterpret_cast<void (**)(Node<T> *, void (**func)(), unsigned)>(
        func)[(p->r).d->header.typeNode]((p->r).d, func, globalStamp);
  }  // destructor

  /**
     Init the two branches using the data coming from the solver.

     @param[in] left, the left branch.
     @param[in] right, the right branch.
   */
  BinaryDeterministicOrNode(DataBranch<Node<T> *> &left,
                            DataBranch<Node<T> *> &right) {
    header.typeNode = TypeNode::TypeIteNode;
    header.stamp = 0;

    l.d = left.d;
    r.d = right.d;

    l.nbUnits = left.unitLits.size();
    r.nbUnits = right.unitLits.size();

    l.nbFree = left.freeVars.size();
    r.nbFree = right.freeVars.size();

    unsigned pos = 0;
    for (auto &lit : left.unitLits) data[pos++] = lit.intern();
    for (auto &var : left.freeVars) data[pos++] = var;
    for (auto &lit : right.unitLits) data[pos++] = lit.intern();
    for (auto &var : right.freeVars) data[pos++] = var;
  }  // constructor

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
    auto *p = reinterpret_cast<BinaryDeterministicOrNode *>(node);

    if (node->header.stamp == globalStamp) return p->nbModels;

    p->nbModels =
        (p->l).computeNbModels(func, p->data, fixedValue, problem,
                               globalStamp) +
        (p)->r.computeNbModels(func, &p->data[p->l.nbUnits + p->l.nbFree],
                               fixedValue, problem, globalStamp);

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
    auto *p = reinterpret_cast<BinaryDeterministicOrNode *>(node);

    if (node->header.stamp == globalStamp) return p->nbModels == 1;
    node->header.stamp = globalStamp;

    p->nbModels = (p->l).isSAT(func, p->data, fixedValue, globalStamp);
    if (p->nbModels == 1) return true;

    p->nbModels = (p->r).isSAT(func, &p->data[p->l.nbUnits + p->l.nbFree],
                               fixedValue, globalStamp);
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
    auto *p = reinterpret_cast<BinaryDeterministicOrNode *>(node);
    if (p->header.stamp == globalStamp) return (unsigned)p->nbModels;
    p->nbModels = idx++;

    out << "o " << (unsigned)p->nbModels << " 0\n";
    p->l.printNNF((unsigned)p->nbModels, p->data, func, out, idx, globalStamp);
    p->r.printNNF((unsigned)p->nbModels, &p->data[p->l.nbUnits + p->l.nbFree],
                  func, out, idx, globalStamp);

    p->header.stamp = globalStamp;
    return (unsigned)p->nbModels;
  }  // printNNF
};
}  // namespace d4
