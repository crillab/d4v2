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

#include <bitset>
#include <vector>

#include "../DataBranch.hpp"
#include "BinaryDeterministicOrNode.hpp"
#include "Branch.hpp"
#include "DecomposableAndNode.hpp"
#include "FalseNode.hpp"
#include "Node.hpp"
#include "TrueNode.hpp"
#include "UnaryNode.hpp"

namespace d4 {
template <class T>
class NodeManager;
template <class T, typename U>
class NodeManagerTyped : public NodeManager<T> {
  using NodeManager<T>::m_memoryPages;
  using NodeManager<T>::m_posInMemoryPage;
  using NodeManager<T>::m_data;
  using NodeManager<T>::m_globalStamp;
  using NodeManager<T>::PAGE_SIZE;

  TrueNode<T> *trueNode;
  FalseNode<T> *falseNode;

 public:
  /**
     The constructor allocate the first memory page.
   */
  NodeManagerTyped() {
    m_data = new uint8_t[PAGE_SIZE];
    m_memoryPages.push_back(m_data);
    m_posInMemoryPage = 0;
    m_globalStamp = 0;

    unsigned memoryNeeded = sizeof(TrueNode<T>);
    uint8_t *data = NodeManager<T>::getMemory(memoryNeeded);
    trueNode = new (data) TrueNode<T>();

    memoryNeeded = sizeof(FalseNode<T>);
    data = NodeManager<T>::getMemory(memoryNeeded);
    falseNode = new (data) FalseNode<T>();
  }  // NodeManagerTyped

  /**
     \return a pointer on a true node.
   */
  inline Node<T> *makeTrueNode() { return trueNode; }  // makeTrueNode

  /**
     \return a pointer on a false node.
   */
  inline Node<T> *makeFalseNode() { return falseNode; }  // makeFalseNode

  /**
     Create a binary deterministic OR node.

     @param[in] left, the left branch.
     @param[in] right, the right branch.

     \return a BinaryDeterministicOrNode that make the disjcution between left
     and right.
  */
  Node<T> *makeBinaryDeterministicOrNode(DataBranch<Node<T> *> &left,
                                         DataBranch<Node<T> *> &right) {
    unsigned memoryNeeded =
        sizeof(BinaryDeterministicOrNode<T, U>) +
        (left.sumFreeUnit() + right.sumFreeUnit()) * sizeof(U);

    uint8_t *data = NodeManager<T>::getMemory(memoryNeeded);
    return new (data) BinaryDeterministicOrNode<T, U>(left, right);
  }  // makeBinaryDeterministicOrNode

  /**
     Create an unary branch.

     @param[in] left, the branch.

     \return a UnaryNode..
  */
  Node<T> *makeUnaryNode(DataBranch<Node<T> *> &branch) {
    unsigned memoryNeeded =
        sizeof(UnaryNode<T, U>) + sizeof(U) * branch.sumFreeUnit();
    uint8_t *data = NodeManager<T>::getMemory(memoryNeeded);
    return new (data) UnaryNode<T, U>(branch);
  }  // makeBinaryDeterministicOrNode

  /**
     Create a decomposable AND node.

     @param[in] sons, the sons.
     @param[in] size, the number of sons.

     \return a DecomposableAndNode that regroup the elements given in parameter.
  */
  inline Node<T> *makeDecomposableAndNode(Node<T> **sons, unsigned size) {
    if (size == 1) return *sons;

    unsigned memoryNeeded =
        sizeof(DecomposableAndNode<T, U>) + size * sizeof(Node<T> *);
    uint8_t *data = NodeManager<T>::getMemory(memoryNeeded);
    return new (data) DecomposableAndNode<T, U>(size, sons);
  }  // makeDecomposableAndNode

  /**
     Compute the number of models regarding a node.

     @param[in] node, the node we start from to get the number of models.
     @param[in] fixedValue, use to know which variables are assigned or not.
     @param[in] problem, the problem specification.

     \return the number of models.
   */
  T computeNbModels(Node<T> *node, std::vector<ValueVar> &fixedValue,
                    ProblemManager &problem) {
    T(*func[TypeNode::count])
    (Node<T> * node, T(*t[])(), std::vector<ValueVar> &, ProblemManager &,
     unsigned);

    func[TypeNode::TypeDecAndNode] = DecomposableAndNode<T, U>::computeNbModels;
    func[TypeNode::TypeIteNode] =
        BinaryDeterministicOrNode<T, U>::computeNbModels;
    func[TypeNode::TypeUnaryNode] = UnaryNode<T, U>::computeNbModels;
    func[TypeNode::TypeFalseNode] = FalseNode<T>::computeNbModels;
    func[TypeNode::TypeTrueNode] = TrueNode<T>::computeNbModels;

    m_globalStamp++;
    return func[node->header.typeNode](node, (T(**)())func, fixedValue, problem,
                                       m_globalStamp);
  }  // computeNbModels

  /**
     Test if the problem is SAT or not.

     @param[in] node, the node we start from to get the number of models.
     @param[in] fixedValue, use to know which variables are assigned or not.

     \return true if the formula conditioned is satisfiable, false otherwise.
  */
  bool isSAT(Node<T> *node, std::vector<ValueVar> &fixedValue) {
    bool (*func[TypeNode::count])(Node<T> * node, bool (*t[])(),
                                  std::vector<ValueVar> &, unsigned);

    func[TypeNode::TypeDecAndNode] = DecomposableAndNode<T, U>::isSAT;
    func[TypeNode::TypeIteNode] = BinaryDeterministicOrNode<T, U>::isSAT;
    func[TypeNode::TypeUnaryNode] = UnaryNode<T, U>::isSAT;
    func[TypeNode::TypeFalseNode] = FalseNode<T>::isSAT;
    func[TypeNode::TypeTrueNode] = TrueNode<T>::isSAT;

    m_globalStamp++;
    return func[node->header.typeNode](node, (bool (**)())func, fixedValue,
                                       m_globalStamp);
  }  // isSAT

  void printNNF(Node<T> *node, std::ostream &out) {
    unsigned (*func[TypeNode::count])(Node<T> * node, unsigned (*t[])(),
                                      std::ostream &, unsigned &, unsigned);
    func[TypeNode::TypeDecAndNode] = DecomposableAndNode<T, U>::printNNF;
    func[TypeNode::TypeIteNode] = BinaryDeterministicOrNode<T, U>::printNNF;
    func[TypeNode::TypeUnaryNode] = UnaryNode<T, U>::printNNF;
    func[TypeNode::TypeFalseNode] = FalseNode<T>::printNNF;
    func[TypeNode::TypeTrueNode] = TrueNode<T>::printNNF;

    m_globalStamp++;
    unsigned idx = 1;
    func[node->header.typeNode](node, (unsigned (**)())func, out, idx,
                                m_globalStamp);
  }  // printNNF

  /**
     Deallocate the memory of the member variables of all the graph from a given
     node.

     @param[in] node, the root node of the graph we want to free the members.
   */
  void deallocate(Node<T> *node) {
    void (*func[TypeNode::count])(Node<T> * node, void (*t[])(), unsigned int);

    func[TypeNode::TypeDecAndNode] = DecomposableAndNode<T, U>::deallocate;
    func[TypeNode::TypeIteNode] = BinaryDeterministicOrNode<T, U>::deallocate;
    func[TypeNode::TypeUnaryNode] = UnaryNode<T, U>::deallocate;
    func[TypeNode::TypeFalseNode] = FalseNode<T>::deallocate;
    func[TypeNode::TypeTrueNode] = TrueNode<T>::deallocate;

    m_globalStamp++;
    func[node->header.typeNode](node, (void (**)())func, m_globalStamp);
  }  // deallocate
};

template <class T>
class NodeManager {
 protected:
  std::vector<uint8_t *> m_memoryPages;
  unsigned m_posInMemoryPage;
  uint8_t *m_data;

  unsigned m_globalStamp;

  /**
     'Allocate' an ammount of bytes.

     @param[in] nbBytes, the ammount of memory we want to allocate.

     \return a pointer on the memory we allocate.
   */
  inline uint8_t *getMemory(unsigned nbBytes) {
    assert(nbBytes < PAGE_SIZE);  // we check out that the PAGE is large enough.
    if (m_posInMemoryPage + nbBytes >= PAGE_SIZE) {
      m_posInMemoryPage = 0;
      m_data = new uint8_t[PAGE_SIZE];
      m_memoryPages.push_back(m_data);
    }

    m_posInMemoryPage += nbBytes;
    return &m_data[m_posInMemoryPage - nbBytes];
  }  // getMemory

 public:
  static const unsigned PAGE_SIZE = 1 << 24;

  /**
     The destructor free the memory.
   */
  virtual ~NodeManager() {
    for (auto &p : m_memoryPages) delete[] p;
    m_posInMemoryPage = 0;
  }  // destructor

  /**
     Node constructor factory.

     @param[in] nbVar, the number of variables in the problem.
   */
  static NodeManager<T> *makeNodeManager(unsigned nbVar) {
    if (nbVar < (1 << 7)) return new NodeManagerTyped<T, uint8_t>();
    if (nbVar < (1 << 15)) return new NodeManagerTyped<T, uint16_t>();
    return new NodeManagerTyped<T, uint32_t>();
  }  // makeNodeManager

  virtual Node<T> *makeTrueNode() = 0;
  virtual Node<T> *makeFalseNode() = 0;

  virtual Node<T> *makeDecomposableAndNode(Node<T> **sons, unsigned size) = 0;

  virtual Node<T> *makeBinaryDeterministicOrNode(
      DataBranch<Node<T> *> &left, DataBranch<Node<T> *> &right) = 0;

  virtual Node<T> *makeUnaryNode(DataBranch<Node<T> *> &branch) = 0;

  virtual T computeNbModels(Node<T> *node, std::vector<ValueVar> &fixedValue,
                            ProblemManager &problem) = 0;

  virtual bool isSAT(Node<T> *node, std::vector<ValueVar> &fixedValue) = 0;

  virtual void printNNF(Node<T> *node, std::ostream &out) = 0;

  virtual void deallocate(Node<T> *node) = 0;
};
}  // namespace d4
