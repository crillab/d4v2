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

#include "src/exceptions/NodeException.hpp"
#include "src/problem/ProblemManager.hpp"

namespace d4 {
template <class T, typename U>
class DecomposableAndNode;
template <class T, typename U>
class BinaryDeterministicOrNode;
template <class T, typename U>
class UnaryNode;
template <class T>
class TrueNode;
template <class T>
class FalseNode;

enum TypeNode {
  TypeIteNode,
  TypeUnaryNode,
  TypeDecAndNode,
  TypeTrueNode,
  TypeFalseNode,
  count
};
enum ValueVar { isTrue, isFalse, isNotAssigned };

template <class T>
class Node {
 public:
  struct {
    unsigned typeNode : 4;
    unsigned stamp : 28;
  } header;

  Node() { header = {0, 0}; }
};
}  // namespace d4
