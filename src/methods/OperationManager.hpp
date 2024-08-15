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
#include <boost/multiprecision/gmp.hpp>

#include "CountingOperation.hpp"
#include "DataBranch.hpp"
#include "DecisionDNNFOperation.hpp"
#include "nnf/Branch.hpp"
#include "nnf/Node.hpp"
#include "nnf/NodeManager.hpp"
#include "src/exceptions/FactoryException.hpp"
#include "src/options/methods/OptionOperationManager.hpp"

namespace d4 {
template <class T, class U>
class Operation {
 public:
  /**
     Operation factory.

     @param[in] options are the options given to the operator manager.
     @param[in] problem, the problem description.
     @param[in] specs, the problem specification.
     @param[in] solver, the SAT solver used for the compiler.
     @param[in] out, where are print out the log.

     \return an operation manager regarding the given options.
  */
  static void *makeOperationManager(const OptionOperationManager &options,
                                    ProblemManager *problem, SpecManager *specs,
                                    WrapperSolver *solver, std::ostream &out) {
    out << "c [OPERATION MANAGER]" << options << '\n';

    if (options.operatorType == OP_COUNTING)
      return new CountingOperation<T>(problem);

    if (options.operatorType == OP_CIRC)
      return new DecisionDNNFOperation<T, Node<T> *>(problem, specs, solver);

    throw(FactoryException("Cannot create a Operation", __FILE__, __LINE__));
  }  // makeOperationManager

  virtual ~Operation() {}

  virtual U createTop() = 0;
  virtual U createBottom() = 0;
  virtual U manageBottom() = 0;
  virtual U manageTop(std::vector<Var> &component) = 0;
  virtual U manageBranch(DataBranch<U> &e) = 0;
  virtual U manageDeterministOr(DataBranch<U> *elts, unsigned size) = 0;
  virtual U manageDecomposableAnd(U *elts, unsigned size) = 0;
  virtual T count(U &result) = 0;
  virtual T count(U &result, std::vector<Lit> &assum) = 0;
};
}  // namespace d4
