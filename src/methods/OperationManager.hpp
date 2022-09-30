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
#include <boost/multiprecision/gmp.hpp>
#include <boost/program_options.hpp>

#include "CountingOperation.hpp"
#include "DataBranch.hpp"
#include "DecisionDNNFOperation.hpp"
#include "nnf/Branch.hpp"
#include "nnf/Node.hpp"
#include "nnf/NodeManager.hpp"
#include "src/exceptions/FactoryException.hpp"

namespace d4 {
namespace po = boost::program_options;
template <class T, class U>
class Operation {
 public:
  /**
     Operation factory.

     @param[in] vm, the option list.
     @param[in] problem, the problem description.
     @param[in] specs, the problem specification.
     @param[in] solver, the SAT solver used for the compiler.
     @param[in] out, where are print out the log.

     \return an operation manager regarding the given options.
  */
  static void *makeOperationManager(std::string &meth, bool isFloat,
                                    ProblemManager *problem, SpecManager *specs,
                                    WrapperSolver *solver, std::ostream &out) {
    out << "c [CONSTRUCTOR] Operation: "
        << "method(" << meth << ") "
        << "float(" << isFloat << ")\n";

    if (meth == "counting") return new CountingOperation<T>(problem);

    if (meth == "ddnnf-compiler")
      return new DecisionDNNFOperation<T, Node<T> *>(problem, specs, solver);

    throw(FactoryException("Cannot create a Operation", __FILE__, __LINE__));
  }  // makeOperationManager

  virtual ~Operation() {}

  virtual U createTop() = 0;
  virtual U createBottom() = 0;
  virtual void manageResult(U &result, po::variables_map &vm,
                            std::ostream &out) = 0;
  virtual U manageBottom() = 0;
  virtual U manageTop(std::vector<Var> &component) = 0;
  virtual U manageBranch(DataBranch<U> &e) = 0;
  virtual U manageDeterministOr(DataBranch<U> *elts, unsigned size) = 0;
  virtual U manageDecomposableAnd(U *elts, unsigned size) = 0;
  virtual T count(U &result) = 0;
  virtual T count(U &result, std::vector<Lit> &assum) = 0;
};
}  // namespace d4
