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

#include <fstream>
#include <iostream>

#include "DataBranch.hpp"
#include "QueryManager.hpp"
#include "nnf/NodeManager.hpp"
#include "src/solvers/WrapperSolver.hpp"
#include "src/specs/SpecManager.hpp"

namespace d4 {
template <class T, class U>
class Operation;
template <class T, class U>
class DecisionDNNFOperation : public Operation<T, U> {
 private:
  ProblemManager *m_problem;
  NodeManager<T> *m_nodeManager;
  WrapperSolver *m_solver;

 public:
  DecisionDNNFOperation() = delete;

  /**
     Constructor.

     @param[in] problem, allows to get information about the problem such as
     weights.
   */
  DecisionDNNFOperation(ProblemManager *problem, SpecManager *specs,
                        WrapperSolver *solver)
      : m_problem(problem), m_solver(solver) {
    m_nodeManager = NodeManager<T>::makeNodeManager(specs->getNbVariable() + 1);
  }  // constructor.

  /**
     Destructor.
   */
  ~DecisionDNNFOperation() { delete m_nodeManager; }  // destructor

  /**
     Create top node and returns it.

     \return a top node.
   */
  U createTop() { return m_nodeManager->makeTrueNode(); }  // createTop

  /**
     Create bottom node and returns it.

     \return a bottom node.
   */
  U createBottom() { return m_nodeManager->makeFalseNode(); }  // createBottom

  /**
     Compute the sum of the given elements.

     @param[in] elts, the elements we want to get the product.
     @param[in] size, the number of elements.

     \return the product of each element of elts.
  */
  U manageDeterministOr(DataBranch<U> *elts, unsigned size) {
    assert(size == 2);
    return m_nodeManager->makeBinaryDeterministicOrNode(elts[0], elts[1]);
  }  // manageDeterministOr

  /**
     Compute the product of the given elements.

     @param[in] elts, the elements we want to get the product.
     @param[in] size, the number of elements.

     \return the product of each element of elts.
   */
  U manageDecomposableAnd(U *elts, unsigned size) {
    return m_nodeManager->makeDecomposableAndNode(elts, size);
  }  // manageDecomposableAnd

  /**
     Manage the case where the problem is unsatisfiable.

     \return 0 as number of models.
   */
  U manageBottom() { return createBottom(); }  // manageBottom

  /**
     Manage the case where the problem is a tautology.

     @param[in] component, the current set of variables (useless here).

     \return 0 as number of models.
   */
  U manageTop(std::vector<Var> &component) {
    DataBranch<U> b;
    m_solver->whichAreUnits(component, b.unitLits);  // collect unit literals
    b.d = m_nodeManager->makeTrueNode();
    if (b.unitLits.size()) return m_nodeManager->makeUnaryNode(b);
    return b.d;
  }  // manageTop

  /**
     Manage the case where we only have a branch in our OR gate.

     @param[in] e, the branch we are considering.

     \return the number of models associate to the given branch.
   */
  U manageBranch(DataBranch<U> &e) {
    return m_nodeManager->makeUnaryNode(e);
  }  // manageBranch

  /**
     Manage the final result compute.

     @param[in] result, the result we are considering.
     @param[in] vm, a set of options that describes what we want to do on the
     given result.
     @param[in] out, the output stream.
  */
  void manageResult(U &result, po::variables_map &vm, std::ostream &out) {
    if (vm.count("dump-ddnnf")) {
      std::ofstream outFile;
      std::string fileName = vm["dump-ddnnf"].as<std::string>();
      outFile.open(fileName);
      m_nodeManager->printNNF(result, outFile);
      outFile.close();
    } else if (vm.count("query")) {
      std::vector<Lit> query;
      std::vector<ValueVar> fixedValue(m_problem->getNbVar() + 1,
                                       ValueVar::isNotAssigned);

      std::string fileName = vm["query"].as<std::string>();
      QueryManager queryManager(fileName);
      TypeQuery typeQuery = TypeQuery::QueryEnd;

      do {
        typeQuery = queryManager.next(query);
        for (auto &l : query) {
          if ((unsigned)l.var() >= fixedValue.size()) continue;
          fixedValue[l.var()] =
              (l.sign()) ? ValueVar::isFalse : ValueVar::isTrue;
        }

        if (typeQuery == TypeQuery::QueryCounting) {
          out << "s " << std::fixed
              << m_nodeManager->computeNbModels(result, fixedValue, *m_problem)
              << "\n";
        } else if (typeQuery == TypeQuery::QueryDecision) {
          bool res = m_nodeManager->isSAT(result, fixedValue);
          out << "s " << ((res) ? "SAT" : "UNS") << "\n";
        }

        for (auto &l : query) {
          if ((unsigned)l.var() >= fixedValue.size()) continue;
          fixedValue[l.var()] = ValueVar::isNotAssigned;
        }
      } while (typeQuery != TypeQuery::QueryEnd);
    } else {
      std::vector<ValueVar> fixedValue(m_problem->getNbVar() + 1,
                                       ValueVar::isNotAssigned);
      out << "s " << std::fixed
          << m_nodeManager->computeNbModels(result, fixedValue, *m_problem)
          << "\n";
    }

    m_nodeManager->deallocate(result);
  }  // manageResult

  /**
     Compute the number of model on the dDNNF.

     @param[in] root, the root of the dDNNF.

     \return the number of models.
   */
  T count(U &root) {
    std::vector<ValueVar> fixedValue(m_problem->getNbVar() + 1,
                                     ValueVar::isNotAssigned);
    return m_nodeManager->computeNbModels(root, fixedValue, *m_problem);
  }  // count

  /**
     Compute the number of model on the dDNNF.

     @param[in] root, the root of the dDNNF.
     @param[in] assum, a set of literals used to conditioned the dDNNF.

     \return the number of models.
   */
  T count(U &root, std::vector<Lit> &assum) {
    std::vector<ValueVar> fixedValue(m_problem->getNbVar() + 1,
                                     ValueVar::isNotAssigned);
    for (auto &l : assum) {
      if ((unsigned)l.var() >= fixedValue.size()) continue;
      fixedValue[l.var()] = (l.sign()) ? ValueVar::isFalse : ValueVar::isTrue;
    }

    return m_nodeManager->computeNbModels(root, fixedValue, *m_problem);
  }  // count
};

}  // namespace d4
