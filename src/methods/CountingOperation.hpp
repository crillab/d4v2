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
#include <boost/multiprecision/cpp_dec_float.hpp>
#include <boost/multiprecision/detail/default_ops.hpp>
#include <boost/multiprecision/gmp.hpp>

#include "DataBranch.hpp"
#include "src/exceptions/BadBehaviourException.hpp"

namespace d4 {
template <class T, class U>
class Operation;
template <class T>
class CountingOperation : public Operation<T, T> {
 private:
  ProblemManager *m_problem;

  const T top = T(1);
  const T bottom = T(0);

 public:
  CountingOperation() = delete;

  /**
     Constructor.

     @param[in] problem, allows to get information about the problem such as
     weights.
   */
  CountingOperation(ProblemManager *problem)
      : m_problem(problem) {}  // constructor.

  /**
     Compute the sum of the given elements.

     @param[in] elts, the elements we want to get the product.
     @param[in] size, the number of elements.

     \return the product of each element of elts.
  */
  T manageDeterministOr(DataBranch<T> *elts, unsigned size) {
    assert(size == 2);
    return elts[0].d * m_problem->computeWeightUnitFree<T>(elts[0].unitLits,
                                                           elts[0].freeVars) +
           elts[1].d * m_problem->computeWeightUnitFree<T>(elts[1].unitLits,
                                                           elts[1].freeVars);
  }  // manageDeterministOr

  /**
     Compute the product of the given elements.

     @param[in] elts, the elements we want to get the product.
     @param[in] size, the number of elements.

     \return the product of each element of elts.
   */
  T manageDecomposableAnd(T *elts, unsigned size) {
    if (size == 1) return elts[0];
    if (size == 2) return elts[0] * elts[1];

    T ret = 1;
    for (unsigned i = 0; i < size; i++) ret = ret * elts[i];
    return ret;
  }  // manageDecomposableAnd

  /**
     Manage the case where the problem is unsatisfiable.

     \return 0 as number of models.
   */
  T manageBottom() { return bottom; }  // manageBottom

  /**
     Return false, that is given by the value 1.

     \return T(0).
   */
  inline T createBottom() { return bottom; }

  /**
     Manage the case where the problem is a tautology.

     @param[in] component, the current set of variables (useless here).

     \return 0 as number of models.
   */
  inline T manageTop(std::vector<Var> &component) { return top; }

  /**
     Return true, that is given by the value 1.

     \return T(1).
   */
  inline T createTop() { return top; }

  /**
     Manage the case where we only have a branch in our OR gate.

     @param[in] e, the branch we are considering.

     \return the number of models associate to the given branch.
   */
  T manageBranch(DataBranch<T> &e) {
    return e.d * m_problem->computeWeightUnitFree<T>(e.unitLits, e.freeVars);
  }  // manageBranch

  /**
     Manage the final result compute.

     @param[in] result, the result we are considering.
     @param[in] vm, a set of options that describes what we want to do on the
     given result.
     @param[in] out, the output stream.
   */
  void manageResult(T &result, po::variables_map &vm, std::ostream &out) {
    std::string format = vm["keyword-output-format-solution"].as<std::string>();
    std::string outFormat = vm["output-format"].as<std::string>();

    if (outFormat == "competition") {
      boost::multiprecision::mpf_float::default_precision(128);
      out.precision(std::numeric_limits<
                    boost::multiprecision::cpp_dec_float_50>::digits10);

      if (result == 0) {
        out << "s UNSATISFIABLE\n";
        out << "c " << format << "\n";
        out << "c s log10-estimate -inf\n";
        out << "c s exact quadruple int 0\n";
      } else {
        out << "s SATISFIABLE\n";
        out << "c " << format << "\n";
        out << "c s log10-estimate "
            << boost::multiprecision::log10(
                   boost::multiprecision::cpp_dec_float_100(result))
            << "\n";
        if (vm["float"].as<bool>())
          out << "c s exact quadruple int " << result << "\n";
        else
          out << "c s exact arb int " << result << "\n";
      }
    } else {
      assert(outFormat == "classic");
      out << format << " ";
      out << std::fixed << std::setprecision(50) << result << "\n";
    }
  }  // manageResult

  /**
     Count the number of model, for this case that means doing noting.

     \return the number of models.
   */
  T count(T &result) { return result; }  // count

  /**
     Cannot be called, then throws an exception!
   */
  T count(T &result, std::vector<Lit> &assum) {
    throw(BadBehaviourException(
        "This operation is not allowed in this context.", __FILE__, __LINE__));
  }  // count
};

}  // namespace d4
