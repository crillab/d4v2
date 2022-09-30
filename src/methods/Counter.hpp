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
#include <vector>

#include "DpllStyleMethod.hpp"
#include "nnf/Node.hpp"
#include "src/exceptions/BadBehaviourException.hpp"
#include "src/problem/ProblemTypes.hpp"

namespace d4 {
template <class T, class U>
class DpllStyleMethod;

template <class T>
class Counter {
 public:
  /**
     As for the method manager, but we return a counter (actually we also verify
     the it is a counter that is requiered).

     @param[in] vm, the map of option.
     @param[in] out, the stream where are print the information.
     @param[in] meth, the method we search to construct.
     @param[in] precision, the precision for the bignum.
     @param[in] out, the stream where are printed the information.

     \return a counter.
  */
  static Counter<T> *makeCounter(po::variables_map &vm, ProblemManager *problem,
                                 std::string meth, bool isFloat, int precision,
                                 std::ostream &out,
                                 LastBreathPreproc &lastBreath) {
    out << "c [CONSTRUCTOR] MethodManager: " << meth << "\n";
    boost::multiprecision::mpf_float::default_precision(
        precision);  // we set the precision

    if (meth == "counting")
      return new DpllStyleMethod<T, T>(vm, meth, isFloat, problem, out,
                                       lastBreath);
    if (meth == "ddnnf-compiler")
      return new DpllStyleMethod<T, Node<T> *>(vm, meth, isFloat, problem, out,
                                               lastBreath);

    throw(BadBehaviourException(
        "Cannot create a counter with the given options.", __FILE__, __LINE__));
  }  // makeCounter

  virtual ~Counter() {}

  virtual T count(std::vector<Var> &setOfVar, std::vector<Lit> &assumption,
                  std::ostream &out) = 0;

  /**
     Count the number of model on the problem.

     @param[in] setOfvar, the set of variables involved in the considered
     problem.
     @param[in] out, the stream where are print out the log.

     \return the number of models.
   */
  T count(std::vector<Var> &setOfVar, std::ostream &out) {
    std::vector<Lit> assum;
    return count(setOfVar, assum, out);
  }
};
}  // namespace d4
