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
#include <vector>

#include "DpllStyleMethod.hpp"
#include "nnf/Node.hpp"
#include "src/configurations/Configuration.hpp"
#include "src/exceptions/BadBehaviourException.hpp"
#include "src/options/methods/OptionDpllStyleMethod.hpp"
#include "src/problem/ProblemTypes.hpp"

namespace d4 {
template <class T, class U>
class DpllStyleMethod;

template <class T>
class Counter {
 public:
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
