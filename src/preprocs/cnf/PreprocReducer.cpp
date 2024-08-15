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

#include "PreprocReducer.hpp"

#include <csignal>

#include "src/problem/cnf/ProblemManagerCnf.hpp"

namespace d4 {

/**
   The constructor.

   @param[in] vm, the options used (solver).
 */
PreprocReducer::PreprocReducer(const std::string &method, int nbIteration,
                               std::ostream &out) {
  m_method = method;
  m_nbIteration = nbIteration;
}  // constructor

/**
   Destructor.
 */
PreprocReducer::~PreprocReducer() {}  // destructor

/**
 * @brief PreprocReducer::run implementation.
 */
ProblemManager *PreprocReducer::run(ProblemManager *pin, unsigned timeout) {
  // prepage the clauses.
  ProblemManagerCnf &pcnf = dynamic_cast<ProblemManagerCnf &>(*pin);
  std::vector<std::vector<bipe::Lit>> clauses;
  for (auto &cl : pcnf.getClauses()) {
    clauses.push_back({});
    for (auto l : cl)
      clauses.back().push_back(bipe::Lit::makeLit(l.var(), l.sign()));
  }

  // create the problem from the reducer side.
  bipe::reducer::Method *rm =
      bipe::reducer::Method::makeMethod("combinaison", std::cout);

  rm->run(pin->getNbVar(), clauses, 10, true, clauses);

  ProblemManagerCnf *ret = new ProblemManagerCnf(
      pin->getNbVar(), pin->getWeightLit(), pin->getWeightVar(),
      pin->getSelectedVar(), pin->getMaxVar(), pin->getIndVar());

  // transfer the clauses to the returned formula.
  std::vector<std::vector<Lit>> &clausesAfter = ret->getClauses();
  for (auto &cl : clauses) {
    clausesAfter.push_back({});
    for (auto &l : cl)
      clausesAfter.back().push_back(Lit::makeLit(l.var(), l.sign()));
  }

  delete rm;
  return ret;
}  // run
}  // namespace d4
