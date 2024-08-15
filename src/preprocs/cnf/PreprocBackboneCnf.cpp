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
#include "PreprocBackboneCnf.hpp"

#include <bits/types/clock_t.h>

#include <csignal>
#include <ctime>

#include "3rdParty/bipe/src/bipartition/methods/Method.hpp"
#include "src/problem/cnf/ProblemManagerCnf.hpp"

namespace d4 {

/**
   The constructor.

   @param[in] vm, the options used (solver).
 */
PreprocBackboneCnf::PreprocBackboneCnf(std::ostream &out) {}  // constructor

/**
   Destructor.
 */
PreprocBackboneCnf::~PreprocBackboneCnf() {}  // destructor

/**
 * @brief PreprocBackboneCnf::run implementation.
 */
ProblemManager *PreprocBackboneCnf::run(ProblemManager *pin, unsigned timeout) {
  // create the problem regarding the bipe library.
  std::vector<Var> protect, selected;
  if (pin->getSelectedVar().size())
    selected = pin->getSelectedVar();
  else
    for (unsigned i = 1; i <= pin->getNbVar(); i++)
      if (pin->getWeightLit(Lit::makeLitTrue(i)) ==
          pin->getWeightLit(Lit::makeLitFalse(i)))
        selected.push_back(i);

  std::vector<double> tmp(pin->getNbVar() + 1, 1.0);
  bipe::Problem pb(pin->getNbVar(), tmp, selected, protect);

  ProblemManagerCnf &pcnf = dynamic_cast<ProblemManagerCnf &>(*pin);
  std::vector<std::vector<bipe::Lit>> &clauses = pb.getClauses();

  for (auto &cl : pcnf.getClauses()) {
    clauses.push_back({});
    for (auto l : cl)
      clauses.back().push_back(bipe::Lit::makeLit(l.var(), l.sign()));
  }

  // call the preprocessor to compute the backbone.
  bipe::bipartition::Method bb;
  std::vector<bipe::Gate> gates;
  std::vector<std::vector<bool>> setOfModels;

  std::cerr << "c [PREPOC BACKBONE] Is running for at most " << timeout
            << " seconds\n";

  PreprocManager::s_isRunning = &bb;

  // change the handler.
  void (*handler)(int) = [](int s) {
    if (PreprocManager::s_isRunning)
      ((bipe::bipartition::Method *)PreprocManager::s_isRunning)->interrupt();
  };
  signal(SIGALRM, handler);
  alarm(timeout);

  bool res = bb.simplifyBackbone(
      pb, (bipe::bipartition::OptionBackbone){true, timeout, true, "glucose"},
      gates, std::cout, setOfModels);
  s_isRunning = nullptr;

  if (!res) {
    std::cout
        << "c [PREPROC BACKBONE] The process has been stopped before the end\n";
  }

  // the list of unit literals.
  std::vector<Lit> units;
  for (auto g : gates)
    units.push_back(Lit::makeLit(g.output.var(), g.output.sign()));

  std::cout << "c [PREPOC BACKBONE] Backone size: " << units.size() << "\n";
  return pin->getConditionedFormula(units);
}  // run
}  // namespace d4
