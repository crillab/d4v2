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
#include "PreprocSharpEquiv.hpp"

#include <csignal>

#include "3rdParty/bipe/src/bipartition/methods/Backbone.hpp"
#include "3rdParty/bipe/src/bipartition/methods/Bipartition.hpp"
#include "3rdParty/bipe/src/bipartition/methods/DACircuit.hpp"

namespace d4 {

/**
 * The constructor.
 *
 * @param[in] vm, the options used (solver).
 */
PreprocSharpEquiv::PreprocSharpEquiv(int nbIteration, std::ostream &out) {
  m_nbIteration = nbIteration;
}  // constructor

/**
 * @brief computeBipartition implementation.
 */
void PreprocSharpEquiv::computeBipartition(ProblemManagerCnf &pcnf,
                                           std::vector<Lit> &units,
                                           std::vector<bipe::Var> &input,
                                           std::vector<bipe::Var> &output,
                                           std::vector<bipe::Gate> &gates,
                                           unsigned timeout) {
  std::vector<Var> protect, selected;
  if (pcnf.getSelectedVar().size())
    selected = pcnf.getSelectedVar();
  else
    for (unsigned i = 1; i <= pcnf.getNbVar(); i++)
      if (pcnf.getWeightLit(Lit::makeLitTrue(i)) ==
          pcnf.getWeightLit(Lit::makeLitFalse(i)))
        selected.push_back(i);
      else
        protect.push_back(i);

  std::vector<double> tmp(pcnf.getNbVar() + 1, 1.0);
  bipe::Problem pb(pcnf.getNbVar(), tmp, selected, protect);
  Lit::rewrite<bipe::Lit>(
      pcnf.getClauses(), units, pb.getClauses(),
      [](unsigned var, bool sign) { return bipe::Lit::makeLit(var, sign); });

  // Options:
  bipe::bipartition::OptionBackbone optionBackbone(false, 0, true, "glucose");
  bipe::bipartition::OptionDac optionDac(false, "glucose");
  bipe::bipartition::OptionBipartition optionBipartition(
      false, true, true, "OCC_ASC", "glucose", 0);

  bipe::bipartition::Bipartition b;
  bipe::Problem *formula = nullptr;

  s_isRunning = &b;
  std::cout << "c [PREPROC #EQUIV] Bipartition is running ...\n";

  // change the handler.
  void (*handler)(int) = [](int s) {
    if (PreprocManager::s_isRunning)
      ((bipe::bipartition::Method *)PreprocManager::s_isRunning)->interrupt();
  };
  signal(SIGALRM, handler);
  alarm(timeout);

  std::vector<std::vector<bool>> setOfModels;
  formula =
      b.simplifyBackbone(pb, optionBackbone, gates, std::cout, setOfModels);

  if (formula) {
    bipe::Problem *tmp = formula;
    formula = b.simplifyDac(*tmp, optionDac, gates, std::cout, setOfModels);
    delete tmp;
  }

  if (!formula) {
    input = pb.getProjectedVar();
  } else {
    std::vector<std::vector<bipe::Var>> symGroup;
    bool res = b.run(*formula, input, gates, optionBipartition, symGroup,
                     setOfModels, std::cout);

    if (!res) {
      std::cout << "c [PREPOC BACKBONE] The preproc has been stopped before "
                   "the end\n";
    }
  }

  // put the remaining variable into the output set.
  std::vector<bool> marked(pb.getNbVar() + 1, false);
  for (auto &v : input) marked[v] = true;
  for (unsigned i = 1; i < pb.getNbVar() + 1; i++)
    if (!marked[i]) output.push_back(i);
  assert(output.size() + input.size() == pb.getNbVar());

  delete formula;
  s_isRunning = NULL;
  std::cout << "c [PREPROC #EQUIV] ... done\n";
}  // computeBipartition

/**
 * @brief Destroy the Preproc Sharp Equiv:: Preproc Sharp Equiv object
 */
PreprocSharpEquiv::~PreprocSharpEquiv() {}  // destructor

/**
 * @brief The preprocessing itself.
 * @param[out] p, the problem we want to preprocess.
 * @param[out] lastBreath gives information about the way the    preproc sees
 * the problem.
 */
ProblemManager *PreprocSharpEquiv::run(ProblemManager *pin, unsigned timeout) {
  std::cout << "c [PREPROC #EQUIV] Start\n";

  std::vector<bool> isUnit(pin->getNbVar() + 1, false);

  // get the cnf.
  // create the problem regarding the bipe library.
  std::vector<Var> protect, selected;
  if (pin->getSelectedVar().size())
    selected = pin->getSelectedVar();
  else
    for (unsigned i = 1; i <= pin->getNbVar(); i++)
      if (pin->getWeightLit(Lit::makeLitTrue(i)) ==
          pin->getWeightLit(Lit::makeLitFalse(i)))
        selected.push_back(i);
      else
        protect.push_back(i);

  std::vector<double> tmp(pin->getNbVar() + 1, 1.0);
  bipe::Problem pb(pin->getNbVar(), tmp, selected, protect);

  ProblemManagerCnf &pcnf = dynamic_cast<ProblemManagerCnf &>(*pin);
  std::vector<std::vector<bipe::Lit>> &clauses = pb.getClauses();
  for (auto &cl : pcnf.getClauses()) {
    clauses.push_back({});
    for (auto l : cl)
      clauses.back().push_back(bipe::Lit::makeLit(l.var(), l.sign()));
  }

  unsigned limitNbClauses = pcnf.getClauses().size();

  // call the preprocessor to compute the bipartition.
  std::vector<bipe::Var> input, output;
  std::vector<bipe::Gate> gates;
  std::vector<Lit> units;
  computeBipartition(pcnf, units, input, output, gates, timeout);

  // create the problem from the reducer side.
  bipe::eliminator::Eliminator e;
  bipe::reducer::Method *rm =
      bipe::reducer::Method::makeMethod("combinaison", std::cout);

  // the reduction + elimination + reduction phase.
  rm->run(pin->getNbVar(), clauses, 5, true, clauses);
  std::vector<bipe::Lit> eliminated;
  e.eliminate(pin->getNbVar(), clauses, input, gates, eliminated, false,
              limitNbClauses);
  rm->run(pin->getNbVar(), clauses, 5, true, clauses);

  // the problem we return.
  ProblemManagerCnf *ret = new ProblemManagerCnf(
      pin->getNbVar(), pin->getWeightLit(), pin->getWeightVar(),
      pin->getSelectedVar(), pin->getMaxVar(), pin->getIndVar());

  // sort the clauses regarding their size.
  std::sort(
      clauses.begin(), clauses.end(),
      [](const std::vector<bipe::Lit> &a, const std::vector<bipe::Lit> &b) {
        return a.size() < b.size();
      });

  // transfer the clauses.
  std::vector<std::vector<Lit>> &clausesAfter = ret->getClauses();
  for (auto &cl : clauses) {
    clausesAfter.push_back({});
    for (auto &l : cl)
      clausesAfter.back().push_back(Lit::makeLit(l.var(), l.sign()));
  }

  // to be sure to expel the removed variables.
  for (auto &l : eliminated)
    clausesAfter.push_back({Lit::makeLit(l.var(), l.sign())});

  delete rm;
  return ret;
}  // run

}  // namespace d4
