/**
 * bipe
 *  Copyright (C) 2021  Lagniez Jean-Marie
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Affero General Public License as published
 *  by the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Affero General Public License for more details.
 *
 *  You should have received a copy of the GNU Affero General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "src/bipartition/methods/Bipartition.hpp"

#include <cassert>
#include <ctime>
#include <iomanip>
#include <string>
#include <utility>
#include <vector>

#include "src/bipartition/heuristic/HeuristicBipartition.hpp"
#include "src/solver/WrapperSolver.hpp"
#include "src/utils/FactoryException.hpp"
#include "src/utils/Gate.hpp"
#include "src/utils/ProblemTypes.hpp"

namespace bipe {
namespace bipartition {

#define TEST 0

/**
 * @brief Print a line to separate the printed line.
 *
 * @param out is the stream.
 */
void Bipartition::separator(std::ostream &out) {
  out << "c ";
  for (unsigned i = 0; i < COLUMN_SIZE; i++) out << "-";
  out << "\n";
}  // separator

/**
 * @brief Print the header of the table.
 *
 * @param out is the stream.
 */
void Bipartition::printHeader(std::ostream &out) {
  separator(out);
  out << "c "
      << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "time"
      << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "time(SAT)"
      << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "time(UNS)"
      << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "time(UND)"
      << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "#call"
      << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "#call(SAT)"
      << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "#call(UNS)"
      << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "#call(UND)"
      << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "#i. mod."
      << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "#o. sym."
      << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "#remains"
      << " |\n";
  separator(out);
}  // printHeader

/**
 * @brief Print the log.
 *
 * @param out is the stream.
 */
void Bipartition::printInfo(HeuristicBipartition *heuristic,
                            std::ostream &out) {
  if (!((m_nbCall - 1) % FREQ_HEADER)) printHeader(out);

  out << "c " << std::fixed << std::setprecision(2) << "|"
      << std::setw(WIDTH_PRINT_COLUMN_MC) << m_timeCall << "|"
      << std::setw(WIDTH_PRINT_COLUMN_MC) << m_timeSATCall << "|"
      << std::setw(WIDTH_PRINT_COLUMN_MC) << m_timeUNSCall << "|"
      << std::setw(WIDTH_PRINT_COLUMN_MC) << m_timeUNDCall << "|"
      << std::setw(WIDTH_PRINT_COLUMN_MC) << m_nbCall << "|"
      << std::setw(WIDTH_PRINT_COLUMN_MC) << m_nbSATCall << "|"
      << std::setw(WIDTH_PRINT_COLUMN_MC) << m_nbUNSCall << "|"
      << std::setw(WIDTH_PRINT_COLUMN_MC) << m_nbUNDCall << "|"
      << std::setw(WIDTH_PRINT_COLUMN_MC) << m_nbInputFromModel << "|"
      << std::setw(WIDTH_PRINT_COLUMN_MC) << m_nbOutputFromSym << "|"
      << std::setw(WIDTH_PRINT_COLUMN_MC)
      << heuristic->remainingAssumptions().size() << " |\n";
}

void Bipartition::updateInfo(Status status) {
  m_nbCall++;
  m_lastCall = m_timeCall;
  m_timeCall = getTimer();
  float delta = m_timeCall - m_lastCall;
  if (status == SAT) {
    m_nbSATCall++;
    m_timeSATCall += delta;
  } else if (status == UNS) {
    m_nbUNSCall++;
    m_timeUNSCall += delta;
  } else {
    m_nbUNDCall++;
    m_timeUNDCall += delta;
  }
}  // updateInfo

/**
 * @brief Initialize the CNF with the formula that consider two versions of the
 * given problem linked with equivalence that can be activated or deactivated
 * using selectors.
 *
 * @param p is the initial problem.
 * @param[out] res is the resulting problem.
 * @param[out] selectors is the set of literals used to control the
 * equivalences.
 * @param[out] mapSelector is a map where we store the equivalence related to
 * the added selectors.
 * @param gates is a set of gates we found out.
 * @param[out] input is the variables that are forced to be in the input set.
 */
void Bipartition::initProblem(Problem &p, Problem &res,
                              std::vector<Lit> &selectors,
                              std::map<Var, std::pair<Var, Var>> &mapSelector,
                              std::vector<Gate> &gates,
                              std::vector<Var> &input) {
  // mark the unit literals.
  std::vector<bool> inOutput(p.getNbVar() + 1, false);
  std::vector<bool> inUnit(p.getNbVar() + 1, false);
  for (auto &g : gates) {
    inOutput[g.output.var()] = true;
    inUnit[g.output.var()] = g.type == UNIT;
  }

  // mark the projected variable
  std::vector<bool> isProjected(p.getNbVar() + 1, false);
  for (auto v : p.getProjectedVar()) isProjected[v] = true;

  // mark the projected variable
  m_isProtectedVar.clear();
  m_isProtectedVar.resize(p.getNbVar() + 1, false);
  for (auto v : p.getProtectedVar()) {
    m_isProtectedVar[v] = true;
    input.push_back(v);
  }

  // create the problem we will give to the solver.
  res.setNbVar(3 * p.getNbVar());

  // create the equivalence.
  std::vector<std::vector<Lit>> &clauses = res.getClauses();
  for (unsigned i = 1, j = p.getNbVar() + 1, k = 2 * p.getNbVar() + 1;
       i <= p.getNbVar(); i++, j++, k++) {
    if (inUnit[i] || (!m_useCore && inOutput[i]) || !isProjected[i]) continue;

    // -i v (j <-> k)
    if (!m_isProtectedVar[i]) {
      clauses.push_back(
          {Lit::makeLitFalse(i), Lit::makeLitFalse(j), Lit::makeLitTrue(k)});
      clauses.push_back(
          {Lit::makeLitFalse(i), Lit::makeLitTrue(j), Lit::makeLitFalse(k)});
    }

    if (inOutput[i]) continue;

    if (!m_isProtectedVar[i]) {
      selectors.push_back(Lit::makeLitTrue(i));
      mapSelector[i] = std::make_pair(j, k);
    }
  }

  // force to be equivalent
  if (m_useCore) {
    for (auto &g : gates) {
      if (g.type == UNIT) continue;
      std::vector<Lit> cl = {Lit::makeLitTrue(g.output.var())};
      for (auto &l : g.input)
        if (!m_isProtectedVar[l.var()])
          cl.push_back(Lit::makeLitFalse(l.var()));
      clauses.push_back(cl);
    }
  }

  // shift the initial formula.
  for (auto &cl : p.getClauses()) {
    clauses.push_back(cl);
    for (auto &l : clauses.back())
      l = Lit::makeLit(l.var() + p.getNbVar(), l.sign());
  }

  // shift the second formula.
  for (auto &cl : p.getClauses()) {
    clauses.push_back(cl);
    for (auto &l : clauses.back())
      if (m_isProtectedVar[l.var()])
        l = Lit::makeLit(l.var() + p.getNbVar(), l.sign());
      else
        l = Lit::makeLit(l.var() + 2 * p.getNbVar(), l.sign());
  }

  m_nbSATCall = m_nbUNDCall = m_nbUNSCall = 0;

  // create the heap variable set.
  std::vector<Var> &heapVariable = res.getHeapVariable();
  for (unsigned i = 1; i <= p.getNbVar(); i++)
    if (!inUnit[i]) heapVariable.push_back(i + p.getNbVar());

  for (unsigned i = 1; i <= p.getNbVar(); i++)
    if (!inUnit[i] && !m_isProtectedVar[i])
      heapVariable.push_back(i + (2 * p.getNbVar()));

  m_lastCall = getTimer();
  m_marked.clear();
  m_marked.resize(3 * (p.getNbVar() + 1), false);
}  // initProblem

/**
 * @brief Initialize the variable used to execute the model exploitation.
 *
 * @param p is the problem we are considering.
 */
void Bipartition::initInputModelVariable(Problem &p) {
  // create the occurrence list on the initial problem.
  m_occurrence.clear();
  m_occurrence.resize(2 * (p.getNbVar() + 1));
  for (unsigned i = 0; i < p.getClauses().size(); i++)
    for (auto &l : p.getClauses()[i]) m_occurrence[l.intern()].push_back(i);

  m_nbSatisfied.clear();
  m_nbSatisfied.resize(p.getClauses().size(), 0);

  m_falsified.clear();
  m_falsified.resize(p.getClauses().size(), 0);

  m_whereIsFalsified.clear();
  m_whereIsFalsified.resize(p.getClauses().size(), 0);

  m_breakcount.clear();
  m_breakcount.resize(p.getNbVar() + 1, 0);

  m_linkedVarTaut.clear();
  m_linkedVarTaut.resize(p.getNbVar() + 1);

  std::vector<bool> wasAdded(p.getNbVar() + 1, false);
  std::vector<bool> marked(2 * (p.getNbVar() + 1), false);
  for (auto v : p.getProjectedVar()) {
    Lit l = Lit::makeLitFalse(v);
    std::vector<Var> &current = m_linkedVarTaut[v];

    // check for tautology.
    for (auto idx : m_occurrence[l.intern()]) {
      std::vector<Lit> &cl = p.getClauses()[idx];
      for (auto m : cl) marked[m.intern()] = true;

      // check the resolvents.
      for (auto idx : m_occurrence[(~l).intern()]) {
        Lit tautLit = lit_Undef;
        for (auto m : p.getClauses()[idx])
          if (m != ~l && !wasAdded[m.var()] && marked[(~m).intern()]) {
            tautLit = m;
            break;
          }

        if (tautLit != lit_Undef) {
          wasAdded[tautLit.var()] = true;
          current.push_back(tautLit.var());
        }
      }

      for (auto m : cl) marked[m.intern()] = false;
    }

    // reinit for the next run.
    for (auto w : current) wasAdded[w] = false;
  }

  m_model.clear();
  m_model.resize(p.getNbVar() + 1);
  m_nbInputFromModel = 0;
  m_breakLit.clear();
  m_breakLit.resize(p.getClauses().size(), lit_Undef);
  m_makecount.clear();
  m_makecount.resize(p.getNbVar() + 1, 0);
}  // initInputModelVariable

/**
 * @brief Bipartition::flip implementation.
 */
void Bipartition::flip(Problem &p, Var v, std::vector<bool> &model) {
  assert(v < model.size());
  Lit l = Lit::makeLit(v, !model[v]);
  model[v] = 1 - model[v];

  for (auto idx : m_occurrence[l.intern()]) {
    m_nbSatisfied[idx]--;
    if (m_nbSatisfied[idx] == 0)  // UNSAT
    {
      m_breakcount[l.var()]--;
      for (auto l : p.getClauses()[idx]) m_makecount[l.var()]++;
      m_nbFalsified++;

      m_whereIsFalsified[idx] = m_falsified.size();
      m_falsified.push_back(idx);
    } else if (m_nbSatisfied[idx] == 1)  // update breakCount
    {
      // search for the break lit.
      Lit breakLit = lit_Undef;
      for (auto l : p.getClauses()[idx])
        if (l.sign() == !model[l.var()]) {
          breakLit = l;
          break;
        }

      assert(breakLit != lit_Undef);
      m_breakcount[breakLit.var()]++;
      m_breakLit[idx] = breakLit;
    }
  }

  for (auto idx : m_occurrence[(~l).intern()]) {
    m_nbSatisfied[idx]++;

    if (m_nbSatisfied[idx] == 1)  // new SAT
    {
      for (auto l : p.getClauses()[idx]) m_makecount[l.var()]--;
      m_nbFalsified--;
      m_breakcount[l.var()]++;
      m_breakLit[idx] = ~l;

      m_falsified[m_whereIsFalsified[idx]] = m_falsified.back();
      m_whereIsFalsified[m_falsified.back()] = m_whereIsFalsified[idx];
      m_falsified.pop_back();
    } else if (m_nbSatisfied[idx] == 2) {
      m_breakcount[m_breakLit[idx].var()]--;
    }
  }

  // debugBreakMakeCount(p, model);
}  // flip

/**
 * @brief Bipartition::debugMakeBreakCount implementation.
 */
void Bipartition::debugBreakMakeCount(Problem &p, std::vector<bool> &model) {
  for (unsigned i = 0; i < p.getClauses().size(); i++) {
    unsigned nbSat = 0;
    for (auto &l : p.getClauses()[i])
      if (l.sign() == !model[l.var()]) nbSat++;
    assert(m_nbSatisfied[i] == nbSat);
  }

  std::vector<unsigned> makeCount(p.getNbVar() + 1, 0);
  std::vector<unsigned> breakCount(p.getNbVar() + 1, 0);

  for (unsigned i = 0; i < p.getClauses().size(); i++) {
    if (m_nbSatisfied[i] == 0) {
      for (auto &l : p.getClauses()[i]) makeCount[l.var()]++;
    }

    [[maybe_unused]] bool isIn = false;
    for (auto &idx : m_falsified)
      if (idx == i) isIn = true;
    assert(isIn == (m_nbSatisfied[i] == 0));

    for (unsigned j = 0; j < m_falsified.size(); j++)
      assert(m_whereIsFalsified[m_falsified[j]] == j);

    if (m_nbSatisfied[i] == 1) {
      for (auto &l : p.getClauses()[i]) {
        if (l.sign() == !model[l.var()]) {
          breakCount[l.var()]++;
          break;
        }
      }
    }
  }

  for (unsigned i = 1; i <= p.getNbVar(); i++) {
    assert(m_makecount[i] == makeCount[i]);
    assert(m_breakcount[i] == breakCount[i]);
  }
}  // debugBreakMakeCount

/**
 * @brief Bipartition::initMakeBreakCount implementation.
 */
void Bipartition::initMakeBreakCount(Problem &p, std::vector<bool> &model) {
  m_nbFalsified = 0;  // sure because we have a model.
  for (auto &nbSat : m_nbSatisfied) nbSat = 0;
  m_falsified.clear();

  for (unsigned i = 1; i <= p.getNbVar(); i++) {
    m_makecount[i] = 0;
    m_breakcount[i] = 0;
    Lit l = Lit::makeLit(i, !model[i]);

    for (auto idx : m_occurrence[l.intern()]) {
      m_nbSatisfied[idx]++;
      if (m_nbSatisfied[idx] == 1) {
        m_breakcount[l.var()]++;
        m_breakLit[idx] = l;
      }

      if (m_nbSatisfied[idx] == 2) m_breakcount[m_breakLit[idx].var()]--;
    }
  }
}  // initMakeBreakCount

/**
 * @brief Bipartition::searchInputFromModel implementation.
 */
void Bipartition::searchInputFromModel(Problem &p, std::vector<Var> &input,
                                       std::vector<Lit> &assums,
                                       std::vector<bool> &model, Lit lastLit) {
  // init the make/break structures.
  initMakeBreakCount(p, model);

  // update the list of model.
  for (unsigned i = 0; i < assums.size(); i++)
    if (!m_breakcount[assums[i].var()]) {
      m_listOfModels.push_back(model);
    }

  // we know for sure that variable that when flipping do not change the
  // consistency of the model should be added in the input set.
  m_nbInputFromCompleteModel += moveToInput(
      assums, input, [this](Var v) -> bool { return !m_breakcount[v]; });

  // try to flip one not decided variable and then flip the already conputed
  // output variables.
  unsigned j = 0;
  for (unsigned i = 0; i < assums.size(); i++) {
    assert(!m_nbFalsified);
    Lit l = assums[i];
    flip(p, l.var(), model);

    // flip the output while we can improve.
    std::vector<Var> justFlip;

    // try to make the formula SAT
    bool inProgress = true;
    while (m_nbFalsified && inProgress) {
      std::vector<Lit> &cl = p.getClauses()[m_falsified[0]];
      inProgress = false;

      for (auto &l : cl) {
        if (!m_isInOutput[l.var()]) continue;
        if (m_makecount[l.var()] > m_breakcount[l.var()]) {
          inProgress = true;
          flip(p, l.var(), model);
          justFlip.push_back(l.var());
        }
      }
    }

    // can we add this variable in the output set?
    if (m_nbFalsified)
      assums[j++] = l;
    else {
      input.push_back(l.var());
      m_listOfModels.push_back(model);
    }

    // unflip.
    for (auto &v : justFlip) flip(p, v, model);
    flip(p, l.var(), model);
  }

  m_nbInputFromPadoaModel += assums.size() - j;
  assums.resize(j);
}  // searchInputFromModel

/**
 * @brief Set the solver to the priority set built from l.
 *
 * @param p is the problem we are considering.
 * @param solver is the solver.
 * @param l is the literal we want to prioritized.
 */
void Bipartition::setPriority(Problem &p, WrapperSolver *solver, Lit l) {
  m_priority.clear();
  if (m_linkedVarTaut[l.var()].size() < 10)
    for (auto v : m_linkedVarTaut[l.var()]) {
      m_priority.push_back(v + p.getNbVar());
      m_priority.push_back(v + (p.getNbVar() << 1));
    }
  solver->oncePriorityVar(m_priority);
}  // setPriority

/**
 * @brief Search for input by considering set of models given in parameter.
 *
 * @param p is the problem we are looking for bipartition.
 * @param setOfModels is a set of models of p.
 * @param[out] collectedInput is the set of input that has been collected.
 */
void Bipartition::exploitSetOfModelAsPreprocToSpotInput(
    Problem &p, std::vector<Gate> &gates,
    std::vector<std::vector<bool>> &setOfModels,
    std::vector<Var> &collectedInput) {
  if (setOfModels.size()) {
    std::vector<Lit> assums;
    std::vector<bool> isInAssum(p.getNbVar() + 1, false);
    for (auto v : p.getProjectedVar()) isInAssum[v] = true;
    for (auto v : p.getProtectedVar()) isInAssum[v] = false;
    for (auto &g : gates) isInAssum[g.output.var()] = false;

    for (auto &v : p.getProjectedVar())
      if (isInAssum[v]) assums.push_back(Lit::makeLitTrue(v));

    for (auto &model : setOfModels) {
      std::vector<Var> newInput;
      searchInputFromModel(p, newInput, assums, model, lit_Undef);

      m_nbInputFromModel += newInput.size();
      for (auto v : newInput) {
        assert(isInAssum[v]);
        isInAssum[v] = false;
        collectedInput.push_back(v);
      }

      unsigned j = 0;
      for (unsigned i = 0; i < assums.size(); i++)
        if (isInAssum[assums[i].var()]) assums[j++] = assums[i];
      assums.resize(j);
    }
  }
}  // exploitSetOfModelAsPreprocToSpotInput

/**
 * @brief Bipartition::generateOutputSym implementation.
 */
void Bipartition::generateOutputSym(
    WrapperSolver *solver, std::map<Var, std::pair<Var, Var>> &mapSelector,
    const std::vector<std::vector<Var>> &symGroup, std::vector<Var> &def,
    std::vector<Lit> &areUnit, std::vector<Var> &outFound) {
  std::vector<std::vector<Var>> listDef;
  listDef.push_back(def);

  while (listDef.size()) {
    def = listDef.back();
    listDef.pop_back();

    // generate sym def:
    for (auto &s : symGroup) {
      if (s[def[0]] != def[0] && !m_isInOutput[s[def[0]]]) {
        bool canBeAdded = true;
        for (auto &v : def)
          if (m_isInOutput[s[v]]) {
            canBeAdded = false;
            break;
          }

        if (canBeAdded) {
          m_isInOutput[s[def[0]]] = true;
          m_nbOutputFromSym++;

          std::vector<Var> addDef;
          for (auto &v : def) addDef.push_back(s[v]);
          listDef.push_back(addDef);

          if (!m_useCore)
            areUnit.push_back(Lit::makeLitFalse(s[def[0]]));
          else {
            std::vector<Lit> symCl;
            for (auto &v : def) symCl.push_back(Lit::makeLitFalse(v));
            symCl[0] = ~symCl[0];
            solver->addClauseInit(symCl);
          }

          outFound.push_back(s[def[0]]);
        }
      }
    }
  }
}  // generateOutputSym

/**
 * @brief Bipartition::tryToConnectSavedModels implementation.
 */
void Bipartition::tryToConnectSavedModels(Problem &p, std::vector<Var> &input,
                                          std::vector<Lit> &assums) {
  for (auto &l : assums) m_marked[l.var()] = true;

  for (unsigned i = 0; i < m_listOfModels.size(); i++) {
    std::vector<bool> &m1 = m_listOfModels[i];
    for (unsigned j = i + 1; j < m_listOfModels.size(); j++) {
      std::vector<bool> &m2 = m_listOfModels[j];

      Var saveV = var_Undef;
      unsigned cptDiff = 0;
      for (unsigned v = 1; cptDiff < 2 && v <= p.getNbVar(); v++) {
        if (m_isInOutput[v]) continue;
        if (m1[v] != m2[v]) {
          if (!m_marked[v])
            cptDiff = 10;
          else {
            saveV = v;
            cptDiff++;
          }
        }
      }

      if (cptDiff == 1) {
        m_marked[saveV] = false;
        input.push_back(saveV);
        std::cout << "we detect one\n";
      }
    }
  }

  unsigned j = 0;
  for (unsigned i = 0; i < assums.size(); i++) {
    if (m_marked[assums[i].var()]) {
      assums[j++] = assums[i];
      m_marked[assums[i].var()] = false;
    }
  }
  m_nbInputFromModel += assums.size() - j;
  assums.resize(j);
}  // tryToConnectSavedModels

/**
 * @brief Bipartition::compute implementation.
 */
void Bipartition::compute(Problem &p, WrapperSolver *solver,
                          std::vector<Var> &input, std::vector<Lit> &und,
                          HeuristicBipartition *heuristic,
                          std::map<Var, std::pair<Var, Var>> &mapSelector,
                          const int nbConflict,
                          const std::vector<std::vector<Var>> &symGroup,
                          std::ostream &out, bool verbose) {
  out << "c \033[1m\033[31mBipartition in progress\033[0m\n";
  std::vector<Lit> areUnit;
  std::vector<Var> newInput;

  // compute the partition.
  while (!m_isInterrupted && !heuristic->isEmpty()) {
    Lit next = heuristic->nextAssumption();

    std::vector<Lit> assums = und;
    for (auto &l : heuristic->remainingAssumptions()) assums.push_back(l);

    solver->setAssumption(assums);
    assert(!solver->varIsAssigned(next.var()));
    assert(!solver->isInAssumption(next.var()));

    // search a model where the values of the selected variable are different.
    solver->pushAssumption(Lit::makeLitTrue(mapSelector[next.var()].first));
    solver->pushAssumption(Lit::makeLitFalse(mapSelector[next.var()].second));
    setPriority(p, solver, next);

    Status status = solver->solveLimited(nbConflict);
    bool isDefined = (status == Status::UNS);
    solver->popAssumption(2);

    if (!isDefined) {
      // input case:
      if (status == Status::UND)
        und.push_back(next);
      else
        input.push_back(next.var());

      solver->restart();
      if (status == Status::SAT && !solver->varIsAssigned(next.var()))
        solver->uncheckedEnqueue(next);

      if (m_inputModel && status == Status::SAT) {
        newInput.resize(0);
        std::vector<bool> &currentModel = solver->getModel();

        // first model.
        for (unsigned i = 1; i <= p.getNbVar(); i++)
          m_model[i] = currentModel[i + p.getNbVar()];
        searchInputFromModel(p, newInput, heuristic->remainingAssumptions(),
                             m_model, next);

        // second model.
        for (unsigned i = 1; i <= p.getNbVar(); i++)
          if (!m_isProtectedVar[i])
            m_model[i] = currentModel[i + (p.getNbVar() << 1)];
        searchInputFromModel(p, newInput, heuristic->remainingAssumptions(),
                             m_model, ~next);

        // manage the compute input and update the assumption accordingly.
        if (newInput.size()) {
          for (auto v : newInput) {
            areUnit.push_back(Lit::makeLitTrue(v));
            input.push_back(v);
            m_nbInputFromModel++;
          }
          solver->cleanAssumption();
          solver->setAssumption(heuristic->remainingAssumptions());
        }
      }

      for (auto m : areUnit)
        if (!solver->varIsAssigned(m.var())) solver->uncheckedEnqueue(m);
    } else {
      // save that next is an output.
      assert(next.var() < m_isInOutput.size());
      m_isInOutput[next.var()] = true;

      // output case: UNSAT
      if (!m_useCore && !symGroup.size())
        areUnit.push_back(~next);
      else {
        assert(status == Status::UNS);
        std::vector<Lit> core;
        solver->getCore(core);

        // remove diff value.
        int nb = 0;
        for (unsigned i = 0; nb < 2 && i < core.size();)
          if (core[i].var() != mapSelector[next.var()].first &&
              core[i].var() != mapSelector[next.var()].second)
            i++;
          else {
            core[i] = core.back();
            core.pop_back();
            nb++;
          }

        core.push_back(next);
        core[core.size() - 1] = core[0];
        core[0] = next;

        if (m_useCore) solver->addClauseInit(core);
        if (symGroup.size()) {
          std::vector<Var> def, outFound;
          for (auto &l : core) def.push_back(l.var());
          generateOutputSym(solver, mapSelector, symGroup, def, areUnit,
                            outFound);

          for (auto &v : outFound) heuristic->popVarFromAssumption(v);
          solver->cleanAssumption();
          solver->setAssumption(heuristic->remainingAssumptions());
        }
      }
    }

    updateInfo(status);
    if (verbose) printInfo(heuristic, out);
  }

  if (m_isInterrupted) {
    out << "c [BIPARTITION] Stop by signal, number of remaining undefined "
           "variables: "
        << heuristic->remainingAssumptions().size() << "\n";
    for (auto &l : heuristic->remainingAssumptions()) input.push_back(l.var());
  }
}  // compute

/**
 * @brief Bipartition::run implementation.
 */
bool Bipartition::run(Problem &p, std::vector<Var> &input,
                      std::vector<Gate> &gates,
                      const OptionBipartition &optionBipartition,
                      const std::vector<std::vector<Var>> &symGroup,
                      std::vector<std::vector<bool>> &setOfModels,
                      std::ostream &out) {
  out << "c [BIPARTITION] #Projected: " << p.getProjectedVar().size() << "\n";
  out << "c [BIPARTITION] #Protected: " << p.getProtectedVar().size() << "\n";
  out << "c [BIPARTITION] #Gates: " << gates.size() << "\n";

  m_nbVar = p.getNbVar();
  initTimer();
  m_inputModel = optionBipartition.useModel;
  m_useCore = optionBipartition.useCore;

  if (m_isInterrupted) {
    out << "c [BIPARTITION] Bipartition stopped early\n";
    std::vector<Lit> units;
    for (auto &g : gates)
      if (g.type == UNIT) units.push_back(g.output);
    constructInputFromUnits(p, units, input);
    return true;
  }

  // mark the variable that are output.
  m_isInOutput.clear();
  m_isInOutput.resize(p.getNbVar() + 1, false);
  for (auto &g : gates) m_isInOutput[g.output.var()] = true;

  if (symGroup.size())
    out << "c [BIPARTITION] The number of group of symmetries is: "
        << symGroup.size() << "\n";

  // Create the CNF formula and get the assumptions.
  std::vector<Lit> selectors, assums;
  std::map<Var, std::pair<Var, Var>> mapSelector;
  initInputModelVariable(p);

  if (m_inputModel) {
    m_listOfModels = setOfModels;
    std::vector<Var> collectInput;
    exploitSetOfModelAsPreprocToSpotInput(p, gates, setOfModels, collectInput);

    std::vector<Var> &protectedInFormula = p.getProtectedVar();
    for (auto v : collectInput) protectedInFormula.push_back(v);
    out << "c [BIPARTITION] #Model considered for preproc with models: "
        << setOfModels.size() << "\n";
    out << "c [BIPARTITION] #Input variables from models as preproc: "
        << m_nbInputFromModel << "\n";
  }

  Problem ptmp;
  initProblem(p, ptmp, selectors, mapSelector, gates, input);
  out << "c [BIPARTITION] Initial input set: " << selectors.size() << "\n";

  // Init the SAT solver.
  m_solver =
      WrapperSolver::makeWrapperSolver(optionBipartition.solverName, std::cout);
  m_solver->initSolver(ptmp);
  m_solver->setIncrementalMode(true);
  if (m_inputModel) m_solver->setNeedModel(true);
  m_solver->setLastIndexAssumption(p.getNbVar() + 1);
  m_solver->setHeapVariable(ptmp.getHeapVariable());

  // sort the assumption regarding the heuristic.
  HeuristicBipartition *heuristic =
      HeuristicBipartition::makeHeuristicBipartition(
          p, selectors, optionBipartition.heuristicSorting, out);

  // compute the partition.
  std::vector<Lit> und;
  unsigned runNumber = 1;
  while (true) {
    std::cout << "c Run for " << runNumber * optionBipartition.solverNbConflict
              << " conflicts\n";
    und.resize(0);
    compute(p, m_solver, input, und, heuristic, mapSelector,
            runNumber * optionBipartition.solverNbConflict, symGroup, out,
            optionBipartition.verbose);
    heuristic->setAssumption(und);
    m_solver->cleanAssumption();
    runNumber *= 2;
    if (!und.size()) break;
  }

  assert(m_nbInputFromCompleteModel + m_nbInputFromPadoaModel ==
         m_nbInputFromModel);
  assert(m_nbInputFromModel <= input.size());

  // manage the result.
  out << "c [BIPARTITION] \033[1m\033[31mStatistics \033[0m\n";

  out << "c [BIPARTITION] #Input variables get from complete model: "
      << m_nbInputFromCompleteModel << "\n";
  out << "c [BIPARTITION] #Input variables get from Padoa model: "
      << m_nbInputFromPadoaModel << "\n";
  out << "c [BIPARTITION] #Input variables get from model: "
      << m_nbInputFromModel << "\n";

  out << "c [BIPARTITION] #Output variables get from symmetries: "
      << m_nbOutputFromSym << "\n";

  out << "c [BIPARTITION] #Input variables computed: " << input.size() << "\n";
  out << "c [BIPARTITION] #Input variables that are undertermined: "
      << und.size() << "\n";

  out << "c [BIPARTITION] Time needed to compute the partition: " << getTimer()
      << "\n";

  // clean.
  delete heuristic;
  return true;
}  // run

}  // namespace bipartition
}  // namespace bipe