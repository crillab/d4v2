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

#include <boost/program_options.hpp>
#include <ctime>
#include <iomanip>
#include <iostream>

#include "Counter.hpp"
#include "DataBranch.hpp"
#include "MethodManager.hpp"
#include "src/caching/CacheManager.hpp"
#include "src/caching/CachedBucket.hpp"
#include "src/caching/TmpEntry.hpp"
#include "src/heuristics/PartitioningHeuristic.hpp"
#include "src/heuristics/PhaseHeuristic.hpp"
#include "src/heuristics/ScoringMethod.hpp"
#include "src/preprocs/PreprocManager.hpp"
#include "src/problem/ProblemManager.hpp"
#include "src/problem/ProblemTypes.hpp"
#include "src/solvers/WrapperSolver.hpp"
#include "src/specs/SpecManager.hpp"
#include "src/utils/MemoryStat.hpp"

#define NB_SEP_MC 104
#define MASK_SHOWRUN_MC ((2 << 13) - 1)
#define WIDTH_PRINT_COLUMN_MC 12
#define MASK_HEADER 1048575

#include "CountingOperation.hpp"
#include "DecisionDNNFOperation.hpp"
#include "OperationManager.hpp"

namespace d4 {
namespace po = boost::program_options;
template <class T>
class Counter;

template <class T, class U>
class DpllStyleMethod : public MethodManager, public Counter<T> {
 private:
  bool optDomConst;
  bool optReversePolarity;

  unsigned m_nbCallCall;
  unsigned m_nbSplit;
  unsigned m_callPartitioner;
  unsigned m_nbDecisionNode;
  unsigned m_optCached;
  unsigned m_stampIdx;
  bool m_isProjectedMode;

  std::vector<unsigned> m_stampVar;
  std::vector<std::vector<Lit>> m_clauses;

  std::vector<unsigned long> nbTestCacheVarSize;
  std::vector<unsigned long> nbPosHitCacheVarSize;
  std::vector<bool> m_isDecisionVariable;

  std::vector<bool> m_currentPrioritySet;

  ProblemManager *m_problem;
  WrapperSolver *m_solver;
  SpecManager *m_specs;
  ScoringMethod *m_hVar;
  PhaseHeuristic *m_hPhase;
  PartitioningHeuristic *m_hCutSet;
  TmpEntry<U> NULL_CACHE_ENTRY;
  CacheManager<U> *m_cache;

  std::ostream m_out;
  bool m_panicMode;

  Operation<T, U> *m_operation;

 public:
  /**
     Constructor.

     @param[in] vm, the list of options.
   */
  DpllStyleMethod(po::variables_map &vm, std::string &meth, bool isFloat,
                  ProblemManager *initProblem, std::ostream &out,
                  LastBreathPreproc &lastBreath)
      : m_problem(initProblem), m_out(nullptr) {
    // init the output stream
    m_out.copyfmt(out);
    m_out.clear(out.rdstate());
    m_out.basic_ios<char>::rdbuf(out.rdbuf());

    // we create the SAT solver.
    m_solver = WrapperSolver::makeWrapperSolver(vm, m_out);
    assert(m_solver);
    m_panicMode = lastBreath.panic;

    m_solver->initSolver(*m_problem);
    m_solver->setCountConflict(lastBreath.countConflict, 1,
                               m_problem->getNbVar());
    m_solver->setNeedModel(true);

    // we initialize the object that will give info about the problem.
    m_specs = SpecManager::makeSpecManager(vm, *m_problem, m_out);
    assert(m_specs);

    // we initialize the object used to compute score and partition.
    m_hVar = ScoringMethod::makeScoringMethod(vm, *m_specs, *m_solver, m_out);
    m_hPhase =
        PhaseHeuristic::makePhaseHeuristic(vm, *m_specs, *m_solver, m_out);

    // specify which variables are decisions, and which are not.
    m_isDecisionVariable.clear();
    m_isDecisionVariable.resize(m_problem->getNbVar() + 1,
                                !m_problem->getNbSelectedVar());
    for (auto v : m_problem->getSelectedVar()) m_isDecisionVariable[v] = true;
    m_currentPrioritySet.resize(m_problem->getNbVar() + 1, false);

    // select the partitioner regarding if it projected model counting or not.
    if ((m_isProjectedMode = m_problem->getNbSelectedVar())) {
      m_out << "c [MODE] projected\n";
      m_hCutSet = PartitioningHeuristic::makePartitioningHeuristicNone(m_out);
    } else {
      m_out << "c [MODE] classic\n";
      m_hCutSet = PartitioningHeuristic::makePartitioningHeuristic(
          vm, *m_specs, *m_solver, m_out);
    }

    assert(m_hVar && m_hPhase && m_hCutSet);
    m_cache = CacheManager<U>::makeCacheManager(vm, m_problem->getNbVar(),
                                                m_specs, m_out);

    // init the clock time.
    initTimer();

    m_optCached = vm["cache-activated"].as<bool>();
    m_callPartitioner = 0;
    m_nbDecisionNode = m_nbSplit = m_nbCallCall = 0;
    m_stampIdx = 0;
    m_stampVar.resize(m_specs->getNbVariable() + 1, 0);
    nbTestCacheVarSize.resize(m_specs->getNbVariable() + 1, 0);
    nbPosHitCacheVarSize.resize(m_specs->getNbVariable() + 1, 0);

    void *op = Operation<T, U>::makeOperationManager(meth, isFloat, m_problem,
                                                     m_specs, m_solver, m_out);
    m_operation = static_cast<Operation<T, U> *>(op);
    m_out << "c\n";
  }  // constructor

  /**
     Destructor.
   */
  ~DpllStyleMethod() {
    delete m_operation;
    delete m_problem;
    delete m_solver;
    delete m_specs;
    delete m_hVar;
    delete m_hPhase;
    delete m_hCutSet;
    delete m_cache;
  }  // destructor

 private:
  /**
     Expel from a set of variables the ones they are marked as being decidable.

     @param[out] vars, the set of variables we search to filter.

     @param[in] isDecisionvariable, a boolean vector that marks as true decision
     variables.
   */
  void expelNoDecisionVar(std::vector<Var> &vars,
                          std::vector<bool> &isDecisionVariable) {
    if (!m_isProjectedMode) return;

    unsigned j = 0;
    for (unsigned i = 0; i < vars.size(); i++)
      if (isDecisionVariable[vars[i]]) vars[j++] = vars[i];
    vars.resize(j);
  }  // expelNoDecisionVar

  /**
     Expel from a set of variables the ones they are marked as being decidable.

     @param[out] lits, the set of literals we search to filter.

     @param[in] isDecisionvariable, a boolean vector that marks as true decision
     variables.
   */
  void expelNoDecisionLit(std::vector<Lit> &lits,
                          std::vector<bool> &isDecisionVariable) {
    if (!m_isProjectedMode) return;

    unsigned j = 0;
    for (unsigned i = 0; i < lits.size(); i++)
      if (isDecisionVariable[lits[i].var()]) lits[j++] = lits[i];
    lits.resize(j);
  }  // expelNoDecisionLit

  /**
     Compute the current priority set.

     @param[in] connected, the current component.
     @param[in] priorityVar, the current priority variables.
     @param[out] currPriority, the intersection of the two previous sets.
  */
  inline void computePrioritySubSet(std::vector<Var> &connected,
                                    std::vector<Var> &priorityVar,
                                    std::vector<Var> &currPriority) {
    currPriority.resize(0);
    m_stampIdx++;
    for (auto &v : connected) m_stampVar[v] = m_stampIdx;
    for (auto &v : priorityVar)
      if (m_stampVar[v] == m_stampIdx && !m_specs->varIsAssigned(v))
        currPriority.push_back(v);
  }  // computePrioritySet

  /**
     Print out information about the solving process.

     @param[in] out, the stream we use to print out information.
  */
  inline void showInter(std::ostream &out) {
    out << "c " << std::fixed << std::setprecision(2) << "|"
        << std::setw(WIDTH_PRINT_COLUMN_MC) << getTimer() << "|"
        << std::setw(WIDTH_PRINT_COLUMN_MC) << m_cache->getNbPositiveHit()
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC)
        << m_cache->getNbNegativeHit() << "|"
        << std::setw(WIDTH_PRINT_COLUMN_MC) << m_cache->usedMemory() << "|"
        << std::setw(WIDTH_PRINT_COLUMN_MC) << m_nbSplit << "|"
        << std::setw(WIDTH_PRINT_COLUMN_MC) << MemoryStat::memUsedPeak() << "|"
        << std::setw(WIDTH_PRINT_COLUMN_MC) << m_nbDecisionNode << "|"
        << std::setw(WIDTH_PRINT_COLUMN_MC) << m_callPartitioner << "|\n";
  }  // showInter

  /**
     Print out a line of dashes.

     @param[in] out, the stream we use to print out information.
   */
  inline void separator(std::ostream &out) {
    out << "c ";
    for (int i = 0; i < NB_SEP_MC; i++) out << "-";
    out << "\n";
  }  // separator

  /**
     Print out the header information.

     @param[in] out, the stream we use to print out information.
  */
  inline void showHeader(std::ostream &out) {
    separator(out);
    out << "c "
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "time"
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "#posHit"
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "#negHit"
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "memory"
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "#split"
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "mem(MB)"
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "#dec. Node"
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "#cutter"
        << "|\n";
    separator(out);
  }  // showHeader

  /**
     Print out information when it is requiered.

     @param[in] out, the stream we use to print out information.
   */
  inline void showRun(std::ostream &out) {
    if (!(m_nbCallCall & (MASK_HEADER))) showHeader(out);
    if (m_nbCallCall && !(m_nbCallCall & MASK_SHOWRUN_MC)) showInter(out);
  }  // showRun

  /**
     Print out the final stat.

     @param[in] out, the stream we use to print out information.
   */
  inline void printFinalStats(std::ostream &out) {
    separator(out);
    out << "c\n";
    out << "c \033[1m\033[31mStatistics \033[0m\n";
    out << "c \033[33mCompilation Information\033[0m\n";
    out << "c Number of recursive call: " << m_nbCallCall << "\n";
    out << "c Number of split formula: " << m_nbSplit << "\n";
    out << "c Number of decision: " << m_nbDecisionNode << "\n";
    out << "c Number of paritioner calls: " << m_callPartitioner << "\n";
    out << "c\n";
    m_cache->printCacheInformation(out);
    if (m_hCutSet) {
      out << "c\n";
      m_hCutSet->displayStat(out);
    }
    out << "c Final time: " << getTimer() << "\n";
    out << "c\n";
  }  // printFinalStat

  /**
     Initialize the assumption in order to compute compiled formula under this
     one.

     @param[in] assums, the assumption
  */
  inline void initAssumption(std::vector<Lit> &assums) {
    m_solver->restart();
    m_solver->popAssumption(m_solver->getAssumption().size());
    m_solver->setAssumption(assums);
  }  // initAssumption

  /**
     Decide if the cache is realized or not.
   */
  bool cacheIsActivated(std::vector<Var> &connected) {
    if (!m_optCached) return false;
    return m_cache->isActivated(connected.size());
  }  // cacheIsActivated

  /**
     Call the CNF formula into a FBDD.

     @param[in] setOfVar, the current set of considered variables
     @param[in] unitsLit, the set of unit literal detected at this level
     @param[in] freeVariable, the variables which become free
     @param[in] out, the stream we use to print out information.

     \return an element of type U that sums up the given CNF sub-formula using a
     DPLL style algorithm with an operation manager.
  */
  U compute_(std::vector<Var> &setOfVar, std::vector<Lit> &unitsLit,
             std::vector<Var> &freeVariable, std::ostream &out) {
    showRun(out);
    m_nbCallCall++;

    if (!m_solver->solve(setOfVar)) return m_operation->manageBottom();

    m_solver->whichAreUnits(setOfVar, unitsLit);  // collect unit literals
    m_specs->preUpdate(unitsLit);

    // compute the connected composant
    std::vector<std::vector<Var>> varConnected;
    int nbComponent = m_specs->computeConnectedComponent(varConnected, setOfVar,
                                                         freeVariable);
    expelNoDecisionVar(freeVariable, m_isDecisionVariable);

    // consider each connected component.
    if (nbComponent) {
      U tab[nbComponent];
      m_nbSplit += (nbComponent > 1) ? nbComponent : 0;
      for (int cp = 0; cp < nbComponent; cp++) {
        std::vector<Var> &connected = varConnected[cp];
        bool cacheActivated = cacheIsActivated(connected);

        TmpEntry<U> cb = cacheActivated ? m_cache->searchInCache(connected)
                                        : NULL_CACHE_ENTRY;

        if (cacheActivated) nbTestCacheVarSize[connected.size()]++;
        if (cacheActivated && cb.defined) {
          nbPosHitCacheVarSize[connected.size()]++;
          tab[cp] = cb.getValue();
        } else {
          // recursive call
          tab[cp] = computeDecisionNode(connected, out);

          if (cacheActivated) m_cache->addInCache(cb, tab[cp]);
        }
      }

      m_specs->postUpdate(unitsLit);
      return m_operation->manageDecomposableAnd(tab, nbComponent);
    }  // else we have a tautology

    m_specs->postUpdate(unitsLit);
    expelNoDecisionLit(unitsLit, m_isDecisionVariable);

    return m_operation->createTop();
  }  // compute_

  /**
   * @brief Set the Current Priority.
   *
   * @param cutSet is the set of variables that become decision variables.
   */
  inline void setCurrentPriority(std::vector<Var> &cutSet) {
    for (auto &v : cutSet)
      if (m_isDecisionVariable[v]) m_currentPrioritySet[v] = true;
  }  // setCurrentPriority

  /**
   * @brief Unset the Current Priority.
   *
   * @param cutSet is the set of variables that become decision variables.
   */
  inline void unsetCurrentPriority(std::vector<Var> &cutSet) {
    for (auto &v : cutSet)
      if (m_isDecisionVariable[v]) m_currentPrioritySet[v] = false;
  }  // setCurrentPriority

  /**
     This function select a variable and compile a decision node.

     @param[in] connected, the set of variable present in the current problem.
     @param[in] out, the stream whare are printed out the logs.

     \return the compiled formula.
  */
  U computeDecisionNode(std::vector<Var> &connected, std::ostream &out) {
    std::vector<Var> cutSet;
    bool hasPriority = false, hasVariable = false;
    for (auto v : connected) {
      if (m_specs->varIsAssigned(v) || !m_isDecisionVariable[v]) continue;
      hasVariable = true;
      if ((hasPriority = m_currentPrioritySet[v])) break;
    }

    if (hasVariable && !hasPriority && m_hCutSet->isReady(connected)) {
      m_hCutSet->computeCutSet(connected, cutSet);
      m_callPartitioner++;
      setCurrentPriority(cutSet);
    }

    // search the next variable to branch on
    Var v = m_hVar->selectVariable(connected, *m_specs, m_currentPrioritySet);
    if (v == var_Undef) {
      unsetCurrentPriority(cutSet);
      return m_operation->manageTop(connected);
    }

    Lit l = Lit::makeLit(v, m_hPhase->selectPhase(v));
    m_nbDecisionNode++;

    // compile the formula where l is assigned to true
    DataBranch<U> b[2];

    assert(!m_solver->isInAssumption(l.var()));
    m_solver->pushAssumption(l);
    b[0].d = compute_(connected, b[0].unitLits, b[0].freeVars, out);
    m_solver->popAssumption();

    if (m_solver->isInAssumption(l))
      b[1].d = m_operation->manageBottom();
    else if (m_solver->isInAssumption(~l))
      b[1].d = compute_(connected, b[1].unitLits, b[1].freeVars, out);
    else {
      m_solver->pushAssumption(~l);
      b[1].d = compute_(connected, b[1].unitLits, b[1].freeVars, out);
      m_solver->popAssumption();
    }

    unsetCurrentPriority(cutSet);
    return m_operation->manageDeterministOr(b, 2);
  }  // computeDecisionNode

  /**
     Compute U using the trace of a SAT solver.

     @param[in] setOfVar, the set of variables of the considered problem.
     @param[in] out, the stream are is print out the logs.
     @param[in] warmStart, to activate/deactivate the warm start strategy.
     /!\ When the warm start is activated the assumptions are reset.

     \return an element of type U that sums up the given CNF formula using a
     DPLL style algorithm with an operation manager.
  */
  U compute(std::vector<Var> &setOfVar, std::ostream &out,
            bool warmStart = true) {
    if (m_problem->isUnsat() || (warmStart && !m_panicMode &&
                                 !m_solver->warmStart(29, 11, setOfVar, m_out)))
      return m_operation->manageBottom();

    DataBranch<U> b;
    b.d = compute_(setOfVar, b.unitLits, b.freeVars, out);
    return m_operation->manageBranch(b);
  }  // compute

 public:
  /**
     Given an assumption, we compute the number of models.  That is different
     from the query strategy, where we first compute and then condition the
     computed structure.

     @param[in] setOfVar, the set of variables of the considered problem.
     @param[in] assumption, the set of literals we want to assign.
     @param[in] out, the stream where are print out the log.

     \return the number of models when the formula is simplified by the given
     assumption.
   */
  T count(std::vector<Var> &setOfVar, std::vector<Lit> &assumption,
          std::ostream &out) {
    initAssumption(assumption);

    // get the unit not in setOfVar.
    std::vector<Lit> shadowUnits;
    m_stampIdx++;
    for (auto &v : setOfVar) m_stampVar[v] = m_stampIdx;
    for (auto &l : assumption)
      if (m_stampVar[l.var()] != m_stampIdx) shadowUnits.push_back(l);

    m_specs->preUpdate(shadowUnits);
    U result = compute(setOfVar, out, false);
    m_specs->postUpdate(shadowUnits);

    return m_operation->count(result);
  }  // count

  /**
     Run the DPLL style algorithm with the operation manager.

     @param[in] vm, the set of options.
   */
  void run(po::variables_map &vm) {
    std::vector<Var> setOfVar;
    for (int i = 1; i <= m_specs->getNbVariable(); i++) setOfVar.push_back(i);

    U result = compute(setOfVar, m_out);
    printFinalStats(m_out);
    m_operation->manageResult(result, vm, m_out);
  }  // run
};
}  // namespace d4
