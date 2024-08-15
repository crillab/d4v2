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

#include <ctime>
#include <iomanip>
#include <iostream>

#include "Counter.hpp"
#include "DataBranch.hpp"
#include "MethodManager.hpp"
#include "src/caching/CacheManager.hpp"
#include "src/caching/CachedBucket.hpp"
#include "src/caching/TmpEntry.hpp"
#include "src/heuristics/partitioning/PartitioningHeuristic.hpp"
#include "src/heuristics/phaseSelection/PhaseHeuristic.hpp"
#include "src/heuristics/scoringVariable/ScoringMethod.hpp"
#include "src/options/cache/OptionCacheManager.hpp"
#include "src/options/methods/OptionDpllStyleMethod.hpp"
#include "src/options/solvers/OptionSolver.hpp"
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
template <class T>
class Counter;

struct GlobalCacheInfo {
  std::string name;
};

struct QueryInfo {
  unsigned id;
};

class GlobalCacheManager {
  GlobalCacheInfo m_info;
  unsigned m_id = 0;

 public:
  GlobalCacheManager() {}

  /**
   * @brief Create the object that will be used in order to manage the
   * communication with the global cache server.
   *
   * @param info
   */
  GlobalCacheManager(const GlobalCacheInfo &info, ProblemManager *problem)
      : m_info(info) {
    std::cout << "Init the communication with the global cache server: "
              << info.name << "\n";

    ProblemManagerCnf *cnf = static_cast<ProblemManagerCnf *>(problem);

    std::cout << "p cnf " << cnf->getNbVar() << " " << cnf->getClauses().size()
              << " ";

    for (auto &cl : cnf->getClauses()) {
      for (auto &l : cl) std::cout << l.human() << " ";
      std::cout << "0 ";
    }
    std::cout << "\n";
  }

  /**
   * @brief Stop the communication with the global cache server.
   *
   */
  ~GlobalCacheManager() {
    std::cout << "Stop the communication with the global cache server: "
              << m_info.name << "\n";
  }

  /**
   * @brief Communicate the unit literals to the server.
   *
   * @param unit is the set of units.
   * @return true if everything is okay, false otherwise.
   */
  bool pushUnits(const std::vector<Lit> &unit) {
    std::cout << "push ";
    for (auto &l : unit) std::cout << l.human() << " ";
    std::cout << "0";
    std::cout << "\n";
    return true;
  }

  /**
   * @brief Communicate with the server to pop some unit literals.
   *
   * @param unit is the set of units.
   * @return true if everything is okay, false otherwise.
   */
  bool popUnits(const std::vector<Lit> &unit) {
    std::cout << "pop ";
    for (auto &l : unit) std::cout << l.human() << " ";
    std::cout << "0";
    std::cout << "\n";
    return true;
  }

  /**
   * @brief Ask for stopping the search for a given query.
   *
   * @param query gives the query information.
   */
  void stopQuery(const QueryInfo &query) {
    std::cout << "stop " << query.id;
    std::cout << "\n";
  }  // stopQuery

  /**
   * @brief Query the server about the current formula idenfied by a given
   * component.
   *
   * @param component is the set of variables we are interested in (it's enough
   * to send one variable normally ...).
   *
   * @return some information about the query.
   */
  QueryInfo query(const std::vector<Var> &component) {
    std::cout << "query ";
    for (auto &v : component) std::cout << v << " ";
    std::cout << "0\n";
    return {++m_id};
  }  // query

  /**
   * @brief Wait for the answer to a query (this query should be answered
   * possitively).
   *
   * @param query gives the query information.
   */
  mpz::mpz_int askCount(const QueryInfo &query) {
    std::cout << "ask " << query.id << "\n";
    std::cout << "get the answer from the server ...\n";
    return 1;
  }  // askCount

  /**
   * @brief Ask to the server if a subproblem has been found as being in the
   * global cache.
   *
   * @param[out] info is filled with the information requiered to spot the
   * query.
   *
   * @return true if a positive hit occurs, false otherwise.
   */
  bool check(QueryInfo &info) {
    std::cout << "check\n";
    return false;
  }  // check
};

class CountingGlobalCache : public MethodManager, public Counter<mpz::mpz_int> {
 private:
  bool optDomConst;
  bool optReversePolarity;

  unsigned m_nbCallCall;
  unsigned m_nbSplit;
  unsigned m_callPartitioner;
  unsigned m_nbDecisionNode;
  unsigned m_optCached;
  unsigned m_stampIdx;
  unsigned m_freqDecay;
  bool m_isProjectedMode;

  std::vector<unsigned> m_stampVar;
  std::vector<std::vector<Lit>> m_clauses;
  std::vector<bool> m_isDecisionVariable;

  std::vector<bool> m_currentPrioritySet;

  ProblemManager *m_problem;
  WrapperSolver *m_solver;
  SpecManager *m_specs;
  ScoringMethod *m_hVar;
  PhaseHeuristic *m_hPhase;
  PartitioningHeuristic *m_hCutSet;
  TmpEntry<mpz::mpz_int> NULL_CACHE_ENTRY;
  CacheManager<mpz::mpz_int> *m_cache;

  std::ostream m_out;

  GlobalCacheManager m_globalCacheManager;
  QueryInfo m_currentQuery;

 public:
  /**
     Constructor.

     @param[in] vm, the list of options.
   */
  CountingGlobalCache(const OptionDpllStyleMethod &options,
                      ProblemManager *initProblem, std::ostream &out)
      : m_problem(initProblem), m_out(nullptr) {
    // init the output stream
    m_out.copyfmt(out);
    m_out.clear(out.rdstate());
    m_out.basic_ios<char>::rdbuf(out.rdbuf());
    m_out.setstate(out.rdstate());

    m_out << "c [COUNTING GLOBAL CACHE METHOD]" << options << "\n";
    m_freqDecay = options.optionBranchingHeuristic.freqDecay;

    // we create and init the SAT solver.
    m_solver = WrapperSolver::makeWrapperSolver(options.optionSolver, m_out);
    m_solver->initSolver(*m_problem);
    m_solver->setNeedModel(true);

    // we initialize the object that will give info about the problem.
    m_specs = SpecManager::makeSpecManager(options.optionSpecManager,
                                           *m_problem, m_out);

    // we initialize the object used to compute score and partition.
    m_hVar = ScoringMethod::makeScoringMethod(options.optionBranchingHeuristic,
                                              *m_specs, *m_solver, m_out);
    m_hPhase = PhaseHeuristic::makePhaseHeuristic(
        options.optionBranchingHeuristic, *m_specs, *m_solver, m_out);

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
          options.optionPartitioningHeuristic, *m_specs, *m_solver, m_out);
    }

    assert(m_hVar && m_hPhase && m_hCutSet);

    m_cache = CacheManager<mpz::mpz_int>::makeCacheManager(
        options.optionCacheManager, m_problem->getNbVar(), m_specs, m_out);

    // init the clock time.
    initTimer();

    m_optCached = options.optionCacheManager.isActivated;
    m_callPartitioner = 0;
    m_nbDecisionNode = m_nbSplit = m_nbCallCall = 0;
    m_stampIdx = 0;
    m_stampVar.resize(m_specs->getNbVariable() + 1, 0);

    m_out << "c\n";

    GlobalCacheInfo info = {"@"};
    m_globalCacheManager = GlobalCacheManager(info, m_problem);
  }  // constructor

  /**
     Destructor.
   */
  ~CountingGlobalCache() {
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
  mpz::mpz_int compute_(std::vector<Var> &setOfVar, std::vector<Lit> &unitsLit,
                        std::vector<Var> &freeVariable, std::ostream &out) {
    // check if a positive hit occurs when considering the global cache.
    if (m_globalCacheManager.check(m_currentQuery)) return -1;

    showRun(out);
    m_nbCallCall++;
    if (!m_solver->solve(setOfVar)) return 0;

    m_solver->whichAreUnits(setOfVar, unitsLit);  // collect unit literals
    m_specs->preUpdate(unitsLit);
    m_globalCacheManager.pushUnits(unitsLit);

    // compute the connected composant
    std::vector<std::vector<Var>> varConnected;
    int nbComponent = m_specs->computeConnectedComponent(varConnected, setOfVar,
                                                         freeVariable);
    expelNoDecisionVar(freeVariable, m_isDecisionVariable);

    // consider each connected component.
    if (nbComponent) {
      mpz::mpz_int count = 1;
      m_nbSplit += (nbComponent > 1) ? nbComponent : 0;
      for (int cp = 0; cp < nbComponent && count != -1; cp++) {
        std::vector<Var> &connected = varConnected[cp];

        bool cacheActivated = cacheIsActivated(connected);
        TmpEntry<mpz::mpz_int> cb = cacheActivated
                                        ? m_cache->searchInCache(connected)
                                        : NULL_CACHE_ENTRY;
        if (cacheActivated && cb.defined)
          count *= cb.getValue();
        else {
          // query the server.
          QueryInfo query = m_globalCacheManager.query(connected);

          // recursive call
          mpz::mpz_int res = computeDecisionNode(connected, out);

          // check if the server found out an entry.
          if (res == -1) {
            if (query.id == m_currentQuery.id) {
              res = m_globalCacheManager.askCount(query);
            } else {
              // we will abort the current search until we reach the good query.
              count = -1;
            }
          }

          if (count != -1) {
            count *= res;
            if (cacheActivated) m_cache->addInCache(cb, res);
          } else {
            m_cache->releaseMemory(cb.e);
          }

          // ask the server to stop searching for the given query.
          m_globalCacheManager.stopQuery(query);
        }
      }

      m_globalCacheManager.popUnits(unitsLit);
      m_specs->postUpdate(unitsLit);
      return count;
    }  // else we have a tautology

    m_globalCacheManager.popUnits(unitsLit);
    m_specs->postUpdate(unitsLit);
    expelNoDecisionLit(unitsLit, m_isDecisionVariable);

    return 1;
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
  mpz::mpz_int computeDecisionNode(std::vector<Var> &connected,
                                   std::ostream &out) {
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
      return 1;
    }
    assert(!hasPriority || m_currentPrioritySet[v]);

    Lit l = Lit::makeLit(v, m_hPhase->selectPhase(v));
    m_nbDecisionNode++;

    if (!(m_nbDecisionNode % m_freqDecay)) m_hVar->decayCountConflict();

    // compile the formula where l is assigned to true
    DataBranch<mpz::mpz_int> b[2];

    assert(!m_solver->isInAssumption(l.var()));
    m_solver->pushAssumption(l);
    b[0].d = compute_(connected, b[0].unitLits, b[0].freeVars, out);
    m_solver->popAssumption();
    if (b[0].d == -1) goto unsetPriority;

    if (m_solver->isInAssumption(l))
      b[1].d = 0;
    else if (m_solver->isInAssumption(~l))
      b[1].d = compute_(connected, b[1].unitLits, b[1].freeVars, out);
    else {
      m_solver->pushAssumption(~l);
      b[1].d = compute_(connected, b[1].unitLits, b[1].freeVars, out);
      m_solver->popAssumption();
    }

  unsetPriority:
    unsetCurrentPriority(cutSet);
    if (b[0].d == -1 || b[1].d == -1) return -1;

    return b[0].d * m_problem->computeWeightUnitFree<mpz::mpz_int>(
                        b[0].unitLits, b[0].freeVars) +
           b[1].d * m_problem->computeWeightUnitFree<mpz::mpz_int>(
                        b[1].unitLits, b[1].freeVars);
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
  mpz::mpz_int compute(std::vector<Var> &setOfVar, std::ostream &out,
                       bool warmStart = true) {
    if (m_problem->isUnsat() ||
        (warmStart && !m_solver->warmStart(29, 11, setOfVar, m_out)))
      return 0;
    DataBranch<mpz::mpz_int> b;
    b.d = compute_(setOfVar, b.unitLits, b.freeVars, out);
    return b.d * m_problem->computeWeightUnitFree<mpz::mpz_int>(b.unitLits,
                                                                b.freeVars);
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
  mpz::mpz_int count(std::vector<Var> &setOfVar, std::vector<Lit> &assumption,
                     std::ostream &out) {
    initAssumption(assumption);

    // get the unit not in setOfVar.
    std::vector<Lit> shadowUnits;
    m_stampIdx++;
    for (auto &v : setOfVar) m_stampVar[v] = m_stampIdx;
    for (auto &l : assumption)
      if (m_stampVar[l.var()] != m_stampIdx) shadowUnits.push_back(l);

    m_specs->preUpdate(shadowUnits);
    m_globalCacheManager.pushUnits(shadowUnits);
    mpz::mpz_int result = compute(setOfVar, out, false);
    m_globalCacheManager.popUnits(shadowUnits);
    m_specs->postUpdate(shadowUnits);

    return result;
  }  // count

  /**
     Run the DPLL style algorithm with the operation manager.

     @param[in] vm, the set of options.
   */
  mpz::mpz_int run() {
    std::vector<Var> setOfVar;
    for (int i = 1; i <= m_specs->getNbVariable(); i++) setOfVar.push_back(i);

    mpz::mpz_int result = compute(setOfVar, m_out);
    printFinalStats(m_out);
    return result;
  }  // run
};
}  // namespace d4
