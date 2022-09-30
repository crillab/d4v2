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

#include <bits/stdint-uintn.h>
#include <sys/types.h>

#include <boost/program_options.hpp>
#include <cstddef>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <ios>
#include <iostream>
#include <string>
#include <vector>

#include "Counter.hpp"
#include "DataBranch.hpp"
#include "MethodManager.hpp"
#include "src/caching/CacheManager.hpp"
#include "src/caching/CachedBucket.hpp"
#include "src/caching/TmpEntry.hpp"
#include "src/heuristics/PartitioningHeuristic.hpp"
#include "src/heuristics/PhaseHeuristic.hpp"
#include "src/heuristics/ScoringMethod.hpp"
#include "src/methods/nnf/Node.hpp"
#include "src/preprocs/PreprocManager.hpp"
#include "src/problem/ProblemManager.hpp"
#include "src/problem/ProblemTypes.hpp"
#include "src/solvers/WrapperSolver.hpp"
#include "src/specs/SpecManager.hpp"
#include "src/utils/MemoryStat.hpp"

namespace d4 {
namespace po = boost::program_options;
template <class T>
class Counter;

template <class T>
class MaxSharpSAT : public MethodManager {
  enum TypeDecision { NO_DEC, EXIST_DEC, MAX_DEC };

  struct MaxSharpSatResult {
    T count;
    u_int8_t *valuation;

    MaxSharpSatResult() : count(T(0)), valuation(NULL) {}
    MaxSharpSatResult(const T c, u_int8_t *v) : count(c), valuation(v) {}

    void display(unsigned size) {
      assert(valuation);
      for (unsigned i = 0; i < size; i++) std::cout << (int)valuation[i] << " ";
      std::cout << "\n";
    }
  };

 private:
  const unsigned NB_SEP = 131;

  bool optDomConst;
  bool optReversePolarity;

  unsigned m_nbCallCall;
  unsigned m_nbCallProj;
  unsigned m_nbSplit;
  unsigned m_nbDecisionNode;
  unsigned m_optCached;
  unsigned m_stampIdx;

  bool m_isUnderAnd = false;
  bool m_greedyInitActivated;

  std::vector<unsigned> m_stampVar;
  std::vector<std::vector<Lit>> clauses;

  std::vector<bool> m_isDecisionVariable;
  std::vector<bool> m_isProjectedVariable;
  std::vector<bool> m_isMaxDecisionVariable;
  std::vector<unsigned> m_redirectionPos;
  unsigned m_countUpdateMaxCount = 0;

  const unsigned c_sizePage = 1 << 18;
  std::vector<u_int8_t *> m_memoryPages;
  unsigned m_posInMemoryPages;
  unsigned m_sizeArray;

  ProblemManager *m_problem;
  WrapperSolver *m_solver;
  SpecManager *m_specs;
  ScoringMethod *m_hVar;
  PhaseHeuristic *m_hPhase;

  CacheManager<T> *m_cacheInd;
  CacheManager<MaxSharpSatResult> *m_cacheMax;

  std::ostream m_out;
  bool m_panicMode;

  double m_threshold = -1;
  bool m_stopProcess = false;
  bool m_andDig = false;
  std::string m_heuristicMax = "none";
  unsigned m_heuristicMaxRdm = 0;

  MaxSharpSatResult m_scale = {T(1), NULL};
  MaxSharpSatResult m_maxCount = {T(0), NULL};

 public:
  /**
     Constructor.

     @param[in] vm, the list of options.
   */
  MaxSharpSAT(po::variables_map &vm, std::string &meth, bool isFloat,
              ProblemManager *initProblem, std::ostream &out,
              LastBreathPreproc &lastBreath)
      : m_problem(initProblem), m_out(nullptr) {
    // init the output stream
    m_out.copyfmt(out);
    m_out.clear(out.rdstate());
    m_out.basic_ios<char>::rdbuf(out.rdbuf());

    m_heuristicMax = vm["maxsharpsat-heuristic-phase"].as<std::string>();
    m_out << "c [CONSTRUCTOR MAX#SAT] Heuristic on MAX variables: "
          << m_heuristicMax << "\n";
    m_heuristicMaxRdm = vm["maxsharpsat-heuristic-phase-random"].as<unsigned>();
    m_out << "c [CONSTRUCTOR MAX#SAT] Use random on MAX variables: "
          << m_heuristicMaxRdm << "\n";
    m_threshold = vm["maxsharpsat-threshold"].as<double>();
    m_out << "c [CONSTRUCTOR MAX#SAT] Threshold: " << m_threshold << "\n";
    m_andDig = vm["maxsharpsat-option-and-dig"].as<bool>();
    m_out << "c [CONSTRUCTOR MAX#SAT] Dig for a partial solution under an AND: "
          << m_andDig << "\n";

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
    m_redirectionPos.clear();
    m_isDecisionVariable.clear();
    m_isMaxDecisionVariable.clear();
    m_isProjectedVariable.clear();

    m_redirectionPos.resize(m_problem->getNbVar() + 1, 0);
    m_isDecisionVariable.resize(m_problem->getNbVar() + 1, false);
    m_isProjectedVariable.resize(m_problem->getNbVar() + 1, false);
    for (unsigned i = 0; i < m_problem->getIndVar().size(); i++) {
      Var v = m_problem->getIndVar()[i];
      m_isDecisionVariable[v] = true;
      m_redirectionPos[v] = i;
      m_isProjectedVariable[v] = true;
    }

    m_isMaxDecisionVariable.resize(m_problem->getNbVar() + 1, false);
    for (unsigned i = 0; i < m_problem->getMaxVar().size(); i++) {
      Var v = m_problem->getMaxVar()[i];
      m_isMaxDecisionVariable[v] = true;
      m_redirectionPos[v] = i;
    }

    // no partitioning heuristic for the moment.
    assert(m_hVar && m_hPhase);
    m_cacheInd = CacheManager<T>::makeCacheManager(vm, m_problem->getNbVar(),
                                                   m_specs, m_out);
    m_cacheMax = CacheManager<MaxSharpSatResult>::makeCacheManager(
        vm, m_problem->getNbVar(), m_specs, m_out);

    // init the clock time.
    initTimer();

    m_greedyInitActivated = vm["maxsharpsat-option-greedy-init"].as<bool>();
    m_out << "c [MAX#SAT] Greedy init activated: " << m_greedyInitActivated
          << "\n";

    m_optCached = vm["cache-activated"].as<bool>();
    m_nbCallProj = m_nbDecisionNode = m_nbSplit = m_nbCallCall = 0;

    m_stampIdx = 0;
    m_stampVar.resize(m_specs->getNbVariable() + 1, 0);
    m_out << "c\n";

    // init the memory required for storing interpretation.
    m_memoryPages.push_back(new u_int8_t[c_sizePage]);
    m_posInMemoryPages = 0;
    m_sizeArray = m_problem->getMaxVar().size();

    // set the m_scale variable if needed.
    m_scale.valuation = getArray();
    for (unsigned i = 0; i < m_sizeArray; i++) m_scale.valuation[i] = 0;
    m_maxCount.valuation = getArray();
  }  // constructor

  /**
     Destructor.
   */
  ~MaxSharpSAT() {
    delete m_problem;
    delete m_solver;
    delete m_specs;
    delete m_hVar;
    delete m_hPhase;
    delete m_cacheInd;
    delete m_cacheMax;

    for (auto page : m_memoryPages) delete[] page;
  }  // destructor

 private:
  /**
   * @brief Print out the solution.
   *
   * @param solution is the maxsharp SAT solution we want to print.
   */
  void printSolution(MaxSharpSatResult &solution, char status) {
    assert(solution.valuation);
    assert(m_problem->getMaxVar().size() == m_sizeArray);
    std::cout << "v ";
    for (unsigned i = 0; i < m_problem->getMaxVar().size(); i++) {
      std::cout << ((solution.valuation[i]) ? "" : "-")
                << m_problem->getMaxVar()[i] << " ";
    }
    std::cout << "0\n";
    std::cout << status << " " << std::fixed << std::setprecision(50)
              << solution.count << "\n";
  }  // printSolution

  /**
     Print out information about the solving process.

     @param[in] out, the stream we use to print out information.
  */
  inline void showInter(std::ostream &out) {
    out << "c "
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << m_nbCallCall << std::fixed
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << m_nbCallProj << std::fixed
        << std::setprecision(2) << "|" << std::setw(WIDTH_PRINT_COLUMN_MC)
        << getTimer() << "|" << std::setw(WIDTH_PRINT_COLUMN_MC)
        << m_cacheInd->getNbPositiveHit() << "|"
        << std::setw(WIDTH_PRINT_COLUMN_MC) << m_cacheInd->getNbNegativeHit()
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << m_cacheInd->usedMemory()
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << m_nbSplit << "|"
        << std::setw(WIDTH_PRINT_COLUMN_MC) << MemoryStat::memUsedPeak() << "|"
        << std::setw(WIDTH_PRINT_COLUMN_MC) << m_nbDecisionNode << "|"
        << std::scientific << std::setw(WIDTH_PRINT_COLUMN_MC)
        << m_maxCount.count << "|\n";
  }  // showInter

  /**
     Print out a line of dashes.

     @param[in] out, the stream we use to print out information.
   */
  inline void separator(std::ostream &out) {
    out << "c ";
    for (int i = 0; i < NB_SEP; i++) out << "-";
    out << "\n";
  }  // separator

  /**
     Print out the header information.

     @param[in] out, the stream we use to print out information.
  */
  inline void showHeader(std::ostream &out) {
    separator(out);
    out << "c "
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "#call(m)"
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "#call(i)"
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "time"
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "#posHit"
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "#negHit"
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "memory"
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "#split"
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "mem(MB)"
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "#dec. Node"
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "max#count"
        << "|\n";
    separator(out);
  }  // showHeader

  /**
     Print out information when it is required.

     @param[in] out, the stream we use to print out information.
   */
  inline void showRun(std::ostream &out) {
    unsigned nbCall = m_nbCallCall + m_nbCallProj;
    if (!(nbCall & (MASK_HEADER))) showHeader(out);
    if (nbCall && !(nbCall & MASK_SHOWRUN_MC)) showInter(out);
  }  // showRun

  /**
     Print out the final stat.

     @param[in] out, the stream we use to print out information.
   */
  inline void printFinalStats(std::ostream &out) {
    separator(out);
    out << "c\n"
        << "c \033[1m\033[31mStatistics \033[0m\n"
        << "c \033[33mCompilation Information\033[0m\n"
        << "c Number of recursive call: " << m_nbCallCall << "\n"
        << "c Number of split formula: " << m_nbSplit << "\n"
        << "c Number of decision: " << m_nbDecisionNode << "\n"
        << "c\n";
    m_cacheInd->printCacheInformation(out);
    out << "c\n";
    m_cacheMax->printCacheInformation(out);
    out << "c Final time: " << getTimer() << "\n";
    out << "c\n";
  }  // printFinalStat

  /**
   * @brief Get a pointer on an allocated array of size m_sizeArray (which is
   * set once in the constructor).
   *
   * @return a pointer on a u_int8_t array.
   */
  u_int8_t *getArray() {
    u_int8_t *ret = &(m_memoryPages.back()[m_posInMemoryPages]);
    m_posInMemoryPages += m_sizeArray;
    if (m_posInMemoryPages > c_sizePage) {
      m_memoryPages.push_back(new u_int8_t[c_sizePage]);
      m_posInMemoryPages = 0;
      ret = m_memoryPages.back();
    }
    return ret;
  }  // getArray

  /**
   * Expel from a set of variables the ones they are marked as being decidable.
   * @param[out] vars, the set of variables we search to filter.
   * @param[in] isDecisionvariable, a type decision vector that marks as true
   * decision variables.
   */
  void expelNoDecisionVar(std::vector<Var> &vars) {
    unsigned j = 0;
    for (unsigned i = 0; i < vars.size(); i++)
      if (m_isDecisionVariable[vars[i]]) vars[j++] = vars[i];
    vars.resize(j);
  }  // expelNoDecisionVar

  /**
     Expel from a set of variables the ones they are marked as being decidable.

     @param[out] lits, the set of literals we search to filter.

     @param[in] isDecisionvariable, a boolean vector that marks as true decision
     variables.
   */
  void expelNoDecisionLit(std::vector<Lit> &lits) {
    unsigned j = 0;
    for (unsigned i = 0; i < lits.size(); i++)
      if (m_isDecisionVariable[lits[i].var()]) lits[j++] = lits[i];
    lits.resize(j);
  }  // expelNoDecisionLit

  /**
   * @brief Estimate the maximum number of models we can get when considering
   * the given set of variables.
   *
   * @param setOfVar is the considered set of variables.
   * @return T is the resulting number of models (upper bound).
   */
  T computeUpper(std::vector<Var> &setOfVar) {
    T ret = 1;
    for (auto v : setOfVar)
      if (m_isProjectedVariable[v]) ret = ret * 2;
    return ret;
  }  // computeUpper

  /**
   * @brief Apply an or logic between resValuation and orValuation (the result
   * is stored in resValuation).
   *
   * @param vars is the set of variables we are considering for the OR.
   * @param[out] resValuation is a 'boolean' vector that also receive the
   * result.
   * @param orValuation is another 'boolean' vector used for the OR
   */
  void orOnMaxVar(std::vector<Var> &vars, u_int8_t *resValuation,
                  u_int8_t *orValuation) {
    for (auto v : vars) {
      if (m_isMaxDecisionVariable[v])
        resValuation[m_redirectionPos[v]] |= orValuation[m_redirectionPos[v]];
    }
  }  // disjunctionOnMaxVariable

  /**
   * @brief Update the bound if needed.
   *
   * @param result is a solution we found.
   * @param vars is the set of variables of the component that is considered to
   * compute result.
   */
  void updateBound(MaxSharpSatResult &result, std::vector<Var> &vars) {
    if (!m_isUnderAnd && result.count * m_scale.count > m_maxCount.count) {
      m_maxCount.count = result.count * m_scale.count;

      assert(result.valuation);
      for (unsigned i = 0; i < m_sizeArray; i++)
        m_maxCount.valuation[i] = m_scale.valuation[i];

      for (auto v : vars)
        if (m_isMaxDecisionVariable[v])
          m_maxCount.valuation[m_redirectionPos[v]] =
              result.valuation[m_redirectionPos[v]];

      if (m_threshold < 0 || m_maxCount.count < T(m_threshold)) {
        m_out << "i " << ++m_countUpdateMaxCount << " " << std::fixed
              << std::setprecision(2) << getTimer() << " ";
        m_out << std::fixed << std::setprecision(50) << m_maxCount.count
              << "\n";
      } else {
        m_out << "c Stop because we found out a good enough solution\n";
        m_out << "r SATISFIABLE\n";
        printSolution(m_maxCount, 's');
        m_stopProcess = true;
      }
    }
  }  // updateBound

  /**
   * @brief Search for a valuation of the max variables that maximizes the
   * number of models on the remaning formula where some variables are
   * forget.
   *
   * @param setOfVar, the current set of considered variables.
   * @param unitsLit, the set of unit literal detected at this level.
   * @param freeVariable, the variables which become free decision node.
   * @param out, the stream we use to print out logs.
   * @param result, the structure where is stored the result.
   */
  void searchMaxValuation(std::vector<Var> &setOfVar,
                          std::vector<Lit> &unitsLit,
                          std::vector<Var> &freeVariable, std::ostream &out,
                          MaxSharpSatResult &result) {
    if (m_stopProcess) return;

    showRun(out);
    m_nbCallCall++;

    // is the problem still satisfiable?
    if (!m_solver->solve(setOfVar)) {
      result = {T(0), NULL};
      return;
    }

    m_solver->whichAreUnits(setOfVar, unitsLit);  // collect unit literals
    m_specs->preUpdate(unitsLit);

    // compute the connected composant
    std::vector<std::vector<Var>> varConnected;
    int nbComponent = m_specs->computeConnectedComponent(varConnected, setOfVar,
                                                         freeVariable);

    // init the returned result.
    result.valuation = getArray();
    for (unsigned i = 0; i < m_sizeArray; i++) result.valuation[i] = 0;

    // set a valuation for fixed variables.
    T saveCount = m_scale.count;
    T fixCount = T(1), fixInd = T(1);

    for (auto &v : freeVariable)
      if (m_isMaxDecisionVariable[v]) {
        Lit l = Lit::makeLitTrue(v);
        if (m_problem->getWeightLit(l) < m_problem->getWeightLit(~l)) l = ~l;

        m_scale.valuation[m_redirectionPos[v]] = 1 - l.sign();
        result.valuation[m_redirectionPos[v]] = 1 - l.sign();
        fixCount *= T(m_problem->getWeightLit(l));
      } else if (m_isDecisionVariable[v])
        fixInd *= T(m_problem->getWeightVar(v));

    // consider the unit literals that belong to max
    for (auto &l : unitsLit)
      if (m_isMaxDecisionVariable[l.var()]) {
        fixCount *= T(m_problem->getWeightLit(l));
        m_scale.valuation[m_redirectionPos[l.var()]] = 1 - l.sign();
        result.valuation[m_redirectionPos[l.var()]] = 1 - l.sign();
      } else if (m_isDecisionVariable[l.var()])
        fixInd *= T(m_problem->getWeightLit(l));

    result.count = fixCount;
    m_scale.count = m_scale.count * fixCount * fixInd;

    expelNoDecisionVar(freeVariable);
    bool wasUnderAnd = m_isUnderAnd;

    // dig for an assignment for each component (execpt the first one).
    std::vector<MaxSharpSatResult> andCount;
    if (!m_andDig)
      m_isUnderAnd = wasUnderAnd || nbComponent > 1;
    else {
      for (int cp = 1; cp < nbComponent; cp++) {
        andCount.push_back({T(0), NULL});
        greedySearch(varConnected[cp], out, andCount.back());
        orOnMaxVar(varConnected[cp], m_scale.valuation,
                   andCount.back().valuation);
        m_scale.count *= andCount.back().count;
      }
    }

    // consider each connected component.
    T mustMultiply = T(1);
    if (nbComponent) {
      m_nbSplit += (nbComponent > 1) ? nbComponent : 0;
      for (int cp = 0; cp < nbComponent; cp++) {
        std::vector<Var> &connected = varConnected[cp];
        TmpEntry<MaxSharpSatResult> cb = m_cacheMax->searchInCache(connected);

        // should divide if we are under an AND and if we manage the option.
        if (m_andDig && cp > 0)
          m_scale.count = m_scale.count / andCount[cp - 1].count;

        if (cb.defined) {
          mustMultiply = cb.getValue().count;
          if (cb.getValue().valuation)
            orOnMaxVar(connected, result.valuation, cb.getValue().valuation);
        } else {
          MaxSharpSatResult tmpResult;
          searchMaxSharpSatDecision(connected, out, tmpResult);
          m_cacheMax->addInCache(cb, tmpResult);
          mustMultiply = tmpResult.count;
          if (tmpResult.valuation)
            orOnMaxVar(connected, result.valuation, tmpResult.valuation);
        }

        result.count = result.count * mustMultiply;

        // should multiply if we are under an AND and if we manage the option.
        if (nbComponent > 1 && m_andDig) {
          m_scale.count = m_scale.count * mustMultiply;

          for (auto &v : connected)
            if (m_isMaxDecisionVariable[v])
              m_scale.valuation[m_redirectionPos[v]] =
                  result.valuation[m_redirectionPos[v]];
        }
      }
    }  // else we have a tautology

    m_isUnderAnd = wasUnderAnd;
    m_scale.count = saveCount;

    m_specs->postUpdate(unitsLit);
    expelNoDecisionLit(unitsLit);

    // update the global maxcount if needed.
    m_scale.count *= fixInd;
    updateBound(result, setOfVar);
    m_scale.count = saveCount;
  }  // searchMaxValuation

  /**
   * @brief Given the selected heuristic, return the way we want to assign the
   * given variable (actually we want the sign).
   *
   * @param v is the variable we want to assign.
   * @return 1 if we want to assign to false, 0 otherwise/
   */
  inline bool selectPhase(Var v) {
    assert(m_isMaxDecisionVariable[v]);
    int rdm = rand() % 100;
    if (rdm <= m_heuristicMaxRdm) return rdm & 1;

    if (m_heuristicMax == "weight")
      return m_problem->getWeightLit(Lit::makeLitTrue(v)) <
             m_problem->getWeightLit(Lit::makeLitFalse(v));
    if (m_scale.count > 0 && m_heuristicMax == "best")
      return m_scale.valuation[m_redirectionPos[v]];
    return m_hPhase->selectPhase(v);
  }  // selectPhase

  /**
   * This function select a variable and compile a decision node.
   *
   * @param[in] connected, the set of variable present in the current
   * problem.
   * @param[in] out, the stream we use to print out logs.
   * @param[out] result, the best solution found.
   */
  void searchMaxSharpSatDecision(std::vector<Var> &connected, std::ostream &out,
                                 MaxSharpSatResult &result) {
    if (m_stopProcess) return;

    // search the next variable to branch on
    Var v =
        m_hVar->selectVariable(connected, *m_specs, m_isMaxDecisionVariable);

    if (v == var_Undef) {
      std::vector<Lit> unitsLit;
      std::vector<Var> freeVar;
      result.count = countInd_(connected, unitsLit, freeVar, out);
      result.count *= m_problem->computeWeightUnitFree<T>(unitsLit, freeVar);
      result.valuation = NULL;
      return;
    }

    Lit l = Lit::makeLit(v, selectPhase(v));
    m_nbDecisionNode++;

    // consider the two value for l
    DataBranch<T> b[2];
    MaxSharpSatResult res[2];

    // search max#sat for the first phase.
    assert(!m_solver->isInAssumption(l.var()));
    m_solver->pushAssumption(l);
    searchMaxValuation(connected, b[0].unitLits, b[0].freeVars, out, res[0]);

    m_solver->popAssumption();
    b[0].d = res[0].count *
             m_problem->computeWeightUnitFree<T>(b[0].unitLits, b[0].freeVars);

    // search max#sat for the next phase.
    if (m_solver->isInAssumption(l))
      res[1].count = T(0);
    else if (m_solver->isInAssumption(~l))
      searchMaxValuation(connected, b[1].unitLits, b[1].freeVars, out, res[1]);
    else {
      m_solver->pushAssumption(~l);
      searchMaxValuation(connected, b[1].unitLits, b[1].freeVars, out, res[1]);
      m_solver->popAssumption();
    }

    b[1].d = res[1].count *
             m_problem->computeWeightUnitFree<T>(b[1].unitLits, b[1].freeVars);

    // aggregation with max.
    result.count = (b[0].d > b[1].d) ? b[0].d : b[1].d;
    result.valuation = (b[0].d > b[1].d) ? res[0].valuation : res[1].valuation;
  }  // searchMaxSharpSatDecision

  /**
   * @brief Count the number of projected models.
   *
   * @param setOfVar, the current set of considered variables
   * @param unitsLit, the set of unit literal detected at this level
   * @param freeVariable, the variables which become free decision node
   * @param out, the stream we use to print out logs.
   *
   * \return the number of models.
   */
  T countInd_(std::vector<Var> &setOfVar, std::vector<Lit> &unitsLit,
              std::vector<Var> &freeVariable, std::ostream &out) {
    if (m_stopProcess) return T(0);

    showRun(out);
    m_nbCallProj++;

    if (!m_solver->solve(setOfVar)) return T(0);

    m_solver->whichAreUnits(setOfVar, unitsLit);  // collect unit literals
    m_specs->preUpdate(unitsLit);

    // compute the connected composant
    std::vector<std::vector<Var>> varConnected;

    int nbComponent = m_specs->computeConnectedComponent(varConnected, setOfVar,
                                                         freeVariable);
    expelNoDecisionVar(freeVariable);

    // consider each connected component.
    T result = T(1);
    if (nbComponent) {
      m_nbSplit += (nbComponent > 1) ? nbComponent : 0;

      for (int cp = 0; cp < nbComponent; cp++) {
        std::vector<Var> &connected = varConnected[cp];
        TmpEntry<T> cb = m_cacheInd->searchInCache(connected);

        if (cb.defined)
          result = result * cb.getValue();
        else {
          T curr = countIndDecisionNode(connected, out);
          m_cacheInd->addInCache(cb, curr);
          result = result * curr;
        }
      }
    }  // else we have a tautology

    m_specs->postUpdate(unitsLit);
    expelNoDecisionLit(unitsLit);
    return result;
  }  // countInd_

  /**
   * @brief This function select a variable and compile a decision node.
   *
   * @param connected, the set of variable present in the current
   * problem.
   * @param out, the stream we use to print out logs.
   * \return the number of computed models.
   */
  T countIndDecisionNode(std::vector<Var> &connected, std::ostream &out) {
    if (m_stopProcess) return T(0);

    // search the next variable to branch on
    Var v = m_hVar->selectVariable(connected, *m_specs, m_isDecisionVariable);

    if (v == var_Undef) return T(1);

    // select a variable for decision.
    Lit l = Lit::makeLit(v, m_hPhase->selectPhase(v));
    m_nbDecisionNode++;

    // consider the two value for l
    DataBranch<T> b[2];

    assert(!m_solver->isInAssumption(l.var()));
    m_solver->pushAssumption(l);
    b[0].d = countInd_(connected, b[0].unitLits, b[0].freeVars, out);
    m_solver->popAssumption();

    // compute the next lower regarding the already compute information.
    b[0].d *= m_problem->computeWeightUnitFree<T>(b[0].unitLits, b[0].freeVars);

    if (m_solver->isInAssumption(l))
      b[1].d = 0;
    else if (m_solver->isInAssumption(~l))
      b[1].d = countInd_(connected, b[1].unitLits, b[1].freeVars, out);
    else {
      m_solver->pushAssumption(~l);
      b[1].d = countInd_(connected, b[1].unitLits, b[1].freeVars, out);
      m_solver->popAssumption();
    }

    b[1].d *= m_problem->computeWeightUnitFree<T>(b[1].unitLits, b[1].freeVars);
    return b[0].d + b[1].d;
  }  // computeDecisionNode

  /**
   * @brief Search for an interpretation that maximize the number of models in
   * a greedy search.
   *
   * @param setOfVar is the set of variables we are considering.
   * @param out is the stream where is printed out the logs.
   * @param result is the interpretation and the related number of models.
   */
  void greedySearch(std::vector<Var> &setOfVar, std::ostream &out,
                    MaxSharpSatResult &result) {
    // first: search for a model to init the interpretation.
    if (!m_solver->solve(setOfVar)) {
      result.count = T(0);
      result.valuation = NULL;
      return;
    }

    // collect the model.
    T multiply = T(1);
    result.valuation = getArray();

    std::vector<Var> vars = setOfVar;
    unsigned cpt = 0, j = 0;
    for (unsigned i = 0; i < vars.size(); i++) {
      Var v = vars[i];
      if (m_isMaxDecisionVariable[v]) {
        Lit l = Lit::makeLit(v, m_solver->getModelVar(v) == l_False);
        m_solver->pushAssumption(l);
        multiply *= T(m_problem->getWeightLit(l));
        result.valuation[m_redirectionPos[l.var()]] = 1 - l.sign();
        cpt++;
      } else
        vars[j++] = vars[i];
    }
    vars.resize(j);

    m_solver->propagateAssumption();

    std::vector<Lit> unitsLit;
    std::vector<Var> freeVariable;
    result.count = countInd_(vars, unitsLit, freeVariable, out);
    if (!m_stopProcess)
      result.count =
          result.count * multiply *
          m_problem->computeWeightUnitFree<T>(unitsLit, freeVariable);
    m_solver->popAssumption(cpt);
  }  // greedySearch

  /**
   * @brief Search an assignation of Max variables that maximize the number of
   * models of the problem conditionned by this assignation.
   *
   * @param setOfVar is the set of variables of the considered problem.
   * @param out is the stream where are printed out the logs.
   * @param[out] result is the place where is stored the result.
   * @param warmStart is an option to activate/deactivate the warm start
   * strategy (by defaut it is deactivate).
   */
  void compute(std::vector<Var> &setOfVar, std::ostream &out,
               MaxSharpSatResult &result, bool warmStart = true) {
    if (m_problem->isUnsat() ||
        (warmStart && !m_panicMode &&
         !m_solver->warmStart(29, 11, setOfVar, m_out))) {
      result = {T(0), NULL};
      return;
    }

    if (m_greedyInitActivated) {
      MaxSharpSatResult greedyResult;
      greedySearch(setOfVar, out, greedyResult);
      updateBound(greedyResult, setOfVar);
      std::cout << "c Greedy search done: " << greedyResult.count << "\n";
    }

    DataBranch<T> b;
    searchMaxValuation(setOfVar, b.unitLits, b.freeVars, out, result);
    assert(result.valuation);
    if (!m_stopProcess)
      result.count *=
          m_problem->computeWeightUnitFree<T>(b.unitLits, b.freeVars);
  }  // compute

 public:
  /**
   * @brief Stop the current search and print out the best solution found so
   * far.
   *
   */
  void interrupt() override {
    if (m_maxCount.count < 0)
      std::cout << "No solution found so far\n";
    else {
      std::cout << "Processus interrupted, here is the best solution found\n";
      printSolution(m_maxCount, 'k');
    }
  }  // interrupt

  /**
   * @brief Search for the instantiation of the variables of
   * m_problem->getMaxVar() that maximize the number of the remaining
   * variables where the variables not belonging to m_problem->getIndVar()
   * are existantially quantified.
   *
   * @param[in] vm, the set of options.
   */
  void run(po::variables_map &vm) {
    std::vector<Var> setOfVar;
    for (int i = 1; i <= m_specs->getNbVariable(); i++) setOfVar.push_back(i);

    MaxSharpSatResult result;
    compute(setOfVar, m_out, result);
    printFinalStats(m_out);
    if (!m_stopProcess) {
      if (m_threshold < 0)
        printSolution(result, 'o');
      else {
        if (result.count >= T(m_threshold)) {
          std::cout << "r SATISFIABLE\n";
          printSolution(result, 's');
        } else {
          std::cout << "r UNSATISFIABLE\n";
          printSolution(result, 'o');
        }
      }
    }
  }  // run

};  // end class
}  // namespace d4
