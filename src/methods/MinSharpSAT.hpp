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
#include <boost/program_options.hpp>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <sys/types.h>

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

#include "Counter.hpp"
#include "DataBranch.hpp"
#include "MethodManager.hpp"

namespace d4 {
namespace po = boost::program_options;
template <class T> class Counter;

template <class T> class MinSharpSAT : public MethodManager {
  enum TypeDecision { NO_DEC, EXIST_DEC, MAX_DEC };

  struct MinSharpSatResult {
    T count;
    u_int8_t *valuation;
  };

private:
  const unsigned NB_SEP = 118;

  bool optDomConst;
  bool optReversePolarity;

  unsigned m_nbCallCall;
  unsigned m_nbSplit;
  unsigned m_nbDecisionNode;
  unsigned m_optCached;
  unsigned m_stampIdx;
  bool m_isProjectedMode;

  std::vector<unsigned> m_stampVar;
  std::vector<std::vector<Lit>> clauses;

  std::vector<bool> m_isDecisionVar;
  std::vector<bool> m_isMaxDecisionVar;
  std::vector<unsigned> m_redirectionPos;
  T m_minCount = T(-1);

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
  CacheManager<MinSharpSatResult> *m_cacheMax;

  std::ostream m_out;
  bool m_panicMode;

public:
  /**
     Constructor.

     @param[in] vm, the list of options.
   */
  MinSharpSAT(po::variables_map &vm, std::string &meth, bool isFloat,
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
    m_redirectionPos.clear();
    m_isDecisionVar.clear();
    m_isMaxDecisionVar.clear();

    m_redirectionPos.resize(m_problem->getNbVar() + 1, 0);
    m_isDecisionVar.resize(m_problem->getNbVar() + 1, false);
    for (unsigned i = 0; i < m_problem->getIndVar().size(); i++) {
      Var v = m_problem->getIndVar()[i];
      m_isDecisionVar[v] = true;
      m_redirectionPos[v] = i;
    }

    m_isMaxDecisionVar.resize(m_problem->getNbVar() + 1, false);
    for (unsigned i = 0; i < m_problem->getMaxVar().size(); i++) {
      Var v = m_problem->getMaxVar()[i];
      m_isMaxDecisionVar[v] = true;
      m_redirectionPos[v] = i;
    }

    // no partitioning heuristic for the moment.
    assert(m_hVar && m_hPhase);
    m_cacheInd = CacheManager<T>::makeCacheManager(vm, m_problem->getNbVar(),
                                                   m_specs, m_out);
    m_cacheMax = CacheManager<MinSharpSatResult>::makeCacheManager(
        vm, m_problem->getNbVar(), m_specs, m_out);

    // init the clock time.
    initTimer();

    m_optCached = vm["cache-activated"].as<bool>();
    m_nbDecisionNode = m_nbSplit = m_nbCallCall = 0;

    m_stampIdx = 0;
    m_stampVar.resize(m_specs->getNbVariable() + 1, 0);
    m_out << "c\n";

    // init the memory requierd for storing interpretation.
    m_memoryPages.push_back(new u_int8_t[c_sizePage]);
    m_posInMemoryPages = 0;
    m_sizeArray = m_problem->getMaxVar().size();
  } // constructor

  /**
     Destructor.
   */
  ~MinSharpSAT() {
    delete m_problem;
    delete m_solver;
    delete m_specs;
    delete m_hVar;
    delete m_hPhase;
    delete m_cacheInd;
    delete m_cacheMax;

    for (auto page : m_memoryPages)
      delete[] page;
  } // destructor

private:
  /**
     Print out information about the solving process.

     @param[in] out, the stream we use to print out information.
  */
  inline void showInter(std::ostream &out) {
    out << "c "
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << m_nbCallCall << std::fixed
        << std::setprecision(2) << "|" << std::setw(WIDTH_PRINT_COLUMN_MC)
        << getTimer() << "|" << std::setw(WIDTH_PRINT_COLUMN_MC)
        << m_cacheInd->getNbPositiveHit() << "|"
        << std::setw(WIDTH_PRINT_COLUMN_MC) << m_cacheInd->getNbNegativeHit()
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << m_cacheInd->usedMemory()
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << m_nbSplit << "|"
        << std::setw(WIDTH_PRINT_COLUMN_MC) << MemoryStat::memUsedPeak() << "|"
        << std::setw(WIDTH_PRINT_COLUMN_MC) << m_nbDecisionNode << "|"
        << std::setw(WIDTH_PRINT_COLUMN_MC) << m_minCount << "|\n";
  } // showInter

  /**
     Print out a line of dashes.

     @param[in] out, the stream we use to print out information.
   */
  inline void separator(std::ostream &out) {
    out << "c ";
    for (int i = 0; i < NB_SEP; i++)
      out << "-";
    out << "\n";
  } // separator

  /**
     Print out the header information.

     @param[in] out, the stream we use to print out information.
  */
  inline void showHeader(std::ostream &out) {
    separator(out);
    out << "c "
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "#compile"
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
  } // showHeader

  /**
     Print out information when it is requiered.

     @param[in] out, the stream we use to print out information.
   */
  inline void showRun(std::ostream &out) {
    if (!(m_nbCallCall & (MASK_HEADER)))
      showHeader(out);
    if (m_nbCallCall && !(m_nbCallCall & MASK_SHOWRUN_MC))
      showInter(out);
  } // showRun

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
    out << "c\n";
    m_cacheInd->printCacheInformation(out);
    out << "c\n";
    m_cacheMax->printCacheInformation(out);
    out << "c Final time: " << getTimer() << "\n";
    out << "c\n";
  } // printFinalStat

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
  } // getArray

  /**
   * Expel from a set of variables the ones they are marked as being decidable.
   * @param[out] vars, the set of variables we search to filter.
   * @param[in] isDecisionvariable, a type decision vector that marks as true
   * decision variables.
   */
  void expelNoDecisionVar(std::vector<Var> &vars,
                          std::vector<bool> &isDecisionVariable) {
    unsigned j = 0;
    for (unsigned i = 0; i < vars.size(); i++)
      if (isDecisionVariable[vars[i]])
        vars[j++] = vars[i];
    vars.resize(j);
  } // expelNoDecisionVar

  /**
   * Expel from a set of variables the ones they are marked as being decidable.
   * @param[out] vars, the set of variables we search to filter.
   * @param[in] isDecisionvariable, a type decision vector that marks as true
   * decision variables.
   */
  void expelNoDecisionLit(std::vector<Lit> &lits,
                          std::vector<bool> &isDecisionVariable) {
    unsigned j = 0;
    for (unsigned i = 0; i < lits.size(); i++)
      if (isDecisionVariable[lits[i].var()])
        lits[j++] = lits[i];
    lits.resize(j);
  } // expelNoDecisionLit

  /**
   * @brief Search for a valuation of the max variables that maximizes the
   * number of models on the remaning formula where some variables are forget.
   *
   * @param setOfVar, the current set of considered variables.
   * @param unitsLit, the set of unit literal detected at this level.
   * @param freeVariable, the variables which become free decision node.
   * @param out, the stream we use to print out logs.
   * @param result, the strucre where is solved the result.
   */
  void searchMinValuation(std::vector<Var> &setOfVar,
                          std::vector<Lit> &unitsLit,
                          std::vector<Var> &freeVariable, std::ostream &out,
                          MinSharpSatResult &result) {
    showRun(out);
    m_nbCallCall++;

    // is the problem still satisifiable?
    if (!m_solver->solve(setOfVar)) {
      result.count = T(0);
      result.valuation = getArray();
      return;
    }

    m_solver->whichAreUnits(setOfVar, unitsLit); // collect unit literals
    m_specs->preUpdate(unitsLit);

    // compute the connected composant
    std::vector<std::vector<Var>> varConnected;

    int nbComponent = m_specs->computeConnectedComponent(varConnected, setOfVar,
                                                         freeVariable);
    expelNoDecisionVar(freeVariable, m_isDecisionVar);

    // init the returned result.
    result.count = T(1);
    result.valuation = getArray();
    for (unsigned i = 0; i < m_sizeArray; i++)
      result.valuation[i] = 0;

    Lit propagateMaxVar = lit_Undef;
    for (auto l : unitsLit)
      if (!m_solver->isInAssumption(l) && m_isMaxDecisionVar[l.var()]) {
        propagateMaxVar = l;
        break;
      }

    if (propagateMaxVar != lit_Undef)
      result.count = 0;
    else {
      // consider each connected component.
      if (nbComponent) {
        m_nbSplit += (nbComponent > 1) ? nbComponent : 0;
        for (int cp = 0; cp < nbComponent; cp++) {
          std::vector<Var> &connected = varConnected[cp];
          TmpEntry<MinSharpSatResult> cb = m_cacheMax->searchInCache(connected);

          if (cb.defined) {
            result.count = result.count * cb.getValue().count;
            if (cb.getValue().valuation) {
              for (auto v : connected) {
                if (m_isMaxDecisionVar[v])
                  result.valuation[m_redirectionPos[v]] |=
                      cb.getValue().valuation[m_redirectionPos[v]];
              }
            }
          } else {
            MinSharpSatResult tmpResult;
            searchMinSharpSatDecision(connected, out, tmpResult);
            m_cacheMax->addInCache(cb, tmpResult);
            result.count = result.count * tmpResult.count;

            if (tmpResult.valuation) {
              for (auto v : connected) {
                if (m_isMaxDecisionVar[v]) {
                  result.valuation[m_redirectionPos[v]] |=
                      tmpResult.valuation[m_redirectionPos[v]];
                }
              }
            }
          }

          if (result.count == 0)
            break;
        }
      } // else we have a tautology
    }

    for (unsigned i = 0; i < m_sizeArray; i++) {
      Lit l = Lit::makeLit(m_problem->getMaxVar()[i], false);
      if (m_specs->litIsAssigned(l))
        result.valuation[i] = m_specs->litIsAssignedToTrue(l);
    }

    if (propagateMaxVar != lit_Undef) {
      result.valuation[m_redirectionPos[propagateMaxVar.var()]] =
          propagateMaxVar.sign();
    }

    m_specs->postUpdate(unitsLit);
    expelNoDecisionLit(unitsLit, m_isDecisionVar);
  } // searchMaxValuation

  /**
   * This function select a variable and compile a decision node.
   *
   * @param[in] connected, the set of variable present in the current problem.
   * @param[in] out, the stream we use to print out logs.
   * @param[out] result, the best solution found.
   *
   * \return the compiled formula.
   */
  void searchMinSharpSatDecision(std::vector<Var> &connected, std::ostream &out,
                                 MinSharpSatResult &result) {
    // search the next variable to branch on
    Var v = m_hVar->selectVariable(connected, *m_specs, m_isMaxDecisionVar);

    if (v == var_Undef) {
      std::vector<Lit> unitsLit;
      std::vector<Var> freeVar;
      result.count = countInd_(connected, unitsLit, freeVar, out);
      result.count *= m_problem->computeWeightUnitFree<T>(unitsLit, freeVar);
      result.valuation = NULL;

      if (result.count > m_minCount)
        m_minCount = result.count;
      return;
    }

    Lit l = Lit::makeLit(v, m_hPhase->selectPhase(v));
    m_nbDecisionNode++;

    // consider the two value for l
    DataBranch<T> b[2];
    MinSharpSatResult res[2];

    assert(!m_solver->isInAssumption(l.var()));
    m_solver->pushAssumption(l);
    searchMinValuation(connected, b[0].unitLits, b[0].freeVars, out, res[0]);
    res[0].valuation[m_redirectionPos[v]] = !l.sign();
    m_solver->popAssumption();

    if (res[0].count == 0)
      result = {0, res[0].valuation};
    else {
      if (m_solver->isInAssumption(l)) {
        res[1].count = T(0);
        res[1].valuation = getArray();
      } else if (m_solver->isInAssumption(~l))
        searchMinValuation(connected, b[1].unitLits, b[1].freeVars, out,
                           res[1]);
      else {
        m_solver->pushAssumption(~l);
        searchMinValuation(connected, b[1].unitLits, b[1].freeVars, out,
                           res[1]);
        m_solver->popAssumption();
      }

      b[0].d = res[0].count * m_problem->computeWeightUnitFree<T>(
                                  b[0].unitLits, b[0].freeVars);
      b[1].d = res[1].count * m_problem->computeWeightUnitFree<T>(
                                  b[1].unitLits, b[1].freeVars);
      res[1].valuation[m_redirectionPos[v]] = l.sign();

      if (b[0].d < b[1].d)
        result = {b[0].d, res[0].valuation};
      else
        result = {b[1].d, res[1].valuation};
    }

    if (m_minCount == -1 || result.count < m_minCount)
      m_minCount = result.count;
  } // searchMaxSharpSatDecision

  /**
   * Count the number of projected models.
   *
   * @param[in] setOfVar, the current set of considered variables
   * @param[in] unitsLit, the set of unit literal detected at this level
   * @param[in] freeVariable, the variables which become free decision node
   * @param[in] out, the stream we use to print out logs.
   *
   * \return an element of type U that sums up the given CNF sub-formula using
   * a DPLL style algorithm with an operation manager.
   */
  T countInd_(std::vector<Var> &setOfVar, std::vector<Lit> &unitsLit,
              std::vector<Var> &freeVariable, std::ostream &out) {
    showRun(out);
    m_nbCallCall++;

    if (!m_solver->solve(setOfVar))
      return T(0);

    m_solver->whichAreUnits(setOfVar, unitsLit); // collect unit literals
    m_specs->preUpdate(unitsLit);

    // compute the connected composant
    std::vector<std::vector<Var>> varConnected;

    int nbComponent = m_specs->computeConnectedComponent(varConnected, setOfVar,
                                                         freeVariable);
    expelNoDecisionVar(freeVariable, m_isDecisionVar);

    // consider each connected component.
    T ret = T(1);
    if (nbComponent) {
      m_nbSplit += (nbComponent > 1) ? nbComponent : 0;
      for (int cp = 0; cp < nbComponent; cp++) {
        std::vector<Var> &connected = varConnected[cp];
        TmpEntry<T> cb = m_cacheInd->searchInCache(connected);

        if (cb.defined)
          ret = ret * cb.getValue();
        else {
          T curr = countIndDecisionNode(connected, out);
          m_cacheInd->addInCache(cb, curr);
          ret = ret * curr;
        }
      }
    } // else we have a tautology

    m_specs->postUpdate(unitsLit);
    return ret;
  } // countInd_

  /**
   * This function select a variable and compile a decision node.
   *
   * @param[in] connected, the set of variable present in the current problem.
   * @param[in] out, the stream we use to print out logs.
   *
   * \return the compiled formula.
   */
  T countIndDecisionNode(std::vector<Var> &connected, std::ostream &out) {
    // search the next variable to branch on
    Var v = m_hVar->selectVariable(connected, *m_specs, m_isDecisionVar);

    if (v == var_Undef)
      return T(1);

    Lit l = Lit::makeLit(v, m_hPhase->selectPhase(v));
    m_nbDecisionNode++;

    // consider the two value for l
    DataBranch<T> b[2];

    assert(!m_solver->isInAssumption(l.var()));
    m_solver->pushAssumption(l);
    b[0].d = countInd_(connected, b[0].unitLits, b[0].freeVars, out);
    m_solver->popAssumption();

    if (m_solver->isInAssumption(l))
      b[1].d = 0;
    else if (m_solver->isInAssumption(~l))
      b[1].d = countInd_(connected, b[1].unitLits, b[1].freeVars, out);
    else {
      m_solver->pushAssumption(~l);
      b[1].d = countInd_(connected, b[1].unitLits, b[1].freeVars, out);
      m_solver->popAssumption();
    }

    b[0].d *= m_problem->computeWeightUnitFree<T>(b[0].unitLits, b[0].freeVars);
    b[1].d *= m_problem->computeWeightUnitFree<T>(b[1].unitLits, b[1].freeVars);
    return b[0].d + b[1].d;
  } // computeDecisionNode

  /**
     Compute U using the trace of a SAT solver.

     @param[in] setOfVar, the set of variables of the considered problem.
     @param[in] out, the stream are is print out the logs.
     @param[in] warmStart, to activate/deactivate the warm start strategy.
     /!\ When the warm start is activated we the assumptions are reset.

     \return an element of type U that sums up the given CNF formula using a
     DPLL style algorithm with an operation manager.
  */
  void compute(std::vector<Var> &setOfVar, std::ostream &out,
               MinSharpSatResult &result, bool warmStart = true) {
    if (m_problem->isUnsat() || (warmStart && !m_panicMode &&
                                 !m_solver->warmStart(29, 11, setOfVar, m_out)))
      result.count = T(0);

    DataBranch<T> b;
    searchMinValuation(setOfVar, b.unitLits, b.freeVars, out, result);
    result.count *= m_problem->computeWeightUnitFree<T>(b.unitLits, b.freeVars);
  } // compute

public:
  /**
   * @brief Search for the instanciation of the variables of
   * m_problem->getMaxVar() that maximize the number of the remaining variables
   * where the variables not belonging to m_problem->getIndVar() are
   * existantially quatified.
   *
   * @param[in] vm, the set of options.
   */
  void run(po::variables_map &vm) {
    std::vector<Var> setOfVar;
    for (int i = 1; i <= m_specs->getNbVariable(); i++)
      setOfVar.push_back(i);

    MinSharpSatResult result;
    compute(setOfVar, m_out, result);
    printFinalStats(m_out);
    std::cout << "v ";
    for (unsigned i = 0; i < m_problem->getMaxVar().size(); i++)
      std::cout << ((result.valuation[i]) ? "" : "-")
                << m_problem->getMaxVar()[i] << " ";
    std::cout << "0\n";

    std::cout << "s " << result.count << "\n";
  } // run
};
} // namespace d4
