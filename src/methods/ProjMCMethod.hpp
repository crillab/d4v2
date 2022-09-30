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
#include <unordered_map>

#include "MethodManager.hpp"
#include "src/methods/Counter.hpp"
#include "src/preprocs/PreprocManager.hpp"
#include "src/problem/ProblemManager.hpp"
#include "src/problem/cnf/ProblemManagerCnf.hpp"

namespace d4 {
namespace po = boost::program_options;

template <class T>
class ProjMCMethod : public MethodManager {
 private:
  struct CoupleNotProjClauseSelector {
    std::vector<Lit> clause;
    Lit selector;

    void display() {
      std::cout << selector << " : ";
      for (const auto &l : clause) std::cout << l << " ";
      std::cout << "\n";
    }
  };

  // constants
  const unsigned c_NB_SEP = 92;
  const long unsigned c_MASK_HEADER = 1048575;
  const long unsigned c_MASK_SHOWRUN = ((2 << 13) - 1);
  const unsigned c_WIDTH_PRINT_COLUMN = 12;

  ProblemManager *m_problem;
  std::ostream m_out;
  std::ostream m_outCounter;

  std::vector<Lit> m_selectors;
  std::vector<std::vector<Lit>> m_selectorToProjClause;
  std::vector<std::vector<unsigned>> m_selectorToNProjClause;
  std::vector<bool> m_isProjectedVar;
  std::vector<bool> m_isSelector;
  std::vector<int> m_marked;
  std::vector<bool> m_flag;
  std::unordered_map<std::string, Lit> mapProjClSelector;
  std::vector<CoupleNotProjClauseSelector> notProjClauses;

  SpecManager *m_specs;
  WrapperSolver *m_solver;
  Counter<T> *m_counter;
  CacheManager<T> *m_cache;
  LastBreathPreproc m_lastBreath;

  long unsigned m_nbCallRec;
  long unsigned m_nbSplit;

  bool m_refinement;

 public:
  /**
     Constructor.

     @param[in] vm, the list of options.
   */
  ProjMCMethod(po::variables_map &vm, bool isFloat, ProblemManager *initProblem,
               LastBreathPreproc &lastBreath)
      : m_problem(initProblem), m_out(nullptr), m_outCounter(nullptr) {
    m_nbCallRec = m_nbSplit = 0;
    m_lastBreath = lastBreath;

    // init the output stream
    m_out.copyfmt(std::cout);
    m_out.clear(std::cout.rdstate());
    m_out.basic_ios<char>::rdbuf(std::cout.rdbuf());

    // mark the projected variables.
    m_isProjectedVar.resize(m_problem->getNbVar() + 1, false);
    m_isSelector.resize(m_problem->getNbVar() + 1, false);
    for (auto v : m_problem->getSelectedVar()) m_isProjectedVar[v] = true;

    m_refinement = vm["projMC-refinement"].as<bool>();
    m_out << "c [CONSTRUCTOR] ProjMCMethod: refinement(" << m_refinement
          << ")\n";

    std::vector<std::vector<Lit>> projClause, nprojClause, mix;
    partitionFormula(m_problem, m_isProjectedVar, projClause, nprojClause, mix);

    // split the mixed clauses.
    unsigned idxVar = m_problem->getNbVar() + 1;
    manageMixedClauses(mix, m_isProjectedVar, projClause, nprojClause,
                       m_selectors, idxVar);

    // prepare the SAT solver.
    std::vector<std::vector<Lit>> satSolverClauses = projClause;
    for (auto &cl : nprojClause) satSolverClauses.push_back(cl);
    initSatSolver(vm, m_problem, satSolverClauses, idxVar - 1);

    // prepare the cache.
    m_cache = CacheManager<T>::makeCacheManager(vm, idxVar - 1, m_specs, m_out);

    // init the clock time.
    initTimer();

    // update the last breath structure with the additional variables.
    m_lastBreath.fitSizeCountConflict(idxVar);

    // prepare the counter.
    initCounter(vm, m_problem, isFloat, projClause, idxVar - 1);
    m_marked.resize(idxVar + 1, -1);
    m_flag.resize((idxVar + 1) << 1, false);
  }  // constructor

  /**
     Destructor.
   */
  ~ProjMCMethod() {
    delete m_counter;
    delete m_solver;
    delete m_specs;
    delete m_cache;
    delete m_problem;
  }  // destructor

 private:
  /**
     Init the counter with the projected clauses.

     @param[in] vm, the option to create the SAT solver.
     @param[in] problem, the input problem (only used to get information about
     weight).
     @param[in] isFloat, specify if the weight are float or int.
     @param[in] clauses, the set of clauses.
     @param[in] nbVar, the number of variables of the formula.
   */
  void initCounter(po::variables_map &vm, ProblemManager *problem, bool isFloat,
                   std::vector<std::vector<Lit>> &clauses, unsigned nbVar) {
    int precision = vm["float-precision"].as<int>();
#if DEBUG
    m_outCounter.copyfmt(m_out);
    m_outCounter.clear(m_out.rdstate());
    m_outCounter.basic_ios<char>::rdbuf(m_out.rdbuf());
#else
    m_outCounter.setstate(std::ios_base::badbit);
#endif
    // init the problem we will pass to the counter.
    std::vector<double> weightLit((nbVar + 1) << 1, 1);
    std::vector<double> weightVar(nbVar + 1, 2);

    std::vector<double> &problemWeightLit = problem->getWeightLit();
    for (unsigned i = 0; i < problemWeightLit.size(); i++)
      weightLit[i] = problemWeightLit[i];

    std::vector<double> &problemWeightVar = problem->getWeightVar();
    for (unsigned i = 0; i < problemWeightVar.size(); i++) {
      if (m_isProjectedVar[i])
        weightVar[i] = problemWeightVar[i];
      else
        weightVar[i] = 1;
    }

    std::vector<Var> emptySelectedVar;
    ProblemManagerCnf *p =
        new ProblemManagerCnf(nbVar, weightLit, weightVar, emptySelectedVar);
    p->setClauses(clauses);

    // create the counter.
    m_out << "c [CONSTRUCTOR] Create an external counter: "
          << "counting"
          << "\n";
    m_counter = Counter<T>::makeCounter(vm, p, "counting", isFloat, precision,
                                        m_outCounter, m_lastBreath);
  }  // initCounter

  /**
     Init the SAT solver with a set of clauses (actually two sets).  Only deals
     with CNF formula.

     @param[in] vm, the option to create the SAT solver.
     @param[in] problem, the input problem (only used to get information about
     weight).
     @param[in] clauses, the set of clauses.
     @param[in] nbVar, the number of variables of the formula.
   */
  void initSatSolver(po::variables_map &vm, ProblemManager *problem,
                     std::vector<std::vector<Lit>> &clauses, unsigned nbVar) {
    m_solver = WrapperSolver::makeWrapperSolver(vm, m_out);
    assert(m_solver);

    // prepare the weight vectors and init the problem.
    std::vector<double> weightLit((nbVar + 1) << 1, 1);
    std::vector<double> weightVar(nbVar + 1, 2);

    std::vector<double> &problemWeightLit = problem->getWeightLit();
    for (unsigned i = 0; i < problemWeightLit.size(); i++)
      weightLit[i] = problemWeightLit[i];

    std::vector<double> &problemWeightVar = problem->getWeightVar();
    for (unsigned i = 0; i < problemWeightVar.size(); i++)
      weightVar[i] = problemWeightVar[i];

    ProblemManagerCnf p(nbVar, weightLit, weightVar, problem->getSelectedVar());
    p.setClauses(clauses);
    m_solver->initSolver(p);
    m_solver->setCountConflict(m_lastBreath.countConflict, 1,
                               m_problem->getNbVar());

    // ask for the witness.
    m_solver->setNeedModel(true);

    // prepare the spec manager.
    m_specs = SpecManager::makeSpecManager(vm, p, m_out);
  }  // initSatSolver

  /**
     Partition the formula in tree sets regarding the the clauses contain or not
     projected variable.

     @param[in] prolem, the problem we want to partition.
     @param[in] isProjectvar, boolean vector that specifies the projected
     variables.
     @param[out] projClause, the clauses that only contain projected variable.
     @param[out] nprojClause, the clauses that do not contain projected
     variable.
     @param[out] mix, the clauses that contain both projected variable and not.
     projected variables.
   */
  void partitionFormula(ProblemManager *problem,
                        std::vector<bool> &isProjectedVar,
                        std::vector<std::vector<Lit>> &projClause,
                        std::vector<std::vector<Lit>> &nprojClause,
                        std::vector<std::vector<Lit>> &mix) {
    ProblemManagerCnf *cnf = static_cast<ProblemManagerCnf *>(problem);
    std::vector<std::vector<Lit>> &clauses = cnf->getClauses();

    for (auto &cl : clauses) {
      unsigned nbp = 0, nbn = 0;
      for (auto l : cl) {
        if (isProjectedVar[l.var()])
          nbp++;
        else
          nbn++;
        if (nbp && nbn) break;
      }

      if (nbp && !nbn)
        projClause.push_back(cl);
      else if (!nbp && nbn)
        nprojClause.push_back(cl);
      else
        mix.push_back(cl);
    }
  }  // partitionFormula

  /**
     Manage the mixed clauses by adding a selector in order to seperate each
     clause between projected and not projected variables.

     @param[int] mix, the clauses that contain both projected variable and not.
     projected variables.
     @param[in] isProjectvar, boolean vector that specifies the projected
     variables.
     @param[out] projClause, the clauses that only contain projected variable.
     @param[out] nprojClause, the clauses that do not contain projected
     variable.
     @param[out] nextVar, the index of the next available variable.

     @param[out] selectors, the added selectors literals (that can be used to
     activate the projected part - ¬s v C).
   */
  void manageMixedClauses(std::vector<std::vector<Lit>> &mix,
                          std::vector<bool> &isProjectedVar,
                          std::vector<std::vector<Lit>> &projClause,
                          std::vector<std::vector<Lit>> &nprojClause,
                          std::vector<Lit> &selectors, unsigned &nextVar) {
    for (auto &cl : mix) {
      std::vector<Lit> clp, clnp;
      std::string key = "";
      for (auto &l : cl)
        if (!isProjectedVar[l.var()])
          clnp.push_back(l);
        else {
          key += "." + std::to_string(l.intern());
          clp.push_back(l);
        }

      Lit s = lit_Undef;
      if (mapProjClSelector.count(key) > 0)
        s = mapProjClSelector[key];
      else {
        s = Lit::makeLitTrue(nextVar++);
        if (s.var() >= (int)m_isSelector.size())
          m_isSelector.resize(s.var() + 1, false);
        m_isSelector[s.var()] = true;

        mapProjClSelector[key] = s;

        if ((int)m_selectorToProjClause.size() <= s.var()) {
          m_selectorToProjClause.resize(s.var() + 1, std::vector<Lit>());
          m_selectorToNProjClause.resize(s.var() + 1, std::vector<unsigned>());
        }
        m_selectorToProjClause[s.var()] = clp;

        clp.push_back(s);
        projClause.push_back(clp);

        m_isProjectedVar.resize(nextVar);
        m_isProjectedVar[s.var()] = false;
      }

      // link the selector to the not projected part of the considered clause.
      m_selectorToNProjClause[s.var()].push_back(notProjClauses.size());
      notProjClauses.push_back({clnp, s});
      clnp.push_back(~s);
      nprojClause.push_back(clnp);
    }
  }  // manageMixedClauses

  /**
     Extract the selector that correspond to the non projected clauses that are
     falsified by a given interpretation.

     @param[in] model, the interpretation we check.
     @param[out] selector, the returned selector.
   */
  void extractSelectorFalsifiedNProj(std::vector<Var> &setOfVar,
                                     std::vector<lbool> &model,
                                     std::vector<Lit> &selector) {
    for (auto &v : setOfVar) {
      if (!m_isSelector[v]) continue;
      if (m_solver->isInAssumption(v)) continue;

      bool allSAT = true;
      for (auto idx : m_selectorToNProjClause[v]) {
        const std::vector<Lit> &cl = notProjClauses[idx].clause;
        bool isSAT = false;
        for (auto &l : cl) {
          if (model[l.var()] == l_Undef)
            continue;
          else if (l.sign())
            isSAT = model[l.var()] == l_False;
          else
            isSAT = model[l.var()] == l_True;

          if (isSAT) break;
        }

        if (!(allSAT = isSAT)) break;
      }

      selector.push_back(Lit::makeLit(v, !allSAT));
    }
  }  // extractSelectorFalsifiedNProj

  /**
     Expel from a list of variables, and a list of literals, the elements that
     are not belonging to the projected set of variables.

     @param[out] litList, the list of literals we want to refine.
     @param[out] varList, the list of variables want to refine.
   */
  void expelNoProjectedElement(std::vector<Lit> &litList,
                               std::vector<Var> &varList) {
    // only keep the projected.
    unsigned j = 0;
    for (unsigned i = 0; i < litList.size(); i++)
      if (m_isProjectedVar[litList[i].var()]) litList[j++] = litList[i];
    litList.resize(j);

    j = 0;
    for (unsigned i = 0; i < varList.size(); i++)
      if (m_isProjectedVar[varList[i]]) varList[j++] = varList[i];
    varList.resize(j);
  }  // expelNoProjectedElement

  /**
     Try to reduce the number of falsified selector.

     @param[in] setOfVar, the set of variableswe consider.
     @param[out] selector, the selector list we try to reduce.
   */
  void refine(std::vector<Var> &setOfVar, std::vector<Lit> &selector) {
    unsigned initSizeAssumption = m_solver->sizeAssumption();

    // split between true and false selector.
    std::vector<Lit> falseSelector;
    unsigned j = 0;
    for (unsigned i = 0; i < selector.size(); i++) {
      Lit l = selector[i];
      if (l.sign())
        falseSelector.push_back(l);
      else {
        selector[j++] = selector[i];
        assert(!m_solver->isInAssumption(l.var()));
        m_solver->pushAssumption(l);
      }
    }
    selector.resize(j);

    // reintegrate false selector regarding if it is possible to flip some.
    for (unsigned i = 0; i < falseSelector.size(); i++) {
      Lit l = falseSelector[i];
      if (m_solver->isInAssumption(l.var())) {
        selector.push_back(m_solver->isInAssumption(l) ? l : ~l);
        continue;
      }

      m_solver->pushAssumption(~l);
      if (m_solver->solve(setOfVar)) {
        std::vector<lbool> &model = m_solver->getModel();
        selector.push_back(~l);

        unsigned kj = i + 1;
        for (unsigned ki = i + 1; ki < falseSelector.size(); ki++) {
          bool allSAT = true;
          for (auto idx : m_selectorToNProjClause[falseSelector[ki].var()]) {
            const std::vector<Lit> &cl = notProjClauses[idx].clause;
            bool isSAT = false;
            for (auto &kl : cl) {
              if (model[kl.var()] == l_Undef)
                continue;
              else if (kl.sign())
                isSAT = model[kl.var()] == l_False;
              else
                isSAT = model[kl.var()] == l_True;

              if (isSAT) break;
            }

            if (!(allSAT = isSAT)) break;
          }

          if (allSAT) {
            selector.push_back(~falseSelector[ki]);
            assert(!m_solver->isInAssumption(falseSelector[ki]));
            if (!m_solver->isInAssumption(~falseSelector[ki]))
              m_solver->pushAssumption(~falseSelector[ki]);
          } else
            falseSelector[kj++] = falseSelector[ki];
        }
        falseSelector.resize(kj);
      } else {
        selector.push_back(l);
        m_solver->popAssumption(1);
        m_solver->pushAssumption(l);
      }
    }

    m_solver->popAssumption(m_solver->sizeAssumption() - initSizeAssumption);
  }  // refine

  /**
     Compute the number of model on the projected variables.

     @param[in] setOfVar, the set of variableswe consider.
     @param[in] out, the stream where are printed the logs.

     \return the number of models.
   */
  T compute_(std::vector<Var> &setOfVar, std::ostream &out) {
    showRun(out);
    m_nbCallRec++;
    // if(m_nbCallRec > 100000) exit(0);

    unsigned initSizeAssumption = m_solver->sizeAssumption();
    if (!m_solver->solve(setOfVar)) return T(0);

    // collect unit literals
    std::vector<Lit> unitLits;
    m_solver->whichAreUnits(setOfVar, unitLits);
    m_specs->preUpdate(unitLits);

    // add the unit in the assumption.
    for (auto &l : unitLits) {
      assert(!m_solver->isInAssumption(~l));
      if (!m_solver->isInAssumption(l)) m_solver->pushAssumption(l);
    }

    std::vector<Var> reallyPresent, freeVariable;
    std::vector<std::vector<Var>> varConnected;
    int nbComponent = m_specs->computeConnectedComponent(varConnected, setOfVar,
                                                         freeVariable);

    // extract the really present variables.
    reallyPresent.reserve(setOfVar.size() - freeVariable.size());
    for (auto &l : varConnected)
      for (auto &v : l) reallyPresent.push_back(v);

    // extract the projected variables.
    std::vector<Var> projectSetOfVar;
    for (auto &v : reallyPresent)
      if (m_isProjectedVar[v]) projectSetOfVar.push_back(v);

    T ret = 1;
    if (projectSetOfVar.size()) {
      if (nbComponent > 1) {
        // consider each component independently
        for (auto &component : varConnected) ret *= compute_(component, out);
        m_nbSplit += nbComponent;
      } else if (nbComponent == 1) {
        // collect the selectors of the unsatisfied non projected clauses.
        std::vector<Lit> selector;
        extractSelectorFalsifiedNProj(reallyPresent, m_solver->getModel(),
                                      selector);
        if (m_refinement) refine(reallyPresent, selector);

        TmpEntry<T> cb = m_cache->searchInCache(reallyPresent);
        if (cb.defined)
          ret = cb.getValue();
        else {
          // prepare the next assumption.
          std::vector<Lit> nextAssums(m_solver->getAssumption());
          for (auto &s : selector) {
            assert(!m_solver->isInAssumption(s.var()));
            if (!m_solver->isInAssumption(s)) nextAssums.push_back(s);
          }
          ret = m_counter->count(reallyPresent, nextAssums, m_outCounter);

          // reajust the selectors by only keeping the negative.
          unsigned j = 0;
          for (unsigned i = 0; i < selector.size(); i++)
            if (selector[i].sign()) selector[j++] = selector[i];
          selector.resize(j);

          for (auto &s : selector) {
            unsigned countPushInAssumption = 0;
            auto &cl = m_selectorToProjClause[s.var()];
            bool isUnsat = false;
            for (auto &l : cl) {
              if (m_solver->isInAssumption(l)) {
                isUnsat = true;
                break;
              }
              if (!m_solver->isInAssumption(~l)) {
                m_solver->pushAssumption(~l);
                countPushInAssumption++;
              }
            }

            if (!isUnsat) ret += compute_(reallyPresent, out);
            m_solver->popAssumption(countPushInAssumption);
            if (m_solver->isInAssumption(~s)) break;  // UNSAT.
            if (!m_solver->isInAssumption(s)) m_solver->pushAssumption(s);
          }

          m_solver->popAssumption(selector.size());
          m_cache->addInCache(cb, ret);
        }
      }
    }

    m_specs->postUpdate(unitLits);
    expelNoProjectedElement(unitLits, freeVariable);

    m_solver->popAssumption(m_solver->sizeAssumption() - initSizeAssumption);
    return ret * m_problem->computeWeightUnitFree<T>(unitLits, freeVariable);
  }  // compute_

  /**
     Prepare the computed process.

     @param[in] out, the stream used to print the information.
   */
  T compute(std::ostream &out) {
    std::vector<Var> setOfVar;
    for (int i = 1; i <= m_specs->getNbVariable(); i++) setOfVar.push_back(i);
    return compute_(setOfVar, out);
  }  // compute

  /**
   Print out information about the solving process.

   @param[in] out, the stream we use to print out information.
*/
  inline void showInter(std::ostream &out) {
    out << "c [PROJMC] "
        << "|" << std::setw(c_WIDTH_PRINT_COLUMN) << m_nbCallRec << std::fixed
        << std::setprecision(2) << "|" << std::setw(c_WIDTH_PRINT_COLUMN)
        << getTimer() << "|" << std::setw(c_WIDTH_PRINT_COLUMN)
        << m_cache->getNbPositiveHit() << "|" << std::setw(c_WIDTH_PRINT_COLUMN)
        << m_cache->getNbNegativeHit() << "|" << std::setw(c_WIDTH_PRINT_COLUMN)
        << m_cache->usedMemory() << "|" << std::setw(c_WIDTH_PRINT_COLUMN)
        << m_nbSplit << "|" << std::setw(c_WIDTH_PRINT_COLUMN)
        << MemoryStat::memUsedPeak() << "|\n";
  }  // showInter

  /**
     Print out a line of dashes.

     @param[in] out, the stream we use to print out information.
   */
  inline void separator(std::ostream &out) {
    out << "c [PROJMC] ";
    for (unsigned i = 0; i < c_NB_SEP; i++) out << "-";
    out << "\n";
  }  // separator

  /**
     Print out the header information.

     @param[in] out, the stream we use to print out information.
  */
  inline void showHeader(std::ostream &out) {
    separator(out);
    out << "c [PROJMC] "
        << "|" << std::setw(c_WIDTH_PRINT_COLUMN) << "#rec. call"
        << "|" << std::setw(c_WIDTH_PRINT_COLUMN) << "time"
        << "|" << std::setw(c_WIDTH_PRINT_COLUMN) << "#posHit"
        << "|" << std::setw(c_WIDTH_PRINT_COLUMN) << "#negHit"
        << "|" << std::setw(c_WIDTH_PRINT_COLUMN) << "memory"
        << "|" << std::setw(c_WIDTH_PRINT_COLUMN) << "#split"
        << "|" << std::setw(c_WIDTH_PRINT_COLUMN) << "mem(MB)"
        << "|\n";
    separator(out);
  }  // showHeader

  /**
     Print out information when it is requiered.

     @param[in] out, the stream we use to print out information.
   */
  inline void showRun(std::ostream &out) {
    if (!(m_nbCallRec & (c_MASK_HEADER))) showHeader(out);
    if (m_nbCallRec && !(m_nbCallRec & c_MASK_SHOWRUN)) showInter(out);
  }  // showRun

  /**
     Print out the final stat.

     @param[in] out, the stream we use to print out information.
  */
  inline void printFinalStats(std::ostream &out) {
    out << "c [PROJMC] \n";
    out << "c [PROJMC] \033[1m\033[31mStatistics \033[0m\n";
    out << "c [PROJMC] \033[33mCompilation Information\033[0m\n";
    /*
    out << "c [PROJMC] Number of recursive call: " << nbCallCall << "\n";
    out << "c [PROJMC] Number of split formula: " << nbSplit << "\n";
    out << "c [PROJMC] Number of decision: " << nbDecisionNode << "\n";
    out << "c [PROJMC] Number of paritioner calls: " << callPartitioner << "\n";
    */
    out << "c [PROJMC]\n";
    m_cache->printCacheInformation(out);
    out << "c [PROJMC] Final time: " << getTimer() << "\n";
    out << "c\n";
  }  // printFinalStat

 public:
  /**
     Run the DPLL style algorithm with the operation manager.

     @param[in] vm, the set of options.
   */
  void run(po::variables_map &vm) {
    T res = compute(m_out);
    printFinalStats(m_out);
    std::cout << "s " << res << "\n";
  }  // run
};
}  // namespace d4
