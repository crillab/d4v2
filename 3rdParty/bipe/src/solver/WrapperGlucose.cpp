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
// #include <bits/stdint-uintn.h>
#include "src/solver/WrapperGlucose.hpp"

#include <iostream>
#include <typeinfo>
#include <vector>

#include "3rdParty/glucose-3.0/core/Solver.h"
#include "3rdParty/glucose-3.0/core/SolverTypes.h"
#include "3rdParty/glucose-3.0/mtl/Vec.h"
#include "src/utils/Problem.hpp"
#include "src/utils/ProblemTypes.hpp"

namespace bipe {

/**
 * @brief Destroy the Wrapper Glucose:: Wrapper Glucose object.
 *
 */
WrapperGlucose::~WrapperGlucose() {}

/**
   This function initializes the SAT solver with a given problem.  Warning: we
   suppose that p is a CNF, otherwise a bad_cast exception is threw.

   @param[in] p, the problem we want to link with the SAT solver.
 */
void WrapperGlucose::initSolver(Problem &pcnf) {
  try {
    // force Glucose_bipe to be in incremental mode in order to restart just
    // after the assumptions. s.setIncrementalMode();

    // say to the solver we have pcnf.getNbVar() variables.
    while ((unsigned)s.nVars() <= pcnf.getNbVar()) s.newVar();
    m_model.resize(pcnf.getNbVar() + 1, l_Undef);

    // load the clauses
    std::vector<std::vector<Lit>> &clauses = pcnf.getClauses();
    for (auto &cl : clauses) {
      Glucose::vec<Glucose::Lit> lits;
      for (auto &l : cl) lits.push(Glucose::mkLit(l.var(), l.sign()));
      s.addClause(lits);
    }
  } catch (std::bad_cast &bc) {
    std::cerr << "bad_cast caught: " << bc.what() << '\n';
    std::cerr << "A CNF formula was expeted\n";
  }

  m_activeModel = false;
  m_needModel = false;
  setNeedModel(m_needModel);
  m_isInAssumption.resize(pcnf.getNbVar() + 1, 0);
}  // initSolver

/**
 * @brief Display a the formual into the stream following the DIMACS format.
 *
 * @param out is the stream.
 */
void WrapperGlucose::displayToCnf(std::ostream &out) {
  unsigned limitUnit = s.trail_lim.size() ? s.trail_lim[0] : 0;
  if (!s.decisionLevel()) limitUnit = s.trail.size();

  out << "p cnf " << s.nVars() << " " << s.clauses.size() + limitUnit << "\n";
  for (int i = 0; i < limitUnit; i++) out << litToInt(s.trail[i]) << " 0\n";

  for (int i = 0; i < s.clauses.size(); i++) {
    Glucose::Clause &c = s.ca[s.clauses[i]];
    for (int j = 0; j < c.size(); j++) out << litToInt(c[j]) << " ";
    out << "0\n";
  }
}  // displayToCnf

/**
 * @brief Strong assumption in the sense we push the literal on the stack.
 *
 * @param l the literal we want to push.
 */
void WrapperGlucose::uncheckedEnqueue(Lit l) {
  s.uncheckedEnqueue(Glucose::mkLit(l.var(), l.sign()));
}  // uncheckedEnqueue

/**
   Call the SAT solver and return its result.

   \return true if the problem is SAT, false otherwise.
 */
bool WrapperGlucose::solve() {
  s.rebuildWithAllVar();
  return s.solveWithAssumptions();
}  // solve

/**
 * @brief Add a clause in the solver.
 *
 * @param cl is the clause we add.
 */
void WrapperGlucose::addClauseInit(std::vector<Lit> &cl) {
  Glucose::vec<Glucose::Lit> addCl;
  for (auto &l : cl) addCl.push(Glucose::mkLit(l.var(), l.sign()));
  s.addClauseInit(addCl);
}  // addClauseInit

/**
 * @brief Set the Last Index Assumption object
 *
 * @param lastIndex is the value.
 */
void WrapperGlucose::setLastIndexAssumption(int lastIndex) {
  s.limitSelector = lastIndex;
}  // setLastIndexAssumption

/**
   Calls the SAT solver in a limited fashion and returns its result.

   \return true SAT if the problem is satisfiable, UNS if the problem is
   unsatisfiable and UND if the solver stops before finding the problem
   satifiability.
 */
Status WrapperGlucose::solveLimited(int nbConflict) {
  s.rebuildOrderHeap();
  Glucose::lbool res = s.solveLimited(nbConflict);
  if (res == Glucose::l_False) return UNS;
  if (res == Glucose::l_True) return SAT;
  return UND;
}  // solveLimited

/**
   Print out the trail on the standard output.
 */
void WrapperGlucose::showTrail() { s.showTrail(); }  // showTrail

/**
 * @brief Set the reverse polarity flag to the solver.
 *
 * @param value is the value we want to assign.
 */
void WrapperGlucose::setReversePolarity(bool value) {
  s.reversePolarity = value;
}  // setReversePolarity

/**
   Collect the unit literal from the affectation of the literal l to the
   formula.

   @param[in] l, the literal we want to branch on.
   @param[out] units, the unit literals

   \return true if assign l and propagate does not give a conflict, false
   otherwise.
 */
bool WrapperGlucose::decideAndComputeUnit(Lit l, std::vector<Lit> &units) {
  Glucose::Lit ml = Glucose::mkLit(l.var(), l.sign());
  if (varIsAssigned(l.var())) {
    if (s.litAssigned(l.var()) != ml) return false;
    units.push_back(l);
    return true;
  }

  int posTrail = (s.trail).size();
  s.newDecisionLevel();
  s.uncheckedEnqueue(ml);
  Glucose::CRef confl = s.propagate();

  if (confl != Glucose::CRef_Undef)  // unit literal
  {
    int bt;
    Glucose::vec<Glucose::Lit> learnt_clause;
    s.analyzeLastUIP(confl, learnt_clause, bt);
    s.cancelUntil(s.decisionLevel() - 1);
    assert(learnt_clause[0] == Glucose::mkLit(l.var(), !l.sign()));
    s.insertClauseAndPropagate(learnt_clause);
    return false;
  }

  for (int j = posTrail; j < s.trail.size(); j++)
    units.push_back(Lit::makeLit(var(s.trail[j]), sign(s.trail[j])));
  s.cancelUntil(s.decisionLevel() - 1);
  return true;
}  // decideAndComputeUnit

/**
 * @brief Decide if the list of literals given in parameter conducts to a
 * conflict by applying BCP. If is the case the return a reason that explain why
 * this situation occurs by computing a core.
 *
 * @param lits is the set of literal we want to test.
 * @param core is the core if the problem is UNSAT.
 * @return true if the problem is SAT, false otherwise.
 */
bool WrapperGlucose::decideAndTest(std::vector<Lit> &lits,
                                   std::vector<Lit> &core) {
  for (auto &l : lits) {
    Glucose::Lit ml = Glucose::mkLit(l.var(), l.sign());

    Glucose::CRef confl = Glucose::CRef_Undef;
    if (!s.isAssigned(l.var())) {
      s.newDecisionLevel();
      s.uncheckedEnqueue(ml);
      confl = s.propagate();

      if (confl != Glucose::CRef_Undef)  // conflict
      {
        Glucose::vec<Glucose::Lit> learnt_clause;
        s.analyzeFinal(confl, learnt_clause);

        for (int i = 0; i < learnt_clause.size(); i++) {
          core.push_back(
              Lit::makeLit(var(learnt_clause[i]), sign(learnt_clause[i])));
        }
        s.cancelUntil(0);
        return false;
      }
    } else if (s.litAssigned(l.var()) != ml)  // conflict!
    {
      Glucose::vec<Glucose::Lit> learnt_clause;
      s.analyzeFinal(~ml, learnt_clause);

      for (int i = 0; i < learnt_clause.size(); i++) {
        core.push_back(
            Lit::makeLit(var(learnt_clause[i]), sign(learnt_clause[i])));
      }
      s.cancelUntil(0);
      return false;
    }
  }

  s.cancelUntil(0);
  return true;
}  // decideAndTest

/**
 * @brief Get the list of unit literals that are in the trail (we suppose that
 * the decision level is zero).
 *
 * @param[out] units is the list of unit literals.
 */
void WrapperGlucose::getUnits(std::vector<Lit> &units) {
  for (int i = 0; i < s.trail.size(); i++) {
    Glucose::Lit l = s.trail[i];
    units.push_back(Lit::makeLit(var(l), sign(l)));
  }
}  // getUnits

/**
   Check out if the given variable is assigned or not by the solver.

   @param[in] v, the variable we search for.

   \return true if the variable is assigned, false otherwise.
 */
bool WrapperGlucose::varIsAssigned(Var v) {
  return s.isAssigned(v);
}  // varIsAssigned

/**
   Restart the solver.
 */
void WrapperGlucose::restart() { s.cancelUntil(0); }  // restart

/**
   Transfer to the solver the fact we have a set of assumption variables we want
   to consider.

   @param[in] assums, the set of assumptions
 */
void WrapperGlucose::setAssumption(std::vector<Lit> &assums) {
  // get the place where the assumption are different.
  unsigned i = 0;
  for (; i < assums.size() && i < m_assumption.size(); i++)
    if (assums[i] != m_assumption[i]) break;

  // unset the literal as being in the assumption.
  for (unsigned j = i; j < m_assumption.size(); j++)
    m_isInAssumption[m_assumption[j].var()] = false;

  Glucose::vec<Glucose::Lit> &assumptions = s.assumptions;
  assert(assumptions.size() == m_assumption.size());
  assumptions.shrink_(assumptions.size() - i);
  m_assumption.resize(i);

  (s.cancelUntil)(assumptions.size());
  for (; i < assums.size(); i++) pushAssumption(assums[i]);
}  // setAssumption

/**
   \return the current assumption.

   @param[in] assums, the set of assumptions
 */
std::vector<Lit> &WrapperGlucose::getAssumption() {
  return m_assumption;
}  // getAssumption

/**
   Print out the assumption.

   @param[in] out, the stream where is print the assumption.
 */
void WrapperGlucose::displayAssumption(std::ostream &out) {
  Glucose::vec<Glucose::Lit> &assumptions = s.assumptions;
  for (int i = 0; i < assumptions.size(); i++) {
    Glucose::Lit l = assumptions[i];
    std::cout << (Glucose::sign(l) ? "-" : "") << Glucose::var(l) << " ";
  }
  std::cout << "\n";
}  // displayAssumption

/**
   Ask for the model.

   @param[in] b, a boolean value to ask the solver to get the model.
 */
void WrapperGlucose::setNeedModel(bool b) {
  m_needModel = b;
  s.setNeedModel(b);
}  // setNeedModel

/**
 * @brief Ask the unsat core to the solver.
 *
 * @param core is the core.
 */
void WrapperGlucose::getCore(std::vector<Lit> &core) {
  core.clear();
  for (int i = 0; i < s.conflict.size(); i++)
    core.push_back(Lit::makeLit(var(s.conflict[i]), sign(s.conflict[i])));
}  // getCore

/**
 * @brief Return the model computed by the solver.
 *
 * @return the model's value (lbool).
 */
std::vector<bool> &WrapperGlucose::getModel() {
  assert(m_needModel);
  assert(s.model.size() <= m_model.size());

  for (int i = 0; i < s.model.size(); i++) {
    if (Glucose::toInt(s.model[i]) == 0)
      m_model[i] = true;
    else if (Glucose::toInt(s.model[i]) == 1)
      m_model[i] = false;
  }

  return m_model;
}  // getModel

/**
   Push a new assumption.

   @param[in] l, the literal we want to push.
 */
void WrapperGlucose::pushAssumption(Lit l) {
  Glucose::Lit ml = Glucose::mkLit(l.var(), l.sign());
  m_activeModel = m_activeModel && !s.isAssigned(var(ml));

  (s.assumptions).push(ml);
  m_assumption.push_back(l);
  assert((s.assumptions).size() == m_assumption.size());

  assert(!m_isInAssumption[l.var()]);
  m_isInAssumption[l.var()] = 1 + l.sign();

  if (m_activeModel && m_needModel) {
    m_activeModel = s.litTrueInLastModel(ml);
    if (m_activeModel) {
      assert(s.decisionLevel() == s.assumptions.size() - 1);
      s.newDecisionLevel();
      assert(!s.isAssigned(var(ml)));
      s.uncheckedEnqueue(ml);
      [[maybe_unused]] Glucose::CRef cref = s.propagate();
      assert(cref == Glucose::CRef_Undef);
    }
  }
}  // pushAssumption

/**
   Remove the last assumption and cancelUntil.

   @param[in] count, the number of element we pop.
 */
void WrapperGlucose::popAssumption(unsigned count) {
  for (unsigned i = m_assumption.size() - count; i < m_assumption.size(); i++) {
    assert(m_isInAssumption[m_assumption[i].var()]);
    m_isInAssumption[m_assumption[i].var()] = 0;
  }
  m_assumption.resize(m_assumption.size() - count);
  (s.assumptions).shrink_(count);
  (s.cancelUntil)((s.assumptions).size());
}  // popAssumption

/**
 * @brief Specify to the solver that some variables should be considered in
 * priority.
 *
 * @param priority is the set of variables we want to branch on in priority.
 */
void WrapperGlucose::oncePriorityVar(std::vector<Var> &priority) {
  s.priorityVar.clear();
  for (auto v : priority) s.priorityVar.push(v);
}

/**
 * @brief Notify the solver that we will deal with the given set of variables.
 *
 * @param heapVar is the set of variables the solver can decide on.
 */
void WrapperGlucose::setHeapVariable(std::vector<Var> &heapVar) {
  s.problemVariable.clear();
  for (auto v : heapVar) s.problemVariable.push(v);
}  // oncePriorityVar

/**
 * @brief Restart the assumption.
 *
 */
void WrapperGlucose::cleanAssumption() {
  for (auto &l : m_assumption) m_isInAssumption[l.var()] = false;

  m_assumption.clear();
  s.assumptions.clear();
  s.cancelUntil(0);
}  // cleanAssumption

/**
 * @brief Notify the solver that it should stop.
 *
 */
void WrapperGlucose::interrupt() { s.interrupt(); }

}  // namespace bipe