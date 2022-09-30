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

#include "WrapperGlucose.hpp"

#include <bits/stdint-uintn.h>

#include <iostream>
#include <typeinfo>
#include <vector>

#include "3rdParty/glucose-3.0/core/Solver.h"
#include "3rdParty/glucose-3.0/core/SolverTypes.h"
#include "3rdParty/glucose-3.0/mtl/Vec.h"
#include "src/problem/ProblemTypes.hpp"
#include "src/problem/cnf/ProblemManagerCnf.hpp"

namespace d4 {
/**
   This function initializes the SAT solver with a given problem.  Warning: we
   suppose that p is a CNF, otherwise a bad_cast exception is threw.

   @param[in] p, the problem we want to link with the SAT solver.
 */
void WrapperGlucose::initSolver(ProblemManager &p) {
  try {
    ProblemManagerCnf &pcnf = dynamic_cast<ProblemManagerCnf &>(p);

    // force glucose to be in incremental mode in order to restart just after
    // the assumptions.
    // s.setIncrementalMode();

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
  m_isInAssumption.resize(p.getNbVar() + 1, 0);
}  // initSolver

/**
   Call the SAT solver and return its result.

   @param[in] setOfvar, the variables we focus the solving process.

   \return true if the problem is SAT, false otherwise.
 */
bool WrapperGlucose::solve(std::vector<Var> &setOfVar) {
  if (m_activeModel && m_needModel) return true;

  m_setOfVar_m.setSize(0);
  for (auto &v : setOfVar) m_setOfVar_m.push(v);
  s.rebuildWithConnectedComponent(m_setOfVar_m);

  m_activeModel = s.solveWithAssumptions();
  return m_activeModel;
}  // solve

/**
 * @brief Strong assumption in the sense we push the literal on the stack.
 *
 * @param l the literal we want to push.
 */
void WrapperGlucose::uncheckedEnqueue(Lit l) {
  s.uncheckedEnqueue(Glucose::mkLit(l.var(), l.sign()));
}  // uncheckedEnqueue

/**
 * @brief Enforce the unit propagation of all the assumption literals.
 *
 * @return true if we did not reach a conflict, false otherwise.
 */
bool WrapperGlucose::propagateAssumption() {
  return false;
}  // propagateAssumption

/**
   Call the SAT solver and return its result.

   \return true if the problem is SAT, false otherwise.
 */
bool WrapperGlucose::solve() {
  s.rebuildWithAllVar();
  return s.solveWithAssumptions();
}  // solve

/**
   An accessor on the activity of a variable.

   @param[in] v, the variable we want the activity.

   \return the activity of v.
 */
double WrapperGlucose::getActivity(Var v) {
  return s.activity[v];
}  // getActivity

/**
   Return the number of times the variable v occurs in a conflict.

   @param[in] v, the variable we want the activity.

   \return the number of times v occurs in a conflict.
 */
double WrapperGlucose::getCountConflict(Var v) {
  return s.scoreActivity[v];
}  // getCountConflict

/**
 * @brief Set the count conflict in the solver.
 * @param[in] v is the variable we want to set the counter of  conflicts.
 * @param[in] count is the count we want to assign.
 */
void WrapperGlucose::setCountConflict(Var v, double count) {
  s.scoreActivity[v] = count;
}  // setCountConflict

/**
   Print out the trail on the standard output.
 */
void WrapperGlucose::showTrail() { s.showTrail(); }  // showTrail

/**
   An accessor on the polarity of a variable.

   @param[in] v, the variable we want the polarity.
 */
bool WrapperGlucose::getPolarity(Var v) {
  return s.polarity[v];
}  // getPolarity

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
   Fill the vector units with the literal l that are units such that l.var() is
   in component.

   @param[in] component, the set of variables we search for.
   @param[out] units, the place where we store the literals found.
 */
void WrapperGlucose::whichAreUnits(std::vector<Var> &component,
                                   std::vector<Lit> &units) {
  for (auto &v : component) {
    if (!s.isAssigned(v)) continue;
    Glucose::Lit l = s.litAssigned(v);
    units.push_back(Lit::makeLit(var(l), sign(l)));
  }
}  // whichAreUnits

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
  popAssumption(m_assumption.size());
  Glucose::vec<Glucose::Lit> &assumptions = s.assumptions;
  assumptions.clear();
  m_assumption.clear();
  for (auto &l : assums) pushAssumption(l);
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
 * @brief Return the model computed by the solver.
 *
 * @return the model's value (lbool).
 */
std::vector<lbool> &WrapperGlucose::getModel() {
  for (int i = 0; i < s.model.size(); i++) {
    if (Glucose::toInt(s.model[i]) == 0)
      m_model[i] = l_True;
    else if (Glucose::toInt(s.model[i]) == 1)
      m_model[i] = l_False;
    else
      m_model[i] = l_Undef;
  }

  return m_model;
}  // getModel

/**
 * @brief Get the value given by the last computed model.
 *
 * @param v is the variable we want to get the assignment.
 * @return the last value of v.
 */
lbool WrapperGlucose::getModelVar(Var v) {
  return Glucose::toInt(s.model[v]);
}  // getModelVar

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

inline unsigned WrapperGlucose::getNbConflict() { return s.conflicts; }

}  // namespace d4