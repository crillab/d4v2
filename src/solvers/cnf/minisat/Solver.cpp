/***************************************************************************************[Solver.cc]
Copyright (c) 2003-2006, Niklas Een, Niklas Sorensson
Copyright (c) 2007-2010, Niklas Sorensson

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
**************************************************************************************************/

#include <iostream>
#include <math.h>

#include "SolverTypes.hpp"
#include "mtl/Alg.hpp"
#include "mtl/Heap.hpp"
#include "mtl/Sort.hpp"
#include "mtl/Vec.hpp"

#include "Solver.hpp"
#include "src/problem/ProblemTypes.hpp"

namespace minisat {
//=================================================================================================
// Constructor/Destructor:
Solver::Solver(std::ostream *certif)
    : // Parameters (user settable):
      //
      cert(certif), verbosity(0), var_decay(0.95), clause_decay(0.99),
      random_var_freq(0), random_seed(91648253), luby_restart(true),
      ccmin_mode(2), phase_saving(2), rnd_pol(false), rnd_init_act(false),
      garbage_frac(0.20), restart_first(100), restart_inc(2)

      // Parameters (the rest):
      //
      ,
      learntsize_factor((double)1 / (double)3), learntsize_inc(1.1)

      // Parameters (experimental):
      ,
      learntsize_adjust_start_confl(100), learntsize_adjust_inc(1.5)

      // Statistics: (formerly in 'SolverStats')
      ,
      solves(0), starts(0), decisions(0), rnd_decisions(0), propagations(0),
      conflicts(0), dec_vars(0), clauses_literals(0), learnts_literals(0),
      max_literals(0), tot_literals(0)

      ,
      ok(true), cla_inc(1), var_inc(1), watches(WatcherDeleted(ca)), qhead(0),
      simpDB_assigns(-1), simpDB_props(0), order_heap(VarOrderLt(activity)),
      progress_estimate(0), remove_satisfied(true)

      // Resource constraints:
      //
      ,
      conflict_budget(-1), propagation_budget(-1), asynch_interrupt(false) {
  limTrailNbSatUns = stampInTheHeap = 0;
  occGtThreeInit = needModel = showDebug = false;
  phantomMode = false;
  phantomLit = lit_Undef;
  idxClausesCpt = 0;
}

Solver::~Solver() {}

//=================================================================================================
// Minor methods:

/**
   Creates a new SAT variable in the solver. If 'decision' is cleared,
   variable will not be used as a decision variable (NOTE! This has
   effects on the meaning of a SATISFIABLE result).

   @param[in] sign,
   @param[in] dvar,

   \return the new variable
*/
Var Solver::newVar(bool sign, bool dvar) {
  int v = nVars();

  watches.init(mkLit(v, false));
  watches.init(mkLit(v, true));

  assigns.push(l_Undef);
  vardata.push(mkVarData(CRef_Undef, 0));
  // activity .push(0);
  scoreActivity.push(1);
  activity.push(rnd_init_act ? drand(random_seed) * 0.00001 : 0);
  seen.push(0);
  polarity.push(sign);
  decision.push();
  trail.capacity(v + 1);

  occGtThree.push();
  occGtThree.push();

  inTheHeap.push(0); // must be added before
  setDecisionVar(v, dvar);
  defined.push(false);
  litFlags.push(lit_Undef);
  flags.push(0);

  problemVariable.push(v);
  model.push(l_Undef);
  saveFree.push(0);
  currentModel.push(l_False);
  insistTruePolarity.push(false);

  return v;
} // newVar

bool Solver::addClause_(vec<Lit> &ps) {
  assert(decisionLevel() == 0);
  if (!ok)
    return false;

  isTautologie = false;

  // Check if clause is satisfied and remove false/duplicate literals:
  sort(ps);
  vec<Lit> oc;

  Lit p;

  int i, j;
  for (i = j = 0, p = lit_Undef; i < ps.size(); i++)
    oc.push(ps[i]);

  for (i = j = 0, p = lit_Undef; i < ps.size(); i++) {
    if (value(ps[i]) == l_True || ps[i] == ~p) {
      isTautologie = true;
      return true;
    } else if (value(ps[i]) != l_False && ps[i] != p)
      ps[j++] = p = ps[i];
  }

  ps.shrink(i - j);

  if ((cert != nullptr)) {
    std::ostream &cval = *cert;
    for (i = j = 0, p = lit_Undef; i < ps.size(); i++)
      cval << (var(ps[i]) + 1) * (-2 * sign(ps[i]) + 1) << " ";
    cval << "0\nd ";
    for (i = j = 0, p = lit_Undef; i < oc.size(); i++)
      cval << (var(oc[i]) + 1) * (-2 * sign(oc[i]) + 1) << " ";
    cval << "0\n";
    idxClausesCpt++;
  }

  if (ps.size() == 0)
    return ok = false;
  else if (ps.size() == 1) {
    uncheckedEnqueue(ps[0]);
    CRef ref = propagate();

    if (cert != nullptr && ref != CRef_Undef) {
      std::ostream &cval = *cert;
      cval << (var(ca[ref][0]) + 1) * (-2 * sign(ca[ref][0]) + 1) << " 0\n";
    }

    return ok = (ref == CRef_Undef);
  } else {
    CRef cr = ca.alloc(ps, false);
    clauses.push(cr);
    attachClause(cr);
    ca[cr].idxReason(idxClausesCpt);
  }

  return true;
} // addClause_

void Solver::collectUnit(vec<Var> &setOfVar, vec<Lit> &unitsLit, Lit dec) {
  unitsLit.clear();
  if (dec != lit_Undef)
    unitsLit.push(dec);

  for (int i = 0; i < setOfVar.size(); i++) {
    Var v = setOfVar[i];
    if (dec != lit_Undef && var(dec) == v)
      continue;
    if (value(v) != l_Undef) {
      unitsLit.push(mkLit(v, value(v) == l_False));
    }
  }
} // collectUnit

void Solver::attachClause(CRef cr) {
  Clause &c = ca[cr];
  assert(!c.attached());
  c.attached(1);
  assert(c.size() > 1);
  watches[~c[0]].push(Watcher(cr, c[1]));
  watches[~c[1]].push(Watcher(cr, c[0]));
  if (c.learnt())
    learnts_literals += c.size();
  else
    clauses_literals += c.size();
} // attachClause

void Solver::detachClause(CRef cr, bool strict) {
  Clause &c = ca[cr];
  assert(c.attached());
  c.attached(0);
  assert(c.size() > 1);

  if (strict) {
    remove(watches[~c[0]], Watcher(cr, c[1]));
    remove(watches[~c[1]], Watcher(cr, c[0]));
  } else {
    // Lazy detaching: (NOTE! Must clean all watcher lists before garbage
    // collecting this clause)
    watches.smudge(~c[0]);
    watches.smudge(~c[1]);
  }

  if (c.learnt())
    learnts_literals -= c.size();
  else
    clauses_literals -= c.size();
}

void Solver::removeClause(CRef cr, bool strict) {
  Clause &c = ca[cr];

  if (cert != nullptr) {
    std::ostream &cval = *cert;
    cval << "d ";
    c.showClause(cval);
  }
  detachClause(cr, strict);

  // Don't leave pointers to free'd memory!
  if (locked(c))
    vardata[var(c[0])].reason = CRef_Undef;
  c.mark(1);
  ca.free(cr);
}

void Solver::removeNotAttachedClause(CRef cr) {
  Clause &c = ca[cr];

  // Don't leave pointers to free'd memory!
  if (locked(c))
    vardata[var(c[0])].reason = CRef_Undef;
  c.mark(1);
  ca.free(cr);
}

bool Solver::satisfied(const Clause &c) const {
  for (int i = 0; i < c.size(); i++)
    if (value(c[i]) == l_True)
      return true;
  return false;
} // satisfied

/**
   Revert to the state at given level (keeping all assignment at
   'level' but not beyond).
*/
void Solver::cancelUntil(int lev) {
  if (decisionLevel() > lev) {
    for (int c = trail.size() - 1; c >= trail_lim[lev]; c--) {
      Var x = var(trail[c]);

      assigns[x] = l_Undef;
      polarity[x] = sign(trail[c]);
      insertVarOrder(x);
    }
    qhead = trail_lim[lev];
    trail.shrink(trail.size() - trail_lim[lev]);
    trail_lim.shrink(trail_lim.size() - lev);
  }
} // cancelUntil

//=================================================================================================
// Major methods:

Lit Solver::pickBranchLit() {
  Var next = var_Undef;

  // Activity based decision:
  while (next == var_Undef || value(next) != l_Undef || !decision[next]) {
    if (order_heap.empty()) {
      next = var_Undef;
      break;
    } else
      next = order_heap.removeMin();
  }

  if (next == minisat::var_Undef)
    return lit_Undef;

  // if (insistTruePolarity[next])
  // return mkLit(next, false);

  if (reversePolarity)
    return mkLit(next, !polarity[next]);
  return mkLit(next, polarity[next]);
} // pickBranchLit

/**
   analyze : (confl : Clause*) (out_learnt : vec<Lit>&) (out_btlevel : int&)  ->
   [void]

   Description:
   Analyze conflict and produce a reason clause.

   Pre-conditions:
   * 'out_learnt' is assumed to be cleared.
   * Current decision level must be greater than root level.

   Post-conditions:
   * 'out_learnt[0]' is the asserting literal at level 'out_btlevel'.
   * If out_learnt.size() > 1 then 'out_learnt[1]' has the greatest decision
   level of the rest of literals. There may be others from the same level
   though.

*/
void Solver::analyze(CRef confl, vec<Lit> &out_learnt, int &out_btlevel) {
  int pathC = 0;
  Lit p = lit_Undef;

  // Generate conflict clause:
  out_learnt.push(); // (leave room for the asserting literal)
  int index = trail.size() - 1;

  do {
    assert(confl != CRef_Undef); // (otherwise should be UIP)
    Clause &c = ca[confl];

    if (c.learnt())
      claBumpActivity(c);

    for (int j = (p == lit_Undef) ? 0 : 1; j < c.size(); j++) {
      Lit q = c[j];

      if (!seen[var(q)] && level(var(q)) > 0) {
        varBumpActivity(var(q));
        // printf("bump %d %lf\n", var(q), activity[var(q)]);
        seen[var(q)] = 1;
        if (level(var(q)) >= decisionLevel())
          pathC++;
        else
          out_learnt.push(q);
      }
    }

    // Select next clause to look at:
    while (!seen[var(trail[index--])])
      ;
    p = trail[index + 1];
    confl = reason(var(p));
    seen[var(p)] = 0;
    pathC--;
  } while (pathC > 0);
  out_learnt[0] = ~p;

  // Simplify conflict clause:
  //
  int i, j;
  out_learnt.copyTo(analyze_toclear);
  if (ccmin_mode == 2) {
    uint32_t abstract_level = 0;
    for (i = 1; i < out_learnt.size(); i++)
      abstract_level |=
          abstractLevel(var(out_learnt[i])); // (maintain an abstraction of
                                             // levels involved in conflict)

    for (i = j = 1; i < out_learnt.size(); i++)
      if (reason(var(out_learnt[i])) == CRef_Undef ||
          !litRedundant(out_learnt[i], abstract_level))
        out_learnt[j++] = out_learnt[i];

  } else if (ccmin_mode == 1) {
    for (i = j = 1; i < out_learnt.size(); i++) {
      Var x = var(out_learnt[i]);

      if (reason(x) == CRef_Undef)
        out_learnt[j++] = out_learnt[i];
      else {
        Clause &c = ca[reason(var(out_learnt[i]))];
        for (int k = 1; k < c.size(); k++)
          if (!seen[var(c[k])] && level(var(c[k])) > 0) {
            out_learnt[j++] = out_learnt[i];
            break;
          }
      }
    }
  } else
    i = j = out_learnt.size();

  max_literals += out_learnt.size();
  out_learnt.shrink(i - j);
  tot_literals += out_learnt.size();

  // Find correct backtrack level:
  //
  if (out_learnt.size() == 1)
    out_btlevel = 0;
  else {
    int max_i = 1;
    // Find the first literal assigned at the next-highest level:
    for (int i = 2; i < out_learnt.size(); i++)
      if (level(var(out_learnt[i])) > level(var(out_learnt[max_i])))
        max_i = i;
    // Swap-in this literal at index 1:
    Lit p = out_learnt[max_i];
    out_learnt[max_i] = out_learnt[1];
    out_learnt[1] = p;
    out_btlevel = level(var(p));
  }

  for (int j = 0; j < analyze_toclear.size(); j++)
    seen[var(analyze_toclear[j])] = 0; // ('seen[]' is now cleared)
}

/**
   analyze : (confl : Clause*) (out_learnt : vec<Lit>&) (out_btlevel : int&)  ->
   [void]

   Description:
   Analyze conflict and produce a reason clause.

   Pre-conditions:
   * 'out_learnt' is assumed to be cleared.
   * Current decision level must be greater than root level.

   Post-conditions:
   * 'out_learnt[0]' is the asserting literal at level 'out_btlevel'.
   * If out_learnt.size() > 1 then 'out_learnt[1]' has the greatest decision
   level of the rest of literals. There may be others from the same level
   though.

*/
void Solver::analyzeLastUIP(CRef confl, vec<Lit> &out_learnt,
                            int &out_btlevel) {
  Lit p = lit_Undef;

  // Generate conflict clause:
  out_learnt.push(); // (leave room for the asserting literal)
  int index = trail.size() - 1;

  do {
    assert(confl != CRef_Undef); // (otherwise should be UIP)
    Clause &c = ca[confl];

    if (c.learnt())
      claBumpActivity(c);

    for (int j = (p == lit_Undef) ? 0 : 1; j < c.size(); j++) {
      Lit q = c[j];

      if (!seen[var(q)] && level(var(q)) > 0) {
        varBumpActivity(var(q));
        seen[var(q)] = 1;
        if (!(level(var(q)) >= decisionLevel()))
          out_learnt.push(q);
      }
    }

    // Select next clause to look at:
    while (!seen[var(trail[index--])])
      assert(index >= 0);
    p = trail[index + 1];
    confl = reason(var(p));
    seen[var(p)] = 0;
  } while (confl != CRef_Undef);
  out_learnt[0] = ~p;

  // Simplify conflict clause:
  //
  int i, j;
  out_learnt.copyTo(analyze_toclear);
  if (ccmin_mode == 2) {
    uint32_t abstract_level = 0;
    for (i = 1; i < out_learnt.size(); i++)
      abstract_level |=
          abstractLevel(var(out_learnt[i])); // (maintain an abstraction of
                                             // levels involved in conflict)

    for (i = j = 1; i < out_learnt.size(); i++)
      if (reason(var(out_learnt[i])) == CRef_Undef ||
          !litRedundant(out_learnt[i], abstract_level))
        out_learnt[j++] = out_learnt[i];

  } else if (ccmin_mode == 1) {
    for (i = j = 1; i < out_learnt.size(); i++) {
      Var x = var(out_learnt[i]);

      if (reason(x) == CRef_Undef)
        out_learnt[j++] = out_learnt[i];
      else {
        Clause &c = ca[reason(var(out_learnt[i]))];
        for (int k = 1; k < c.size(); k++)
          if (!seen[var(c[k])] && level(var(c[k])) > 0) {
            out_learnt[j++] = out_learnt[i];
            break;
          }
      }
    }
  } else
    i = j = out_learnt.size();

  max_literals += out_learnt.size();
  out_learnt.shrink(i - j);
  tot_literals += out_learnt.size();

  // Find correct backtrack level:
  //
  if (out_learnt.size() == 1)
    out_btlevel = 0;
  else {
    int max_i = 1;
    // Find the first literal assigned at the next-highest level:
    for (int i = 2; i < out_learnt.size(); i++)
      if (level(var(out_learnt[i])) > level(var(out_learnt[max_i])))
        max_i = i;
    // Swap-in this literal at index 1:
    Lit p = out_learnt[max_i];
    out_learnt[max_i] = out_learnt[1];
    out_learnt[1] = p;
    out_btlevel = level(var(p));
  }

  for (int j = 0; j < analyze_toclear.size(); j++)
    seen[var(analyze_toclear[j])] = 0; // ('seen[]' is now cleared)
}

// Check if 'p' can be removed. 'abstract_levels' is used to abort early if the
// algorithm is visiting literals at levels that cannot be removed later.
bool Solver::litRedundant(Lit p, uint32_t abstract_levels) {
  analyze_stack.clear();
  analyze_stack.push(p);
  int top = analyze_toclear.size();
  while (analyze_stack.size() > 0) {
    assert(reason(var(analyze_stack.last())) != CRef_Undef);
    Clause &c = ca[reason(var(analyze_stack.last()))];
    analyze_stack.pop();

    for (int i = 1; i < c.size(); i++) {
      Lit p = c[i];
      if (!seen[var(p)] && level(var(p)) > 0) {
        if (reason(var(p)) != CRef_Undef &&
            (abstractLevel(var(p)) & abstract_levels) != 0) {
          seen[var(p)] = 1;
          analyze_stack.push(p);
          analyze_toclear.push(p);
        } else {
          for (int j = top; j < analyze_toclear.size(); j++)
            seen[var(analyze_toclear[j])] = 0;
          analyze_toclear.shrink(analyze_toclear.size() - top);
          return false;
        }
      }
    }
  }

  return true;
} // litRedundant

/*_________________________________________________________________________________________________
  |
  |  analyzeFinal : (p : Lit)  ->  [void]
  |
  |  Description:
  |    Specialized analysis procedure to express the final conflict in terms of
  assumptions. |    Calculates the (possibly empty) set of assumptions that led
  to the assignment of 'p', and |    stores the result in 'out_conflict'.
  |________________________________________________________________________________________________@*/
void Solver::analyzeFinal(Lit p, vec<Lit> &out_conflict) {
  out_conflict.clear();
  out_conflict.push(p);

  if (decisionLevel() == 0)
    return;
  seen[var(p)] = 1;

  for (int i = trail.size() - 1; i >= trail_lim[0]; i--) {
    Var x = var(trail[i]);
    scoreActivity[x]++;
    if (seen[x]) {
      if (reason(x) == CRef_Undef) {
        assert(level(x) > 0);
        out_conflict.push(~trail[i]);
      } else {
        Clause &c = ca[reason(x)];
        for (int j = 1; j < c.size(); j++)
          if (level(var(c[j])) > 0)
            seen[var(c[j])] = 1;
      }
      seen[x] = 0;
    }
  }
  seen[var(p)] = 0;
} // analyzeFinal

void Solver::uncheckedEnqueue(Lit p, CRef from) {
  Var v = var(p);

  assert(value(p) == l_Undef);
  assigns[v] = lbool(!sign(p));
  vardata[v] = mkVarData(from, decisionLevel());
  trail.push_(p);

  if (cert != nullptr && !decisionLevel()) {
    idxClausesCpt++;
    std::ostream &cval = *cert;
    cval << (var(p) + 1) * (-2 * sign(p) + 1) << " 0\n";
  }
} // uncheckedEnqueue

/*_________________________________________________________________________________________________
  |
  |  propagate : [void]  ->  [Clause*]
  |
  |  Description:
  |    Propagates all enqueued facts. If a conflict arises, the conflicting
  clause is returned, |    otherwise CRef_Undef.
  |
  |    Post-conditions:
  |      * the propagation queue is empty, even if there was a conflict.
  |________________________________________________________________________________________________@*/
CRef Solver::propagate() {
  CRef confl = CRef_Undef;
  int num_props = 0;
  watches.cleanAll();

  while (qhead < trail.size()) {
    Lit p = trail[qhead++]; // 'p' is enqueued fact to propagate.

    vec<Watcher> &ws = watches[p];
    Watcher *i, *j, *end;
    num_props++;

    for (i = j = (Watcher *)ws, end = i + ws.size(); i != end;) {
      // Try to avoid inspecting the clause:
      Lit blocker = i->blocker;
      if (value(blocker) == l_True) {
        *j++ = *i++;
        continue;
      }

      // Make sure the false literal is data[1]:
      CRef cr = i->cref;
      Clause &c = ca[cr];

      Lit false_lit = ~p;
      if (c[0] == false_lit)
        c[0] = c[1], c[1] = false_lit;
      assert(c[1] == false_lit);
      i++;

      // If 0th watch is true, then clause is already satisfied.
      Lit first = c[0];
      Watcher w = Watcher(cr, first);
      if (first != blocker && value(first) == l_True) {
        *j++ = w;
        continue;
      }

      // Look for new watch:
      for (int k = 2; k < c.size(); k++)
        if (value(c[k]) != l_False) {
          c[1] = c[k];
          c[k] = false_lit;
          watches[~c[1]].push(w);
          goto NextClause;
        }

      // Did not find watch -- clause is unit under assignment:
      *j++ = w;
      if (value(first) == l_False) {
        confl = cr;
        qhead = trail.size();
        // Copy the remaining watches:
        while (i < end)
          *j++ = *i++;
      } else
        uncheckedEnqueue(first, cr);

    NextClause:;
    }
    ws.shrink(i - j);
  }

  propagations += num_props;
  simpDB_props -= num_props;

  return confl;
} // propagate

/**

   reduceDB : ()  ->  [void]

   Description:
   Remove half of the learnt clauses, minus the clauses locked by the current
   assignment. Locked clauses are clauses that are reason to some assignment.
   Binary clauses are never removed.
*/
struct reduceDB_lt {
  ClauseAllocator &ca;
  reduceDB_lt(ClauseAllocator &ca_) : ca(ca_) {}
  bool operator()(CRef x, CRef y) {
    return ca[x].size() > 2 &&
           (ca[y].size() == 2 || ca[x].activity() < ca[y].activity());
  }
};
void Solver::reduceDB() {
  int i, j;
  double extra_lim =
      cla_inc / learnts.size(); // Remove any clause below this activity

  sort(learnts, reduceDB_lt(ca));
  // Don't delete binary or locked clauses. From the rest, delete clauses from
  // the first half and clauses with activity smaller than 'extra_lim':
  for (i = j = 0; i < learnts.size(); i++) {
    Clause &c = ca[learnts[i]];
    if (c.size() > 2 && !locked(c) &&
        (i < learnts.size() / 2 || c.activity() < extra_lim)) {
      removeClause(learnts[i]);
    } else
      learnts[j++] = learnts[i];
  }
  learnts.shrink(i - j);
  checkGarbage();
} // reduceDB

void Solver::removeSatisfied(vec<CRef> &cs) {
  int i, j;
  for (i = j = 0; i < cs.size(); i++) {
    Clause &c = ca[cs[i]];
    if (satisfied(c))
      removeClause(cs[i]);
    else
      cs[j++] = cs[i];
  }
  cs.shrink(i - j);
} // removeSatisfied

void Solver::rebuildOrderHeap() {
  vsRebuildOrderHeap.setSize(0);
  for (int i = 0; i < problemVariable.size(); i++) {
    Var v = problemVariable[i];
    if (decision[v] && value(v) == l_Undef)
      vsRebuildOrderHeap.push(v);
  }
  order_heap.build(vsRebuildOrderHeap);
} // rebuildOrderHeap

/**
   simplify : [void]  ->  [bool]

   Description:
   Simplify the clause database according to the current top-level assigment.
   Currently, the only thing done here is the removal of satisfied clauses, but
   more things can be put here.
*/
bool Solver::simplify() {
  assert(decisionLevel() == 0);

  if (!ok || propagate() != CRef_Undef)
    return ok = false;
  if (nAssigns() == simpDB_assigns || (simpDB_props > 0))
    return true;

  // Remove satisfied clauses:
  removeSatisfied(learnts);
  if (remove_satisfied)
    removeSatisfied(clauses); // Can be turned off.

  checkGarbage();
  rebuildOrderHeap();

  simpDB_assigns = nAssigns();
  //(shouldn't depend on stats really, but it will do for now)
  simpDB_props = clauses_literals + learnts_literals;

  return true;
} // simplify

/**

   search : (nof_conflicts : int) (params : const SearchParams&)  ->  [lbool]

   Description:
   Search for a model the specified number of conflicts.
   NOTE! Use negative value for 'nof_conflicts' indicate infinity.

   Output:
   'l_True' if a partial assigment that is consistent with respect to the
   clauseset is found. If all variables are decision variables, this means that
   the clause set is satisfiable. 'l_False' if the clause set is unsatisfiable.
   'l_Undef' if the bound on number of conflicts is reached.
*/
lbool Solver::search(int nof_conflicts) {
  assert(ok);
  int backtrack_level;
  int conflictC = 0;
  vec<Lit> learnt_clause;
  starts++;

  for (;;) {
    CRef confl = propagate();

    if (confl != CRef_Undef) // CONFLICT
    {
      // printf("conflict\n");
      conflicts++;
      conflictC++;
      if (decisionLevel() == 0) {
        // we add a reason for the conflict, because it cannot exist
        if (cert != nullptr) {
          std::ostream &cval = *cert;
          cval << (var(ca[confl][0]) + 1) * (-2 * sign(ca[confl][0]) + 1)
               << " 0\n";
        }
        return l_False;
      }

      learnt_clause.clear();
      analyze(confl, learnt_clause, backtrack_level);
      cancelUntil(backtrack_level);

      if (cert != nullptr) {
        std::ostream &cval = *cert;
        for (int i = 0; i < learnt_clause.size(); i++)
          cval << (var(learnt_clause[i]) + 1) *
                      (-2 * sign(learnt_clause[i]) + 1)
               << " ";
        cval << "0\n";
      }

      idxClausesCpt++;
      if (learnt_clause.size() == 1) {
        cancelUntil(0);
        uncheckedEnqueue(learnt_clause[0]);
      } else {
        CRef cr = ca.alloc(learnt_clause, true);
        learnts.push(cr);
        attachClause(cr);
        claBumpActivity(ca[cr]);
        uncheckedEnqueue(learnt_clause[0], cr);
        ca[cr].idxReason(idxClausesCpt);
      }

      varDecayActivity();
      claDecayActivity();

      if (--learntsize_adjust_cnt == 0) {
        learntsize_adjust_confl *= learntsize_adjust_inc;
        learntsize_adjust_cnt = (int)learntsize_adjust_confl;
        max_learnts *= learntsize_inc;
      }
    } else // NO CONFLICT
    {
      if (decisionLevel() > assumptions.size() && nof_conflicts >= 0 &&
          (conflictC >= nof_conflicts || !withinBudget())) {
        // Reached bound on number of conflicts:
        progress_estimate = progressEstimate();
        cancelUntil(assumptions.size());
        return l_Undef;
      }

      if (decisionLevel() == 0 && !simplify())
        return l_False; // Simplify the set of problem clauses:
      if (decisionLevel() >= assumptions.size() &&
          learnts.size() - nAssigns() >= max_learnts)
        reduceDB();

      Lit next = lit_Undef;
      while (decisionLevel() <
             assumptions.size()) // Perform user provided assumption:
      {
        Lit p = assumptions[decisionLevel()];

        if (value(p) == l_True)
          newDecisionLevel();
        else if (value(p) == l_False) {
          analyzeFinal(~p, conflict);
          if (cert != nullptr) {
            std::ostream &cval = *cert;
            for (int i = 0; i < conflict.size(); i++)
              cval << (var(conflict[i]) + 1) * (-2 * sign(conflict[i]) + 1)
                   << " ";
            cval << "0\n";
          }
          idxClausesCpt++;
          if (conflict.size() == 1) {
            cancelUntil(0);
            idxReasonFinal = -1;
            if (decisionLevel())
              uncheckedEnqueue(conflict[0]);
          } else {
            CRef cr = ca.alloc(conflict, true);
            learnts.push(cr);
            attachClause(cr);
            claBumpActivity(ca[cr]);
            ca[cr].idxReason(idxClausesCpt);
            idxReasonFinal = ca[cr].idxReason();
          }
          return l_False;
        } else {
          next = p;
          break;
        }
      }

      if (next == lit_Undef) // New variable decision:
      {
        decisions++;
        next = pickBranchLit();
        if (next == lit_Undef)
          return l_True; // Model found:
      }

      // Increase decision level and enqueue 'next'
      newDecisionLevel();
      uncheckedEnqueue(next);
    }
  }
} // search

double Solver::progressEstimate() const {
  double progress = 0;
  double F = 1.0 / nVars();

  for (int i = 0; i <= decisionLevel(); i++) {
    int beg = i == 0 ? 0 : trail_lim[i - 1];
    int end = i == decisionLevel() ? trail.size() : trail_lim[i];
    progress += pow(F, i) * (end - beg);
  }

  return progress / nVars();
}

/*
  Finite subsequences of the Luby-sequence:

  0: 1
  1: 1 1 2
  2: 1 1 2 1 1 2 4
  3: 1 1 2 1 1 2 4 1 1 2 1 1 2 4 8
  ...

*/
static double luby(double y, int x) {
  // Find the finite subsequence that contains index 'x', and the size of that
  // subsequence:
  int size, seq;
  for (size = 1, seq = 0; size < x + 1; seq++, size = 2 * size + 1)
    ;

  while (size - 1 != x) {
    size = (size - 1) >> 1;
    seq--;
    x = x % size;
  }
  return pow(y, seq);
} // luby

// NOTE: assumptions passed in member-variable 'assumptions'.
lbool Solver::solve_(bool rebuildHeap, int nbConflict) {
  conflict.clear();
  if (!ok)
    return l_False;

  if (rebuildHeap)
    rebuildOrderHeap();

  solves++;

  if (solves == 1) {
    max_learnts = nClauses() * learntsize_factor;
    learntsize_adjust_confl = learntsize_adjust_start_confl;
    learntsize_adjust_cnt = (int)learntsize_adjust_confl;
  }
  lbool status = l_Undef;

  // Search:
  int curr_restarts = 0;
  uint64_t initConflicts = conflicts;

  while (status == l_Undef &&
         (!nbConflict || (conflicts - initConflicts < (uint64_t)nbConflict))) {
    double rest_base = luby_restart ? luby(restart_inc, curr_restarts)
                                    : pow(restart_inc, curr_restarts);
    status =
        nbConflict ? search(nbConflict) : search(rest_base * restart_first);
    if (!withinBudget())
      break;
    curr_restarts++;
  }

  if (needModel && status == l_True) {
    // Extend & copy model:
    for (int i = 0; i < problemVariable.size(); i++) {
      // printf("%d %d\n", problemVariable[i], value(problemVariable[i]) ==
      // l_True);
      model[problemVariable[i]] = value(problemVariable[i]);
    }
    // printf("*** %d %d\n", 3, model[3] == l_True);
  }

  cancelUntil(assumptions.size());
  return status;
} // solve_

//=================================================================================================
// Writing CNF to DIMACS:
//
// FIXME: this needs to be rewritten completely.
static Var mapVar(Var x, vec<Var> &map, Var &max) {
  if (map.size() <= x || map[x] == -1) {
    map.growTo(x + 1, -1);
    map[x] = max++;
  }
  return map[x];
}

void Solver::toDimacs(FILE *f, Clause &c, vec<Var> &map, Var &max) {
  if (satisfied(c))
    return;

  for (int i = 0; i < c.size(); i++)
    if (value(c[i]) != l_False)
      fprintf(f, "%s%d ", sign(c[i]) ? "-" : "",
              mapVar(var(c[i]), map, max) + 1);
  fprintf(f, "0\n");
}

void Solver::toDimacs(const char *file, const vec<Lit> &assumps) {
  FILE *f = fopen(file, "wr");
  if (f == NULL)
    fprintf(stderr, "could not open file %s\n", file), exit(1);
  toDimacs(f, assumps);
  fclose(f);
}

void Solver::toDimacs(FILE *f, const vec<Lit> &assumps) {
  // Handle case when solver is in contradictory state:
  if (!ok) {
    fprintf(f, "p cnf 1 2\n1 0\n-1 0\n");
    return;
  }

  vec<Var> map;
  Var max = 0;

  // Cannot use removeClauses here because it is not safe
  // to deallocate them at this point. Could be improved.
  int cnt = 0;
  for (int i = 0; i < clauses.size(); i++)
    if (!satisfied(ca[clauses[i]]))
      cnt++;

  for (int i = 0; i < clauses.size(); i++)
    if (!satisfied(ca[clauses[i]])) {
      Clause &c = ca[clauses[i]];
      for (int j = 0; j < c.size(); j++)
        if (value(c[j]) != l_False)
          mapVar(var(c[j]), map, max);
    }

  // Assumptions are added as unit clauses:
  cnt += assumptions.size();

  fprintf(f, "p cnf %d %d\n", max, cnt);

  for (int i = 0; i < assumptions.size(); i++) {
    assert(value(assumptions[i]) != l_False);
    fprintf(f, "%s%d 0\n", sign(assumptions[i]) ? "-" : "",
            mapVar(var(assumptions[i]), map, max) + 1);
  }

  for (int i = 0; i < clauses.size(); i++)
    toDimacs(f, ca[clauses[i]], map, max);

  if (verbosity > 0)
    printf("Wrote %d clauses with %d variables.\n", cnt, max);
}

//=================================================================================================
// Garbage Collection methods:

void Solver::relocAll(ClauseAllocator &to) {
  // All watchers:
  // for (int i = 0; i < watches.size(); i++)
  watches.cleanAll();
  for (int v = 0; v < nVars(); v++)
    for (int s = 0; s < 2; s++) {
      Lit p = mkLit(v, s);
      // printf(" >>> RELOCING: %s%d\n", sign(p)?"-":"", var(p)+1);
      vec<Watcher> &ws = watches[p];
      for (int j = 0; j < ws.size(); j++)
        ca.reloc(ws[j].cref, to);
    }

  // All reasons:
  //
  for (int i = 0; i < trail.size(); i++) {
    Var v = var(trail[i]);
    if (reason(v) != CRef_Undef &&
        (ca[reason(v)].reloced() || locked(ca[reason(v)])))
      ca.reloc(vardata[v].reason, to);
  }

  for (int i = 0; i < learnts.size(); i++)
    ca.reloc(learnts[i], to); // All learnt:
  for (int i = 0; i < clauses.size(); i++)
    ca.reloc(clauses[i], to); // All original:
  for (int i = 0; i < phantoms.size(); i++)
    ca.reloc(phantoms[i], to); // All phantoms:
}

void Solver::garbageCollect() {
  // Initialize the next region to a size corresponding to the estimated
  // utilization degree. This is not precise but should avoid some unnecessary
  // reallocations for the new region:
  ClauseAllocator to(ca.size() - ca.wasted());

  relocAll(to);
  if (verbosity >= 2)
    printf("|  Garbage collection:   %12d bytes => %12d bytes             |\n",
           ca.size() * ClauseAllocator::Unit_Size,
           to.size() * ClauseAllocator::Unit_Size);
  to.moveTo(ca);
} // garbageCollect

/**
 * @brief Propagate the assumption.
 *
 * @return true if no conflict, false otherwise.
 */
bool Solver::propagateAssumption() {
  while (decisionLevel() < assumptions.size()) {
    Lit p = assumptions[decisionLevel()];

    if (value(p) == l_False)
      return false;

    // add a level for the assumption.
    newDecisionLevel();

    if (value(p) == l_True)
      continue;

    // push the assumption and apply the unit propagation.
    uncheckedEnqueue(p);
    if (propagate() != CRef_Undef)
      return false;
  }

  return true;
} // propagateAssumption

} // namespace minisat
