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
#include "ScoringMethod.hpp"

#include "cnf/ScoringMethodDlcs.hpp"
#include "cnf/ScoringMethodJwts.hpp"
#include "cnf/ScoringMethodMom.hpp"
#include "cnf/ScoringMethodVsads.hpp"
#include "cnf/ScoringMethodVsids.hpp"
#include "src/exceptions/FactoryException.hpp"

namespace d4 {

/**
   Select from the arguments store in vm the good scoring method and return it.

   @param[in] vm, the arguments on the command line.
   @pararm[in] p, the problem manager.

   \return the scoring method
 */
ScoringMethod *ScoringMethod::makeScoringMethod(
    const OptionBranchingHeuristic &options, SpecManager &p,
    ActivityManager &am, std::ostream &out) {
  try {
    SpecManagerCnf &ps = dynamic_cast<SpecManagerCnf &>(p);

    if (options.scoringMethodType == SCORE_MOM) return new ScoringMethodMom(ps);
    if (options.scoringMethodType == SCORE_DLCS)
      return new ScoringMethodDlcs(ps);
    if (options.scoringMethodType == SCORE_VSIDS)
      return new ScoringMethodVsids(am);
    if (options.scoringMethodType == SCORE_VSADS)
      return new ScoringMethodVsads(ps, am);
    if (options.scoringMethodType == SCORE_JWTS)
      return new ScoringMethodJwts(ps);
  } catch (std::bad_cast &bc) {
    std::cerr << "c bad_cast caught: " << bc.what() << '\n';
    std::cerr << "c A CNF formula was expeted\n";
    assert(0);
  }

  throw(FactoryException("Cannot create a ScoringMethod", __FILE__, __LINE__));
}  // makeScoringMethod

/**
 * @brief ScoringMethod::selectLitSet implementation.
 */
Var ScoringMethod::selectVariable(std::vector<Var> &vars, SpecManager &s,
                                  std::vector<bool> &isDecisionVariable) {
  Var ret = var_Undef;
  double bestScore = -1;
  assert(isDecisionVariable.size() >= (unsigned)s.getNbVariable());

  for (auto &v : vars) {
    if (s.varIsAssigned(v) || !isDecisionVariable[v]) continue;

    double current = computeScore(v);
    if (ret == var_Undef || current > bestScore) {
      ret = v;
      bestScore = current;
    }
  }

  return ret;
}  // selectLitSet

}  // namespace d4
