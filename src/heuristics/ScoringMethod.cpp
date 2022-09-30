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
ScoringMethod *ScoringMethod::makeScoringMethod(po::variables_map &vm,
                                                SpecManager &p,
                                                ActivityManager &am,
                                                std::ostream &out) {
  std::string inType = vm["input-type"].as<std::string>();
  std::string meth = vm["scoring-method"].as<std::string>();

  out << "c [CONSTRUCTOR] Variable heuristic: " << meth << "\n";

  if (inType == "cnf" || inType == "dimacs") {
    try {
      SpecManagerCnf &ps = dynamic_cast<SpecManagerCnf &>(p);

      if (meth == "mom") return new ScoringMethodMom(ps);
      if (meth == "dlcs") return new ScoringMethodDlcs(ps);
      if (meth == "vsids") return new ScoringMethodVsids(am);
      if (meth == "vsads") return new ScoringMethodVsads(ps, am);
      if (meth == "jwts") return new ScoringMethodJwts(ps);
      return NULL;
    } catch (std::bad_cast &bc) {
      std::cerr << "bad_cast caught: " << bc.what() << '\n';
      std::cerr << "A CNF formula was expeted\n";
    }
  }

  throw(FactoryException("Cannot create a ScoringMethod", __FILE__, __LINE__));
}  // makeScoringMethod

/**
   Select the best variable in vars and return it.

   \param[in] vars, the set of variables we search in.

   \return the best variable if exists, var_Undef otherwise.
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
}  // selectVariable

}  // namespace d4
