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

#include "src/bipartition/methods/Method.hpp"

#include <vector>

#include "src/bipartition/methods/Backbone.hpp"
#include "src/bipartition/methods/DACircuit.hpp"
#include "src/solver/WrapperSolver.hpp"
#include "src/utils/FactoryException.hpp"
#include "src/utils/Gate.hpp"

namespace bipe {
namespace bipartition {
Method::~Method() {
  if (m_solver != nullptr) delete m_solver;
  m_solver = nullptr;

  if (m_backboneMethod != nullptr) delete m_backboneMethod;
  m_backboneMethod = nullptr;

  if (m_dacMethod != nullptr) delete m_dacMethod;
  m_dacMethod = nullptr;
}  // destuctor

/**
 * @brief We collect the input variables in the case where some units have been
 * detected but the preprocessor received a signal to stop its work.
 *
 * @param p is the problem we are searching for the bipartition.
 * @param units is the set of found units.
 * @param input is the input set.
 */
void Method::constructInputFromUnits(Problem &p, std::vector<Lit> &units,
                                     std::vector<Var> &input) {
  std::vector<bool> markedUnits(p.getNbVar() + 1, false);
  for (auto l : units) markedUnits[l.var()] = true;

  std::vector<bool> markedProtected(p.getNbVar() + 1, false);
  for (auto v : p.getProtectedVar()) markedProtected[v] = true;

  input.clear();
  for (auto v : p.getProjectedVar()) {
    if (markedProtected[v] || !markedUnits[v]) input.push_back(v);
  }
}  // constructInputFromUnits

/**
 * @brief This function calls a SAT solver (limited if asked) and return the
 * simplified formula if the problem SAT and nullptr otherwise.
 *
 * @param p is the problem we want to simplify.
 * @param nbConflict is the number of conflict the solve can do.
 * @param[out] gates is the computed gates (here a set of units).
 * @param out is the stream where are printed out the logs.
 * @return a simplify version of p if the problem is SAT, nullptr otherwise.
 */
Problem *Method::simplifyOneCall(Problem &p, const std::string &solverName,
                                 const unsigned nbConflict,
                                 std::vector<Gate> &gates, std::ostream &out,
                                 std::vector<std::vector<bool>> &setOfModels) {
  Problem *formula = nullptr;
  // Init the SAT solver.
  m_solver = WrapperSolver::makeWrapperSolver(solverName, out);
  m_solver->initSolver(p);
  m_solver->setNeedModel(true);

  std::vector<bool> markedProjected(p.getNbVar() + 1, false);
  for (auto &v : p.getProjectedVar()) markedProjected[v] = true;

  // test if the problem is SAT.
  if ((m_solver->solveLimited(nbConflict) != Status::UNS) && !m_isInterrupted) {
    setOfModels.push_back(m_solver->getModel());

    std::vector<Lit> units;
    m_solver->getUnits(units);
    formula = p.getConditionedFormula(units);

    for (auto &l : units)
      if (markedProjected[l.var()]) gates.push_back({UNIT, l, {}});
  }

  // clean the solver memory.
  WrapperSolver *tmp = m_solver;
  m_solver = nullptr;
  delete tmp;

  return formula;
}  // simplifyOneCall

/**
 * @brief This function calls a the backbone extractor method (limited if asked)
 * and return the simplified formula if the problem SAT and nullptr otherwise.
 *
 * @param p is the problem we want to simplify.
 * @param solverName is the name of the solver we want to use.
 * @param nbConflict is the number of conflict the solve can do.
 * @param[out] gates is the computed gates (here a set of units).
 * @param out is the stream where are printed out the logs.
 * @return a simplify version of p if the problem is SAT, nullptr otherwise.
 */
Problem *Method::simplifyBackbone(Problem &p,
                                  const OptionBackbone &optionBackbone,
                                  std::vector<Gate> &gates, std::ostream &out,
                                  std::vector<std::vector<bool>> &setOfModels) {
  Problem *formula = nullptr;

  m_backboneMethod = new Backbone();
  bool isSAT =
      m_backboneMethod->run(p, gates, setOfModels, optionBackbone, out);

  if (isSAT && !m_isInterrupted) {
    std::vector<Lit> units;
    for (auto &g : gates)
      if (g.type == UNIT) units.push_back(g.output);
    formula = p.getConditionedFormula(units);
  }

  Backbone *tmp = m_backboneMethod;
  m_backboneMethod = nullptr;
  delete tmp;

  return formula;
}  // simplifyBackbone

/**
 * @brief
 *
 * @param p is the problem we want to simplify.
 * @param solverName is the name of the solver we want to use.
 * @param nbConflict is the number of conflict the solve can do.
 * @param[out] gates is the computed gates.
 * @param out is the stream where are printed out the logs.
 * @return a simplify version of p if the problem is SAT, nullptr otherwise.
 */
Problem *Method::simplifyDac(Problem &p, const OptionDac &optionDac,
                             std::vector<Gate> &gates, std::ostream &out,
                             std::vector<std::vector<bool>> &setOfModels) {
  Problem *formula = nullptr;

  m_dacMethod = new DACircuit();
  bool isSAT = m_dacMethod->run(p, gates, setOfModels, optionDac, out);

  if (isSAT && !m_isInterrupted) {
    std::vector<Lit> units;
    for (auto &g : gates)
      if (g.type == UNIT) units.push_back(g.output);
    formula = p.getConditionedFormula(units);
  }

  DACircuit *tmp = m_dacMethod;
  m_dacMethod = nullptr;
  delete tmp;
  return formula;
}  // simplifyDac

/**
 * @brief Interrupt the method.
 *
 */
void Method::interrupt() {
  std::cout << "c [BIPE] The bi-partition process has been stopped\n";
  m_isInterrupted = true;
  if (m_solver != nullptr) m_solver->interrupt();
  if (m_backboneMethod != nullptr) m_backboneMethod->interrupt();
  if (m_dacMethod != nullptr) m_dacMethod->interrupt();
}  // interrupt

}  // namespace bipartition
}  // namespace bipe