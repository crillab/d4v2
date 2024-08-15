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
#include "src/bipartition/methods/Backbone.hpp"

#include <ctime>
#include <vector>

#include "src/solver/WrapperSolver.hpp"

namespace bipe {
namespace bipartition {
/**
 * @brief Destroy the Backbone:: Backbone object
 *
 */
Backbone::~Backbone() {}

/**
 * @brief Construct a new Backbone:: Backbone object
 *
 */
Backbone::Backbone() {}

/**
 * @brief Stop the process of computing the backbone.
 *
 */
void Backbone::interrupt() {
  m_isInterrupted = true;
  if (m_solver != nullptr) m_solver->interrupt();
}  // interrupt

/**
 * @brief Compute the backbone of the given problem.
 *
 * @param p is the problem we are looking for the backbone.
 * @param[out] gates is the vector where is stored the results.
 * @param nbConflict controls how many conflict the solver can do (0 ->
 * infinity) or one restart.
 * @param out is the stream where are displayed the logs.
 * @param solverName is name of the solver we use for computing the backbone
 * (this option is given to makeWrapperSolver)
 * \return true if the problem is satisfiable, false otherwise.
 */
bool Backbone::run(Problem &p, std::vector<Gate> &gates,
                   std::vector<std::vector<bool>> &setOfModels,
                   OptionBackbone option, std::ostream &out) {
  std::clock_t currentTime = clock();

  // Init the SAT solver.
  m_solver = WrapperSolver::makeWrapperSolver(option.solverName, out);
  m_solver->initSolver(p);
  m_solver->setNeedModel(true);

  // some statistics.
  unsigned nbSatCalls = 1;
  unsigned nbFoundUnit = 0;

  if (!m_solver->solve()) return false;
  m_solver->setReversePolarity(option.reversePolarity);

  std::vector<bool> marked(p.getNbVar() + 1, false);
  std::vector<bool> markedProtected(p.getNbVar() + 1, false);
  std::vector<bool> markedProjected(p.getNbVar() + 1, false);
  std::vector<bool> &model = m_solver->getModel();

  for (auto &v : p.getProjectedVar()) markedProjected[v] = true;
  for (auto &v : p.getProtectedVar()) markedProtected[v] = true;

  for (auto &v : p.getProjectedVar()) {
    if (marked[v] || m_solver->varIsAssigned(v) || markedProtected[v]) continue;
    if (m_isInterrupted) break;

    nbSatCalls++;

    // test the negation of the literal in order to verify if it is implied
    Lit l = Lit::makeLit(v, model[v]);
    m_solver->pushAssumption(l);
    bool isUnsat = m_solver->solveLimited(option.nbConflict) == Status::UNS;
    m_solver->popAssumption();

    if (!isUnsat) {
      // update the model.
      std::vector<bool> &solver_model = m_solver->getModel();
      for (unsigned j = 0; j < model.size(); j++)
        marked[j] = marked[j] || (model[j] != solver_model[(Var)j]);
      setOfModels.push_back(solver_model);
    } else {
      nbFoundUnit++;
      if (!m_solver->varIsAssigned(v)) m_solver->uncheckedEnqueue(~l);
    }
  }

  // the list of unit literals.
  std::vector<Lit> backbone;
  m_solver->getUnits(backbone);

  for (auto &l : backbone)
    if (markedProjected[l.var()]) gates.push_back({UNIT, l, {}});

  // some statistics.
  out << "c [BACKBONE] Time to compute the backbone: "
      << ((float)(clock() - currentTime) / CLOCKS_PER_SEC) << "\n";
  out << "c [BACKBONE] Number of SAT calls: " << nbSatCalls << "\n";
  out << "c [BACKBONE] Backbone size: " << backbone.size() << "\n";
  out << "c [BACKBONE] Number of units detected by calling the solver: "
      << nbFoundUnit << "\n";

  WrapperSolver *tmp = m_solver;
  m_solver = nullptr;
  delete tmp;
  return true;
}  // run
}  // namespace bipartition
}  // namespace bipe