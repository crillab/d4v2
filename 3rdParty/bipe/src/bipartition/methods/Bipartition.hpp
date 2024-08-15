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
#pragma once

#include <functional>
#include <map>
#include <ostream>
#include <string>
#include <vector>

#include "src/bipartition/heuristic/HeuristicBipartition.hpp"
#include "src/bipartition/methods/Backbone.hpp"
#include "src/bipartition/methods/Method.hpp"
#include "src/bipartition/option/OptionBipartition.hpp"
#include "src/solver/WrapperSolver.hpp"
#include "src/utils/Gate.hpp"
#include "src/utils/Problem.hpp"
#include "src/utils/ProblemTypes.hpp"

namespace bipe {
namespace bipartition {

class Bipartition : public Method {
 private:
  const unsigned FREQ_HEADER = 50;
  const unsigned COLUMN_SIZE = 122;
  const unsigned WIDTH_PRINT_COLUMN_MC = 10;

  unsigned m_nbVar = 0;
  unsigned m_nbSATCall = 0;
  unsigned m_nbUNSCall = 0;
  unsigned m_nbUNDCall = 0;
  unsigned m_nbCall = 0;
  unsigned m_nbInputFromModel = 0;
  unsigned m_nbOutputFromSym = 0;
  unsigned m_nbInputFromCompleteModel = 0;
  unsigned m_nbInputFromPadoaModel = 0;

  float m_timeCall = 0.f;
  float m_timeSATCall = 0.f;
  float m_timeUNSCall = 0.f;
  float m_timeUNDCall = 0.f;
  float m_lastCall = 0.f;
  bool m_useCore = false;
  bool m_inputModel = false;
  bool m_useSym = false;

  std::vector<std::vector<int>> m_occurrence;
  std::vector<int> m_nbSatisfied;
  std::vector<int> m_breakcount;
  std::vector<int> m_makecount;
  std::vector<int> m_falsified;
  std::vector<int> m_whereIsFalsified;
  std::vector<Var> m_priority;
  std::vector<Lit> m_breakLit;
  std::vector<bool> m_isInOutput;
  unsigned m_nbFalsified;

  std::vector<bool> m_isProtectedVar;
  std::vector<std::vector<Var>> m_linkedVarTaut;

  std::vector<bool> m_model;
  std::vector<std::vector<bool>> m_listOfModels;
  std::vector<bool> m_marked;

  std::clock_t currentTime;

  inline void initTimer() { currentTime = clock(); }
  inline float getTimer() {
    return (float)(clock() - currentTime) / CLOCKS_PER_SEC;
  }

  void initInputModelVariable(Problem &p);

  void separator(std::ostream &out);
  void printHeader(std::ostream &out);
  void printInfo(HeuristicBipartition *heuristic, std::ostream &out);
  void updateInfo(Status status);

  void setPriority(Problem &p, WrapperSolver *solver, Lit l);

  void tryToConnectSavedModels(Problem &p, std::vector<Var> &input,
                               std::vector<Lit> &assums);

  /**
   * @brief Turn around a given set of models.
   *
   * @param p is the problem we are considering.
   * @param gates is a set of gates.
   * @param setOfModels is the set of models we use.
   * @param[out] collectedInput is the set of collected input.
   */
  void exploitSetOfModelAsPreprocToSpotInput(
      Problem &p, std::vector<Gate> &gates,
      std::vector<std::vector<bool>> &setOfModels,
      std::vector<Var> &collectedInput);

  /**
   * @brief Move a set of literals from the assums to the input variables.
   *
   * @param assums is a set of literals.
   * @param[out] input is the set of input.
   * @param test is used to decide if we keep or not the value.
   * @return the number of removed elements.
   */
  inline unsigned moveToInput(std::vector<Lit> &assums, std::vector<Var> &input,
                              std::function<bool(Var)> test) {
    unsigned j = 0;
    for (unsigned i = 0; i < assums.size(); i++) {
      if (!test(assums[i].var()))
        assums[j++] = assums[i];
      else {
        input.push_back(assums[i].var());
      }
    }

    unsigned ret = assums.size() - j;
    assums.resize(j);
    return ret;
  }  // moveToInput

  /**
   * @brief Function to test if the structures break and make count are okay.
   *
   * @param p is the probleme we are dealing with.
   * @param model is the complete interpretation we are considering.
   */
  void debugBreakMakeCount(Problem &p, std::vector<bool> &model);

  /**
   * @brief Flip the given variable and update the make/break strucutres.
   *
   * @param p is the problem.
   * @param v is the variable we want to flip.
   * @param model is the model we are considering.
   */
  void flip(Problem &p, Var v, std::vector<bool> &model);

  /**
   * @brief Take as an input a satisfiable assignement and initialize the
   * structure break and make count.
   *
   * @param p is the problem we are dealing with.
   * @param model is a model of p.
   */
  void initMakeBreakCount(Problem &p, std::vector<bool> &model);

  /**
   * @brief Look if from the model we can get input variable for free.
   *
   * @param p is the problem we are solving.
   * @param solver is the solver where we can get the model.
   * @param[out] input is the set of computed input variables.
   * @param heuristic is the heuristic where we can find out the remaining
   * assumptions.
   * @param lastLit is the literal we consider last for input testing.
   */
  void searchInputFromModel(Problem &p, std::vector<Var> &input,
                            std::vector<Lit> &assums, std::vector<bool> &model,
                            Lit lastLit);

  void initProblem(Problem &p, Problem &res, std::vector<Lit> &assums,
                   std::map<Var, std::pair<Var, Var>> &mapSelector,
                   std::vector<Gate> &gates, std::vector<Var> &input);

  /**
   * @brief Generate a set of definition regarding a definition and a group of
   * symmetries.
   *
   * @param solver is the solver we use.
   * @param symGroup is the group of symmetries.
   * @param def is the definition we start from.
   * @param areUnit is the set of literals that have to be unit.
   * @param outFound is the set of output found.
   */
  void generateOutputSym(WrapperSolver *solver,
                         std::map<Var, std::pair<Var, Var>> &mapSelector,
                         const std::vector<std::vector<Var>> &symGroup,
                         std::vector<Var> &def, std::vector<Lit> &areUnit,
                         std::vector<Var> &outFound);

  /**
   * @brief Compute a bipartition given a initialized solver.
   *
   * @param p is the initial problem.
   * @param solver is where the problem is loaded.
   * @param[out] input is the computed bi-partition (only the input).
   * @param[out] und is the set of input that are undetermined.
   * @param heuristic is used to select the next variable to test.
   * @param mapSelector gives the mapping between the selector and the
   * @param projected is the variables the problem is projected on.
   * equivalences.
   * @param nbConflict is the number of conflicts the SAT solver can do.
   * @param out is the stream where are printed out the logs.
   * @param verbose is set to true if we print out the logs about the solver.
   */
  void compute(Problem &p, WrapperSolver *solver, std::vector<Var> &input,
               std::vector<Lit> &und, HeuristicBipartition *heuristic,
               std::map<Var, std::pair<Var, Var>> &mapSelector,
               const int nbConflict,
               const std::vector<std::vector<Var>> &symGroup, std::ostream &out,
               bool verbose);

 public:
  /**
   * @brief Compute a bi-partition following the approach presented in:
   * Jean-Marie Lagniez, Emmanuel Lonca, Pierre Marquis: Definability for
   * model counting. Artif. Intell. 281: 103229 (2020)
   *
   * @param p, the problem we are considering.
   * @param input, where is stored the computed partition (only the input).
   * @param gates are the gates we identified.
   * @param verbose is set to true if we print out the logs regarding the
   * solver.
   * @param solverName, the solver we want to use.
   * @param nbConflict is the number of conflict the solver can do (or one
   * restart).
   * @param sortingMethod is the method used for heuristic to construct the
   * bipartition.
   * @param optBackbone is set to true if we compute the backbone in
   * preprocessing.
   * @param out, the stream where will be printed out the log.
   * @return true if the problem is satisfiable, false otherwise.
   */
  bool run(Problem &p, std::vector<Var> &input, std::vector<Gate> &gates,
           const OptionBipartition &optionBipartition,
           const std::vector<std::vector<Var>> &symGroup,
           std::vector<std::vector<bool>> &setOfModels, std::ostream &out);
};
}  // namespace bipartition
}  // namespace bipe