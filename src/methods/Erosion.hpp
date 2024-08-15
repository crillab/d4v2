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
#pragma once

#include "MethodManager.hpp"
#include "src/configurations/Configuration.hpp"
#include "src/methods/Counter.hpp"
#include "src/preprocs/PreprocManager.hpp"
#include "src/problem/ProblemManager.hpp"
#include "src/problem/cnf/ProblemManagerCnf.hpp"
#include "src/problem/cnf/ProblemManagerErosionCnf.hpp"

namespace d4 {

template <class T>
class Erosion : public MethodManager {
 private:
  /**
   * @brief Apply the erosion process on the clauses.
   *
   * @param[in,out] clauses is the set of clauses (we suppose that the clauses
   * are sorted).
   * @return true if the problem is not trivially SAT, false otherwise.
   */
  bool erode(std::vector<std::vector<Lit>> &clauses, int nbVar) {
    std::vector<std::vector<Lit>> tmpClauses = clauses;
    std::vector<unsigned long> hashValue;
    clauses.clear();

    // generate.
    for (auto &c : tmpClauses) {
      if (c.size() == 1) return false;

      // generate the clauses.
      for (unsigned i = 0; i < c.size(); i++) {
        clauses.push_back(std::vector<Lit>());
        std::vector<Lit> &cl = clauses.back();
        cl.reserve(c.size());
        unsigned long currentHash = 0;

        for (unsigned j = 0; j < c.size(); j++) {
          if (i == j) continue;
          cl.push_back(c[j]);
          currentHash |= 1 << (c[j].intern() & 63);
        }
        hashValue.push_back(currentHash);
      }
    }

    // reduce.
    std::vector<bool> marked(2 * (nbVar + 1), false);
    for (unsigned i = 0; i < clauses.size(); i++) {
      std::vector<Lit> &c = clauses[i];
      for (auto &l : c) marked[l.intern()] = true;

      // check if the clause is already subsume.
      bool isSubsume = false;
      for (unsigned j = 0; j < i; j++) {
        std::vector<Lit> &d = clauses[j];
        if (!d.size()) continue;
        isSubsume = true;
        for (auto &l : d) {
          isSubsume = marked[l.intern()];
          if (!isSubsume) break;
        }
        if (isSubsume) break;
      }

      // restore mark.
      for (auto &l : c) marked[l.intern()] = false;
      if (isSubsume) c.clear();
    }

    unsigned j = 0;
    for (unsigned i = 0; i < clauses.size(); i++)
      if (clauses[i].size()) clauses[j++] = clauses[i];
    clauses.resize(j);

    return true;
  }  // erode

 public:
  /**
   * @brief Run the method.
   *
   * @param vm is the options.
   */
  void run(ProblemManagerErosionCnf *problem, int depth,
           ConfigurationDpllStyleMethod configCounter, std::ostream &out) {
    // preprare the stream.
    std::ostream outCounter(nullptr);
    outCounter.setstate(std::ios_base::badbit);

    bool isUnsat = false;
    int nbErosion = depth < 0 ? problem->getNbVar() : depth;

    // the CNF formula.
    std::vector<std::vector<Lit>> softClauses = problem->getSoftClauses();
    std::vector<std::vector<Lit>> hardClauses = problem->getHardClauses();

    // require to init the solver
    std::vector<Var> setOfVar;
    for (unsigned i = 1; i < problem->getNbVar() + 1; i++)
      setOfVar.push_back(i);

    // iterate until the number of erosion realized is less than a given value
    // or until the formula is UNSAT.
    T lastCount = T(0);
    for (int cptErosion = 0; cptErosion <= nbErosion; cptErosion++) {
      if (isUnsat) {
        out << "c Erosion finished because empty clause.\n";
        out << "s " << cptErosion << " " << 0 << " \n";
        break;
      }

      // prepare the counter.
      ProblemManagerCnf *p = new ProblemManagerCnf(
          problem->getNbVar(), problem->getWeightLit(), problem->getWeightVar(),
          problem->getSelectedVar());
      std::vector<std::vector<Lit>> tmpClauses = softClauses;

      // add the theory clauses.
      for (auto &cl : hardClauses) tmpClauses.push_back(cl);

      p->setClauses(tmpClauses);

      // create the counter.
      outCounter << "c [CONSTRUCTOR] Create an external counter: counting\n";

      DpllStyleMethod<T, T> *counter = new DpllStyleMethod<T, T>(
          OptionDpllStyleMethod(configCounter), p, std::cout);

      // count to test.
      std::vector<Lit> assumption;
      T count = counter->count(setOfVar, assumption, outCounter);
      out << "s " << cptErosion << " " << count << "\n";
      delete counter;

      if (cptErosion) assert(count <= lastCount);
      lastCount = count;

      if (count == 0) break;

      // erosion step.
      isUnsat = !erode(softClauses, problem->getNbVar());
    }
  }

  void interrupt() {}
};
}  // namespace d4