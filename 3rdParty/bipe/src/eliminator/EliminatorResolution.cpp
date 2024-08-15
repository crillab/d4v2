/**
 * eliminator
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

#include "EliminatorResolution.hpp"

namespace bipe {
namespace eliminator {

/**
 * @brief selectVarAndPop implementation.
 *
 */
Var EliminatorResolution::selectVarAndPop(
    std::vector<Var> &vars, std::vector<std::vector<unsigned>> &occClauses) {
  if (!vars.size()) return var_Undef;
  Lit l = Lit::makeLitTrue(vars[0]);

  int best = 0,
      score = occClauses[l.intern()].size() * occClauses[(~l).intern()].size();

  for (unsigned i = 1; score && i < vars.size(); i++) {
    l = Lit::makeLitTrue(vars[i]);
    int tmp = occClauses[l.intern()].size() * occClauses[(~l).intern()].size();
    if (tmp < score) {
      score = tmp;
      best = i;
    }
  }

  Var tmp = vars[best];
  vars[best] = vars.back();
  vars.pop_back();
  return tmp;
}  // selectVarFromPos

/**
 * @brief generateAllResolution implementation.
 *
 */
bool EliminatorResolution::generateAllResolution(
    Var v, std::vector<std::vector<Lit>> &clauses,
    std::vector<std::vector<unsigned>> &occClauses,
    std::vector<std::vector<Lit>> &result) {
  Lit l = Lit::makeLitFalse(v);
  if (!occClauses[l.intern()].size() || !occClauses[(~l).intern()].size())
    return true;

  unsigned limit =
      occClauses[l.intern()].size() + occClauses[(~l).intern()].size();

  std::vector<Lit> tmp;
  tmp.reserve(occClauses.size());

  for (auto idxPos : occClauses[l.intern()]) {
    std::vector<Lit> &clPos = clauses[idxPos];
    for (auto &m : clPos)
      if (m != l) m_marked[m.intern()] = true;

    bool tooLarge = false;
    unsigned currentPos = result.size();
    for (auto &idxNeg : occClauses[(~l).intern()]) {
      std::vector<Lit> &clNeg = clauses[idxNeg];
      bool isTaut = false;

      tmp.resize(0);
      for (auto &m : clNeg) {
        if (m == ~l) continue;
        if ((isTaut = m_marked[(~m).intern()])) break;
        if (!m_marked[m.intern()]) tmp.push_back(m);
      }

      if (isTaut) continue;

      if (!tmp.size())  // selfSubsum
      {
        result.resize(currentPos);
        result.push_back(std::vector<Lit>());
        result.back().reserve(clPos.size() - 1);
        for (auto &m : clPos)
          if (m != l) result.back().push_back(m);
        break;
      }

      if (!isTaut) {
        result.push_back(std::vector<Lit>());
        result.back().reserve(clPos.size() - 1 + tmp.size());

        if (clPos.size() - 1 + tmp.size() > m_largerClauses) {
          tooLarge = true;
          break;
        }

        for (auto &m : clPos)
          if (m != l) result.back().push_back(m);
        for (auto &m : tmp) result.back().push_back(m);
      }

      if (tooLarge) break;
    }

    for (auto &m : clPos) m_marked[m.intern()] = false;
    if (result.size() > limit) return false;
    if (tooLarge) return false;
  }

  return true;
}  // generateAllResolution

/**
 * @brief eliminate implementation.
 *
 */
void EliminatorResolution::eliminate(unsigned nbVar,
                                     std::vector<std::vector<Lit>> &clauses,
                                     std::vector<Var> &input,
                                     std::vector<Lit> &eliminated, bool verbose,
                                     unsigned limitNbClauses) {
  // prepare the marker.
  m_marked.resize((nbVar + 1) << 1, false);
  for (unsigned i = 0; i < m_marked.size(); i++) m_marked[i] = false;

  // generate the occurrence list.
  std::vector<unsigned> freePlace;
  std::vector<std::vector<unsigned>> occClauses((nbVar + 1) << 1);
  m_largerClauses = 0;
  for (unsigned i = 0; i < clauses.size(); i++) {
    if (!clauses[i].size()) {
      freePlace.push_back(i);
      continue;
    }
    if (clauses[i].size() > m_largerClauses)
      m_largerClauses = clauses[i].size();
    for (auto &l : clauses[i]) occClauses[l.intern()].push_back(i);
  }

  // compute the set of variables we want to eliminate.
  std::vector<Var> output, inProcess;
  for (auto &v : input) m_marked[v] = true;
  for (auto &l : eliminated) m_marked[l.var()] = true;
  for (unsigned i = 1; i <= nbVar; i++)
    if (!m_marked[i]) output.push_back(i);
  for (auto &v : input) m_marked[v] = false;
  for (auto &l : eliminated) m_marked[l.var()] = false;

  // perform the elimination
  bool forgetApplied = true;
  inProcess = output;

  while (!m_isInterrupt && forgetApplied) {
    forgetApplied = false;

    std::vector<Var> pass;
    while (!m_isInterrupt && inProcess.size()) {
      Var v = selectVarAndPop(inProcess, occClauses);

      std::vector<std::vector<Lit>> allResolution;
      bool canForget =
          generateAllResolution(v, clauses, occClauses, allResolution);

      if (!canForget)
        pass.push_back(v);
      else {
        eliminated.push_back(Lit::makeLitFalse(v));
        forgetApplied = true;

        // remove the old clauses.
        for (unsigned ite = 0; ite < 2; ite++) {
          Lit l = Lit::makeLit(v, ite);

          for (auto &idx : occClauses[l.intern()]) {
            for (auto &m : clauses[idx])
              if (m != l) removeOcc(occClauses[m.intern()], idx);

            clauses[idx].clear();
            freePlace.push_back(idx);
          }

          occClauses[l.intern()].clear();
        }

        // add the resolved clauses.
        for (auto &cl : allResolution) {
          unsigned pos = clauses.size();
          if (freePlace.size()) {
            pos = freePlace.back();
            freePlace.pop_back();
          } else
            clauses.push_back(std::vector<Lit>());

          clauses[pos] = cl;
          for (auto &l : cl) occClauses[l.intern()].push_back(pos);
        }
      }
    }

    inProcess = pass;
  }
}  // eliminate

}  // namespace eliminator
}  // namespace bipe