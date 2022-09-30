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
#pragma once

#include <bitset>
#include <iostream>
#include <vector>

#include "../BucketAllocator.hpp"
#include "../BucketManager.hpp"
#include "../CachedBucket.hpp"
#include "src/problem/ProblemTypes.hpp"
#include "src/specs/cnf/SpecManagerCnf.hpp"
#include "src/utils/Enum.hpp"

namespace d4 {
template <class T>
class BucketManager;
template <class T>
class CacheManager;

template <class T>
class BucketManagerCnf : public BucketManager<T> {
 protected:
  SpecManagerCnf &m_specManager;

  ModeStore m_modeStore;
  unsigned m_nbClauseCnf;
  unsigned m_nbVarCnf;
  unsigned m_maxSizeClause;

  std::vector<bool> m_varInComponent;
  std::vector<int> m_idxClauses;

 public:
  /**
     Constructor.

     @param[in] occM, the CNF occurrence manager.
     @param[in] cache, the cache the bucket is linked with.
     @param[in] mdStore, the storing mode for the clause.
     @param[in] sizeFirstPage, the amount of bytes for the first page.
     @param[in] sizeAdditionalPage, the amount of bytes for the additional
     pages.
     @param[in] bucketAllocator, a bucket allocator.
  */
  BucketManagerCnf(SpecManagerCnf &occM, CacheManager<T> *cache,
                   ModeStore mdStore, unsigned long sizeFirstPage,
                   unsigned long sizeAdditionalPage,
                   BucketAllocator *bucketAllocator)
      : m_specManager(occM) {
    this->m_cache = cache;
    this->m_bucketAllocator = bucketAllocator;
    m_modeStore = mdStore;
    m_nbClauseCnf = occM.getNbClause();
    m_nbVarCnf = occM.getNbVariable();
    m_maxSizeClause = occM.getMaxSizeClause();
    m_varInComponent.resize(m_nbVarCnf, false);

    this->m_bucketAllocator->init(sizeFirstPage, sizeAdditionalPage);
  }  // BucketManager

  virtual ~BucketManagerCnf() { ; }
  virtual void storeFormula(std::vector<Var> &component,
                            CachedBucket<T> &b) = 0;

  /**
     Tell if the clause given as parameter (which is represented by its index in
     the spec manager) should be considered or not.

     @param[in] idx, the index of the clause.

     \return true if the clause is kept, false otherwise.
   */
  bool isKeptClause(int idx) {
    switch (m_modeStore) {
      case NT:
        return m_specManager.getNbUnsat(idx);
      case NB:
        return m_specManager.getClause(idx).size() > 2;
      default:
        return true;
    }
  }  // isKeptClause

  /**
     Get the clauses that will be used, that are the clause that respect the
     modeStore.

     @param[in] component, the variables in the current component.
     @param[out] idxClauses, the resulting clauses (index).
  */
  void collectIdActiveClauses(std::vector<Var> &component,
                              std::vector<unsigned> &idxClauses) {
    // collect the clauses
    idxClauses.resize(0);
    if (m_modeStore == ALL)
      m_specManager.getCurrentClauses(idxClauses, component);
    else
      m_specManager.getCurrentClausesNotBin(idxClauses, component);

    unsigned i, j;
    for (i = j = 0; i < idxClauses.size(); i++) {
      if (!isKeptClause(idxClauses[i])) continue;
      idxClauses[j++] = idxClauses[i];
    }
    idxClauses.resize(j);
  }  // collectIdActiveClauses
};
}  // namespace d4
