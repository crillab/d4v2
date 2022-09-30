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

#include <algorithm>

#include "BucketInConstruction.hpp"
#include "BucketManagerCnf.hpp"
#include "BucketManagerCnfCl.hpp"
#include "BucketManagerCnfIndex.hpp"
#include "BucketManagerCnfSym.hpp"
#include "BucketSortInfo.hpp"
#include "src/caching/BucketManager.hpp"
#include "src/exceptions/BucketException.hpp"
#include "src/problem/ProblemTypes.hpp"

namespace d4 {
template <class T>
class BucketManagerCnfSym;
template <class T>
class BucketManagerCnfCl;
template <class T>
class BucketManagerCnfIndex;

template <class T>
class BucketManagerCnfCombi : public BucketManagerCnf<T> {
 private:
  std::vector<BucketSortInfo> m_vecBucketSortInfo;
  int m_unusedBucket;
  std::vector<unsigned long int> m_mapVar;

  std::vector<int> m_mustUnMark;
  std::vector<int> m_markIdx;
  std::vector<unsigned> m_idInVecBucket;

  BucketInConstruction m_inConstruction;
  unsigned *m_offsetClauses;

  BucketManagerCnfCl<T> *clBucketManagerBis;
  BucketManagerCnfCl<T> *clBucketManager;
  BucketManagerCnfIndex<T> *indexBucketManager;
  BucketManagerCnfSym<T> *symBucketManager;

  unsigned m_limitNbVarSym;
  unsigned m_limitNbVarIndex;

 public:
  /**
     Function called in order to initialized variables before using

     @param[in] occM, the CNF occurrence manager
     @param[in] cache, the cache the bucket is linked with.
     @param[in] mdStore, the storing mode for the clause
     @param[in] sizeFirstPage, the amount of bytes for the first page.
     @param[in] sizeAdditionalPage, the amount of bytes for the additional
     pages.
     @param[in] limitNbVarSym, if we have less than this number we use the sym
     cache.
     @param[in] pourcentNbVarIndex, if we have more than some pourcentage of
                varaibles then we use the index cache representation.
     @param[in] bucketAllocator, the object used to manage the memory
     allocation.
  */
  BucketManagerCnfCombi(
      SpecManagerCnf &occM, CacheManager<T> *cache, ModeStore mdStore,
      unsigned long sizeFirstPage, unsigned long sizeAdditionalPage,
      unsigned limitNbVarSym, unsigned limitNbVarIndex,
      BucketAllocator *bucketAllocator = new BucketAllocator())
      : BucketManagerCnf<T>::BucketManagerCnf(occM, cache, mdStore,
                                              sizeFirstPage, sizeAdditionalPage,
                                              bucketAllocator),
        m_inConstruction(occM) {
    clBucketManager =
        new BucketManagerCnfCl<T>(occM, cache, mdStore, sizeFirstPage,
                                  sizeAdditionalPage, this->m_bucketAllocator);

    clBucketManagerBis =
        new BucketManagerCnfCl<T>(occM, cache, mdStore, sizeFirstPage,
                                  sizeAdditionalPage, this->m_bucketAllocator);

    symBucketManager =
        new BucketManagerCnfSym<T>(occM, cache, mdStore, sizeFirstPage,
                                   sizeAdditionalPage, this->m_bucketAllocator);

    indexBucketManager = new BucketManagerCnfIndex<T>(
        occM, cache, mdStore, sizeFirstPage, sizeAdditionalPage,
        this->m_bucketAllocator);

    m_limitNbVarIndex = limitNbVarIndex;
    m_limitNbVarSym = limitNbVarSym;
    this->m_bucketAllocator->deactiveCleanUp();
  }  // BucketManagerCnfCombi

  /**
     Destructor.
   */
  ~BucketManagerCnfCombi() {
    delete symBucketManager;
    delete indexBucketManager;
    delete clBucketManagerBis;
    delete clBucketManager;
    this->m_bucketAllocator->activeCleanUp();
  }  // destructor

  /**
     Transfer the formula store in distib in a table given in parameter.

     @param[in] component, the input variables.
     @param[out] tmpFormula, the place where is stored the formula.
     @param[out] szTmpFormula, to collect the size of the stored formula.
  */
  inline void storeFormula(std::vector<Var> &component, CachedBucket<T> &b) {
    if (component.size() < m_limitNbVarSym)
      return symBucketManager->storeFormula(component, b);
    if (component.size() > m_limitNbVarIndex)
      return indexBucketManager->storeFormula(component, b);
    return clBucketManager->storeFormula(component, b);
  }  // storeFormula
};
}  // namespace d4
