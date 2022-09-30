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

#include "BucketManagerCnf.hpp"
#include "src/caching/BucketManager.hpp"
#include "src/problem/ProblemTypes.hpp"

namespace d4 {

template <class T>
class BucketManagerCnf;

template <class T>
class BucketManagerCnfIndex : public BucketManagerCnf<T> {
 private:
  std::vector<unsigned> m_idxClauses;

 public:
  /**
     Function called in order to initialized variables before using

     @param[in] occM, the CNF occurrence manager
     @param[in] cache, the cache the bucket is linked with.
     @param[in] mdStore, the storing mode for the clause
     @param[in] sizeFirstPage, the amount of bytes for the first page.
     @param[in] sizeAdditionalPage, the amount of bytes for the additional
     pages.
  */
  BucketManagerCnfIndex(
      SpecManagerCnf &occM, CacheManager<T> *cache, ModeStore mdStore,
      unsigned long sizeFirstPage, unsigned long sizeAdditionalPage,
      BucketAllocator *bucketAllocator = new BucketAllocator())
      : BucketManagerCnf<T>::BucketManagerCnf(occM, cache, mdStore,
                                              sizeFirstPage, sizeAdditionalPage,
                                              bucketAllocator) {
  }  // BucketManagerCnfIndex

  /**
     Destructor.
   */
  ~BucketManagerCnfIndex() {}  // destructor

  /**
     Store the variables respecting the information of size concerning the type
     T to encode each elements and returns the pointer just after the end of the
     data.

     @param[]
  */
  template <typename U, typename W>
  void *storeData(void *data, std::vector<W> &value) {
    U *p = static_cast<U *>(data);
    for (auto &v : value) {
      *p = static_cast<U>(v);
      p++;
    }

    return p;
  }  // storeVariables

  /**
     Transfer the formula store in distib in a table given in parameter.

     @param[in] component, the input variables
     @param[out] tmpFormula, the place where is stored the formula
     @param[out] szTmpFormula, to collect the size of the stored formula
  */
  inline void storeFormula(std::vector<Var> &component, CachedBucket<T> &b) {
    // collect the clauses
    this->collectIdActiveClauses(component, m_idxClauses);

    // nb bytes we need to store the information.
    unsigned int nbOVar = this->nbOctetToEncodeInt(component.back() + 1);
    unsigned int nbOData =
        m_idxClauses.size() ? this->nbOctetToEncodeInt(m_idxClauses.back() + 1)
                            : 1;

    // ask for memory
    unsigned szData = nbOVar * component.size() + nbOData * m_idxClauses.size();
    char *data = this->m_bucketAllocator->getArray(szData);
    void *p = data;

    // store the variables
    switch (nbOVar) {
      case 1:
        p = storeData<uint8_t, Var>(p, component);
        break;
      case 2:
        p = storeData<uint16_t, Var>(p, component);
        break;
      default:
        p = storeData<uint32_t, Var>(p, component);
        break;
    }
    assert(static_cast<char *>(p) == &data[nbOVar * component.size()]);
    if (!m_idxClauses.size()) goto fillTheBucket;

    // strore the clauses
    switch (nbOData) {
      case 1:
        p = storeData<uint8_t, unsigned>(p, m_idxClauses);
        break;
      case 2:
        p = storeData<uint16_t, unsigned>(p, m_idxClauses);
        break;
      default:
        p = storeData<uint32_t, unsigned>(p, m_idxClauses);
        break;
    }

  fillTheBucket:
    assert(0);
    // DataInfoCnf di(szData, component.size(), 0, m_idxClauses.size(), nbOVar,
    // 1,
    //               nbOData);
    // assert(di.szData() == szData);
    // b.set(data, di);
  }  // storeFormula
};
}  // namespace d4
