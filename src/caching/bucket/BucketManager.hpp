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

#include <string.h>

#include <cassert>
#include <deque>
#include <iostream>
#include <vector>

#include "../CachedBucket.hpp"
#include "cnf/BucketManagerCnfCl.hpp"
#include "cnf/BucketManagerCnfCombi.hpp"
#include "cnf/BucketManagerCnfIndex.hpp"
#include "cnf/BucketManagerCnfSym.hpp"
#include "src/caching/CacheManager.hpp"
#include "src/exceptions/FactoryException.hpp"
#include "src/options/cache/OptionBucketManager.hpp"
#include "src/problem/ProblemTypes.hpp"
#include "src/specs/SpecManager.hpp"

namespace d4 {
// forward declaration
class BucketAllocator;

template <class T>
class BucketManager {
 protected:
  BucketAllocator *m_bucketAllocator;
  CachedBucket<T> m_bucket;
  CacheManager<T> *m_cache;  // the cache linked with this BucketManager.

 public:
  virtual ~BucketManager() {
    if (m_bucketAllocator->getCleanup()) delete m_bucketAllocator;
  }  // destructor

  /**
   * @brief Create a bucket manager regarding the given options.
   *
   * @param options are the options.
   * @param cache is the cache manager which is linked to the bucket manager.
   * @param s is the spect manager which is linked to the bucket manager.
   * @param out is the stream where is printed out the logs.
   * @return BucketManager<T>*
   */
  static BucketManager<T> *makeBucketManager(OptionBucketManager options,
                                             CacheManager<T> *cache,
                                             SpecManager &s,
                                             std::ostream &out) {
    out << "c [BUCKET MANAGER] " << options << "\n";

    SpecManagerCnf &scnf = dynamic_cast<SpecManagerCnf &>(s);
    if (options.clauseRepresentation == CACHE_CLAUSE)
      return new BucketManagerCnfCl<T>(scnf, cache, options.modeStore,
                                       options.sizeFirstPage,
                                       options.sizeAdditionalPage);
    if (options.clauseRepresentation == CACHE_SYM)
      return new BucketManagerCnfSym<T>(scnf, cache, options.modeStore,
                                        options.sizeFirstPage,
                                        options.sizeAdditionalPage);
    if (options.clauseRepresentation == CACHE_INDEX)
      return new BucketManagerCnfIndex<T>(scnf, cache, options.modeStore,
                                          options.sizeFirstPage,
                                          options.sizeAdditionalPage);
    if (options.clauseRepresentation == CACHE_COMBI) {
      return new BucketManagerCnfCombi<T>(
          scnf, cache, options.modeStore, options.sizeFirstPage,
          options.sizeAdditionalPage, options.limitNbVarSym,
          options.limitNbVarIndex);
    }

    throw(
        FactoryException("Cannot create a BucketManager", __FILE__, __LINE__));
  }  // makeBucketManager

  inline int nbOctetToEncodeInt(unsigned int v) {
    // we know that we cannot have more than 1<<32 variables
    if (v < (1 << 8)) return 1;
    if (v < (1 << 16)) return 2;
    return 4;
  }  // nbOctetToEncodeInt

  /**
     Collect the bucket associtated to the set of variable given in
     parameter.
     @param[in] component, the variable belonging to the connected component
     \return a formula put in a bucket
  */
  CachedBucket<T> *collectBucket(std::vector<Var> &component) {
    storeFormula(component, m_bucket);
    return &m_bucket;
  }  // collectBuckect

  inline unsigned long int usedMemory() {
    return m_bucketAllocator->usedMemory();
  }

  inline bool getComsumedMemory() {
    return m_bucketAllocator->getComsumedMemory();
  }
  inline void reinitComsumedMemory() {
    m_bucketAllocator->reinitComsumedMemory();
  }

  inline void releaseMemory(char *m, unsigned size) {
    m_bucketAllocator->releaseMemory(m, size);
  }  // releaseMemory

  inline double remainingMemory() {
    return m_bucketAllocator->remainingMemory();
  }  // remainingMemory

  virtual void storeFormula(std::vector<Var> &component,
                            CachedBucket<T> &b) = 0;
};
}  // namespace d4
