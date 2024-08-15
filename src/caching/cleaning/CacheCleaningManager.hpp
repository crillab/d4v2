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

#include "../CacheManager.hpp"
#include "CacheCleaningExpectation.hpp"
#include "CacheCleaningNone.hpp"
#include "src/exceptions/FactoryException.hpp"
#include "src/options/cache/OptionCacheCleaningManager.hpp"

namespace d4 {
template <class T>
class CachedBucket;

template <class T>
class CacheCleaningManager {
 protected:
  CacheManager<T> *m_cache;

 public:
  virtual ~CacheCleaningManager() {}

  /**
     Create an operator to manage the cache reduction process.

     @param[in] options, the option list.
     @param[in] cache, the cache where we want to clean.
     @param[in] nbVar, the number of variables in the problem.
     @param[in] out, the stream where are print out the information.
   */
  static CacheCleaningManager<T> *makeCacheCleaningManager(
      const OptionCacheCleaningManager &options, CacheManager<T> *cache,
      int nbVar, std::ostream &out) {
    out << "c [CACHE CLEANING MANAGER] " << options << '\n';

    if (options.cacheCleaningStrategy == CACHE_EXPECTATION)
      return new CacheCleaningExpectation<T>(cache, nbVar);
    if (options.cacheCleaningStrategy == CACHE_NONE)
      return new CacheCleaningNone<T>(cache);

    throw(FactoryException("Cannot create a CacheCleaningManager", __FILE__,
                           __LINE__));
  }  // makeCacheCleaningManager

  virtual void initCountCachedBucket(CachedBucket<T> *cb) = 0;
  virtual void updateCountCachedBucket(CachedBucket<T> *cb, int nbVar) = 0;
  virtual void reduceCache() = 0;
  virtual void printCleaningInfo(std::ostream &out) = 0;

  /**
     Ask to the bucket manager to release some memory block.

     @param[in] data, the memory we release.
     @param[in] size, the size of the memory block.
     @param[in] posInHash, where in the cache this block oppears.
     @param[in] smudge, control if we want to directly remove the entry or not.

   */
  void releaseMemory(char *data, int size) {
    m_cache->getBucketManager()->releaseMemory(data, size);
  }  // releaseMemory
};

}  // namespace d4
