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

#include "CacheCleaningManager.hpp"
#include "CachedBucket.hpp"

namespace d4 {
template <class T>
class CacheCleaningManager;
template <class T>
class CacheManager;
template <class T>
class CacheCleaningNone : public CacheCleaningManager<T> {
 public:
  /**
     Constructor.

     @param[in] cache, the cache where is applied the cleaning process.
   */
  CacheCleaningNone(CacheManager<T> *cache) {
    this->m_cache = cache;
  }  // constructor

  void initCountCachedBucket(CachedBucket<T> *cb) {}  // nothing to do.
  void updateCountCachedBucket(CachedBucket<T> *cb, int nbVar) {
  }                                             // nothing to do.
  void reduceCache() {}                         // nothing to do.
  void printCleaningInfo(std::ostream &out) {}  // nothing to do.
};

}  // namespace d4
