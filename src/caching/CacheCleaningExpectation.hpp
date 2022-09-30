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

#include <functional>

#include "BucketManager.hpp"
#include "CacheCleaningManager.hpp"
#include "CachedBucket.hpp"

namespace d4 {
template <class T>
class CacheCleaningManager;
template <class T>
class CacheManager;

struct StatVarSizeCache {
  unsigned long negative;
  unsigned long positive;
  unsigned long number;
};

template <class T>
class CacheCleaningExpectation : public CacheCleaningManager<T> {
 private:
  const double INC_THRESHOD = 0.05;
  const double INIT_THRESHOD = 0;

  unsigned m_nbReduceCall;
  unsigned long m_nbRemoveEntry;
  unsigned long m_nbPositiveHit;
  unsigned long m_nbNegativeHit;
  int m_nbVar;
  double m_threshold;

  std::vector<StatVarSizeCache> m_statVar;
  using CacheCleaningManager<T>::m_cache;

 public:
  /**
     Constructor.

     @param[in] cache, the cache where is applied the cleaning process.
     @param[in] nbVar, the number of variables in the problem.
     @param[in] limit, the number of negative hits before calling the
     reduction.
     @param[in] ratio, the limit ratio.
   */
  CacheCleaningExpectation(CacheManager<T> *cache, int nbVar) {
    m_nbVar = nbVar;
    m_nbNegativeHit = 0;
    m_nbPositiveHit = 0;
    m_nbReduceCall = 0;
    m_nbRemoveEntry = 0;
    m_threshold = INIT_THRESHOD;
    this->m_cache = cache;

    m_statVar.resize(nbVar + 1, {0, 0, 0});
  }  // constructor

  /**
     We init the count of a bucket with the number of times we ask for an entry
     in the cache.

     @param[out] cb, the cached bucket we want to init.
   */
  void initCountCachedBucket(CachedBucket<T> *cb) {
    m_statVar[cb->nbVar()].number++;
  }  // initCountCachedBucket

  /**
   * @brief Update the information about the bucket.
   *
   * @param cb is the cached bucket we want to init.
   * @param nbVar a number of variables (because cb can be NULL the number of
   * variables cannot be related to cb).
   */
  void updateCountCachedBucket(CachedBucket<T> *cb, int nbVar) {
    if (cb) {
      m_statVar[nbVar].positive++;
      m_nbPositiveHit++;
    } else {
      m_statVar[nbVar].negative++;
      m_nbNegativeHit++;
    }
  }  // updateCountCachedBucket

  /**
   * @brief We remove the entry regarding if they have been used recently and
   * depending their number of variables.
   */
  void reduceCache() {
    m_nbReduceCall++;
    unsigned limit = m_cache->getLimitVarCached();

    for (int i = limit; i > m_cache->MIN_NBVAR_NOTCACHED; i--) {
      double ratio = 0;
      if (m_statVar[i].negative) {
        ratio = ((double)m_statVar[i].positive / (double)m_statVar[i].negative);

        // aging.
        m_statVar[i].negative >>= 1;
        m_statVar[i].positive >>= 1;
      }

      if (ratio > m_threshold) break;

      limit--;
    }

    // bonus and set the limit for the cache.
    limit *= 1.1;
    m_cache->setLimitVarCache(limit);
    m_threshold += INC_THRESHOD;

    unsigned nbRemoveEntry = m_cache->removeEntry([limit](CachedBucket<T> &c) {
      return c.nbVar() && c.nbVar() >= limit;
    });

    m_nbRemoveEntry += nbRemoveEntry;
    std::cout << "c #rm=" << nbRemoveEntry << " #allRm=" << m_nbRemoveEntry
              << " #entries=" << m_cache->getNbEntry() << " limit=" << limit
              << "\n";
  }  // reduceCache

  /**
     Print out statistics about the cleaning process.

     @param[in] out, the stream where are print out the information.
   */
  void printCleaningInfo(std::ostream &out) {
    out << "c Number of reduce calls: " << m_nbReduceCall << "\n";
    out << "c Number of entry removedreduce: " << m_nbRemoveEntry << "\n";
  }
};

}  // namespace d4
