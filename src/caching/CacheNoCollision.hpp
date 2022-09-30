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

#include <boost/program_options.hpp>
#include <vector>

#include "CacheManager.hpp"
#include "CachedBucket.hpp"
#include "TmpEntry.hpp"
#include "src/hashing/HashString.hpp"
#include "src/specs/SpecManager.hpp"

namespace d4 {
namespace po = boost::program_options;

template <class T>
class CacheManager;

template <class T>
class CacheNoCollision : public CacheManager<T> {
 private:
  const unsigned SIZE_HASH = 22041997;

 protected:
  std::vector<CachedBucket<T>> hashTable;

 public:
  /**
   * @brief Construct a new Cache No Collision object.
   *
   * @param vm is a map to get the option.
   * @param nbVar is the number of variables.
   * @param specs is a structure to get data about the formula.
   * @param out is the stream where are printed out the logs.
   */
  CacheNoCollision(po::variables_map &vm, unsigned nbVar, SpecManager *specs,
                   std::ostream &out)
      : CacheManager<T>(vm, nbVar, specs, out) {
    out << "c [CACHE NO-COLLISION CONSTRUCTOR]\n";
    initHashTable(nbVar);
  }  // constructor

  /**
   * @brief Destroy the Cache No Collision object.
   *
   */
  ~CacheNoCollision() {}  // destructor

  /**
   * @brief Add an entry in the cache.
   *
   * @param cb is the bucket we want to add.
   * @param hashValue is the hash value of the bucket.
   * @param val is the value we associate with the bucket.
   */
  void pushInHashTable(CachedBucket<T> &cb, unsigned int hashValue,
                       T val) override {
    CachedBucket<T> &cbi = hashTable[hashValue % SIZE_HASH];

    // remove the previous entry if needed.
    if (cbi.nbVar()) this->releaseMemory(cbi.data, cbi.szData());

    cbi = cb;
    cbi.lockedBucket(val);
    this->m_cacheCleaningManager->initCountCachedBucket(&cbi);

    this->m_nbCreationBucket++;
    this->m_sumDataSize += cb.szData();
    this->m_nbEntry++;
  }  // pushInCache

  /**
   * @brief Research in the set of buckets if the bucket pointed by i
   * already exist.
   *
   * @param cb is the index of the researched bucket.
   * @param hashValue is the hash value of this bucket.
   * @return the index of the identical bucket if this one exists, NULL
   * otherwise
   */
  CachedBucket<T> *bucketAlreadyExist(CachedBucket<T> &cb,
                                      unsigned hashValue) override {
    char *refData = cb.data;

    CachedBucket<T> &cbi = hashTable[hashValue % SIZE_HASH];
    if (cbi.nbVar() && cb.sameHeader(cbi) &&
        !memcmp(refData, cbi.data, cbi.szData())) {
      this->m_nbPositiveHit++;
      return &cbi;
    }

    this->m_nbNegativeHit++;
    return NULL;
  }  // bucketAlreadyExist

  /**
   * @brief Initialized the hashTable
   *
   * @param maxVar is the maximum number of variables a entry can have.
   */
  void initHashTable(unsigned maxVar) override {
    this->setInfoFormula(maxVar);

    // init hash tables
    hashTable.clear();
    hashTable.resize(SIZE_HASH);
  }  // initHashTable

  /**
   * @brief Clean up the cache.
   *
   * @param test is a function that is used to decide if an entry must be
   * removed.
   *
   * @return the number of entry we removed.
   */
  unsigned removeEntry(std::function<bool(CachedBucket<T> &c)> test) {
    unsigned nbRemoveEntry = 0;
    for (auto &cb : hashTable) {
      if (test(cb)) {
        assert((int)cb.szData() > 0);
        this->releaseMemory(cb.data, cb.szData());
        cb.reset();
        nbRemoveEntry++;
      }
    }
    return nbRemoveEntry;
  }  // removeEntry
};
}  // namespace d4
