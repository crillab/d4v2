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

#include "BucketManager.hpp"
#include "CacheCleaningManager.hpp"
#include "CachedBucket.hpp"
#include "src/caching/cnf/BucketManagerCnf.hpp"
#include "src/hashing/HashString.hpp"
#include "src/specs/SpecManager.hpp"

namespace d4 {
namespace po = boost::program_options;
template <class T>
class CacheList : public CacheManager<T> {
 private:
  const unsigned SIZE_HASH = 999331;
  std::vector<std::vector<CachedBucket<T>>> hashTable;

 public:
  /**
   * @brief Construct a new Cache List object
   *
   * @param vm is a map to get the option.
   * @param nbVar is the number of variables.
   * @param specs is a structure to get data about the formula.
   * @param out is the stream where are printed out the logs.
   */
  CacheList(po::variables_map &vm, unsigned nbVar, SpecManager *specs,
            std::ostream &out)
      : CacheManager<T>(vm, nbVar, specs, out) {
    out << "c [CACHE LIST CONSTRUCTOR]\n";
    initHashTable(nbVar);
  }  // constructor

  /**
   * @brief Destroy the Cache List object
   */
  ~CacheList() { hashTable.clear(); }  // destructor

  /**
   * @brief Add an entry in the cache.
   *
   * @param cb is the bucket we want to add.
   * @param hashValue is the hash value of the bucket.
   * @param val is the value we associate with the bucket.
   */
  inline void pushInHashTable(CachedBucket<T> &cb, unsigned int hashValue,
                              T val) {
    hashTable[hashValue % SIZE_HASH].push_back(cb);

    CachedBucket<T> &cbIn = (hashTable[hashValue % SIZE_HASH].back());
    cbIn.lockedBucket(val);
    this->m_nbCreationBucket++;
    this->m_sumDataSize += cb.szData();
    this->m_cacheCleaningManager->initCountCachedBucket(&cbIn);
    this->m_nbEntry++;
  }  // pushinhashtable

  /**
   * @brief Research in the set of buckets if the bucket pointed by i already
   * exist.
   *
   * @param cb is the bucket we are looking for.
   * @param hashValue is the hash value computed from the bucket.
   * @return a valid entry if it is in the cache, null otherwise.
   */
  CachedBucket<T> *bucketAlreadyExist(CachedBucket<T> &cb, unsigned hashValue) {
    char *refData = cb.data;
    std::vector<CachedBucket<T>> &listCollision =
        hashTable[hashValue % SIZE_HASH];

    for (auto &cbi : listCollision) {
      if (!cb.sameHeader(cbi)) continue;

      if (!memcmp(refData, cbi.data, cbi.szData())) {
        this->m_nbPositiveHit++;
        return &cbi;
      }
    }

    this->m_nbNegativeHit++;
    return NULL;
  }  // bucketAlreadyExist

  /**
   * Create a bucket and store it in the cache.
   *
   * @param varConnected is the set of variable.
   * @param c is the value we want to store.
   */
  inline void createAndStoreBucket(std::vector<Var> &varConnected, T &c) {
    CachedBucket<T> *formulaBucket =
        this->m_bucketManager->collectBuckect(varConnected);
    unsigned int hashValue = computeHash(*formulaBucket);
    pushInHashTable(*formulaBucket, hashValue, c);
  }  // createBucket

  /**
   * @brief Init the hashTable.
   *
   * @param maxVar is the number of variable.
   */
  void initHashTable(unsigned maxVar) override {
    this->setInfoFormula(maxVar);

    // init hash tables
    hashTable.clear();
    hashTable.resize(SIZE_HASH, std::vector<CachedBucket<T>>());
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
    for (auto &list : hashTable) {
      unsigned j = 0;
      for (unsigned i = 0; i < list.size(); i++) {
        CachedBucket<T> &cb = list[i];

        if (test(cb)) {
          assert((int)cb.szData() > 0);
          this->releaseMemory(cb.data, cb.szData());
          cb.reset();
          nbRemoveEntry++;
        } else
          list[j++] = list[i];
      }
      list.resize(j);
    }
    return nbRemoveEntry;
  }  // removeEntry
};
}  // namespace d4
