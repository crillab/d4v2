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
#include <functional>
#include <vector>

#include "BucketManager.hpp"
#include "CacheCleaningManager.hpp"
#include "CacheList.hpp"
#include "CacheNoCollision.hpp"
#include "CachedBucket.hpp"
#include "TmpEntry.hpp"
#include "src/exceptions/FactoryException.hpp"
#include "src/hashing/HashString.hpp"
#include "src/specs/SpecManager.hpp"

namespace d4 {
namespace po = boost::program_options;

template <class T>
class CacheCleaningManager;
template <class T>
class BucketManager;

template <class T>
class CacheManager {
 public:
  bool verb;

  // statistics
  unsigned long m_nbEntry = 0;
  unsigned long m_nbPositiveHit = 0;
  unsigned long m_nbNegativeHit = 0;
  unsigned minAffectedHitCache = 0;
  double sumAffectedHitCache = 0;

  // data info
  unsigned m_nbInitVar;
  unsigned m_nbRemoveEntry;
  unsigned long int m_nbCreationBucket;
  unsigned long int m_sumDataSize;
  unsigned int m_limitVarCached;

  std::ostream m_out;
  HashString hashMethod;

  const unsigned int MAX_NBVAR_CACHED = 100000;
  const unsigned int MIN_NBVAR_NOTCACHED = 100;

  BucketManager<T> *m_bucketManager;
  CacheCleaningManager<T> *m_cacheCleaningManager;

  /**
   * @brief Construct a new Cache Manager object
   *
   * @param vm is a map to get the option.
   * @param nbVar is the number of variables.
   * @param specs is a structure to get data about the formula.
   * @param out is the stream where are printed out the logs.
   */
  CacheManager(po::variables_map &vm, unsigned nbVar, SpecManager *specs,
               std::ostream &out)
      : m_out(nullptr) {
    m_out.copyfmt(out);
    m_out.clear(out.rdstate());
    m_out.basic_ios<char>::rdbuf(out.rdbuf());

    m_sumDataSize = m_nbEntry = m_nbCreationBucket = 0;
    verb = m_nbRemoveEntry = sumAffectedHitCache = 0;
    m_limitVarCached = (nbVar < MAX_NBVAR_CACHED) ? nbVar : MAX_NBVAR_CACHED;

    m_cacheCleaningManager =
        CacheCleaningManager<T>::makeCacheCleaningManager(vm, this, nbVar, out);
    m_bucketManager =
        BucketManager<T>::makeBucketManager(vm, this, *specs, out);
  }  // constructor

  /**
   * @brief Destroy the Cache Manager object
   */
  virtual ~CacheManager() {
    delete m_cacheCleaningManager;
    delete m_bucketManager;
  }  // destructor

  /**
   * @brief Factory.
   *
   * @param vm are the options.
   * @param nbVar is the number of variables.
   * @param specs gives the information about the input formula.
   * @param out is the stream where are printed out the logs.
   * @return CacheManager<T>*
   */
  static CacheManager<T> *makeCacheManager(po::variables_map &vm,
                                           unsigned nbVar, SpecManager *specs,
                                           std::ostream &out) {
    std::string method = vm["cache-method"].as<std::string>();
    out << "c [CACHE] Cache method used: " << method << "\n";

    if (method == "no-collision")
      return new CacheNoCollision<T>(vm, nbVar, specs, out);
    if (method == "list") return new CacheList<T>(vm, nbVar, specs, out);

    throw(
        FactoryException("Cannot create a ProblemManager", __FILE__, __LINE__));
  }  // makeCacheManager

  virtual void pushInHashTable(CachedBucket<T> &cb, unsigned int hashValue,
                               T val) = 0;
  virtual CachedBucket<T> *bucketAlreadyExist(CachedBucket<T> &cb,
                                              unsigned hashValue) = 0;
  virtual void initHashTable(unsigned maxVar) = 0;

  virtual unsigned removeEntry(
      std::function<bool(CachedBucket<T> &c)> test) = 0;

  /**
   * @brief Get the memory used by the cache (to store the ).
   *
   * @return unsigned long int
   */
  inline unsigned long int usedMemory() {
    return m_bucketManager->usedMemory();
  }

  inline unsigned getLimitVarCached() { return m_limitVarCached; }
  inline void setLimitVarCache(unsigned val) { m_limitVarCached = val; }
  inline bool isActivated(unsigned nbVar) { return nbVar <= m_limitVarCached; }
  inline unsigned long int nbCreationBucket() { return m_nbCreationBucket; }
  inline unsigned long int sumDataSize() { return m_sumDataSize; }

  inline unsigned long int getNbPositiveHit() { return m_nbPositiveHit; }
  inline unsigned long int getNbNegativeHit() { return m_nbNegativeHit; }
  inline unsigned long getNbEntry() { return m_nbEntry; }
  inline void decrementNbEntry() { m_nbEntry--; }
  inline BucketManager<T> *getBucketManager() { return m_bucketManager; }

  /**
   * @brief Release memory.
   *
   * @param data is the data we want to free.
   * @param size is the number of bytes.
   */
  inline void releaseMemory(char *data, int size) {
    this->m_bucketManager->releaseMemory(data, size);
  }  // releaseMemory

  inline void printCacheInformation(std::ostream &out) {
    out << "c \033[1m\033[34mCache Information\033[0m\n";
    out << "c Number of positive hit: " << m_nbPositiveHit << "\n";
    out << "c Number of negative hit: " << m_nbNegativeHit << "\n";
    m_cacheCleaningManager->printCleaningInfo(out);
    out << "c\n";
  }  // printCacheInformation

  /**
   * @brief Compute the hash value of an entry.
   *
   * @param bucket is the entry we want to compute the hash.
   * @return the value.
   */
  inline unsigned computeHash(CachedBucket<T> &bucket) {
    return hashMethod.hash(bucket.data, bucket.szData(), bucket.getInfo());
  }  // computeHash

  /**
   * @brief Add an entry in the cache structure.
   *
   * @param cb
   * @param val
   */
  void addInCache(TmpEntry<T> &cb, T val) {
    pushInHashTable(cb.e, cb.hashValue, val);
  }  // addInCache

  /**
   * @brief Take a bucket manager (in attribute) as well as a set of variables
   * consisting in the variables in the current component and search in the
   * cache if the related formula is present in the cache, if it is not the case
   * the bucket is created and added.
   *
   * @param varConnected is the set of variables.
   * @return TmpEntry<T>
   */
  TmpEntry<T> searchInCache(std::vector<Var> &varConnected) {
    if (m_bucketManager->getComsumedMemory()) {
      m_cacheCleaningManager->reduceCache();
      m_bucketManager->reinitComsumedMemory();
    }

    CachedBucket<T> *formulaBucket =
        m_bucketManager->collectBucket(varConnected);
    unsigned hashValue = computeHash(*formulaBucket);

    CachedBucket<T> *cacheBucket =
        bucketAlreadyExist(*formulaBucket, hashValue);

    m_cacheCleaningManager->updateCountCachedBucket(cacheBucket,
                                                    varConnected.size());
    if (!cacheBucket) return TmpEntry<T>(*formulaBucket, hashValue, false);

    m_bucketManager->releaseMemory(formulaBucket->data,
                                   formulaBucket->szData());
    return TmpEntry<T>(*cacheBucket, hashValue, true);
  }  // searchInCache

  /**
   * @brief Release the memory allocated to store a bucket.
   *
   * @param formulaBucket is the bucket we want to release the memory.
   */
  void releaseMemory(CachedBucket<T> &formulaBucket) {
    m_bucketManager->releaseMemory(formulaBucket.data, formulaBucket.szData());
  }  // releaseMemory

  /**
   * @brief Set the information concerning the number of clauses, variables and
   * the maximum size of the clauses (these information are useful to know the
   * size of the memory blocks we have to allocate).
   *
   * @param mVar is the number of variable.
   */
  void setInfoFormula(unsigned mVar) {
    minAffectedHitCache = mVar;
    m_nbInitVar = mVar;
  }  // setInfoFormula
};
}  // namespace d4
