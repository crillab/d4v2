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

#include <string.h>

#include <boost/program_options.hpp>
#include <cassert>
#include <deque>
#include <iostream>
#include <vector>

#include "BucketAllocator.hpp"
#include "CachedBucket.hpp"
#include "cnf/BucketManagerCnf.hpp"
#include "cnf/BucketManagerCnfCl.hpp"
#include "cnf/BucketManagerCnfCombi.hpp"
#include "cnf/BucketManagerCnfIndex.hpp"
#include "cnf/BucketManagerCnfSym.hpp"
#include "src/caching/CacheManager.hpp"
#include "src/exceptions/FactoryException.hpp"
#include "src/problem/ProblemTypes.hpp"
#include "src/specs/SpecManager.hpp"
#include "src/utils/Enum.hpp"

namespace d4 {
namespace po = boost::program_options;

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

  static BucketManager<T> *makeBucketManager(po::variables_map &vm,
                                             CacheManager<T> *cache,
                                             SpecManager &s,
                                             std::ostream &out) {
    std::string css = vm["cache-store-strategy"].as<std::string>();
    std::string ccr = vm["cache-clause-representation"].as<std::string>();
    std::string crs = vm["cache-reduction-strategy"].as<std::string>();

    unsigned long sizeFirstPage =
        vm["cache-size-first-page"].as<unsigned long>();
    unsigned long sizeAdditionalPage =
        vm["cache-size-additional-page"].as<unsigned long>();

    out << "c [CONSTRUCTOR] Cache bucket manager:"
        << " storage(" << css << ") "
        << " representation(" << ccr << ") "
        << " size_first_page(" << sizeFirstPage << ")"
        << " size_additional_page(" << sizeAdditionalPage << ")"
        << "\n";

    ModeStore mode = ALL;
    if (css == "not-binary") mode = NB;
    if (css == "not-touched") mode = NT;

    SpecManagerCnf &scnf = dynamic_cast<SpecManagerCnf &>(s);
    if (ccr == "clause")
      return new BucketManagerCnfCl<T>(scnf, cache, mode, sizeFirstPage,
                                       sizeAdditionalPage);
    if (ccr == "sym")
      return new BucketManagerCnfSym<T>(scnf, cache, mode, sizeFirstPage,
                                        sizeAdditionalPage);
    if (ccr == "index")
      return new BucketManagerCnfIndex<T>(scnf, cache, mode, sizeFirstPage,
                                          sizeAdditionalPage);
    if (ccr == "combi") {
      unsigned limitNbVarSym =
          vm["cache-clause-representation-combi-limitVar-sym"].as<unsigned>();
      unsigned limitNbVarIndex =
          vm["cache-clause-representation-combi-limitVar-index"].as<unsigned>();

      out << "c [CONSTRUCTOR] Cache bucket manager mixed strategy:"
          << " limit #var sym(" << limitNbVarSym << ") "
          << " limit #var index (" << limitNbVarIndex << ") "
          << "\n";

      return new BucketManagerCnfCombi<T>(scnf, cache, mode, sizeFirstPage,
                                          sizeAdditionalPage, limitNbVarSym,
                                          limitNbVarIndex);
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
