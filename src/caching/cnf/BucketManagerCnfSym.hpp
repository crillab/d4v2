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
#include "BucketSortInfo.hpp"
#include "src/caching/BucketManager.hpp"
#include "src/caching/CacheManager.hpp"
#include "src/exceptions/BucketException.hpp"
#include "src/problem/ProblemTypes.hpp"

namespace d4 {
template <class T>
class BucketManagerCnf;

template <class T>
class BucketManagerCnfSym : public BucketManagerCnf<T> {
 private:
  std::vector<BucketSortInfo> m_vecBucketSortInfo;
  int m_unusedBucket;
  std::vector<unsigned long int> m_mapVar;

  std::vector<int> m_mustUnMark;
  std::vector<int> m_markIdx;
  std::vector<unsigned> m_idInVecBucket;

  BucketInConstruction m_inConstruction;
  unsigned *m_offsetClauses;

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
  BucketManagerCnfSym(SpecManagerCnf &occM, CacheManager<T> *cache,
                      ModeStore mdStore, unsigned long sizeFirstPage,
                      unsigned long sizeAdditionalPage,
                      BucketAllocator *bucketAllocator = new BucketAllocator())
      : BucketManagerCnf<T>::BucketManagerCnf(occM, cache, mdStore,
                                              sizeFirstPage, sizeAdditionalPage,
                                              bucketAllocator),
        m_inConstruction(occM) {
    this->m_mapVar.resize(this->m_nbVarCnf + 1, 0);
    this->m_markIdx.resize(this->m_nbClauseCnf, -1);
    this->m_offsetClauses = new unsigned[this->m_nbClauseCnf];
  }  // BucketManagerCnfSym

  /**
     Destructor.
   */
  ~BucketManagerCnfSym() { delete[] m_offsetClauses; }  // destructor

  /**
     Get an index store the distribution information.

     @param[out] inConstruction, place where we store the bucket in
     construction.

     \return the index of a reserved bucket.
   */
  inline int getIdxBucketSortInfo(BucketInConstruction &inConstruction) {
    int ret = m_unusedBucket;

    if (m_unusedBucket == -1) {
      ret = m_vecBucketSortInfo.size();
      m_vecBucketSortInfo.emplace_back(
          BucketSortInfo(inConstruction.nbClauseInDistrib));
    } else
      m_unusedBucket = -1;

    return ret;
  }  // getIdxBucketSortInfo

  /**
     Push sorted, use the natural order.

   */
  inline void pushSorted(unsigned *tab, unsigned pos, unsigned val) {
    tab[pos] = val;
    for (unsigned i = pos; i > 0; i--)
      if (tab[i] < tab[i - 1])
        std::swap(tab[i], tab[i - 1]);
      else
        break;
  }  // pushSorted

  /**
     It is used in order to construct a sorted residual formula.

     @param[in] l, we considere the clause containing l
     @param[out] inConstruction, place where we store the bucket in
     construction.
     @param[in] repLit, the representation of the literal l in the formula in
     construction.
  */
  void createDistribWrTLit(const Lit &l, BucketInConstruction &inConstruction,
                           const Lit repLit) {
    unsigned currentPos =
        inConstruction.sizeDistrib;   // the place where we put l.
    inConstruction.sizeDistrib += 2;  // save memory for l and the size.

    // associate a bucket to the literal.
    unsigned counter = 0, nbElt = 0;
    unsigned *tab = &inConstruction.distrib[inConstruction.sizeDistrib];
    int ownBucket = getIdxBucketSortInfo(inConstruction);

    // visit each clause
    m_idInVecBucket.resize(0);
    unsigned nextBucket = m_vecBucketSortInfo.size();

    IteratorIdxClause listIndex = this->m_specManager.getVecIdxClause(l);
    for (int *ptr = listIndex.start; ptr != listIndex.end; ptr++) {
      int idx = *ptr;
      assert((unsigned)idx < m_markIdx.size());
      if (m_markIdx[idx] == -1) {
        inConstruction.sizeClauses[idx] = 1;
        m_mustUnMark.push_back(idx);
        m_markIdx[idx] = ownBucket;
        pushSorted(tab, nbElt++, inConstruction.nbClauseInDistrib + counter);
        counter++;
      } else {
        inConstruction.sizeClauses[idx]++;
        BucketSortInfo &b = m_vecBucketSortInfo[m_markIdx[idx]];
        if (!b.counter) {
          assert(nextBucket ==
                 m_vecBucketSortInfo.size() + m_idInVecBucket.size());
          b.redirected = nextBucket++;
          m_idInVecBucket.push_back(m_markIdx[idx]);
        }
        m_markIdx[idx] = b.redirected;
        pushSorted(tab, nbElt++, b.start + b.counter);
        b.counter++;
      }
    }

    inConstruction.sizeDistrib += nbElt;
    assert(inConstruction.sizeDistrib < inConstruction.capacityDistrib);

    m_vecBucketSortInfo.resize(m_vecBucketSortInfo.size() +
                               m_idInVecBucket.size());
    for (auto &bid : m_idInVecBucket) {
      BucketSortInfo &b = m_vecBucketSortInfo[bid];
      assert(b.counter);

      // we split out the bucket.
      m_vecBucketSortInfo[b.redirected].reset(b.start, b.start + b.counter);
      b.start += b.counter;
      b.counter = 0;
    }

    if (!counter)
      m_unusedBucket = ownBucket;
    else {
      m_vecBucketSortInfo[ownBucket].reset(
          inConstruction.nbClauseInDistrib,
          inConstruction.nbClauseInDistrib + counter);
      inConstruction.nbClauseInDistrib += counter;
    }

    if (currentPos == inConstruction.sizeDistrib - 2)
      inConstruction.sizeDistrib -= 2;
    else {
      inConstruction.distrib[currentPos] = l.intern();
      inConstruction.distrib[currentPos + 1] =
          inConstruction.sizeDistrib - currentPos - 2;
    }
  }  // createDistribWrTLit

  /**
     Collect the clause distribution. The result is stored in distrib.

     @param[in] component, the set of variables we consider.
     @param[out] inConstruction, place where we store the bucket in
     construction.

     \return the number of elements we have in the distribution once the
     redundant clauses have been removed.
  */
  inline unsigned collectDistrib(std::vector<Var> &component,
                                 BucketInConstruction &inConstruction) {
    // sort the set of clauses
    for (auto &v : component) {
      if (this->m_specManager.varIsAssigned(v)) continue;
      Lit l = Lit::makeLitFalse(v);
      createDistribWrTLit(l, inConstruction, l);
      createDistribWrTLit(~l, inConstruction, ~l);
    }

    // mark the clause we do not keep.
    unsigned realSizeDistrib = inConstruction.sizeDistrib;
    for (auto &idx : m_mustUnMark) {
      BucketSortInfo &b = m_vecBucketSortInfo[m_markIdx[idx]];
      m_markIdx[idx] = -1;
      inConstruction.shiftedSizeClause[b.start] =
          inConstruction.sizeClauses[idx];
      if (b.end != b.start + 1) {
        realSizeDistrib -=
            (b.end - b.start - 1) * this->m_specManager.getCurrentSize(idx);
        for (unsigned j = b.start + 1; j < b.end; j++)
          inConstruction.markedAsRedundant[j] = true;
        b.end = b.start + 1;
      }
    }
    m_mustUnMark.resize(0);

    // shift the clauses indices if requiered.
    unsigned index = 0;
    for (unsigned i = 0; i < inConstruction.nbClauseInDistrib; i++) {
      if (!inConstruction.markedAsRedundant[i]) {
        inConstruction.distribDiffSize[inConstruction.shiftedSizeClause[i]]++;
        inConstruction.shiftedSizeClause[index] =
            inConstruction.shiftedSizeClause[i];
        inConstruction.shiftedIndexClause[i] = index++;
      } else
        inConstruction.shiftedIndexClause[i] = inConstruction.sizeDistrib;
      inConstruction.markedAsRedundant[i] = false;
    }
    inConstruction.nbClauseInDistrib = index;  // resize

    return realSizeDistrib;
  }  // collectDistrib

  /**
     Prepare the data to store a new bucket.

     @param[out] inConstruction, place where we store the bucket in
     construction.
   */
  inline void initSortBucket(BucketInConstruction &inConstruction) {
    inConstruction.reinit();
    m_unusedBucket = -1;
    m_vecBucketSortInfo.resize(0);
  }  // initSortBucket

  inline void showListBucketSort(std::vector<BucketSortInfo> &v,
                                 std::ostream &out) {
    out << "size = " << v.size() << "\n";
    for (auto &e : v)
      out << "[" << e.start << " " << e.end << " " << e.counter << " "
          << e.redirected << "]";
    out << "\n";
  }  // showListBucketSort

  /**
     Compute the number of bytes requiered to store the data.
   */
  inline unsigned computeNeededBytes(unsigned nBda, unsigned nbD,
                                     unsigned nbEltData, unsigned nbEltDist) {
    return (nBda * nbEltData) + (nbD * (nbEltDist << 1));
  }  // computeNeededBytes

  /**
     Store the variables respecting the information of size concerning the type
     T to encode each elements and returns the pointer just after the end of the
     data.

     @param[in] data, the place where we print the data.
     @param[in] component, the set of variables.
   */
  template <typename U>
  void *storeVariables(void *data, std::vector<Var> &component) {
    U *p = static_cast<U *>(data);
    for (auto &v : component) {
      *p = static_cast<U>(v);
      p++;
    }

    return p;
  }  // storeVariables

  /**
   Store the variables respecting the information of size concerning the type T
   to encode each elements and returns the pointer just after the end of the
   data.

   @param[in] data, the place where we store the information.
   @param[out] inConstruction, place where we store the bucket in construction.
 */
  template <typename U>
  void *storeDistribInfo(void *data, BucketInConstruction &inConstruction) {
    U *p = static_cast<U *>(data);
    for (unsigned i = 0; i <= inConstruction.maxSizeClause; i++) {
      if (!inConstruction.distribDiffSize[i]) continue;
      *p = static_cast<U>(i);
      p++;
      *p = static_cast<U>(inConstruction.distribDiffSize[i]);
      p++;
    }

    return p;
  }  // storeDistribInfo

  /**
     Store the formula representation respecting the information of size
     concerning the type T to encode each elements and returns the pointer just
     after the end of the data.

     Information about the formula is store in member variables:
      - m_sizeDistrib
      - m_distrib

     @param[in] data, the place where we store the information
     @param[in] component, is the set of variables.
     @param[out] inConstruction, place where we store the bucket in
     construction.

     \return a pointer to the end of the data we added
  */
  template <typename U>
  void *storeClauses(void *data, std::vector<Var> &component,
                     BucketInConstruction &inConstruction) {
    // we map the variable to another index regarding their poistion in
    // component.
    for (unsigned i = 0; i < component.size(); i++) m_mapVar[component[i]] = i;

    // get the information about the starting offset for the different clause
    // size.
    unsigned offSet = 0;
    unsigned memoryPlaceWrtSizeClause[this->m_maxSizeClause + 1];
    for (unsigned i = 0; i <= this->m_maxSizeClause; i++) {
      memoryPlaceWrtSizeClause[i] = offSet;
      offSet += inConstruction.distribDiffSize[i] * i;
    }

    // allocate an offset for each clauses.
    for (unsigned i = 0; i < inConstruction.nbClauseInDistrib; i++) {
      unsigned szClause = inConstruction.shiftedSizeClause[i];
      if (!szClause) continue;

      m_offsetClauses[i] = memoryPlaceWrtSizeClause[szClause];
      memoryPlaceWrtSizeClause[szClause] += szClause;
      inConstruction.shiftedSizeClause[i] = 0;
    }

    // we store the data.
    U *p = static_cast<U *>(data);
    unsigned i = 0;
    while (i < inConstruction.sizeDistrib) {
      unsigned lit = inConstruction.distrib[i++];

      U l = static_cast<U>((m_mapVar[lit >> 1] << 1) | (lit & 1));
      unsigned szLitList = inConstruction.distrib[i++];

      while (szLitList) {
        szLitList--;

        unsigned idx =
            inConstruction.shiftedIndexClause[inConstruction.distrib[i++]];
        if (idx >= inConstruction.nbClauseInDistrib) continue;
        p[m_offsetClauses[idx]] = l;
        m_offsetClauses[idx]++;
      }
    }

    p += offSet;
    return p;
  }  // storeClauses

  /**
     Compute from the m_distribDiffSize the number of different size and the
     maximum size.

     @param[out] maxNbSizeDistr, the clause size with the maximum number of
     elements.
     @parar[out] largestSizeClause, store the size of the largest clause.
     @param[out] nbDiffClauseSize, the number of different size.
     @param[out] nbLit, the number of literals in the distribution.
     @param[out] inConstruction, place where we store the bucket in
     construction.
   */
  inline void getInfoDistributionSize(unsigned &maxNbSizeClause,
                                      unsigned &largestSizeClause,
                                      unsigned &nbDiffClauseSize,
                                      unsigned &nbLit,
                                      BucketInConstruction &inConstruction) {
    largestSizeClause = 0;
    maxNbSizeClause = 0;
    nbDiffClauseSize = 0;
    for (unsigned i = 0; i <= this->m_maxSizeClause; i++)
      if (inConstruction.distribDiffSize[i]) {
        largestSizeClause = i;
        if (maxNbSizeClause < inConstruction.distribDiffSize[i])
          maxNbSizeClause = inConstruction.distribDiffSize[i];
        nbDiffClauseSize++;
        nbLit += inConstruction.distribDiffSize[i] * i;
      }
  }  // getInfoDistributionSize

  /**
     Transfer the formula store in distib in a table given in parameter.

     @param[in] component, the input variables.
     @param[out] tmpFormula, the place where is stored the formula.
     @param[out] szTmpFormula, to collect the size of the stored formula.
  */
  inline void storeFormula(std::vector<Var> &component, CachedBucket<T> &b) {
    initSortBucket(m_inConstruction);
    collectDistrib(component, m_inConstruction);  // built the sorted formula

    // get information about the clause distribution
    unsigned nbLit = 0, nbVar = component.size(), maxNbSizeClause,
             nbDiffClauseSize, largestSizeClause;
    getInfoDistributionSize(maxNbSizeClause, largestSizeClause,
                            nbDiffClauseSize, nbLit, m_inConstruction);

    unsigned nbODistrib =
        this->nbOctetToEncodeInt(std::max(maxNbSizeClause, largestSizeClause));
    unsigned nbOLit = this->nbOctetToEncodeInt(nbVar << 1);

    // ask for memory
    unsigned szData =
        computeNeededBytes(nbOLit, nbODistrib, nbLit, nbDiffClauseSize);
    char *data = this->m_bucketAllocator->getArray(szData);
    void *p = data;

    // store the clause distribution of the size.
    switch (nbODistrib) {
      case 1:
        p = storeDistribInfo<uint8_t>(p, m_inConstruction);
        break;
      case 2:
        p = storeDistribInfo<uint16_t>(p, m_inConstruction);
        break;
      case 4:
        p = storeDistribInfo<uint32_t>(p, m_inConstruction);
        break;
      default:
        throw(BucketException("Bad number of bytes", __FILE__, __LINE__));
    }
    assert(static_cast<char *>(p) ==
           &data[nbODistrib * (nbDiffClauseSize << 1)]);

    // store the clauses.
    switch (nbOLit) {
      case 1:
        p = storeClauses<uint8_t>(p, component, m_inConstruction);
        break;
      case 2:
        p = storeClauses<uint16_t>(p, component, m_inConstruction);
        break;
      case 4:
        p = storeClauses<uint32_t>(p, component, m_inConstruction);
        break;
      default:
        throw(BucketException("Bad number of bytes", __FILE__, __LINE__));
    }
    assert(static_cast<char *>(p) == &data[szData]);

    // put the information into the bucket
    assert(0);
  }  // storeFormula
};
}  // namespace d4
