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

#include <cstdint>
#include <iostream>
#include <vector>

namespace d4 {

typedef int Var;
typedef uint8_t lbool;

const Var var_Undef = -1;
const lbool l_True = 0;
const lbool l_False = 1;
const lbool l_Undef = 2;

struct Lit {
  int m_x;

  inline bool sign() const { return m_x & 1; }
  inline Var var() const { return m_x >> 1; }
  inline Lit neg() const { return {m_x ^ 1}; }
  inline unsigned intern() const { return m_x; }
  inline int human() const { return (m_x & 1) ? -var() : var(); }

  bool operator==(Lit p) const { return m_x == p.m_x; }
  bool operator!=(Lit p) const { return m_x != p.m_x; }
  bool operator<(Lit p) const {
    return m_x < p.m_x;
  }  // '<' makes p, ~p adjacent in the ordering.

  friend Lit operator~(Lit p);
  friend std::ostream &operator<<(std::ostream &os, Lit l);

  static inline Lit makeLit(Var v, bool sign) { return {(v << 1) + sign}; }
  static inline Lit makeLitFalse(Var v) { return {(v << 1) + 1}; }
  static inline Lit makeLitTrue(Var v) { return {v << 1}; }

  template <class T>
  inline static void rewrite(std::vector<std::vector<Lit>> &clauses,
                             std::vector<Lit> &units,
                             std::vector<std::vector<T>> &res,
                             T (*createLit)(unsigned var, bool sign)) {
    res.clear();
    for (auto &l : units) res.push_back({createLit(l.var(), l.sign())});

    for (auto &cl : clauses) {
      if (cl.size() == 0) continue;
      res.push_back(std::vector<T>());
      for (auto &l : cl) res.back().push_back(createLit(l.var(), l.sign()));
    }
  }  // rewrite

  template <class T1, class T2>
  inline static void rewrite(std::vector<std::vector<T1>> &clauses,
                             std::vector<std::vector<T2>> &res,
                             T2 (*createLit)(T1)) {
    for (auto &cl : clauses) {
      if (cl.size() == 0) continue;
      res.push_back(std::vector<T2>());
      for (auto &l : cl) res.back().push_back(createLit(l));
    }
  }  // rewrite
};

const Lit lit_Undef = {-2};  // }- Useful special constants.
const Lit lit_Error = {-1};  // }

inline void showListLit(std::ostream &out, std::vector<Lit> &v) {
  for (auto &l : v) out << l << " ";
}  // showListLit

inline Lit operator~(Lit p) { return {p.m_x ^ 1}; }
}  // namespace d4
