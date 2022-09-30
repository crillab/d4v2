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
#include <cassert>
#include <iterator>

namespace d4 {

class HyperEdge {
 private:
  unsigned m_id;
  unsigned *m_data;

 public:
  HyperEdge(unsigned id, unsigned *data);
  inline unsigned getId() const { return m_id; }
  inline unsigned getSize() { return *m_data; }
  inline unsigned *getData() { return m_data; }

  inline void next() {
    m_id++;
    m_data += 1 + *m_data;
  }

  inline unsigned operator[](unsigned i) const { return m_data[1 + i]; }

  class Iterator {
   private:
    unsigned *m_edge;
    unsigned m_pos;

   public:
    Iterator(unsigned *ptr, unsigned pos) : m_edge(ptr), m_pos(pos) {}

    inline unsigned &operator*() { return m_edge[1 + m_pos]; }
    inline unsigned operator->() { return m_edge[1 + m_pos]; }

    Iterator &operator++() {
      m_pos++;
      return *this;
    }
    Iterator operator++(int) {
      Iterator tmp = *this;
      ++(*this);
      return tmp;
    }

    friend bool operator==(const Iterator &a, const Iterator &b) {
      return a.m_pos == b.m_pos;
    }
    friend bool operator!=(const Iterator &a, const Iterator &b) {
      return a.m_pos != b.m_pos;
    }
  };

  Iterator begin() { return Iterator(m_data, 0); }
  Iterator end() { return Iterator(m_data, *m_data); }
};

}  // namespace d4
