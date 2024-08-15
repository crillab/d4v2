/**
 * bipe
 *  Copyright (C) 2021  Lagniez Jean-Marie
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Affero General Public License as published
 *  by the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Affero General Public License for more details.
 *
 *  You should have received a copy of the GNU Affero General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "SymGenerate.hpp"

namespace bipe {

void SymGenerate::getSymmetries(const std::string &path,
                                const std::string &file, bool verb,
                                unsigned nbVar,
                                std::vector<std::vector<Var>> &symGroup) {
  if (verb) {
    std::cout << "c [SYMMETRY] Run Saucy to get the symmetries of the input "
                 "file given by "
                 "the file "
              << file << "\n";
    std::cout << "c [SYMMETRY] The number of variable is: " << nbVar << "\n";
  }
  FILE *fp;

  /* Open the command for reading. */
  std::string cmd = path + " " + file;
  fp = popen(cmd.c_str(), "r");
  assert(fp);

  int c = 0;

  // normally we first shoulw read [
  while ((c = fgetc(fp)) != EOF && c != '[')
    ;

  // know we read the symmetries/
  std::vector<std::pair<unsigned, unsigned>> couples;
  while ((c = fgetc(fp)) != EOF) {
    if (c == ' ' || c == '\n') continue;

    // we are done with the current symmetry.
    if (c == ',' || c == ']') {
      if (couples.size()) {
        symGroup.push_back(std::vector<Var>());
        std::vector<Var> &currentSym = symGroup.back();

        for (unsigned i = 0; i <= nbVar; i++) currentSym.push_back(i);
        for (auto c : couples) {
          if (currentSym[c.first] == c.first &&
              currentSym[c.second] == c.second) {
            currentSym[c.first] = c.second;
            currentSym[c.second] = c.first;
          }
          assert(currentSym[c.first] == c.second &&
                 currentSym[c.second] == c.first);
        }

        couples.clear();
      }
    }

    // we read a couple
    if (c == '(') {
      // first value
      unsigned v1 = 0;
      while ((c = fgetc(fp)) != EOF && c >= '0' && c <= '9')
        v1 = v1 * 10 + c - '0';

      assert(c == ',');

      // second value
      unsigned v2 = 0;
      while ((c = fgetc(fp)) != EOF && c >= '0' && c <= '9')
        v2 = v2 * 10 + c - '0';

      assert(c == ')');

      if (v1 > 2 * nbVar)
        assert(v2 > 2 * nbVar);
      else {
        if (v1 > nbVar) v1 -= nbVar;
        if (v2 > nbVar) v2 -= nbVar;
        couples.push_back(std::make_pair(v1, v2));
      }
    }
  }

  pclose(fp);
  if (verb) std::cout << "c [SYMMETRY] Done\n";
}

}  // namespace bipe