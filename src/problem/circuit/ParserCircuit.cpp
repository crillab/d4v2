/*
 * d4
 * Copyright (C) 2024  Univ. Artois & CNRS & KU Leuven
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

#include "ParserCircuit.hpp"

#include <stdio.h>
#include <stdlib.h>

#include <algorithm>
#include <cassert>
#include <iostream>
#include <limits>
#include <unordered_map>
#include <vector>

#include "src/problem/ProblemManager.hpp"
#include "src/problem/circuit/ProblemManagerCircuit.hpp"

namespace d4 {

/**
 * @brief LitNameMap is a wrapper around an unordered_map,
 * used to map literal names (string) to their corresponding Literal.
 *
 * It keeps track of a nextVar, to assign newly observed litnames.
 */
class LitNameMap {
 private:
  std::unordered_map<std::string, Lit> name_map;

 public:
  Var nextVar = 1;

  Lit get_lit(std::string &name) {
    // TODO: QoL can we change/pass a constructor to name_map, to replace this
    // if-check?
    if (!name_map.contains(name)) name_map[name] = Lit::makeLitTrue(nextVar++);
    return name_map[name];
  }

  Lit add_new(std::string &name) {
    if (name_map.contains(name))
      throw std::invalid_argument("Key " + name + " already defined.");
    name_map[name] = Lit::makeLitTrue(nextVar++);
    return name_map[name];
  }
};

/**
 * @brief Auxiliary method to process gate definition.
 * This expects a line of the form `G varname := FlatFormula`
 * It then adds this gate to gates.
 *
 * @param line The line to process.
 * @param nextWord A string to store the next word in.
 * @param gates The vector of gates to add the new gate to.
 * @param litname_map A map from varname to the assigned literal.
 */
inline void process_gate_definition(std::string &line, std::string &nextWord,
                                    std::vector<BcGate> &gates,
                                    LitNameMap &litname_map) {
  // expected line format: G name := (A|O) name1 name2 ... namen
  assert(line[0] == 'G');
  std::stringstream linestream(line);
  linestream >> nextWord;  // eat 'G'

  // prepare gate
  gates.emplace_back();
  BcGate &gate = gates.back();

  // - gate_name and output
  linestream >> nextWord;  // eat gate name
  assert(nextWord[0] != '-');
  gate.output = litname_map.get_lit(nextWord);

  linestream >> nextWord;  // eat ':='
  assert(nextWord.compare(":=") == 0);

  // - gate_type
  linestream >> nextWord;  // eat 'A' or 'O'
  assert(nextWord[0] == 'A' || nextWord[0] == 'O' || nextWord[0] == 'I');
  switch(nextWord[0]) {
    case 'A':
      gate.gate_type = BcGateType::AND;
      break;
    case 'O':
      gate.gate_type = BcGateType::OR;
      break;
    case 'I':
      gate.gate_type = BcGateType::IDENTITY;
      break;
  }

  // - gate_inputs
  //      fill input by converting names to literals
  //      line format: "name1 name2 ... namen\n"
  std::vector<Lit> &lits = gate.input;
  while (linestream >> nextWord) {
    bool sign = nextWord[0] == '-';
    if (sign) {
      std::string nextWordp = nextWord.substr(1, std::string::npos);
      Lit lit = litname_map.get_lit(nextWordp);
      lits.push_back(~lit);
    } else {
      lits.push_back(litname_map.get_lit(nextWord));
    }
  }
  assert(lits.size() >= 2 || (gate.gate_type == BcGateType::IDENTITY && lits.size() == 1));
  std::sort(lits.begin(), lits.end());
  // TODO: QoL check for redundant gate? (e.g., l v -l,  l ^ -l,   l v l,  or l
  // ^ l)
}

/**
 * @brief Auxiliary method to process a "true statement" in a Bc file.
 * This expects a line of the form `T litname`.
 * It then adds the literal corresponding to litname, to true_lits
 *
 * @param line The line to process.
 * @param nextWord A string to store the next word in.
 * @param litname_map A map from literal name to the assigned literal.
 * @param true_lits A vector of literals that must be true to satisfy the
 * formula. These literals may both be gate output literals, or input literals.
 */
inline void process_true_statement(std::string &line, std::string &nextWord,
                                   LitNameMap &litname_map,
                                   std::vector<Lit> &true_lits) {
  // expected line format: T var
  assert(line[0] == 'T');
  std::stringstream linestream(line);
  linestream >> nextWord;  // eat 'T'
  linestream >> nextWord;
  bool sign = nextWord[0] == '-';
  if (sign) {
    std::string nextWordp = nextWord.substr(1, std::string::npos);
    Lit lit = litname_map.get_lit(nextWordp);
    true_lits.push_back(~lit);
  } else {
    true_lits.push_back(litname_map.get_lit(nextWord));
  }
}

/**
 * Auxiliary method to process a weight comment.
 * This expects line of the form `c w litname weight`.
 * It then does `weightLit[litname_map[litname]] = weight`.
 *
 * @param line The line to process.
 * @param nextWord A string to store the next word in.
 * @param litname_map A map from literal name to the assigned literal.
 * @param weightLit A vector to store the weights of each literal.
 */
inline void process_weight_comment(std::string &line, std::string &nextWord,
                                   LitNameMap &litname_map,
                                   std::vector<mpz::mpf_float> &weightLit) {
  // expected line format: c w litname weight
  assert(line.starts_with("c w "));
  std::stringstream linestream(line);
  linestream >> nextWord;  // eat 'c'
  linestream >> nextWord;  // eat 'w'
  // litname
  linestream >> nextWord;
  bool sign = nextWord[0] == '-';
  std::string name = (sign) ? nextWord.substr(1, std::string::npos) : nextWord;
  Lit lit = litname_map.get_lit(name);
  if (sign) {
    lit = ~lit;
  }
  // weight
  linestream >> nextWord;
  boost::multiprecision::mpf_float weight =
      boost::multiprecision::mpf_float(nextWord);
  assert(weightLit.size() > lit.intern());  // TODO: should we resize prior?
  weightLit[lit.intern()] = weight;
}

/**
 * @brief Auxiliary method to process projected vars.
 * This expects a line of the form `P v1 v2 v3 ... vn`.
 *
 * @param line The line to process.
 * @param nextWord A string to store the next word in.
 * @param projected_vars The output vector containing projected vars.
 * @param litname_map A map from varname to the assigned literal.
 */
inline void process_projected_vars(std::string &line, std::string &nextWord,
                                    std::vector<Var> &projected_vars,
                                    LitNameMap &litname_map) {
  // expected line format: P v1 v2 v3 ... vn
  assert(line[0] == 'P');
  std::stringstream linestream(line);
  linestream >> nextWord; 

  while (linestream >> nextWord) {
    // Intentionally skip the sign for projected vars
    bool sign = nextWord[0] == '-';
    if (sign) {
      nextWord = nextWord.substr(1, std::string::npos);
    }

    Lit lit = litname_map.get_lit(nextWord);
    projected_vars.push_back(lit.var());
  }
  assert(projected_vars.size() >= 1);
  std::sort(projected_vars.begin(), projected_vars.end());
}

/**
 * @brief Parse the BC-S1.2 format in order to extract the formula
 * and literal weights.
 *
 * The BC-S1.0 format is the following:
 * Comment		->	c *\n
 * WeightInfo	->	c w literal weight\n
 * var	        ->  name
 * literal      ->  var | -var
 * circuit		->	statement | statement\ncircuit
 * statement	->	G var := FlatFormula\n |
 *                  I var\n |
 *                  T literal\n |
 *                  P LiteralList\n
 * LiteralList	->	literal | literal LiteralList
 * FlatFormula	->	A LiteralList |
 *                   O LiteralList
 *
 * A statement defines a gate represented by a formula, or
 * declares an input variable that is no gate, or
 * declares a literal that must be true. Multiple of each statements may occur.
 *
 * A formula is either an AND (A), or OR (O).
 * Negation is supported by minus sign in front of a gate or input name.
 * A formula literal is supported by T litname.
 *
 * A LiteralList in O, or A must contain at least 2 literals!
 *
 * @param in the input file stream from which to parse.
 * @param problemManager the place where is store the result.
 * @return an integer that gives the problem's number of variables.
 */
int ParserCircuit::parse_circuit_main(std::ifstream &in,
                                      ProblemManagerCircuit *problemManager) {
  LitNameMap litname_map;  // TODO: Should this be part of problemManager? We
                           // later need those names? Only for input vars?
  Var &nextVar = litname_map.nextVar;

  std::vector<Lit> &true_lits = problemManager->getTrueLiterals();
  std::vector<BcGate> &gates = problemManager->getGates();
  std::vector<mpz::mpf_float> &weightLit = problemManager->getWeightLit();
  std::string line;
  std::string nextWord;
  unsigned int lineNb = 0;
  while (std::getline(in, line)) {
    lineNb++;
    if (line[0] == 'G') {  // gate definition
      // TODO: gate weight; are default weights 1,1?
      process_gate_definition(line, nextWord, gates, litname_map);
    } else if (line[0] == 'T') {  // lit that must be true
      process_true_statement(line, nextWord, litname_map, true_lits);
    } else if (line[0] == 'I') {  // named input var (so no gate)
                                  // nbInputVars++;
      // TODO: read name and assign it literal in case this did not happen yet?
      //  allows a user to force an order on literal assignments...
    } else if (line[0] == 'c') {  // comment
      if (line.starts_with("c w ")) {
        process_weight_comment(line, nextWord, litname_map, weightLit);
      }
    } else if (line[0] == 'P') {
      process_projected_vars(line, nextWord,
        problemManager->getSelectedVar(),
        litname_map);
      // TODO: anything else? 
    } else {
      std::cerr << "ERROR parsing line " << lineNb
                << ". Unknown start character.\n",
          exit(1);
    }
  }
  weightLit.resize((nextVar << 1), 1);
  return nextVar - 1;
}  // parse_circuit_main

int ParserCircuit::parse_circuit(const std::string &input_stream,
                                 ProblemManagerCircuit *problemManager) {
  std::cout << "c [PARSING CIRCUIT] Start:\n";
  std::ifstream istrm(input_stream, std::ios::in);
  if (!istrm.is_open())
    std::cerr << "ERROR! Could not open file: " << input_stream << "\n",
        exit(1);
  int nbVars = parse_circuit_main(istrm, problemManager);

  istrm.close();
  return nbVars;
}  // parse_circuit

}  // namespace d4
