#pragma once

#include <string>

using namespace std;

namespace d4 {
  class Config {
  public:
    string input;
    string input_type;
    string method;
    double maxsharpsat_threshold;
    string maxsharpsat_heuristic_phase;
    unsigned maxsharpsat_heuristic_phase_random;
    bool maxsharpsat_option_and_dig;
    bool maxsharpsat_option_greedy_init;
    string preproc;
    string scoring_method;
    string occurrence_manager;
    string phase_heuristic;
    string partitioning_heuristic;
    string partitioning_heuristic_bipartite_phase;
    double partitioning_heuristic_bipartite_phase_dynamic;
    int partitioning_heuristic_bipartite_phase_static;
    bool partitioning_heuristic_simplification_equivalence;
    bool partitioning_heuristic_simplification_hyperedge;
    string cache_reduction_strategy;
    unsigned long cache_reduction_strategy_cachet_limit;
    unsigned long cache_reduction_strategy_expectation_limit;
    double cache_reduction_strategy_expectation_ratio;
    unsigned long cache_size_first_page;
    unsigned long cache_size_additional_page;
    string cache_store_strategy;
    string cache_clause_representation;
    unsigned cache_clause_representation_combi_limitVar_sym;
    unsigned cache_clause_representation_combi_limitVar_index;
    unsigned cache_limit_number_variable;
    double cache_limit_ratio;
    bool cache_activated;
    string cache_method;
    bool phase_heuristic_reversed;
    int float_precision;
    bool isFloat;
    string dump_ddnnf;
    string query;
    bool projMC_refinement;
    string keyword_output_format_solution;
    string output_format;

    static Config default_values();
  };
};
