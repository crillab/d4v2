#include "Config.hpp"

namespace d4 {
  Config Config::default_values() {
    Config config;

    // input is required
    config.input_type = "cnf";
    // method is required
    config.maxsharpsat_threshold = -1.0;
    config.maxsharpsat_heuristic_phase = "best";
    config.maxsharpsat_heuristic_phase_random = 5;
    config.maxsharpsat_option_and_dig = true;
    config.maxsharpsat_option_greedy_init = false;
    config.solver = "minisat";
    config.preproc_solver = "minisat";
    config.preproc = "basic";
    config.scoring_method = "vsads";
    config.occurrence_manager = "dynamic";
    config.phase_heuristic = "polarity";
    config.partitioning_heuristic = "decomposition-static-dual";
    config.partitioning_heuristic_bipartite_phase = "none";
    config.partitioning_heuristic_bipartite_phase_dynamic = 0;
    config.partitioning_heuristic_bipartite_phase_static = 0;
    config.partitioning_heuristic_simplification_equivalence = true;
    config.partitioning_heuristic_simplification_hyperedge = true;
    config.cache_reduction_strategy = "expectation";
    config.cache_reduction_strategy_cachet_limit = 10UL * (1<<21);
    config.cache_reduction_strategy_expectation_limit = 100000;
    config.cache_reduction_strategy_expectation_ratio = 0.3;
    config.cache_size_first_page = (1UL<<31);
    config.cache_size_additional_page = (1UL<<29);
    config.cache_store_strategy = "not-touched";
    config.cache_clause_representation = "clause";
    config.cache_clause_representation_combi_limitVar_sym = 20;
    config.cache_clause_representation_combi_limitVar_index = 2000;
    config.cache_limit_number_variable = 100000;
    config.cache_limit_ratio = 0;
    config.cache_activated = true;
    config.cache_method = "list";
    config.phase_heuristic_reversed = false;
    config.float_precision = 128;
    config.isFloat = false;
    config.dump_ddnnf = "";
    config.query = "";
    config.projMC_refinement = false;
    config.keyword_output_format_solution = "s";
    config.output_format = "classic";

    return config;
  }
}
