#include "ConfigConverter.hpp"

namespace d4 {
  Config ConfigConverter::fromVariablesMap(boost::program_options::variables_map &vm) {
    Config config;
    config.input = vm["input"].as<string>();
    config.input_type = vm["input-type"].as<string>();
    config.method = vm["method"].as<string>();
    config.maxsharpsat_threshold = vm["maxsharpsat-threshold"].as<double>();
    config.maxsharpsat_heuristic_phase = vm["maxsharpsat-heuristic-phase"].as<string>();
    config.maxsharpsat_heuristic_phase_random = vm["maxsharpsat-heuristic-phase-random"].as<unsigned>();
    config.maxsharpsat_option_and_dig = vm["maxsharpsat-option-and-dig"].as<bool>();
    config.maxsharpsat_option_greedy_init = vm["maxsharpsat-option-greedy-init"].as<bool>();
    config.solver = vm["solver"].as<string>();
    config.preproc_solver = vm["preproc-solver"].as<string>();
    config.preproc = vm["preproc"].as<string>();
    config.scoring_method = vm["scoring-method"].as<string>();
    config.occurrence_manager = vm["occurrence-manager"].as<string>();
    config.phase_heuristic = vm["phase-heuristic"].as<string>();
    config.partitioning_heuristic = vm["partitioning-heuristic"].as<string>();
    config.partitioning_heuristic_bipartite_phase = vm["partitioning-heuristic-bipartite-phase"].as<string>();
    config.partitioning_heuristic_bipartite_phase_dynamic = vm["partitioning-heuristic-bipartite-phase-dynamic"].as<double>();
    config.partitioning_heuristic_bipartite_phase_static = vm["partitioning-heuristic-bipartite-phase-static"].as<int>();
    config.partitioning_heuristic_simplification_equivalence = vm["partitioning-heuristic-simplification-equivalence"].as<bool>();
    config.partitioning_heuristic_simplification_hyperedge = vm["partitioning-heuristic-simplification-hyperedge"].as<bool>();
    config.cache_reduction_strategy = vm["cache-reduction-strategy"].as<string>();
    config.cache_reduction_strategy_cachet_limit = vm["cache-reduction-strategy-cachet-limit"].as<unsigned long>();
    config.cache_reduction_strategy_expectation_limit = vm["cache-reduction-strategy-expectation-limit"].as<unsigned long>();
    config.cache_reduction_strategy_expectation_ratio = vm["cache-reduction-strategy-expectation-ratio"].as<double>();
    config.cache_size_first_page = vm["cache-size-first-page"].as<unsigned long>();
    config.cache_size_additional_page = vm["cache-size-additional-page"].as<unsigned long>();
    config.cache_store_strategy = vm["cache-store-strategy"].as<string>();
    config.cache_clause_representation = vm["cache_clause_representation"].as<string>();
    config.cache_clause_representation_combi_limitVar_sym = vm["cache-clause-representation-combi-limitVar-sym"].as<unsigned>();
    config.cache_clause_representation_combi_limitVar_index = vm["cache-clause-representation-combi-limitVar-index"].as<unsigned>();
    config.cache_limit_number_variable = vm["cache-limit-number-variable"].as<unsigned>();
    config.cache_limit_ratio = vm["cache-limit-ratio"].as<double>();
    config.cache_activated = vm["cache-activated"].as<bool>();
    config.cache_method = vm["cache-method"].as<string>();
    config.phase_heuristic_reversed = vm["phase-heuristic-reversed"].as<bool>();
    config.float_precision = vm["float-precision"].as<int>();
    config.isFloat = vm["float"].as<bool>();
    config.dump_ddnnf = vm["dump-ddnnf"].as<string>();
    config.query = vm["query"].as<string>();
    config.projMC_refinement = vm["projMC-refinement"].as<bool>();
    config.keyword_output_format_solution = vm["keyword-output-format-solution"].as<string>();
    config.output_format = vm["output-format"].as<string>();
  }
};
