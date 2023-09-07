cmake_minimum_required(VERSION 3.12)

project(D4 VERSION 2.0 LANGUAGES CXX)

option(BUILD_SHARED_LIBS OFF "Whether to build a shared (instead of static) library.")

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(Boost_USE_STATIC_LIBS ON)

find_package(Boost COMPONENTS program_options REQUIRED)

set(CMAKE_FIND_LIBRARY_SUFFIXES .a .lib)

find_library(GMP_LIBRARY gmp REQUIRED)
message(STATUS "Found GMP: ${GMP_LIBRARY}")

find_library(GMPXX_LIBRARY gmpxx REQUIRED)
message(STATUS "Found GMP C++: ${GMPXX_LIBRARY}")

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/glucose-3.0)
include_directories(${Boost_INCLUDE_DIRS})

# Workaround for kahypar's CMake config to find its own modules.
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/kahypar/cmake/modules)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/glucose-3.0 EXCLUDE_FROM_ALL)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/kahypar EXCLUDE_FROM_ALL)

add_executable(d4
    src/Main.cpp
    src/caching/BucketAllocator.cpp
    src/caching/cnf/BucketInConstruction.cpp
    src/caching/DataInfo.cpp
    src/heuristics/cnf/PartitioningHeuristicBipartite.cpp
    src/heuristics/cnf/PartitioningHeuristicBipartiteDual.cpp
    src/heuristics/cnf/PartitioningHeuristicBipartitePrimal.cpp
    src/heuristics/cnf/PartitioningHeuristicStatic.cpp
    src/heuristics/cnf/PartitioningHeuristicStaticMulti.cpp
    src/heuristics/cnf/PartitioningHeuristicStaticNone.cpp
    src/heuristics/cnf/PartitioningHeuristicStaticSingle.cpp
    src/heuristics/cnf/PartitioningHeuristicStaticSingleDual.cpp
    src/heuristics/cnf/PartitioningHeuristicStaticSinglePrimal.cpp
    src/heuristics/cnf/PhaseSelectorDynamic.cpp
    src/heuristics/cnf/PhaseSelectorManager.cpp
    src/heuristics/cnf/PhaseSelectorNone.cpp
    src/heuristics/cnf/PhaseSelectorStatic.cpp
    src/heuristics/cnf/ScoringMethodDlcs.cpp
    src/heuristics/cnf/ScoringMethodJwts.cpp
    src/heuristics/cnf/ScoringMethodMom.cpp
    src/heuristics/cnf/ScoringMethodVsads.cpp
    src/heuristics/cnf/ScoringMethodVsids.cpp
    src/heuristics/PartitioningHeuristic.cpp
    src/heuristics/PartitioningHeuristicNone.cpp
    src/heuristics/PhaseHeuristic.cpp
    src/heuristics/PhaseHeuristicFalse.cpp
    src/heuristics/PhaseHeuristicOccurrence.cpp
    src/heuristics/PhaseHeuristicPolarity.cpp
    src/heuristics/PhaseHeuristicTrue.cpp
    src/heuristics/ScoringMethod.cpp
    src/hyperGraph/HyperEdge.cpp
    src/hyperGraph/HyperGraph.cpp
    src/hyperGraph/HyperGraphExtractorDual.cpp
    src/hyperGraph/HyperGraphExtractorPrimal.cpp
    src/methods/MethodManager.cpp
    src/methods/QueryManager.cpp
    src/partitioner/PartitionerKahypar.cpp
    src/partitioner/PartitionerManager.cpp
    src/partitioner/PartitionerPatoh.cpp
    src/preprocs/cnf/PreprocBackboneCnf.cpp
    src/preprocs/cnf/PreprocBasicCnf.cpp
    src/preprocs/PreprocManager.cpp
    src/problem/cnf/ParserDimacs.cpp
    src/problem/cnf/ProblemManagerCnf.cpp
    src/problem/ProblemManager.cpp
    src/problem/ProblemTypes.cpp
    src/solvers/ActivityManager.cpp
    src/solvers/cnf/minisat/Solver.cpp
    src/solvers/cnf/WrapperGlucose.cpp
    src/solvers/cnf/WrapperMinisat.cpp
    src/solvers/WrapperSolver.cpp
    src/specs/cnf/SpecManagerCnf.cpp
    src/specs/cnf/SpecManagerCnfDyn.cpp
    src/specs/SpecManager.cpp
    src/utils/AtMost1Extractor.cpp
    src/utils/cnf/AndGatesExtractor.cpp
    src/utils/EquivExtractor.cpp
)

target_link_libraries(d4 gmp gmpxx glucose kahypar ${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/patoh/libpatoh.a)

install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/d4 DESTINATION bin)
