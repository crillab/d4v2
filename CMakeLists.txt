cmake_minimum_required(VERSION 3.16)

project(D4 VERSION 2.0 LANGUAGES CXX)

option(BUILD_SHARED_LIBS "Build shared libraries." OFF)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT BUILD_SHARED_LIBS)
    set(Boost_USE_STATIC_LIBS ON)
endif()

find_package(Boost COMPONENTS program_options REQUIRED)

set(_gmp_ORIG_CMAKE_FIND_LIBRARY_SUFFIXES ${CMAKE_FIND_LIBRARY_SUFFIXES})

if(NOT BUILD_SHARED_LIBS)
    set(CMAKE_FIND_LIBRARY_SUFFIXES .a .lib)
endif()

find_library(GMP_LIBRARY gmp REQUIRED)
message(STATUS "Found GMP: ${GMP_LIBRARY}")

find_library(GMPXX_LIBRARY gmpxx REQUIRED)
message(STATUS "Found GMP C++: ${GMPXX_LIBRARY}")

set(CMAKE_FIND_LIBRARY_SUFFIXES ${_gmp_ORIG_CMAKE_FIND_LIBRARY_SUFFIXES})

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/glucose-3.0)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/mt-kahypar/include)
include_directories(${Boost_INCLUDE_DIRS})

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/glucose-3.0 EXCLUDE_FROM_ALL)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/mt-kahypar EXCLUDE_FROM_ALL)

add_library(caching OBJECT
    src/caching/BucketAllocator.cpp
    src/caching/cnf/BucketInConstruction.cpp
    src/caching/DataInfo.cpp
)

add_library(heuristics OBJECT
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
)

add_library(hyperGraph OBJECT
    src/hyperGraph/HyperEdge.cpp
    src/hyperGraph/HyperGraph.cpp
    src/hyperGraph/HyperGraphExtractorDual.cpp
    src/hyperGraph/HyperGraphExtractorPrimal.cpp
)

add_library(methods OBJECT
    src/methods/MethodManager.cpp
    src/methods/QueryManager.cpp
)

add_library(partitioner OBJECT
    src/partitioner/PartitionerKahyparMT.cpp
    src/partitioner/PartitionerManager.cpp
)

add_library(preprocs OBJECT
    src/preprocs/cnf/PreprocBackboneCnf.cpp
    src/preprocs/cnf/PreprocBasicCnf.cpp
    src/preprocs/PreprocManager.cpp
)

add_library(problem OBJECT
    src/problem/cnf/ParserDimacs.cpp
    src/problem/cnf/ProblemManagerCnf.cpp
    src/problem/ProblemManager.cpp
    src/problem/ProblemTypes.cpp
)

add_library(solvers OBJECT
    src/solvers/ActivityManager.cpp
    src/solvers/cnf/minisat/Solver.cpp
    src/solvers/cnf/WrapperGlucose.cpp
    src/solvers/cnf/WrapperMinisat.cpp
    src/solvers/WrapperSolver.cpp
)

add_library(spec OBJECT
    src/specs/cnf/SpecManagerCnf.cpp
    src/specs/cnf/SpecManagerCnfDyn.cpp
    src/specs/SpecManager.cpp
)

add_library(utils OBJECT
    src/utils/AtMost1Extractor.cpp
    src/utils/cnf/AndGatesExtractor.cpp
    src/utils/EquivExtractor.cpp
)

add_library(core STATIC
    $<TARGET_OBJECTS:caching>
    $<TARGET_OBJECTS:heuristics>
    $<TARGET_OBJECTS:hyperGraph>
    $<TARGET_OBJECTS:methods>
    $<TARGET_OBJECTS:partitioner>
    $<TARGET_OBJECTS:preprocs>
    $<TARGET_OBJECTS:problem>
    $<TARGET_OBJECTS:solvers>
    $<TARGET_OBJECTS:spec>
    $<TARGET_OBJECTS:utils>
)

target_link_libraries(core ${GMPXX_LIBRARY} ${GMP_LIBRARY} glucose mtkahypar)

add_executable(d4 src/Main.cpp)
target_link_libraries(d4 core)
install(TARGETS d4 DESTINATION ${CMAKE_INSTALL_BINDIR})

add_library(ddnnf_compiler src/DdnnfCompiler.cpp)
target_link_libraries(ddnnf_compiler core)
target_include_directories(ddnnf_compiler PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
set_target_properties(ddnnf_compiler PROPERTIES PUBLIC_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/include/DdnnfCompiler.hpp)
install(TARGETS ddnnf_compiler
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
