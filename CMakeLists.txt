cmake_minimum_required(VERSION 3.27)

project(D4 VERSION 2.0 LANGUAGES CXX)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(GNUInstallDirs)
include(Solver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# FIXME: libmtkahypar currently can not be built statically (https://github.com/kahypar/mt-kahypar/pull/161)
option(BUILD_SHARED_LIBS "Build shared libraries." ON)

if(NOT BUILD_SHARED_LIBS)
    set(GMP_USE_STATIC_LIBS ON)
    set(Boost_USE_STATIC_LIBS ON)
    set(MtKaHyPar_USE_STATIC_LIBS ON)
    set(glucose_USE_STATIC_LIBS ON)
endif()

find_package(GMP REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)
find_package(MtKaHyPar REQUIRED)

# glucose is only needed when requested, which it is not by default.
if(USE_GLUCOSE)
    find_package(glucose REQUIRED)
    include_directories(${glucose_INCLUDE_DIR})
endif()

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${GMP_INCLUDE_DIRS})
include_directories(${GMPXX_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${MtKaHyPar_INCLUDE_DIR})

add_library(caching OBJECT
    src/caching/BucketAllocator.cpp
    src/caching/cnf/BucketInConstruction.cpp
    src/caching/DataInfo.cpp
)

add_library(config OBJECT
    src/config/Config.cpp
    src/config/ConfigConverter.cpp
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
    src/solvers/cnf/WrapperMinisat.cpp
    src/solvers/WrapperSolver.cpp
)

if(USE_GLUCOSE)
    target_sources(solvers PRIVATE src/solvers/cnf/WrapperGlucose.cpp)
endif()

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
    $<TARGET_OBJECTS:config>
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

target_link_libraries(core GMP::GMPXX GMP::GMP MtKaHyPar::MtKaHyPar)

if(USE_GLUCOSE)
    target_link_libraries(core glucose::glucose)
endif()

add_executable(d4 src/Main.cpp)
target_link_libraries(d4 core Boost::program_options)
install(TARGETS d4 DESTINATION ${CMAKE_INSTALL_BINDIR})

add_library(ddnnf_compiler_lib src/DdnnfCompiler.cpp)
target_link_libraries(ddnnf_compiler_lib core)
target_include_directories(ddnnf_compiler_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
set_target_properties(ddnnf_compiler_lib PROPERTIES OUTPUT_NAME ddnnf_compiler)
set_target_properties(ddnnf_compiler_lib PROPERTIES PUBLIC_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/include/DdnnfCompiler.hpp)
install(TARGETS ddnnf_compiler_lib
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

add_executable(ddnnf_compiler src/DdnnfCompilerRunnable.cpp)
target_link_libraries(ddnnf_compiler ddnnf_compiler_lib)
install(TARGETS ddnnf_compiler DESTINATION ${CMAKE_INSTALL_BINDIR})
