cmake_minimum_required(VERSION 3.23)

project(glucose VERSION 3.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

option(BUILD_SHARED_LIBS "Build shared libraries." ON)

if(NOT BUILD_SHARED_LIBS)
    set(ZLIB_USE_STATIC_LIBS ON)
endif()

find_package(ZLIB REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${ZLIB_INCLUDE_DIRS})

add_library(glucose)

target_sources(glucose PRIVATE
    core/Solver.cc
    simp/SimpSolver.cc
    utils/Options.cc
    utils/System.cc
)

target_sources(glucose PUBLIC FILE_SET HEADERS FILES
    core/BoundedQueue.h
    core/Constants.h
    core/Dimacs.h
    core/Solver.h
    core/SolverTypes.h
    mtl/Alg.h
    mtl/Alloc.h
    mtl/Heap.h
    mtl/IntTypes.h
    mtl/Map.h
    mtl/Queue.h
    mtl/Sort.h
    mtl/Vec.h
    mtl/XAlloc.h
    simp/SimpSolver.h
    utils/Options.h
    utils/ParseUtils.h
    utils/System.h
)

target_link_libraries(glucose ZLIB::ZLIB)
set_target_properties(glucose PROPERTIES VERSION ${PROJECT_VERSION})

add_executable(binary simp/Main.cc)
set_target_properties(binary PROPERTIES OUTPUT_NAME glucose)
target_link_libraries(binary glucose)

install(
    TARGETS glucose
    FILE_SET HEADERS DESTINATION include/glucose
)
