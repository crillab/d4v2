cmake_minimum_required(VERSION 3.1)

# set the project name and version
project(D4 VERSION 0.1)

add_custom_target(clean-cmake-files
   COMMAND ${CMAKE_COMMAND} -P clean-all.cmake
)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_BUILD_TYPE Release)

# specify the C++ standard
set(CMAKE_CXX_STANDARD_REQUIRED 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# source/include files
file(GLOB_RECURSE SOURCES ./src/**.cpp)
file(GLOB_RECURSE INCLUDES ./src/**.hpp)

set(SRCS ${SOURCES})
set(HEADERS ${INCLUDES})

set(Boost_USE_STATIC_LIBS ON)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR})
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/3rdParty/glucose-3.0/)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/3rdParty/kahypar/)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/3rdParty/bipe/)

INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})

if (NOT DEFINED BUILD_MODE)
   set(BUILD_MODE 1)
endif()

set(libList ${CMAKE_SOURCE_DIR}/3rdParty/kahypar/build/lib/libkahypar.a ${Boost_LIBRARIES} ${CMAKE_SOURCE_DIR}/3rdParty/bipe/build/libbipe.a  ${CMAKE_SOURCE_DIR}/3rdParty/bipe/3rdParty/glucose-3.0/core/libglucose.a ${CMAKE_SOURCE_DIR}/3rdParty/patoh/libpatoh.a ${CMAKE_SOURCE_DIR}/3rdParty/glucose-3.0/core/lib_glucose.a)
set(wnoOption -Wno-strict-aliasing -Wno-class-memaccess -Wno-format -Wno-misleading-indentation -Wno-literal-suffix -Wno-sign-compare -Wno-parentheses)

# build.
if(${BUILD_MODE} EQUAL 1)
  message("Library debug D4")  
  add_library(d4 ${SRCS} ${HEADERS})

  target_compile_options(d4 PUBLIC -DNOMAIN -std=c++20 -Wall -g -O2 ${wnoOption} -U NDEBUG)
  target_link_libraries (d4 -lz -O2  -U NDEBUG -lgmpxx -lgmp ${libList})
  target_include_directories (d4 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

  install (TARGETS d4 DESTINATION lib)
  install(FILES ${HEADER} DESTINATION include/${CMAKE_PROJECT_NAME}-debug)
elseif(${BUILD_MODE} EQUAL 2)
  message("Library static")  
  add_library(d4 ${SRCS} ${HEADERS})

  target_compile_options(d4 PUBLIC -DNOMAIN -std=c++20 -Wall -O3 ${wnoOption} -D NDEBUG -static)
  target_link_libraries (d4 -lz -O3 -lgmpxx -lgmp ${libList} -static)
  target_include_directories (d4 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

  install (TARGETS d4 DESTINATION lib)
  install(FILES ${HEADER} DESTINATION include/${CMAKE_PROJECT_NAME}-static)
elseif(${BUILD_MODE} EQUAL 3)
  message("Library profil")  
  add_library(d4 ${SRCS} ${HEADERS})

  target_compile_options(d4 PUBLIC -DNOMAIN -std=c++20 -Wall -pg -O3 ${wnoOption} -D NDEBUG)
  target_link_libraries (d4 -lz -O3 -pg -lgmpxx -lgmp ${libList})
  target_include_directories (d4 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

  install (TARGETS d4 DESTINATION lib)
  install(FILES ${HEADER} DESTINATION include/${CMAKE_PROJECT_NAME}-profil)
else()  
  message("Library standard")  
  add_library(d4 ${SRCS} ${HEADERS})

  target_compile_options(d4 PUBLIC -DNOMAIN -std=c++20 -Wall -O3 ${wnoOption} -D NDEBUG)
  target_link_libraries (d4 -lz -O3 -lgmpxx -lgmp ${libList})
  target_include_directories (d4 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

  install (TARGETS d4 DESTINATION lib)
  install(FILES ${HEADER} DESTINATION include/${CMAKE_PROJECT_NAME}-standard)
endif()


file(GLOB cmake_generated ${CMAKE_SOURCE_DIR}/src/*~)

foreach(file ${cmake_generated})

  if (EXISTS ${file})
     file(REMOVE_RECURSE ${file})
  endif()

endforeach(file)
