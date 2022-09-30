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

FIND_PACKAGE(Boost COMPONENTS program_options REQUIRED)
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})

if (NOT DEFINED BUILD_MODE)
   set(BUILD_MODE 1)
endif()

# build.
if(${BUILD_MODE} EQUAL 1)
  # create an executable
  add_executable(d4_debug ${SRCS} ${HEADERS})

  # PROFILE
  message("Debug mode activated")
  set(CMAKE_CXX_COMPILER /usr/bin/g++-8 CACHE PATH "" FORCE)
  
  set(CMAKE_BUILD_TYPE Debug)
  target_compile_options(d4_debug PUBLIC -std=c++17 -Wall -g -pg -O3 -Wno-class-memaccess -Wno-format -Wno-misleading-indentation -Wno-literal-suffix -Wno-sign-compare -Wno-parentheses -D DEBUG -DCMAKE_EXPORT_COMPILE_COMMANDS=1)
  target_link_libraries (d4_debug -lz -g -pg -lgmpxx -lgmp -O3 ${CMAKE_SOURCE_DIR}/3rdParty/kahypar/build/lib/libkahypar.a ${Boost_LIBRARIES} ${CMAKE_SOURCE_DIR}/3rdParty/patoh/libpatoh.a ${CMAKE_SOURCE_DIR}/3rdParty/glucose-3.0/core/lib_debug.a)

  #target_compile_options(d4_debug PUBLIC -std=c++17 -Wall -g -O1 -Wno-class-memaccess -Wno-format -Wno-misleading-indentation -Wno-literal-suffix -Wno-sign-compare -Wno-parentheses -D DEBUG -DCMAKE_EXPORT_COMPILE_COMMANDS=1)
  #target_link_libraries (d4_debug -lz -g -lgmpxx -lgmp -O1 ${CMAKE_SOURCE_DIR}/3rdParty/kahypar/build/lib/libkahypar.a ${Boost_LIBRARIES} ${CMAKE_SOURCE_DIR}/3rdParty/patoh/libpatoh.a ${CMAKE_SOURCE_DIR}/3rdParty/glucose-3.0/core/lib_debug.a)

  install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/d4_debug
    DESTINATION bin
    RENAME ${CMAKE_PROJECT_NAME}-d4_debug)
elseif(${BUILD_MODE} EQUAL 2)
  message("Static mode activated")

  add_executable(d4_static ${SRCS} ${HEADERS})

  target_compile_options(d4_static PUBLIC -std=c++17 -Wall -O3 -Wno-class-memaccess -Wno-format -Wno-misleading-indentation -Wno-literal-suffix -Wno-sign-compare -Wno-parentheses -D NDEBUG -static)
  target_link_libraries (d4_static -lz -O3 -lgmpxx -lgmp ${CMAKE_SOURCE_DIR}/3rdParty/kahypar/build/lib/libkahypar.a ${Boost_LIBRARIES} ${CMAKE_SOURCE_DIR}/3rdParty/patoh/libpatoh.a -D NDEBUG -static ${CMAKE_SOURCE_DIR}/3rdParty/glucose-3.0/core/lib_static.a)
  
  install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/d4_static
    DESTINATION bin
    RENAME ${CMAKE_PROJECT_NAME}-d4_static)
elseif(${BUILD_MODE} EQUAL 3)
  message("Library creation")
  add_library(d4 ${SRCS} ${HEADERS})

  target_compile_options(d4 PUBLIC -DNOMAIN -std=c++17 -Wall -O3 -Wno-class-memaccess -Wno-format -Wno-misleading-indentation -Wno-literal-suffix -Wno-sign-compare -Wno-parentheses -D NDEBUG)
  target_link_libraries (d4 -lz -O3 -lgmpxx -lgmp ${CMAKE_SOURCE_DIR}/3rdParty/kahypar/build/lib/libkahypar.a ${Boost_LIBRARIES} ${CMAKE_SOURCE_DIR}/3rdParty/patoh/libpatoh.a -D NDEBUG ${CMAKE_SOURCE_DIR}/3rdParty/glucose-3.0/core/lib_standard.a)
  target_include_directories (d4 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

  install (TARGETS d4 DESTINATION lib)
  install(FILES ${HEADER} DESTINATION include/${CMAKE_PROJECT_NAME})
else()  
# create an executable
add_executable(d4 ${SRCS} ${HEADERS})

message("Classical mode activated")
target_compile_options(d4 PUBLIC -std=c++17 -Wall -O3 -Wno-class-memaccess -Wno-format -Wno-misleading-indentation -Wno-literal-suffix -Wno-sign-compare -Wno-parentheses)
target_link_libraries (d4 -lz -O3 -lgmpxx -lgmp ${CMAKE_SOURCE_DIR}/3rdParty/kahypar/build/lib/libkahypar.a ${Boost_LIBRARIES} ${CMAKE_SOURCE_DIR}/3rdParty/patoh/libpatoh.a ${CMAKE_SOURCE_DIR}/3rdParty/glucose-3.0/core/lib_standard.a)

install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/d4
  DESTINATION bin
  RENAME ${CMAKE_PROJECT_NAME}-d4)
endif()


file(GLOB cmake_generated ${CMAKE_SOURCE_DIR}/src/*~)

foreach(file ${cmake_generated})

  if (EXISTS ${file})
     file(REMOVE_RECURSE ${file})
  endif()

endforeach(file)



#
# Testing
#

# enable_testing()

# include(cmake_includes/AddGoogleTest.cmake)
# add_executable(HashString_test tests/HashString_test.cpp)
# add_gtest(HashString_test)
