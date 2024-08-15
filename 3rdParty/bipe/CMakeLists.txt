cmake_minimum_required(VERSION 3.1)

# set the project name and version
project(bipe VERSION 0.1)

add_custom_target(clean-cmake-files
   COMMAND ${CMAKE_COMMAND} -P clean-all.cmake
)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

# specify the C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# source/include files
file(GLOB_RECURSE SOURCES ./src/**.cpp)
file(GLOB_RECURSE INCLUDES ./src/**.hpp)

set(SRCS ${SOURCES})
set(HEADERS ${INCLUDES})

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR})
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/3rdParty/glucose-3.0/)

if (NOT DEFINED BUILD_MODE)
   set(BUILD_MODE 1)
endif()


set(libList ${CMAKE_SOURCE_DIR}/3rdParty/glucose-3.0/core/libglucose.a)

# flags
if(${BUILD_MODE} EQUAL 1)  
  message("Debug: Library creation")
  set(CMAKE_BUILD_TYPE Debug)    
  add_library(bipe ${SRCS})    

  target_compile_options(bipe PUBLIC -DNOMAIN -std=c++20 -Wall -pg -O2 -Wno-class-memaccess -Wno-format -Wno-misleading-indentation -Wno-literal-suffix -Wno-sign-compare -Wno-parentheses)
  target_link_libraries (bipe -lz -O2 ${libList})
  target_include_directories (bipe PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

  install (TARGETS bipe DESTINATION lib)
  install (FILES ${HEADER} DESTINATION include/${CMAKE_PROJECT_NAME})  
elseif(${BUILD_MODE} EQUAL 2)  
  message("Static: Library creation")
  set(CMAKE_BUILD_TYPE Release)
  add_library(bipe ${SRCS})

  target_compile_options(bipe PUBLIC -DNOMAIN -std=c++20 -Wall -O3 -Wno-class-memaccess -Wno-format -Wno-misleading-indentation -Wno-literal-suffix -Wno-sign-compare -Wno-parentheses -static)
  target_link_libraries (bipe -lz -O3 ${libList} -static)
  target_include_directories (bipe PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

  install (TARGETS bipe DESTINATION lib)
  install (FILES ${HEADER} DESTINATION include/${CMAKE_PROJECT_NAME})
elseif(${BUILD_MODE} EQUAL 0)
  message("Library creation")
  add_library(bipe ${SRCS})

  target_compile_options(bipe PUBLIC -DNOMAIN -std=c++20 -Wall -O3 -Wno-class-memaccess -Wno-format -Wno-misleading-indentation -Wno-literal-suffix -Wno-sign-compare -Wno-parentheses)
  target_link_libraries (bipe -lz -O3 ${libList})
  target_include_directories (bipe PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

  install (TARGETS bipe DESTINATION lib)
  install (FILES ${HEADER} DESTINATION include/${CMAKE_PROJECT_NAME})
else()  
  message("Error!")  
endif()


file(GLOB cmake_generated ${CMAKE_SOURCE_DIR}/srcBipe/*~)

foreach(file ${cmake_generated})

  if (EXISTS ${file})
     file(REMOVE_RECURSE ${file})
  endif()

endforeach(file)
