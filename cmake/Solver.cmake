set(D4_SOLVER_OPTIONS "minisat" "glucose")

mark_as_advanced(D4_SOLVER_OPTIONS)

set(D4_SOLVER "minisat" CACHE STRING "The SAT solver to use. Options: ${D4_SOLVER_OPTIONS}")
set(D4_PREPROC_SOLVER "minisat" CACHE STRING "The SAT solver to use for preprocessing. Options: ${D4_SOLVER_OPTIONS}")

if(NOT D4_SOLVER IN_LIST D4_SOLVER_OPTIONS)
    message(FATAL_ERROR "D4_SOLVER must be one of: ${D4_SOLVER_OPTIONS}")
endif()

if(NOT D4_PREPROC_SOLVER IN_LIST D4_SOLVER_OPTIONS)
    message(FATAL_ERROR "D4_PREPROC_SOLVER must be one of: ${D4_SOLVER_OPTIONS}")
endif()

if(D4_SOLVER STREQUAL "glucose" OR D4_PREPROC_SOLVER STREQUAL "glucose")
    set(USE_GLUCOSE ON)
    add_compile_definitions(USE_GLUCOSE)
endif()

add_compile_definitions(D4_SOLVER=${D4_SOLVER})
add_compile_definitions(D4_PREPROC_SOLVER=${D4_PREPROC_SOLVER})
