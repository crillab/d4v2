include(FindPackageHandleStandardArgs)

# Keep track of the original library suffixes to reset them later.
set(_gmp_ORIG_CMAKE_FIND_LIBRARY_SUFFIXES ${CMAKE_FIND_LIBRARY_SUFFIXES})

# Look for .a or .lib libraries in case of a static library.
if(GMP_USE_STATIC_LIBS)
    set(CMAKE_FIND_LIBRARY_SUFFIXES .a .lib)
endif()

# Find libraries and headers.
find_library(GMP_LIBRARY NAMES gmp)
find_path(GMP_INCLUDE_DIR NAMES gmp.h)

find_library(GMPXX_LIBRARY NAMES gmpxx)
find_path(GMPXX_INCLUDE_DIR NAMES gmpxx.h)

# Windows (dynamic): Also find import libraries.
if(WIN32 AND NOT GMP_USE_STATIC_LIBS)
    set(CMAKE_FIND_LIBRARY_SUFFIXES .dll.a .lib)
    find_library(GMP_IMPORT_LIBRARY NAMES gmp)
    find_library(GMPXX_IMPORT_LIBRARY NAMES gmpxx)
endif()

# Reset library suffixes.
set(CMAKE_FIND_LIBRARY_SUFFIXES ${_gmp_ORIG_CMAKE_FIND_LIBRARY_SUFFIXES})

# Register the found package.
if(WIN32 AND NOT GMP_USE_STATIC_LIBS)
    # Windows (dynamic): also require import libraries.
    find_package_handle_standard_args(GMP REQUIRED_VARS GMP_LIBRARY GMP_IMPORT_LIBRARY GMP_INCLUDE_DIR GMPXX_LIBRARY GMPXX_IMPORT_LIBRARY GMPXX_INCLUDE_DIR)
else()
    find_package_handle_standard_args(GMP REQUIRED_VARS GMP_LIBRARY GMP_INCLUDE_DIR GMPXX_LIBRARY GMPXX_INCLUDE_DIR)
endif()

if(GMP_FOUND)
    mark_as_advanced(GMP_LIBRARY)
    mark_as_advanced(GMP_IMPORT_LIBRARY)
    mark_as_advanced(GMP_INCLUDE_DIR)
    mark_as_advanced(GMPXX_LIBRARY)
    mark_as_advanced(GMPXX_IMPORT_LIBRARY)
    mark_as_advanced(GMPXX_INCLUDE_DIR)

    # Create targets in case not already done.
    if(NOT TARGET GMP::GMP)
        if(GMP_USE_STATIC_LIBS)
            add_library(GMP::GMP STATIC IMPORTED)
        else()
            add_library(GMP::GMP SHARED IMPORTED)
        endif()

        # Set library and include paths.
        set_target_properties(GMP::GMP PROPERTIES IMPORTED_LOCATION ${GMP_LIBRARY})
        target_include_directories(GMP::GMP INTERFACE ${GMP_INCLUDE_DIR})

        # Windows (dynamic): Also set import library.
        if(WIN32 AND NOT GMP_USE_STATIC_LIBS)
            set_target_properties(GMP::GMP PROPERTIES IMPORTED_IMPLIB ${GMP_IMPORT_LIBRARY})
        endif()
    endif()

    if(NOT TARGET GMP::GMPXX)
        if(GMP_USE_STATIC_LIBS)
            add_library(GMP::GMPXX STATIC IMPORTED)
        else()
            add_library(GMP::GMPXX SHARED IMPORTED)
        endif()

        # Set library and include paths.
        set_target_properties(GMP::GMPXX PROPERTIES IMPORTED_LOCATION ${GMPXX_LIBRARY})
        target_include_directories(GMP::GMPXX INTERFACE ${GMPXX_INCLUDE_DIR})

        # Windows (dynamic): Also set import library.
        if(WIN32 AND NOT GMP_USE_STATIC_LIBS)
            set_target_properties(GMP::GMPXX PROPERTIES IMPORTED_IMPLIB ${GMPXX_IMPORT_LIBRARY})
        endif()
    endif()
endif()
