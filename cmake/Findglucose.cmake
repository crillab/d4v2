include(FindPackageHandleStandardArgs)

# Keep track of the original library suffixes to reset them later.
set(_glucose_ORIG_CMAKE_FIND_LIBRARY_SUFFIXES ${CMAKE_FIND_LIBRARY_SUFFIXES})

# Look for .a or .lib libraries in case of a static library.
if(glucose_USE_STATIC_LIBS)
    set(CMAKE_FIND_LIBRARY_SUFFIXES .a .lib)
endif()

# Find library and header.
find_library(glucose_LIBRARY NAMES glucose)
find_path(glucose_INCLUDE_DIR NAMES glucose.h)

# Windows (dynamic): Also find import library.
if(WIN32 AND NOT glucose_USE_STATIC_LIBS)
    set(CMAKE_FIND_LIBRARY_SUFFIXES .dll.a .lib)
    find_library(glucose_IMPORT_LIBRARY NAMES glucose)
endif()

# Reset library suffixes.
set(CMAKE_FIND_LIBRARY_SUFFIXES ${_glucose_ORIG_CMAKE_FIND_LIBRARY_SUFFIXES})

# Register the found package.
if(WIN32 AND NOT glucose_USE_STATIC_LIBS)
    # Windows (dynamic): also require import library.
    find_package_handle_standard_args(glucose REQUIRED_VARS glucose_LIBRARY glucose_IMPORT_LIBRARY glucose_INCLUDE_DIR)
else()
    find_package_handle_standard_args(glucose REQUIRED_VARS glucose_LIBRARY glucose_INCLUDE_DIR)
endif()

if(glucose_FOUND)
    mark_as_advanced(glucose_LIBRARY)
    mark_as_advanced(glucose_IMPORT_LIBRARY)
    mark_as_advanced(glucose_INCLUDE_DIR)

    # Create target in case not already done.
    if(NOT TARGET glucose::glucose)
        if(glucose_USE_STATIC_LIBS)
            add_library(glucose::glucose STATIC IMPORTED)
        else()
            add_library(glucose::glucose SHARED IMPORTED)
        endif()

        # Set library and include paths.
        set_target_properties(glucose::glucose PROPERTIES IMPORTED_LOCATION ${glucose_LIBRARY})
        target_include_directories(glucose::glucose INTERFACE ${glucose_INCLUDE_DIR})

        # Windows (dynamic): Also set import library.
        if(WIN32 AND NOT glucose_USE_STATIC_LIBS)
            set_target_properties(glucose::glucose PROPERTIES IMPORTED_IMPLIB ${glucose_IMPORT_LIBRARY})
        endif()
    endif()
endif()
