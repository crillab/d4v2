include(FindPackageHandleStandardArgs)

# Keep track of the original library suffixes to reset them later.
set(_mtkahypar_ORIG_CMAKE_FIND_LIBRARY_SUFFIXES ${CMAKE_FIND_LIBRARY_SUFFIXES})

# Look for .a or .lib libraries in case of a static library.
if(MtKaHyPar_USE_STATIC_LIBS)
    set(CMAKE_FIND_LIBRARY_SUFFIXES .a .lib)
endif()

# Find library and header.
find_library(MtKaHyPar_LIBRARY NAMES mtkahypar)
find_path(MtKaHyPar_INCLUDE_DIR NAMES libmtkahypar.h)

# Windows (dynamic): Also find import library.
if(WIN32 AND NOT MtKaHyPar_USE_STATIC_LIBS)
    set(CMAKE_FIND_LIBRARY_SUFFIXES .dll.a .lib)
    find_library(MtKaHyPar_IMPORT_LIBRARY NAMES mtkahypar)
endif()

# Reset library suffixes.
set(CMAKE_FIND_LIBRARY_SUFFIXES ${_mtkahypar_ORIG_CMAKE_FIND_LIBRARY_SUFFIXES})

# Register the found package.
if(WIN32 AND NOT MtKaHyPar_USE_STATIC_LIBS)
    # Windows (dynamic): also require import library.
    find_package_handle_standard_args(MtKaHyPar REQUIRED_VARS MtKaHyPar_LIBRARY MtKaHyPar_IMPORT_LIBRARY MtKaHyPar_INCLUDE_DIR)
else()
    find_package_handle_standard_args(MtKaHyPar REQUIRED_VARS MtKaHyPar_LIBRARY MtKaHyPar_INCLUDE_DIR)
endif()

if(MtKaHyPar_FOUND)
    mark_as_advanced(MtKaHyPar_LIBRARY)
    mark_as_advanced(MtKaHyPar_IMPORT_LIBRARY)
    mark_as_advanced(MtKaHyPar_INCLUDE_DIR)

    # Create target in case not already done.
    if(NOT TARGET MtKaHyPar::MtKaHyPar)
        if(MtKaHyPar_USE_STATIC_LIBS)
            add_library(MtKaHyPar::MtKaHyPar STATIC IMPORTED)
        else()
            add_library(MtKaHyPar::MtKaHyPar SHARED IMPORTED)
        endif()

        # Set library and include paths.
        set_target_properties(MtKaHyPar::MtKaHyPar PROPERTIES IMPORTED_LOCATION ${MtKaHyPar_LIBRARY})
        target_include_directories(MtKaHyPar::MtKaHyPar INTERFACE ${MtKaHyPar_INCLUDE_DIR})

        # Windows (dynamic): Also set import library.
        if(WIN32 AND NOT MtKaHyPar_USE_STATIC_LIBS)
            set_target_properties(MtKaHyPar::MtKaHyPar PROPERTIES IMPORTED_IMPLIB ${MtKaHyPar_IMPORT_LIBRARY})
        endif()
    endif()
endif()
