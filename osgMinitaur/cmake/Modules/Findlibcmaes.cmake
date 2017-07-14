#[[# - Try to find libcmaes, https://github.com/beniz/libcmaes
# Once done this will define
#
#  LIBCMAES_FOUND - system has libcmaes
#  LIBCMES_INCLUDE_DIR - the libcmaes include directory
#  LIBCMAES_LIBRARIES - Link these to use libcmaes
#  LIBCMAES_DEFINITIONS - Compiler switches required for using libcmaes
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.
#


# Copyright (c) 2014, Emmanuel Benazera, <emmanuel.benazera@lri.fr>
#
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.

if ( LIBCMAES_INCLUDE_DIR AND LIBCMAES_LIBRARIES )
   # in cache already
   SET(Libcmaes_FIND_QUIETLY TRUE)
endif ( LIBCMAES_INCLUDE_DIR AND LIBCMAES_LIBRARIES )

# use pkg-config to get the directories and then use these values
# in the FIND_PATH() and FIND_LIBRARY() calls
if( NOT WIN32 )
  find_package(PkgConfig)
  pkg_check_modules(PC_LIBCMAES QUIET libcmaes)

  if( PKG_CONFIG_FOUND )
    set(LIBCMAES_INCLUDE_DIR ${PC_LIBCMAES_INCLUDE_DIRS})
    set(LIBCMAES_LIBRARIES ${PC_LIBCMAES_LIBRARY_DIRS})
    set(LIBCMAES_DEFINITIONS ${PC_LIBCMAES_CFLAGS_OTHER})
  endif( PKG_CONFIG_FOUND )

endif( NOT WIN32 )

find_path(LIBCMAES_INCLUDE_DIR NAMES cmaes.h
  PATHS
  ${PC_LIBCMAES_INCLUDEDIR}
  ${PC_LIBCMAES_INCLUDE_DIRS}
)

find_library(LIBCMAES_LIBRARIES NAMES libcmaes
  PATHS
  ${PC_LIBCMAES_LIBDIR}
  ${PC_LIBCMAES_LIBRARY_DIRS}
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Libcmaes DEFAULT_MSG LIBCMAES_INCLUDE_DIR LIBCMAES_LIBRARIES )

# show the LIBCMAES_INCLUDE_DIR and LIBCMAES_LIBRARIES variables only in the advanced view
mark_as_advanced(LIBCMAES_INCLUDE_DIR LIBCMAES_LIBRARIES )#]]

SET(libcmaes_INCLUDE_SEARCH_PATHS
  /usr/include
  /usr/local/include
  /usr/local/include/libcmaes
  /usr/local/include/surrogates
)

SET(libcmaes_LIB_SEARCH_PATHS
        /lib/
        /lib64/
        /usr/lib
        /usr/lib64
        /usr/local/lib
        /usr/local/lib64
        ~/libcmaes/lib/python2.7/dist-packages
 )

FIND_PATH(libcmaes_INCLUDE_DIR NAMES cmaes.h PATHS ${libcmaes_INCLUDE_SEARCH_PATHS})
FIND_LIBRARY(libcmaes_LIB NAMES cmaes PATHS ${libcmaes_LIB_SEARCH_PATHS})

SET(libcmaes_FOUND ON)

#    Check include files
IF(NOT libcmaes_INCLUDE_DIR)
    SET(libcmaes_FOUND OFF)
    MESSAGE(STATUS "Could not find libcmaes include. Turning libcmaes_FOUND off")
ENDIF()

#    Check libraries
IF(NOT libcmaes_LIB)
    SET(libcmaes_FOUND OFF)
    MESSAGE(STATUS "Could not find libcmaes lib. Turning libcmaes_FOUND off")
ENDIF()

IF (libcmaes_FOUND)
  IF (NOT libcmaes_FIND_QUIETLY)
    MESSAGE(STATUS "Found libcmaes libraries: ${libcmaes_LIB}")
    MESSAGE(STATUS "Found libcmaes include: ${libcmaes_INCLUDE_DIR}")
  ENDIF (NOT libcmaes_FIND_QUIETLY)
ELSE (libcmaes_FOUND)
  IF (libcmaes_FIND_REQUIRED)
    MESSAGE(FATAL_ERROR "Could not find libcmaes")
  ENDIF (libcmaes_FIND_REQUIRED)
ENDIF (libcmaes_FOUND)

MARK_AS_ADVANCED(
    libcmaes_INCLUDE_DIR
    libcmaes_LIB
    libcmaes
)

