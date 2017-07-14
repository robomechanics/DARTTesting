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
        ~/libcmaes/lib/python2.7/dist-packages/
 )

FIND_PATH(libcmaes_INCLUDE_DIR NAMES cmaes.h PATHS ${libcmaes_INCLUDE_SEARCH_PATHS})
FIND_LIBRARY(libcmaes_LIB NAMES lcmaes.so PATHS ${libcmaes_LIB_SEARCH_PATHS})

SET(libcmaes_FOUND ON)

#    Check include files
IF(NOT libcmaes_INCLUDE_DIR)
    SET(libcmaes_FOUND OFF)
    MESSAGE(STATUS "Could not find libcmaes include. Turning libcmaes_FOUND off")
ENDIF()

#    Check libraries
IF(NOT libcmaes_LIB)
    SET(libcmaes_FOUND OFF)
    MESSAGE(STATUS "Could not find libcmaes lib. Turning libcmaes_FOUND off.")
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