# - Check for the presence of QHULL
#
# The following variables are set when QHULL is found:
#  HAVE_QHULL       = Set to true, if all components of QHULL
#                          have been found.
#  QHULL_INCLUDE_DIR   = Include path for the header files of QHULL
#  QHULL_LIBRARIES  = Link these to use QHULL

## -----------------------------------------------------------------------------
## Check for the header files

find_path (QHULL_INCLUDE_DIR qhull_a.h
  PATHS ${QHULL_INC} /usr/local/include /usr/include /sw/include /opt/local/include  $ENV{ROBOTPKG_BASE}/include
  PATH_SUFFIXES qhull
  )

## -----------------------------------------------------------------------------
## Check for the library

find_library (QHULL_LIBRARIES qhull
  PATHS ${QHULL_LIB} /usr/local/lib /usr/lib /lib /sw/lib /opt/local/lib  $ENV{ROBOTPKG_BASE}/lib
  )

## -----------------------------------------------------------------------------
## Actions taken when all components have been found

if (QHULL_INCLUDE_DIR AND QHULL_LIBRARIES)
  set (HAVE_QHULL TRUE)
else (QHULL_INCLUDE_DIR AND QHULL_LIBRARIES)
  if (NOT QHULL_FIND_QUIETLY)
    if (NOT QHULL_INCLUDE_DIR)
      message (STATUS "Unable to find QHULL header files!")
    endif (NOT QHULL_INCLUDE_DIR)
    if (NOT QHULL_LIBRARIES)
      message (STATUS "Unable to find QHULL library files!")
    endif (NOT QHULL_LIBRARIES)
  endif (NOT QHULL_FIND_QUIETLY)
endif (QHULL_INCLUDE_DIR AND QHULL_LIBRARIES)

if (HAVE_QHULL)
  if (NOT QHULL_FIND_QUIETLY)
    message (STATUS "Found components for QHULL")
    message (STATUS "QHULL_INCLUDE_DIR = ${QHULL_INCLUDE_DIR}")
    message (STATUS "QHULL_LIBRARIES = ${QHULL_LIBRARIES}")
  endif (NOT QHULL_FIND_QUIETLY)
else (HAVE_QHULL)
  if (QHULL_FIND_REQUIRED)
    SET(QHULL_LIB "" CACHE PATH "Paths where to additionally look for
    libs")
    SET(QHULL_INC "" CACHE PATH "Paths where to additionally look for
    includes")
    message (FATAL_ERROR "Could not find QHULL!")
  endif (QHULL_FIND_REQUIRED)
endif (HAVE_QHULL)

mark_as_advanced (
  HAVE_QHULL
  QHULL_LIBRARIES
  QHULL_INCLUDE_DIR
  )
