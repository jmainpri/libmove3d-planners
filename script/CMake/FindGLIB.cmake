# - Check for the presence of GLIB
#
# The following variables are set when GLIB is found:
#  HAVE_GLIB       = Set to true, if all components of GLIB
#                          have been found.
#  GLIB_INCLUDE_DIR   = Include path for the header files of GLIB
#  GLIB_LIBRARIES  = Link these to use GLIB

## -----------------------------------------------------------------------------
## Check for the header files

find_path (GLIB_INCLUDE_DIR glib.h
  PATHS ${GLIB_INC} /usr/local/include /usr/include /sw/include /opt/local/include
  PATH_SUFFIXES glib-2.0
  )

## -----------------------------------------------------------------------------
## Check for the library

find_library (GLIB_LIBRARIES glib-2.0
  PATHS ${GLIB_LIB} /usr/local/lib /usr/lib /lib /sw/lib /opt/local/lib
  )

## -----------------------------------------------------------------------------
## Actions taken when all components have been found

if (GLIB_INCLUDE_DIR AND GLIB_LIBRARIES)
  set (HAVE_GLIB TRUE)
else (GLIB_INCLUDE_DIR AND GLIB_LIBRARIES)
  if (NOT GLIB_FIND_QUIETLY)
    if (NOT GLIB_INCLUDE_DIR)
      message (STATUS "Unable to find GLIB header files!")
    endif (NOT GLIB_INCLUDE_DIR)
    if (NOT GLIB_LIBRARIES)
      message (STATUS "Unable to find GLIB library files!")
    endif (NOT GLIB_LIBRARIES)
  endif (NOT GLIB_FIND_QUIETLY)
endif (GLIB_INCLUDE_DIR AND GLIB_LIBRARIES)

if (HAVE_GLIB)
  if (NOT GLIB_FIND_QUIETLY)
    message (STATUS "Found components for GLIB")
    message (STATUS "GLIB_INCLUDE_DIR = ${GLIB_INCLUDE_DIR}")
    message (STATUS "GLIB_LIBRARIES = ${GLIB_LIBRARIES}")
  endif (NOT GLIB_FIND_QUIETLY)
else (HAVE_GLIB)
  if (GLIB_FIND_REQUIRED)
    SET(GLIB_LIB "" CACHE PATH "Paths where to additionally look for
    libs")
    SET(GLIB_INC "" CACHE PATH "Paths where to additionally look for
    includes")
    message (FATAL_ERROR "Could not find GLIB!")
  endif (GLIB_FIND_REQUIRED)
endif (HAVE_GLIB)

mark_as_advanced (
  HAVE_GLIB
  GLIB_LIBRARIES
  GLIB_INCLUDE_DIR
  )
