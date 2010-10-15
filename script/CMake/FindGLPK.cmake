# - Check for the presence of GLPK
#
# The following variables are set when GLPK is found:
#  HAVE_GLPK       = Set to true, if all components of GLPK
#                          have been found.
#  GLPK_INCLUDE_DIR   = Include path for the header files of GLPK
#  GLPK_LIBRARIES  = Link these to use GLPK

## -----------------------------------------------------------------------------
## Check for the header files

find_path (GLPK_INCLUDE_DIR glpk.h
  PATHS ${GLPK_INC} /usr/local/include /usr/include /sw/include /opt/local/include
 PATH_SUFFIXES glpk   
)

## -----------------------------------------------------------------------------
## Check for the library

find_library (GLPK_LIBRARIES glpk
  PATHS ${GLPK_LIB} /usr/local/lib /usr/lib /lib /sw/lib /opt/local/lib
  )

## -----------------------------------------------------------------------------
## Actions taken when all components have been found

if (GLPK_INCLUDE_DIR AND GLPK_LIBRARIES)
  set (HAVE_GLPK TRUE)
else (GLPK_INCLUDE_DIR AND GLPK_LIBRARIES)
  if (NOT GLPK_FIND_QUIETLY)
    if (NOT GLPK_INCLUDE_DIR)
      message (STATUS "Unable to find GLPK header files!")
    endif (NOT GLPK_INCLUDE_DIR)
    if (NOT GLPK_LIBRARIES)
      message (STATUS "Unable to find GLPK library files!")
    endif (NOT GLPK_LIBRARIES)
  endif (NOT GLPK_FIND_QUIETLY)
endif (GLPK_INCLUDE_DIR AND GLPK_LIBRARIES)

if (HAVE_GLPK)
  if (NOT GLPK_FIND_QUIETLY)
    message (STATUS "Found components for GLPK")
    message (STATUS "GLPK_INCLUDE_DIR = ${GLPK_INCLUDE_DIR}")
    message (STATUS "GLPK_LIBRARIES = ${GLPK_LIBRARIES}")
  endif (NOT GLPK_FIND_QUIETLY)
else (HAVE_GLPK)
  if (GLPK_FIND_REQUIRED)
    SET(GLPK_LIB "" CACHE PATH "Paths where to additionally look for
    libs")
    SET(GLPK_INC "" CACHE PATH "Paths where to additionally look for
    includes")
    message (FATAL_ERROR "Could not find GLPK!")
  endif (GLPK_FIND_REQUIRED)
endif (HAVE_GLPK)

mark_as_advanced (
  HAVE_GLPK
  GLPK_LIBRARIES
  GLPK_INCLUDE_DIR
  )
