# - Check for the presence of GTS
#
# The following variables are set when GTS is found:
#  HAVE_GTS       = Set to true, if all components of GTS
#                          have been found.
#  GTS_INCLUDE_DIR   = Include path for the header files of GTS
#  GTS_LIBRARIES  = Link these to use GTS

## -----------------------------------------------------------------------------
## Check for the header files

find_path (GTS_INCLUDE_DIR gts.h
  PATHS ${GTS_INC} /usr/local/include /usr/include /sw/include /opt/local/include
  )

## -----------------------------------------------------------------------------
## Check for the library

find_library (GTS_LIBRARIES gts
  PATHS ${GTS_LIB} /usr/local/lib /usr/lib /lib /sw/lib /opt/local/lib
  )

## -----------------------------------------------------------------------------
## Actions taken when all components have been found

if (GTS_INCLUDE_DIR AND GTS_LIBRARIES)
  set (HAVE_GTS TRUE)
else (GTS_INCLUDE_DIR AND GTS_LIBRARIES)
  if (NOT GTS_FIND_QUIETLY)
    if (NOT GTS_INCLUDE_DIR)
      message (STATUS "Unable to find GTS header files!")
    endif (NOT GTS_INCLUDE_DIR)
    if (NOT GTS_LIBRARIES)
      message (STATUS "Unable to find GTS library files!")
    endif (NOT GTS_LIBRARIES)
  endif (NOT GTS_FIND_QUIETLY)
endif (GTS_INCLUDE_DIR AND GTS_LIBRARIES)

if (HAVE_GTS)
  if (NOT GTS_FIND_QUIETLY)
    message (STATUS "Found components for GTS")
    message (STATUS "GTS_INCLUDE_DIR = ${GTS_INCLUDE_DIR}")
    message (STATUS "GTS_LIBRARIES = ${GTS_LIBRARIES}")
  endif (NOT GTS_FIND_QUIETLY)
else (HAVE_GTS)
  if (GTS_FIND_REQUIRED)
    SET(GTS_LIB "" CACHE PATH "Paths where to additionally look for
    libs")
    SET(GTS_INC "" CACHE PATH "Paths where to additionally look for
    includes")
    message (FATAL_ERROR "Could not find GTS!")
  endif (GTS_FIND_REQUIRED)
endif (HAVE_GTS)

mark_as_advanced (
  HAVE_GTS
  GTS_LIBRARIES
  GTS_INCLUDE_DIR
  )
