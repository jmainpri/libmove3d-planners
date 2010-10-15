# - Check for the presence of GMODULE
#
# The following variables are set when GMODULE is found:
#  HAVE_GMODULE       = Set to true, if all components of GMODULE
#                          have been found.
#  GMODULE_INCLUDE_DIR   = Include path for the header files of GMODULE
#  GMODULE_LIBRARIES  = Link these to use GMODULE

## -----------------------------------------------------------------------------
## Check for the header files

#find_path (GMODULE_INCLUDE_DIR glib.h
#  PATHS ${GMODULE_INC} /usr/local/include /usr/include /sw/include /opt/local/include
#  PATH_SUFFIXES glib-2.0
#  )

## -----------------------------------------------------------------------------
## Check for the library

find_library (GMODULE_LIBRARIES glib-2.0
  PATHS ${GMODULE_LIB} /usr/local/lib /usr/lib /lib /sw/lib /opt/local/lib
  )

## -----------------------------------------------------------------------------
## Actions taken when all components have been found

if (GMODULE_LIBRARIES)
  set (HAVE_GMODULE TRUE)
else (GMODULE_LIBRARIES)
  if (NOT GMODULE_FIND_QUIETLY)
    if (NOT GMODULE_LIBRARIES)
      message (STATUS "Unable to find GMODULE library files!")
    endif (NOT GMODULE_LIBRARIES)
  endif (NOT GMODULE_FIND_QUIETLY)
endif (GMODULE_LIBRARIES)

if (HAVE_GMODULE)
  if (NOT GMODULE_FIND_QUIETLY)
    message (STATUS "Found components for GMODULE")
    message (STATUS "GMODULE_LIBRARIES = ${GMODULE_LIBRARIES}")
  endif (NOT GMODULE_FIND_QUIETLY)
else (HAVE_GMODULE)
  if (GMODULE_FIND_REQUIRED)
    SET(GMODULE_LIB "" CACHE PATH "Paths where to additionally look for
    libs")
    message (FATAL_ERROR "Could not find GMODULE!")
  endif (GMODULE_FIND_REQUIRED)
endif (HAVE_GMODULE)

mark_as_advanced (
  HAVE_GMODULE
  GMODULE_LIBRARIES
  )
