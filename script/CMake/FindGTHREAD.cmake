# - Check for the presence of GTHREAD
#
# The following variables are set when GTHREAD is found:
#  HAVE_GTHREAD       = Set to true, if all components of GTHREAD
#                          have been found.
#  GTHREAD_INCLUDE_DIR   = Include path for the header files of GTHREAD
#  GTHREAD_LIBRARIES  = Link these to use GTHREAD

## -----------------------------------------------------------------------------


## -----------------------------------------------------------------------------
## Check for the library

find_library (GTHREAD_LIBRARIES gthread
  PATHS ${GTHREAD_LIB} /usr/local/lib /usr/lib /lib /sw/lib /opt/local/lib
  )

## -----------------------------------------------------------------------------
## Actions taken when all components have been found

if (GTHREAD_LIBRARIES)
  set (HAVE_GTHREAD TRUE)
else (GTHREAD_LIBRARIES)
  if (NOT GTHREAD_FIND_QUIETLY)
    if (NOT GTHREAD_LIBRARIES)
      message (STATUS "Unable to find GTHREAD library files!")
    endif (NOT GTHREAD_LIBRARIES)
  endif (NOT GTHREAD_FIND_QUIETLY)
endif (GTHREAD_LIBRARIES)

if (HAVE_GTHREAD)
  if (NOT GTHREAD_FIND_QUIETLY)
    message (STATUS "Found components for GTHREAD")
    message (STATUS "GTHREAD_LIBRARIES = ${GTHREAD_LIBRARIES}")
  endif (NOT GTHREAD_FIND_QUIETLY)
else (HAVE_GTHREAD)
  if (GTHREAD_FIND_REQUIRED)
    SET(GTHREAD_LIB "" CACHE PATH "Paths where to additionally look for
    libs")
    message (FATAL_ERROR "Could not find GTHREAD!")
  endif (GTHREAD_FIND_REQUIRED)
endif (HAVE_GTHREAD)

mark_as_advanced (
  HAVE_GTHREAD
  GTHREAD_LIBRARIES
  )
