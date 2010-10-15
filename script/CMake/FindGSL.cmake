# - Check for the presence of GSL
#
# The following variables are set when GSL is found:
#  HAVE_GSL       = Set to true, if all components of GSL
#                          have been found.
#  GSL_INCLUDE_DIR   = Include path for the header files of GSL
#  GSL_LIBRARIES  = Link these to use GSL

## -----------------------------------------------------------------------------
## Check for the header files

find_path (GSL_INCLUDE_DIR gsl/gsl_randist.h
  PATHS ${GSL_INC} /usr/local/include /usr/include /sw/include /opt/local/include
  )

## -----------------------------------------------------------------------------
## Check for the library

find_library (GSL_LIBRARIES gsl
  PATHS ${GSL_LIB} /usr/local/lib /usr/lib /usr/lib64/ /lib /sw/lib /opt/local/lib
  )
find_library (GSLCBLAS_LIBRARIES gslcblas
  PATHS ${GSL_LIB} /usr/local/lib /usr/lib /usr/lib64/ /lib /sw/lib /opt/local/lib
  )
SET(GSL_LIBRARIES ${GSL_LIBRARIES} ${GSLCBLAS_LIBRARIES})
UNSET(GSLCBLAS_LIBRARIES CACHE)
## -----------------------------------------------------------------------------
## Actions taken when all components have been found

if (GSL_INCLUDE_DIR AND GSL_LIBRARIES)
  set (HAVE_GSL TRUE)
else (GSL_INCLUDE_DIR AND GSL_LIBRARIES)
  if (NOT GSL_FIND_QUIETLY)
    if (NOT GSL_INCLUDE_DIR)
      message (STATUS "Unable to find GSL header files!")
    endif (NOT GSL_INCLUDE_DIR)
    if (NOT GSL_LIBRARIES)
      message (STATUS "Unable to find GSL library files!")
    endif (NOT GSL_LIBRARIES)
  endif (NOT GSL_FIND_QUIETLY)
endif (GSL_INCLUDE_DIR AND GSL_LIBRARIES)

if (HAVE_GSL)
  if (NOT GSL_FIND_QUIETLY)
    message (STATUS "Found components for GSL")
    message (STATUS "GSL_INCLUDE_DIR = ${GSL_INCLUDE_DIR}")
    message (STATUS "GSL_LIBRARIES = ${GSL_LIBRARIES}")
  endif (NOT GSL_FIND_QUIETLY)
else (HAVE_GSL)
  if (GSL_FIND_REQUIRED)
    SET(GSL_LIB "" CACHE PATH "Paths where to additionally look for
    libs")
    SET(GSL_INC "" CACHE PATH "Paths where to additionally look for
    includes")
    message (FATAL_ERROR "Could not find GSL!")
  endif (GSL_FIND_REQUIRED)
endif (HAVE_GSL)

mark_as_advanced (
  HAVE_GSL
  GSL_LIBRARIES
  GSL_INCLUDE_DIR
  )
