# - Check for the presence of GLEW
#
# The following variables are set when GLEW is found:
#  HAVE_GLEW       = Set to true, if all components of GLEW
#                          have been found.
#  GLEW_INCLUDE_DIR   = Include path for the header files of GLEW
#  GLEW_LIBRARIES  = Link these to use GLEW

## -----------------------------------------------------------------------------
## Check for the header files

find_path (GLEW_INCLUDE_DIR GL/glew.h
  PATHS ${GLEW_INC} /usr/local/include /usr/include /sw/include /opt/local/include
  )

## -----------------------------------------------------------------------------
## Check for the library

find_library (GLEW_LIBRARIES GLEW
  PATHS ${GLEW_LIB} /usr/local/lib /usr/lib /lib /sw/lib /opt/local/lib
  )

## -----------------------------------------------------------------------------
## Actions taken when all components have been found

if (GLEW_INCLUDE_DIR AND GLEW_LIBRARIES)
  set (HAVE_GLEW TRUE)
else (GLEW_INCLUDE_DIR AND GLEW_LIBRARIES)
  if (NOT GLEW_FIND_QUIETLY)
    if (NOT GLEW_INCLUDE_DIR)
      message (STATUS "Unable to find GLEW header files!")
    endif (NOT GLEW_INCLUDE_DIR)
    if (NOT GLEW_LIBRARIES)
      message (STATUS "Unable to find GLEW library files!")
    endif (NOT GLEW_LIBRARIES)
  endif (NOT GLEW_FIND_QUIETLY)
endif (GLEW_INCLUDE_DIR AND GLEW_LIBRARIES)

if (HAVE_GLEW)
  if (NOT GLEW_FIND_QUIETLY)
    message (STATUS "Found components for GLEW")
    message (STATUS "GLEW_INCLUDE_DIR = ${GLEW_INCLUDE_DIR}")
    message (STATUS "GLEW_LIBRARIES = ${GLEW_LIBRARIES}")
  endif (NOT GLEW_FIND_QUIETLY)
else (HAVE_GLEW)
  if (GLEW_FIND_REQUIRED)
    message (FATAL_ERROR "Could not find GLEW!")
  endif (GLEW_FIND_REQUIRED)
endif (HAVE_GLEW)

mark_as_advanced (
  HAVE_GLEW
  GLEW_LIBRARIES
  GLEW_INCLUDE_DIR
  )
