# - Check for the presence of GLU
#
# The following variables are set when GLU is found:
#  HAVE_GLU       = Set to true, if all components of GLU
#                          have been found.
#  GLU_INCLUDE_DIR   = Include path for the header files of GLU
#  GLU_LIBRARIES  = Link these to use GLU

## -----------------------------------------------------------------------------
## Check for the header files

find_path (GLU_INCLUDE_DIR GL/glu.h
  PATHS ${GLU_INC} /usr/local/include /usr/include /sw/include /opt/local/include
  )

## -----------------------------------------------------------------------------
## Check for the library

find_library (GLU_LIBRARIES GLU
  PATHS ${GLU_LIB} /usr/local/lib /usr/lib /lib /sw/lib /opt/local/lib
  )

## -----------------------------------------------------------------------------
## Actions taken when all components have been found

if (GLU_INCLUDE_DIR AND GLU_LIBRARIES)
  set (HAVE_GLU TRUE)
else (GLU_INCLUDE_DIR AND GLU_LIBRARIES)
  if (NOT GLU_FIND_QUIETLY)
    if (NOT GLU_INCLUDE_DIR)
      message (STATUS "Unable to find GLU header files!")
    endif (NOT GLU_INCLUDE_DIR)
    if (NOT GLU_LIBRARIES)
      message (STATUS "Unable to find GLU library files!")
    endif (NOT GLU_LIBRARIES)
  endif (NOT GLU_FIND_QUIETLY)
endif (GLU_INCLUDE_DIR AND GLU_LIBRARIES)

if (HAVE_GLU)
  if (NOT GLU_FIND_QUIETLY)
    message (STATUS "Found components for GLU")
    message (STATUS "GLU_INCLUDE_DIR = ${GLU_INCLUDE_DIR}")
    message (STATUS "GLU_LIBRARIES = ${GLU_LIBRARIES}")
  endif (NOT GLU_FIND_QUIETLY)
else (HAVE_GLU)
  if (GLU_FIND_REQUIRED)
    message (FATAL_ERROR "Could not find GLU!")
  endif (GLU_FIND_REQUIRED)
endif (HAVE_GLU)

mark_as_advanced (
  HAVE_GLU
  GLU_LIBRARIES
  GLU_INCLUDE_DIR
  )
