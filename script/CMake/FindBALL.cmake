# - Check for the presence of BALL
#
# The following variables are set when BALL is found:
#  HAVE_BALL       = Set to true, if all components of BALL
#                          have been found.
#  BALL_INCLUDE_DIR   = Include path for the header files of BALL
#  BALL_LIBRARIES  = Link these to use BALL

## -----------------------------------------------------------------------------
## Check for the header files

find_path (BALL_INCLUDE_DIR "BALL/common.h">
  PATHS ${BALL_INC} /usr/local/include /usr/include /sw/include /opt/local/include
  )

## -----------------------------------------------------------------------------
## Check for the library

find_library (BALL_LIBRARIES "BALL"
  PATHS ${BALL_LIB} /usr/local/lib /usr/lib /lib /sw/lib /opt/local/lib
  )

## -----------------------------------------------------------------------------
## Actions taken when all components have been found

if (BALL_INCLUDE_DIR AND BALL_LIBRARIES)
  set (HAVE_BALL TRUE)
else (BALL_INCLUDE_DIR AND BALL_LIBRARIES)
  if (NOT BALL_FIND_QUIETLY)
    if (NOT BALL_INCLUDE_DIR)
      message (STATUS "Unable to find BALL header files!")
    endif (NOT BALL_INCLUDE_DIR)
    if (NOT BALL_LIBRARIES)
      message (STATUS "Unable to find BALL library files!")
    endif (NOT BALL_LIBRARIES)
  endif (NOT BALL_FIND_QUIETLY)
endif (BALL_INCLUDE_DIR AND BALL_LIBRARIES)

if (HAVE_BALL)
  if (NOT BALL_FIND_QUIETLY)
    message (STATUS "Found components for BALL")
    message (STATUS "BALL_INCLUDE_DIR = ${BALL_INCLUDE_DIR}")
    message (STATUS "BALL_LIBRARIES = ${BALL_LIBRARIES}")
  endif (NOT BALL_FIND_QUIETLY)
else (HAVE_BALL)
  if (BALL_FIND_REQUIRED)
    SET(BALL_LIB "" CACHE PATH "Paths where to additionally look for
    libs")
    SET(BALL_INC "" CACHE PATH "Paths where to additionally look for
    includes")
    message (FATAL_ERROR "Could not find BALL!")
  endif (BALL_FIND_REQUIRED)
endif (HAVE_BALL)

mark_as_advanced (
  HAVE_BALL
  BALL_LIBRARIES
  BALL_INCLUDE_DIR
  )
