# - Check for the presence of BALL
#
# The following variables are set when BALL is found:
#  HAVE_ROBOPTIM     = Set to true, if all components of BALL
#                          have been found.
#  ROBOPTIM_INCLUDE_DIR   = Include path for the header files of BALL
#  ROBOPTIM_LIBRARIES  = Link these to use BALL

## -----------------------------------------------------------------------------
## Check for the header files

find_path ( ROBOPTIM_INCLUDE_DIR roboptim/trajectory/trajectory.hh
  PATHS ${ROBOPTIM_INC} /opt/openrobots/include ~/openrobots/include
  )

## -----------------------------------------------------------------------------
## Check for the library

MESSAGE(${ROBOPTIM_CORE_LIBRARIES})
MESSAGE(${ROBOPTIM_TRAJ_LIBRARIES})
#MESSAGE(${ROBOPTIM_FSQP_LIBRARIES})

find_library (ROBOPTIM_CORE_LIBRARIES roboptim-core
  PATHS ${ROBOPTIM_LIB} /opt/openrobots/lib ~/openrobots/lib
  )
  
find_library (ROBOPTIM_TRAJ_LIBRARIES roboptim-trajectory
  PATHS ${ROBOPTIM_LIB} /opt/openrobots/lib ~/openrobots/lib
  )
  
find_library (ROBOPTIM_FSQP_LIBRARIES roboptim-core-cfsqp-plugin
 PATHS ${ROBOPTIM_LIB} /opt/openrobots/lib ~/openrobots/lib
)

#find_file(ROBOPTIM_FSQP_LIBRARIES roboptim-core-cfsqp-plugin.so 
#  PATHS ${ROBOPTIM_LIB} /opt/openrobots/lib ~/openrobots/lib
#  )


MESSAGE(${ROBOPTIM_CORE_LIBRARIES})
MESSAGE(${ROBOPTIM_TRAJ_LIBRARIES})
MESSAGE(${ROBOPTIM_FSQP_LIBRARIES})

SET(ROBOPTIM_LIBRARIES ${ROBOPTIM_TRAJ_LIBRARIES} ${ROBOPTIM_CORE_LIBRARIES} ${ROBOPTIM_FSQP_LIBRARIES})

## -----------------------------------------------------------------------------
## Actions taken when all components have been found

if (ROBOPTIM_INCLUDE_DIR AND ROBOPTIM_LIBRARIES)
  set (HAVE_ROBOPTIM TRUE)
else (ROBOPTIM_INCLUDE_DIR AND ROBOPTIM_LIBRARIES)
  if (NOT ROBOPTIM_FIND_QUIETLY)
    if (NOT ROBOPTIM_INCLUDE_DIR)
      message (STATUS "Unable to find ROBOPTIM header files!")
    endif (NOT ROBOPTIM_INCLUDE_DIR)
    if (NOT ROBOPTIM_LIBRARIES)
      message (STATUS "Unable to find ROBOPTIM library files!")
    endif (NOT ROBOPTIM_LIBRARIES)
  endif (NOT ROBOPTIM_FIND_QUIETLY)
endif (ROBOPTIM_INCLUDE_DIR AND ROBOPTIM_LIBRARIES)

if (HAVE_ROBOPTIM)
  if (NOT ROBOPTIM_FIND_QUIETLY)
    message (STATUS "Found components for ROBOPTIM")
    message (STATUS "ROBOPTIM_INCLUDE_DIR = ${ROBOPTIM_INCLUDE_DIR}")
    message (STATUS "ROBOPTIM_LIBRARIES = ${ROBOPTIM_LIBRARIES}")
  endif (NOT ROBOPTIM_FIND_QUIETLY)
else (HAVE_ROBOPTIM)
  if (ROBOPTIM_FIND_REQUIRED)
    SET(ROBOPTIM_LIB "" CACHE PATH "Paths where to additionally look for
    libs")
    SET(ROBOPTIM_INC "" CACHE PATH "Paths where to additionally look for
    includes")
    message (FATAL_ERROR "Could not find ROBOPTIM!")
  endif (ROBOPTIM_FIND_REQUIRED)
endif (HAVE_ROBOPTIM)

mark_as_advanced (
  HAVE_ROBOPTIM
  ROBOPTIM_LIBRARIES
  ROBOPTIM_INCLUDE_DIR
  )
