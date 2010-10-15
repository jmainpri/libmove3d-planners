# - Check for the presence of HRP2 GIK library
#
# The following variables are set when HRP2 GIK is found:
#  HAVE_HRP2_GIK       = Set to true, if all components of HRP2_GIK
#                          have been found.
#  HRP2_GIK_INCLUDE_DIR   = Include path for the header files of HRP2_GIK
#  HRP2_GIK_LIBRARIES  = Link these to use HRP2_GIK

## -----------------------------------------------------------------------------
## Check for the header files

find_path (HRP2_GIK_INCLUDE_DIR_TEMP genom-openhrp/genom-hrp2.h
  PATHS ${HRP2_GIK_INC} /usr/local/openrobots/include /home/$ENV{LOGNAME}/openrobots/include
  )

SET(HRP2_GIK_INCLUDE_DIR ${HRP2_GIK_INCLUDE_DIR} ${HRP2_GIK_INCLUDE_DIR_TEMP})
UNSET(HRP2_GIK_INCLUDE_DIR_TEMP CACHE)

find_path (HRP2_GIK_INCLUDE_DIR_TEMP hppGikTools.h 
  PATHS ${HRP2_GIK_INC} /usr/local/openrobots/include/hppGik /home/$ENV{LOGNAME}/openrobots/include/hppGik
  )

SET(HRP2_GIK_INCLUDE_DIR ${HRP2_GIK_INCLUDE_DIR} ${HRP2_GIK_INCLUDE_DIR_TEMP})
UNSET(HRP2_GIK_INCLUDE_DIR_TEMP CACHE)

#find_path (HRP2_GIK_INCLUDE_DIR hrp2Dynamics/hrp2OptHumanoidDynamicRobot.h
#  PATHS ${HRP2_GIK_INC} /usr/local/openrobots/include /home/$ENV{LOGNAME}/openrobots/include
#  )
#find_path (HRP2_GIK_INCLUDE_DIR hppGikPlaneConstraint.h
#  PATHS ${HRP2_GIK_INC} /usr/local/openrobots/include/hppGik/constraints /home/$ENV{LOGNAME}/openrobots/include/hppGik/constraints
#  )
#find_path (HRP2_GIK_INCLUDE_DIR hppGikPlaneConstraint.h
#  PATHS ${HRP2_GIK_INC} /usr/local/openrobots/include/hppGik/constraints /home/$ENV{LOGNAME}/openrobots/include/hppGik/constraints
#  )


## -----------------------------------------------------------------------------
## Check for the library

find_library (HRP2_GIK_LIBRARIES hppGik-2.3 
  PATHS ${HRP2_GIK_LIB} /usr/local/openrobots/lib /home/$ENV{LOGNAME}/openrobots/lib/ 
  )
find_library (HRP2_GIK_DYN_LIBRARIES hrp2Dynamics
  PATHS ${HRP2_GIK_LIB} /usr/local/openrobots/lib /home/$ENV{LOGNAME}/openrobots/lib/
  )
SET(HRP2_GIK_LIBRARIES ${HRP2_GIK_LIBRARIES} ${HRP2_GIK_DYN_LIBRARIES})
UNSET(HRP2_GIK_DYN_LIBRARIES CACHE)
## -----------------------------------------------------------------------------
## Actions taken when all components have been found

if (HRP2_GIK_INCLUDE_DIR AND HRP2_GIK_LIBRARIES)
  set (HAVE_HRP2_GIK TRUE)
else (HRP2_GIK_INCLUDE_DIR AND HRP2_GIK_LIBRARIES)
  if (NOT HRP2_GIK_FIND_QUIETLY)
    if (NOT HRP2_GIK_INCLUDE_DIR)
      message (STATUS "Unable to find HRP2_GIK header files!")
    endif (NOT HRP2_GIK_INCLUDE_DIR)
    if (NOT HRP2_GIK_LIBRARIES)
      message (STATUS "Unable to find HRP2_GIK library files!")
    endif (NOT HRP2_GIK_LIBRARIES)
  endif (NOT HRP2_GIK_FIND_QUIETLY)
endif (HRP2_GIK_INCLUDE_DIR AND HRP2_GIK_LIBRARIES)

if (HAVE_HRP2_GIK)
  if (NOT HRP2_GIK_FIND_QUIETLY)
    message (STATUS "Found components for HRP2_GIK")
    message (STATUS "HRP2_GIK_INCLUDE_DIR = ${HRP2_GIK_INCLUDE_DIR}")
    message (STATUS "HRP2_GIK_LIBRARIES = ${HRP2_GIK_LIBRARIES}")
  endif (NOT HRP2_GIK_FIND_QUIETLY)
else (HAVE_HRP2_GIK)
  if (HRP2_GIK_FIND_REQUIRED)
    SET(HRP2_GIK_LIB "" CACHE PATH "Paths where to additionally look for
    libs")
    SET(HRP2_GIK_INC "" CACHE PATH "Paths where to additionally look for
    includes")
    message (FATAL_ERROR "Could not find HRP2_GIK!")
  endif (HRP2_GIK_FIND_REQUIRED)
endif (HAVE_HRP2_GIK)

mark_as_advanced (
  HAVE_HRP2_GIK
  HRP2_GIK_LIBRARIES
  HRP2_GIK_INCLUDE_DIR
  )
