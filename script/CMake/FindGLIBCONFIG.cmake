# - Check for the presence of GLIBCONFIG
#
# The following variables are set when GLIBCONFIG is found:
#  HAVE_GLIBCONFIG       = Set to true, if all components of GLIBCONFIG
#                          have been found.
#  GLIBCONFIG_INCLUDE_DIR   = Include path for the header files of GLIBCONFIG
#  GLIBCONFIG_LIBRARIES  = Link these to use GLIBCONFIG

## -----------------------------------------------------------------------------
## Check for the header files

find_path (GLIBCONFIG_INCLUDE_DIR glibconfig.h
  PATHS ${GLIBCONFIG_INC} /usr/local/lib/ /usr/lib/ /lib/ /sw/lib/ /opt/local/lib/
  PATH_SUFFIXES glib-2.0/include/ 
  )


## -----------------------------------------------------------------------------
## Actions taken when all components have been found

if (GLIBCONFIG_INCLUDE_DIR)
  set (HAVE_GLIBCONFIG TRUE)
else (GLIBCONFIG_INCLUDE_DIR)
  if (NOT GLIBCONFIG_FIND_QUIETLY)
    if (NOT GLIBCONFIG_INCLUDE_DIR)
      message (STATUS "Unable to find GLIBCONFIG header files!")
    endif (NOT GLIBCONFIG_INCLUDE_DIR)
  endif (NOT GLIBCONFIG_FIND_QUIETLY)
endif (GLIBCONFIG_INCLUDE_DIR)

if (HAVE_GLIBCONFIG)
  if (NOT GLIBCONFIG_FIND_QUIETLY)
    message (STATUS "Found components for GLIBCONFIG")
    message (STATUS "GLIBCONFIG_INCLUDE_DIR = ${GLIBCONFIG_INCLUDE_DIR}")
  endif (NOT GLIBCONFIG_FIND_QUIETLY)
else (HAVE_GLIBCONFIG)
  if (GLIBCONFIG_FIND_REQUIRED)
    SET(GLIBCONFIG_INC "" CACHE PATH "Paths where to additionally look for
    includes")
    message (FATAL_ERROR "Could not find GLIBCONFIG!")
  endif (GLIBCONFIG_FIND_REQUIRED)
endif (HAVE_GLIBCONFIG)

mark_as_advanced (
  HAVE_GLIBCONFIG
  GLIBCONFIG_INCLUDE_DIR
  )
