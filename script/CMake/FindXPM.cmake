# - Check for the presence of XPM
#
# The following variables are set when XPM is found:
#  HAVE_XPM       = Set to true, if all components of XPM
#                          have been found.
#  XPM_INCLUDE_DIR   = Include path for the header files of XPM
#  XPM_LIBRARIES  = Link these to use XPM

## -----------------------------------------------------------------------------
## Check for the library

find_library (XPM_LIBRARIES Xpm
  PATHS ${XPM_LIB} /usr/local/lib /usr/lib /lib /sw/lib /opt/local/lib
  )

## -----------------------------------------------------------------------------
## Actions taken when all components have been found

if (XPM_LIBRARIES)
  set (HAVE_XPM TRUE)
else (XPM_LIBRARIES)
  if (NOT XPM_FIND_QUIETLY)
    if (NOT XPM_LIBRARIES)
      message (STATUS "Unable to find XPM library files!")
    endif (NOT XPM_LIBRARIES)
  endif (NOT XPM_FIND_QUIETLY)
endif (XPM_LIBRARIES)

if (HAVE_XPM)
  if (NOT XPM_FIND_QUIETLY)
    message (STATUS "Found components for XPM")
    message (STATUS "XPM_LIBRARIES     = ${XPM_LIBRARIES}")
  endif (NOT XPM_FIND_QUIETLY)
else (HAVE_XPM)
  if (XPM_FIND_REQUIRED)
    message (FATAL_ERROR "Could not find XPM!")
  endif (XPM_FIND_REQUIRED)
endif (HAVE_XPM)

mark_as_advanced (
  HAVE_XPM
  XPM_LIBRARIES
  )
