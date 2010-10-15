# - Check for the presence of <PACKAGE>
#
# The following variables are set when <PACKAGE> is found:
#  HAVE_<PACKAGE>       = Set to true, if all components of <PACKAGE>
#                          have been found.
#  <PACKAGE>_INCLUDE_DIR   = Include path for the header files of <PACKAGE>
#  <PACKAGE>_LIBRARIES  = Link these to use <PACKAGE>

## -----------------------------------------------------------------------------
## Check for the header files

find_path (<PACKAGE>_INCLUDE_DIR <header file(s)>
  PATHS ${<PACKAGE>_INC} /usr/local/include /usr/include /sw/include /opt/local/include
  PATH_SUFFIXES <optional path extension>
  )

## -----------------------------------------------------------------------------
## Check for the library

find_library (<PACKAGE>_LIBRARIES <package name>
  PATHS ${<PACKAGE>_LIB} /usr/local/lib /usr/lib /lib /sw/lib /opt/local/lib
  )

## -----------------------------------------------------------------------------
## Actions taken when all components have been found

if (<PACKAGE>_INCLUDE_DIR AND <PACKAGE>_LIBRARIES)
  set (HAVE_<PACKAGE> TRUE)
else (<PACKAGE>_INCLUDE_DIR AND <PACKAGE>_LIBRARIES)
  if (NOT <PACKAGE>_FIND_QUIETLY)
    if (NOT <PACKAGE>_INCLUDE_DIR)
      message (STATUS "Unable to find <PACKAGE> header files!")
    endif (NOT <PACKAGE>_INCLUDE_DIR)
    if (NOT <PACKAGE>_LIBRARIES)
      message (STATUS "Unable to find <PACKAGE> library files!")
    endif (NOT <PACKAGE>_LIBRARIES)
  endif (NOT <PACKAGE>_FIND_QUIETLY)
endif (<PACKAGE>_INCLUDE_DIR AND <PACKAGE>_LIBRARIES)

if (HAVE_<PACKAGE>)
  if (NOT <PACKAGE>_FIND_QUIETLY)
    message (STATUS "Found components for <PACKAGE>")
    message (STATUS "<PACKAGE>_INCLUDE_DIR = ${<PACKAGE>_INCLUDE_DIR}")
    message (STATUS "<PACKAGE>_LIBRARIES = ${<PACKAGE>_LIBRARIES}")
  endif (NOT <PACKAGE>_FIND_QUIETLY)
else (HAVE_<PACKAGE>)
  if (<PACKAGE>_FIND_REQUIRED)
    SET(<PACKAGE>_LIB "" CACHE PATH "Paths where to additionally look for
    libs")
    SET(<PACKAGE>_INC "" CACHE PATH "Paths where to additionally look for
    includes")
    message (FATAL_ERROR "Could not find <PACKAGE>!")
  endif (<PACKAGE>_FIND_REQUIRED)
endif (HAVE_<PACKAGE>)

mark_as_advanced (
  HAVE_<PACKAGE>
  <PACKAGE>_LIBRARIES
  <PACKAGE>_INCLUDE_DIR
  )
