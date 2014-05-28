# - Check for the presence of SOFTMOTION
#
# The following variables are set when SOFTMOTION is found:
#  HAVE_SOFTMOTION       = Set to true, if all components of SOFTMOTION
#                          have been found.
#  SOFTMOTION_INCLUDE_DIR   = Include path for the header files of SOFTMOTION
#  SOFTMOTION_LIBRARIES  = Link these to use SOFTMOTION

## -----------------------------------------------------------------------------
## Check for the header files

find_path (SOFTMOTION_INCLUDE_DIR softMotion/softMotion.h
  PATHS 
  ${SOFTMOTION_INC} 
  /usr/local/include 
  /usr/include 
  /sw/include 
  /opt/local/include 
  ${CMAKE_CURRENT_SOURCE_DIR}/../dependencies/install/include 
  $ENV{ROBOTPKG_BASE}/include
  $ENV{MOVE3D_INSTALL_DIR}/include
  )
#if(${SOFTMOTION_INCLUDE_DIR} MATCHES "SOFTMOTION_INCLUDE_DIR-NOTFOUND")
#  add_subdirectory(${BioMove3D_SOURCE_DIR}/other_libraries/softMotion)
#endif(${SOFTMOTION_INCLUDE_DIR} MATCHES "SOFTMOTION_INCLUDE_DIR-NOTFOUND")
#find_path (SOFTMOTION_INCLUDE_DIR softMotion/softMotion.h 
#  PATHS /usr/local/include /usr/include /sw/include /opt/local/include ${CMAKE_CURRENT_SOURCE_DIR}/other_libraries/softMotion-libs/build/install/include
#)

## -----------------------------------------------------------------------------
## Check for the library

find_library (SOFTMOTION_LIBRARIES softMotion
  PATHS 
  ${SOFTMOTION_LIB} 
  /usr/local/lib 
  /usr/lib 
  /lib /sw/lib 
  /opt/local/lib 
  ${CMAKE_CURRENT_SOURCE_DIR}/../dependencies/install/lib  
  $ENV{ROBOTPKG_BASE}/lib
  $ENV{MOVE3D_INSTALL_DIR}/lib
  )

## -----------------------------------------------------------------------------
## Actions taken when all components have been found

if (SOFTMOTION_INCLUDE_DIR AND SOFTMOTION_LIBRARIES)
  set (HAVE_SOFTMOTION TRUE)
else (SOFTMOTION_INCLUDE_DIR AND SOFTMOTION_LIBRARIES)
  if (NOT SOFTMOTION_FIND_QUIETLY)
    if (NOT SOFTMOTION_INCLUDE_DIR)
      message (STATUS "Unable to find SOFTMOTION header files!")
    endif (NOT SOFTMOTION_INCLUDE_DIR)
    if (NOT SOFTMOTION_LIBRARIES)
      message (STATUS "Unable to find SOFTMOTION library files!")
    endif (NOT SOFTMOTION_LIBRARIES)
  endif (NOT SOFTMOTION_FIND_QUIETLY)
endif (SOFTMOTION_INCLUDE_DIR AND SOFTMOTION_LIBRARIES)

if (HAVE_SOFTMOTION)
  if (NOT SOFTMOTION_FIND_QUIETLY)
    message (STATUS "Found components for SOFTMOTION")
    message (STATUS "SOFTMOTION_INCLUDE_DIR = ${SOFTMOTION_INCLUDE_DIR}")
    message (STATUS "SOFTMOTION_LIBRARIES = ${SOFTMOTION_LIBRARIES}")
  endif (NOT SOFTMOTION_FIND_QUIETLY)
else (HAVE_SOFTMOTION)
  if (SOFTMOTION_FIND_REQUIRED)
    SET(SOFTMOTION_LIB "" CACHE PATH "Paths where to additionally look for
    libs")
    SET(SOFTMOTION_INC "" CACHE PATH "Paths where to additionally look for
    includes")
    message (FATAL_ERROR "Could not find SOFTMOTION ${CMAKE_CURRENT_SOURCE_DIR}/other_libraries/softMotion/build/install/{include,lib,bin}!")
  endif (SOFTMOTION_FIND_REQUIRED)
endif (HAVE_SOFTMOTION)

mark_as_advanced (
  HAVE_SOFTMOTION
  SOFTMOTION_LIBRARIES
  SOFTMOTION_INCLUDE_DIR
  )
