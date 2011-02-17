# - Check for the presence of MOVE3D-CORE
#
# The following variables are set when BioMove3D is found:
#  HAVE_BioMove3D       = Set to true, if all components of BioMove3D
#                          have been found.
#  BioMove3D_INCLUDE_DIR   = Include path for the header files of BioMove3D
#  BioMove3D_LIBRARIES  = Link these to use ooMove3d-core

## -----------------------------------------------------------------------------
## Check for the header files

find_path (p3d_INCLUDE_DIR env.hpp
 PATHS $ENV{HOME}/workspace/Move3D-core/p3d  $ENV{ROBOTPKG_BASE}/include/Move3D-core/p3d/
 )

find_path (MOVE3D-CORE_INCLUDE_DIR P3d-pkg.h
 PATHS $ENV{HOME}/workspace/Move3D-core/include $ENV{ROBOTPKG_BASE}/include/Move3D-core/include/
 )

find_library (MOVE3D-CORE_LIBRARIES move3d-viewer
  PATHS ${MOVE3D-CORE_LIB} $ENV{HOME}/workspace/Move3D-core/build_lib/Debug/lib/$ENV{HOSTTYPE} $ENV{ROBOTPKG_BASE}/lib/
  )
message(blalbla)
message(${p3d_INCLUDE_DIR})
message(${MOVE3D-CORE_INCLUDE_DIR})
message(${MOVE3D-CORE_LIBRARIES})
## -----------------------------------------------------------------------------
## Actions taken when all components have been found

if (MOVE3D-CORE_INCLUDE_DIR AND p3d_INCLUDE_DIR AND MOVE3D-CORE_LIBRARIES)
 set (HAVE_MOVE3D-CORE TRUE)
else (MOVE3D-CORE_INCLUDE_DIR AND p3d_INCLUDE_DIR)
 if (NOT MOVE3D-CORE_FIND_QUIETLY)
   if (NOT (MOVE3D-CORE_INCLUDE_DIR AND p3d_INCLUDE_DIR))
     message (STATUS "WARNING : Unable to find ooMove3d-core header files !")
   endif (NOT (MOVE3D-CORE_INCLUDE_DIR AND p3d_INCLUDE_DIR))
 endif (NOT MOVE3D-CORE_FIND_QUIETLY)
endif (MOVE3D-CORE_INCLUDE_DIR AND p3d_INCLUDE_DIR AND MOVE3D-CORE_LIBRARIES)

if (HAVE_MOVE3D-CORE)
 if (NOT MOVE3D-CORE_FIND_QUIETLY)
   message (STATUS "Found components for ooMove3d-core")
   message (STATUS "p3d_INCLUDE_DIR = ${p3d_INCLUDE_DIR}")
   message (STATUS "MOVE3D-CORE_INCLUDE_DIR = ${MOVE3D-CORE_INCLUDE_DIR}")
   message (STATUS "MOVE3D-CORE_LIBRARIES = ${MOVE3D-CORE_LIBRARIES}")
 endif (NOT MOVE3D-CORE_FIND_QUIETLY)
else (HAVE_MOVE3D-CORE)
 if (MOVE3D-CORE_FIND_REQUIRED)
   message (FATAL_ERROR "Could not find ooMove3d-core!")
 endif (MOVE3D-CORE_FIND_REQUIRED)
endif (HAVE_MOVE3D-CORE)

#mark_as_advanced (
# HAVE_MOVE3D-CORE
# MOVE3D-CORE_INCLUDE_DIR
# p3d_INCLUDE_DIR
# MOVE3D-CORE_LIBRARIES
# MOVE3D-CORE_SOURCE_DIR
# )
