# - Check for the presence of MOVE3D-CORE
#
# The following variables are set when BioMove3D is found:
#  HAVE_BioMove3D       = Set to true, if all components of BioMove3D
#                          have been found.
#  BioMove3D_INCLUDE_DIR   = Include path for the header files of BioMove3D
#  BioMove3D_LIBRARIES  = Link these to use ooMove3d-core

## -----------------------------------------------------------------------------
## Check for the header files

find_path (LIBMOVE3D_LIBMOVE3D_P3D_INCLUDE_DIR env.hpp
 PATHS $ENV{ROBOTPKG_BASE}/include/libmove3d/p3d/ $ENV{HOME}/workspace/libmove3d/p3d  
 )

find_path (LIBMOVE3D_INCLUDE_DIR P3d-pkg.h
 PATHS $ENV{ROBOTPKG_BASE}/include/libmove3d/include/ $ENV{HOME}/workspace/libmove3d/p3d  
 )

find_library (LIBMOVE3D_LIBRARIES move3d
  PATHS ${LIBMOVE3D_LIB} $ENV{ROBOTPKG_BASE}/lib/ $ENV{HOME}/workspace/libmove3d/build_lib/Debug/lib/$ENV{HOSTTYPE}
  )

message(${LIBMOVE3D_LIBMOVE3D_P3D_INCLUDE_DIR})
message(${LIBMOVE3D_INCLUDE_DIR})
message(${LIBMOVE3D_LIBRARIES})
## -----------------------------------------------------------------------------
## Actions taken when all components have been found

if (LIBMOVE3D_INCLUDE_DIR AND LIBMOVE3D_P3D_INCLUDE_DIR AND MOVE3D-CORE_LIBRARIES)
 set (HAVE_MOVE3D-CORE TRUE)
else (LIBMOVE3D_INCLUDE_DIR AND LIBMOVE3D_P3D_INCLUDE_DIR)
 if (NOT MOVE3D-CORE_FIND_QUIETLY)
   if (NOT (LIBMOVE3D_INCLUDE_DIR AND LIBMOVE3D_P3D_INCLUDE_DIR))
     message (STATUS "WARNING : Unable to find libmove3d header files !")
   endif (NOT (LIBMOVE3D_INCLUDE_DIR AND LIBMOVE3D_P3D_INCLUDE_DIR))
 endif (NOT MOVE3D-CORE_FIND_QUIETLY)
endif (LIBMOVE3D_INCLUDE_DIR AND LIBMOVE3D_P3D_INCLUDE_DIR AND MOVE3D-CORE_LIBRARIES)

if (HAVE_MOVE3D-CORE)
 if (NOT MOVE3D-CORE_FIND_QUIETLY)
   message (STATUS "Found components for libmove3d")
   message (STATUS "LIBMOVE3D_LIBMOVE3D_P3D_INCLUDE_DIR = ${LIBMOVE3D_LIBMOVE3D_P3D_INCLUDE_DIR}")
   message (STATUS "LIBMOVE3D_INCLUDE_DIR = ${LIBMOVE3D_INCLUDE_DIR}")
   message (STATUS "LIBMOVE3D_LIBRARIES = ${LIBMOVE3D_LIBRARIES}")
 endif (NOT MOVE3D-CORE_FIND_QUIETLY)
else (HAVE_MOVE3D-CORE)
 if (MOVE3D-CORE_FIND_REQUIRED)
   message (FATAL_ERROR "Could not find libmove3d!")
 endif (MOVE3D-CORE_FIND_REQUIRED)
endif (HAVE_MOVE3D-CORE)

#mark_as_advanced (
# HAVE_MOVE3D-CORE
# LIBMOVE3D_INCLUDE_DIR
# LIBMOVE3D_P3D_INCLUDE_DIR
# MOVE3D-CORE_LIBRARIES
# MOVE3D-CORE_SOURCE_DIR
# )
