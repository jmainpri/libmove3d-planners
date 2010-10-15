# - Check for the presence of OOMOVE3D-CORE
#
# The following variables are set when BioMove3D is found:
#  HAVE_BioMove3D       = Set to true, if all components of BioMove3D
#                          have been found.
#  BioMove3D_INCLUDE_DIR   = Include path for the header files of BioMove3D
#  BioMove3D_LIBRARIES  = Link these to use ooMove3d-core

## -----------------------------------------------------------------------------
## Check for the header files

find_path (p3d_INCLUDE_DIR env.hpp
 PATHS /Users/jmainpri/workspace/ooMove3d-core/p3d  $ENV{ROBOTPKG_BASE}/include/ooMove3D-core/p3d/ /usr/local/include/ooMove3D-core/p3d
 )

find_path (OOMOVE3D-CORE_INCLUDE_DIR P3d-pkg.h
 PATHS /Users/jmainpri/workspace/ooMove3d-core/include $ENV{ROBOTPKG_BASE}/include/ooMove3D-core/include/ /usr/local/include/ooMove3D-core/include
 )

find_library (OOMOVE3D-CORE_LIBRARIES ooMove3D-core
  PATHS ${OOMOVE3D-CORE_LIB} /Users/jmainpri/workspace/ooMove3d-core/build_lib/Debug/lib/macintel $ENV{ROBOTPKG_BASE}/lib /usr/local/lib
  )
message(blalbla)
message(${p3d_INCLUDE_DIR})
message(${OOMOVE3D-CORE_INCLUDE_DIR})
message(${OOMOVE3D-CORE_LIBRARIES})
## -----------------------------------------------------------------------------
## Actions taken when all components have been found

if (OOMOVE3D-CORE_INCLUDE_DIR AND p3d_INCLUDE_DIR AND OOMOVE3D-CORE_LIBRARIES)
 set (HAVE_OOMOVE3D-CORE TRUE)
else (OOMOVE3D-CORE_INCLUDE_DIR AND p3d_INCLUDE_DIR)
 if (NOT OOMOVE3D-CORE_FIND_QUIETLY)
   if (NOT (OOMOVE3D-CORE_INCLUDE_DIR AND p3d_INCLUDE_DIR))
     message (STATUS "WARNING : Unable to find ooMove3d-core header files !")
   endif (NOT (OOMOVE3D-CORE_INCLUDE_DIR AND p3d_INCLUDE_DIR))
 endif (NOT OOMOVE3D-CORE_FIND_QUIETLY)
endif (OOMOVE3D-CORE_INCLUDE_DIR AND p3d_INCLUDE_DIR AND OOMOVE3D-CORE_LIBRARIES)

if (HAVE_OOMOVE3D-CORE)
 if (NOT OOMOVE3D-CORE_FIND_QUIETLY)
   message (STATUS "Found components for ooMove3d-core")
   message (STATUS "p3d_INCLUDE_DIR = ${p3d_INCLUDE_DIR}")
   message (STATUS "OOMOVE3D-CORE_INCLUDE_DIR = ${OOMOVE3D-CORE_INCLUDE_DIR}")
   message (STATUS "OOMOVE3D-CORE_LIBRARIES = ${OOMOVE3D-CORE_LIBRARIES}")
 endif (NOT OOMOVE3D-CORE_FIND_QUIETLY)
else (HAVE_OOMOVE3D-CORE)
 if (OOMOVE3D-CORE_FIND_REQUIRED)
   message (FATAL_ERROR "Could not find ooMove3d-core!")
 endif (OOMOVE3D-CORE_FIND_REQUIRED)
endif (HAVE_OOMOVE3D-CORE)

#mark_as_advanced (
# HAVE_OOMOVE3D-CORE
# OOMOVE3D-CORE_INCLUDE_DIR
# p3d_INCLUDE_DIR
# OOMOVE3D-CORE_LIBRARIES
# OOMOVE3D-CORE_SOURCE_DIR
# )
