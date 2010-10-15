# - Check for the presence of BioMove3D
#
# The following variables are set when BioMove3D is found:
#  HAVE_BioMove3D       = Set to true, if all components of BioMove3D
#                          have been found.
#  BioMove3D_INCLUDE_DIR   = Include path for the header files of BioMove3D
#  BioMove3D_LIBRARIES  = Link these to use BioMove3D

## -----------------------------------------------------------------------------
## Check for the header files

find_path (p3d_INCLUDE_DIR env.hpp
 PATHS /Users/jmainpri/workspace/BioMove3D/p3d  $ENV{ROBOTPKG_BASE}/include/BioMove3D/p3d/
 )

find_path (BioMove3D_INCLUDE_DIR P3d-pkg.h
 PATHS /Users/jmainpri/workspace/BioMove3D/include $ENV{ROBOTPKG_BASE}/include/BioMove3D/include/
 )

find_library (BioMove3D_LIBRARIES BioMove3D
  PATHS ${BioMove3D_LIB} /Users/jmainpri/workspace/BioMove3D/build_lib/Debug/lib/macintel $ENV{ROBOTPKG_BASE}/lib
  )

## -----------------------------------------------------------------------------
## Actions taken when all components have been found

if (BioMove3D_INCLUDE_DIR AND p3d_INCLUDE_DIR AND BioMove3D_LIBRARIES)
 set (HAVE_BioMove3D TRUE)
else (BioMove3D_INCLUDE_DIR AND p3d_INCLUDE_DIR)
 if (NOT BioMove3D_FIND_QUIETLY)
   if (NOT (BioMove3D_INCLUDE_DIR AND p3d_INCLUDE_DIR))
     message (STATUS "WARNING : Unable to find BioMove3D header files !")
   endif (NOT (BioMove3D_INCLUDE_DIR AND p3d_INCLUDE_DIR))
 endif (NOT BioMove3D_FIND_QUIETLY)
endif (BioMove3D_INCLUDE_DIR AND p3d_INCLUDE_DIR AND BioMove3D_LIBRARIES)

if (HAVE_BioMove3D)
 if (NOT BioMove3D_FIND_QUIETLY)
   message (STATUS "Found components for BioMove3D")
   message (STATUS "p3d_INCLUDE_DIR = ${p3d_INCLUDE_DIR}")
   message (STATUS "BioMove3D_INCLUDE_DIR = ${BioMove3D_INCLUDE_DIR}")
   message (STATUS "BioMove3D_LIBRARIES = ${BioMove3D_LIBRARIES}")
 endif (NOT BioMove3D_FIND_QUIETLY)
else (HAVE_BioMove3D)
 if (BioMove3D_FIND_REQUIRED)
   message (FATAL_ERROR "Could not find BioMove3D!")
 endif (BioMove3D_FIND_REQUIRED)
endif (HAVE_BioMove3D)

#mark_as_advanced (
# HAVE_BioMove3D
# BioMove3D_INCLUDE_DIR
# p3d_INCLUDE_DIR
# BioMove3D_LIBRARIES
# BioMove3D_SOURCE_DIR
# )
