# - Check for the presence of Eigen
#
# The following variables are set when Eigen is found:
#  HAVE_Eigen       = Set to true, if all components of Eigen
#                          have been found.
#  Eigen_INCLUDE_DIR   = Include path for the header files of Eigen
#  Eigen_LIBRARIES  = Link these to use Eigen

## -----------------------------------------------------------------------------
## Check for the header files

find_package(PkgConfig)

find_path (Eigen_INCLUDE_DIR Eigen/Cholesky
 PATHS $ENV{ROBOTPKG_BASE}/include /usr/local/include /usr/include /sw/include /opt/local/include ${PC_EIGEN_INCLUDEDIR} ${PC_EIGEN_INCLUDE_DIRS}
 PATH_SUFFIXES eigen2 eigen
 NO_CMAKE_SYSTEM_PATH
 )

## -----------------------------------------------------------------------------
## Actions taken when all components have been found

if (Eigen_INCLUDE_DIR)
 set (HAVE_Eigen TRUE)
else (Eigen_INCLUDE_DIR)
 if (NOT Eigen_FIND_QUIETLY)
   if (NOT Eigen_INCLUDE_DIR)
     message (STATUS "WARNING : Unable to find Eigen header files !")
   endif (NOT Eigen_INCLUDE_DIR)
 endif (NOT Eigen_FIND_QUIETLY)
endif (Eigen_INCLUDE_DIR)

if (HAVE_Eigen)
 if (NOT Eigen_FIND_QUIETLY)
   message (STATUS "Found components for Eigen")
   message (STATUS "Eigen_INCLUDE_DIR = ${Eigen_INCLUDE_DIR}")
 endif (NOT Eigen_FIND_QUIETLY)
else (HAVE_Eigen)
 if (Eigen_FIND_REQUIRED)
   message (FATAL_ERROR "Could not find Eigen!")
 endif (Eigen_FIND_REQUIRED)
endif (HAVE_Eigen)

mark_as_advanced (
 HAVE_Eigen
 Eigen_INCLUDE_DIR
 )
