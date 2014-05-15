#
# Copyright (c) 2010 LAAS/CNRS
# All rights reserved.
#
# Permission to use, copy, modify, and distribute this software for any purpose
# with or without   fee is hereby granted, provided   that the above  copyright
# notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH
# REGARD TO THIS  SOFTWARE INCLUDING ALL  IMPLIED WARRANTIES OF MERCHANTABILITY
# AND FITNESS. IN NO EVENT SHALL THE AUTHOR  BE LIABLE FOR ANY SPECIAL, DIRECT,
# INDIRECT, OR CONSEQUENTIAL DAMAGES OR  ANY DAMAGES WHATSOEVER RESULTING  FROM
# LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR
# OTHER TORTIOUS ACTION,   ARISING OUT OF OR IN    CONNECTION WITH THE USE   OR
# PERFORMANCE OF THIS SOFTWARE.
#
#                                            Jim Mainprice on Sat Sept 26 2010

CMAKE_MINIMUM_REQUIRED (VERSION 2.6.2)

PROJECT (libmove3d-planners C CXX)
	
# Set the project version ------------------------------------------------------

if( NOT PROJECT_VERSION)

if( NOT PROJECT_MAJOR_VERSION)
set( PROJECT_MAJOR_VERSION 1)
endif( NOT PROJECT_MAJOR_VERSION)

if( NOT PROJECT_MINOR_VERSION)
set( PROJECT_MINOR_VERSION 2)
endif( NOT PROJECT_MINOR_VERSION)

if( NOT PROJECT_REVISION)
set( PROJECT_REVISION 0)
endif( NOT PROJECT_REVISION)

set( PROJECT_VERSION "${PROJECT_MAJOR_VERSION}.${PROJECT_MINOR_VERSION}.${PROJECT_REVISION}")

endif( NOT PROJECT_VERSION)

if( NOT DEFINED PROJECT_MAJOR_VERSION)
message( FATAL_ERROR "The project major version has not been defined.")
endif( NOT DEFINED PROJECT_MAJOR_VERSION)
	
if( NOT DEFINED PROJECT_MINOR_VERSION)
message( FATAL_ERROR "The project minor version has not been defined.")
endif( NOT DEFINED PROJECT_MINOR_VERSION)

#if( NOT DEFINED PROJECT_REVISION)
#message( FATAL_ERROR "The project revision has not been defined.")
#endif( NOT DEFINED PROJECT_REVISION)

#message( STATUS "Current project major version is ${PROJECT_MAJOR_VERSION}")
#message( STATUS "Current project minor version is ${PROJECT_MINOR_VERSION}")
#message( STATUS "Current project revision is ${PROJECT_REVISION}")
message( STATUS "Current project version is ${PROJECT_VERSION}")

# Define the project version ----------------------------------------------------


if( NOT NON_NATIVE_TARGET)
add_definitions( -DPROJECT_VERSION="${PROJECT_VERSION}")
endif( NOT NON_NATIVE_TARGET)

# CMake information -------------------------------------------------------------

message( STATUS "CMake System = ${CMAKE_SYSTEM_NAME}")
message( STATUS "CMake Processor = ${CMAKE_SYSTEM_PROCESSOR}")
message( STATUS "CMake Source Dir = ${CMAKE_SOURCE_DIR}")
message( STATUS "CMake Binary Dir = ${CMAKE_BINARY_DIR}")
message( STATUS "CMake Install Prefix = ${CMAKE_INSTALL_PREFIX}")

#-------------------------------------------------------------------------------
# ------------------------------------------------------------------------------
# --- Scripts to search source files -------------------------------------------
# ------------------------------------------------------------------------------
#-------------------------------------------------------------------------------
SET(Move3DMPLIB_CompilationFlags "")
SET(Move3DMPLIB_LinkedCompilationFlags "")

MACRO(Move3DMPLIB_flag flagName description status)
  SET(flagName "${flagName}")
  SET(description "${description}")
  SET(status "${status}")
#  message(status "enter in Move3DMPLIB_flag : ${Move3DMPLIB_CompilationFlags}")
  OPTION(${flagName} ${description} ${status})
  IF(${flagName})
    LIST(APPEND Move3DMPLIB_CompilationFlags ${flagName})
  ENDIF()
  FOREACH(linkFlag ${ARGN})
    LIST(APPEND Move3DMPLIB_LinkedCompilationFlags "${flagName}:${linkFlag}")
  ENDFOREACH(linkFlag)
#  set_property(CACHE ANIMATION PROPERTY VALUE ON)
ENDMACRO(Move3DMPLIB_flag)

MACRO(Move3DMPLIB_CheckCompilationFlags)
  SET(Move3DMPLIB_ActiveFlags "")
  MESSAGE ("Active Flags :")
  FILE(WRITE ${CMAKE_CURRENT_BINARY_DIR}/smpFlags "#Generated File Do not modify!!!\n" )
  FILE(APPEND ${CMAKE_CURRENT_BINARY_DIR}/smpFlags "Move3DMPLIB_COMPILATION_FLAGS = ")
  FOREACH(flag ${Move3DMPLIB_CompilationFlags})
    IF(${flag})
      FILE(APPEND ${CMAKE_CURRENT_BINARY_DIR}/Move3DMPLIBFlags "-D${flag} ")
      MESSAGE("${flag}")
      ADD_DEFINITIONS("-D${flag}")
      LIST(APPEND Move3DMPLIB_ActiveFlags ${flag})
    ENDIF()
  ENDFOREACH(flag)
ENDMACRO(Move3DMPLIB_CheckCompilationFlags)

#-------------------------------------------------------------------------------
# ------------------------------------------------------------------------------
# --- Compilation Flags --------------------------------------------------------
# ------------------------------------------------------------------------------
#-------------------------------------------------------------------------------

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/script/CMake")

#IF(NOT DISABLE_QT)
#	SET(QT_LIBRARY)
#	MESSAGE("WARNING HAVE YOU SET QT_LIBRARY ON!!!")
#ENDIF()

#-------------- Qt Library -----------------------------------------------------
FIND_PACKAGE(Qt4) #Qt4 is not required. If not found, set QT_LIBRARY to OFF.

IF(QT_INCLUDE_DIR)
  MESSAGE("I've found Qt4 libraries. I will compile libmove3d-planners with it.")
  SET(QT_LIBRARY ON CACHE BOOL "" FORCE)
  INCLUDE_DIRECTORIES(${QT_INCLUDE_DIR})
  SET(Move3DMPLIB_COMPILATION_INCLUDES
  ${Move3DMPLIB_COMPILATION_INCLUDES} ${QT_INCLUDE_DIR})
  Move3DMPLIB_flag(QT_LIBRARY "Sets QtLibrary compiling mode" ON)
  IF(QT_GL)
	SET(QT_USE_QTOPENGL TRUE)
  ENDIF(QT_GL)
  INCLUDE(${QT_USE_FILE})
  SET(LIBS ${LIBS} ${QT_LIBRARIES})
ELSE()
  MESSAGE("I've not found Qt4 libraries. I won't compile libmove3d-planners with support for Qt.")
  SET(QT_LIBRARY OFF CACHE BOOL "" FORCE)
ENDIF()

#Move3DMPLIB_flag(QT_LIBRARY "Sets QtLibrary compiling mode" OFF)

Move3DMPLIB_flag(P3D_CONSTRAINTS "Compilation with module Constraints" ON)
Move3DMPLIB_flag(P3D_COLLISION_CHECKING "Compilation with module Collision Checking" ON)
Move3DMPLIB_flag(P3D_LOCALPATH "Compilation with module Localpath" ON)
Move3DMPLIB_flag(P3D_PLANNER "Compilation with module Planner" ON)

Move3DMPLIB_flag(LIGHT_PLANNER "Compilation with module Light Planner" ON)
Move3DMPLIB_flag(GRASP_PLANNING "Compilation with module Manipulation Planner" ON)
Move3DMPLIB_flag(MULTILOCALPATH "Compilation with module Multilocalpath" ON)
Move3DMPLIB_flag(MULTIGRAPH "Compilation with module Multi-Graph" ON)

#Move3DMPLIB_flag(USE_GBM "Compilation with module GBM" ON)
Move3DMPLIB_flag(USE_GSL "Compilation with module Hri-planner" ON)

Move3DMPLIB_flag(HRI_COSTSPACE "With the hri work" ON)
Move3DMPLIB_flag(HRI_PLANNER "With the hri work done in BioMove3D" ON USE_GSL)

Move3DMPLIB_flag(DEBUG_STATUS "Writting for debugs" ON)

Move3DMPLIB_CheckCompilationFlags()

# ------------------------------------------------------------------------------
# --- Default compilation option : ---------------------------------------------
# --- DO NOT MODIFY!!! use ccmake ----------------------------------------------
#-------------------------------------------------------------------------------
#-------------- Modules --------------------------------------------------------
#-------------------------------------------------------------------------------


#-------------- X11 Library -----------------------------------------------------
FIND_PACKAGE(X11 REQUIRED)
INCLUDE_directories(${X11_INCLUDE_DIR})
SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${X11_INCLUDE_DIR})
SET(LIBS ${LIBS} ${X11_LIBRARIES})

FIND_PACKAGE(LibXml2 REQUIRED)
INCLUDE_directories(${LIBXML2_INCLUDE_DIR})
SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${LIBXML2_INCLUDE_DIR})
SET(LIBS ${LIBS} ${LIBXML2_LIBRARIES})

IF(APPLE)
	IF(QT_LIBRARY)
  		FIND_PACKAGE(OpenGL REQUIRED)
  		INCLUDE_directories("${OPENGL_INCLUDE_DIR}/Headers")
  		SET(INCLUDE_DIRS ${INCLUDE_DIRS} "${OPENGL_INCLUDE_DIR}/Headers")
  		SET(LIBS ${LIBS} ${OPENGL_LIBRARIES})
  		Message(${OPENGL_INCLUDE_DIR})
  		Message(${OPENGL_LIBRARIES})
  		IF(WITH_XFORMS)
  			FIND_PACKAGE(GLU REQUIRED)
  			INCLUDE_directories(${GLU_INCLUDE_DIR})
  			SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${GLU_INCLUDE_DIR})
  			SET(LIBS ${LIBS} ${GLU_LIBRARIES})
  			FIND_PACKAGE(GL REQUIRED)
  			INCLUDE_directories(${GL_INCLUDE_DIR})
  			SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${GL_INCLUDE_DIR})
  			SET(LIBS ${LIBS} ${GL_LIBRARIES})
  		ENDIF(WITH_XFORMS)
	ENDIF(QT_LIBRARY)
ELSE(APPLE)
  FIND_PACKAGE(OpenGL REQUIRED)
  INCLUDE_directories(${OPENGL_INCLUDE_DIR})
  SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${OPENGL_INCLUDE_DIR})
  SET(LIBS ${LIBS} ${OPENGL_LIBRARIES})
ENDIF(APPLE)

#-------------- Move3D ---------------------------------------------------------
FIND_PACKAGE (PkgConfig REQUIRED)

#IF(NOT QT_LIBRARY)
#  MESSAGE("IMPORTANT : ----- linking against ----> libmove3d ")
#  PKG_CHECK_MODULES(MOVE3D REQUIRED libmove3d)
#  INCLUDE_DIRECTORIES(${MOVE3D_INCLUDE_DIRS})
#  LINK_DIRECTORIES( ${MOVE3D_LIBRARY_DIRS} )
#  SET(LIBS ${LIBS} ${MOVE3D_LIBRARIES})
#  ADD_DEFINITIONS(${MOVE3D_CFLAGS})
#ELSE()
  MESSAGE("IMPORTANT : ----- linking against ----> libmove3d-viewer ")
  PKG_CHECK_MODULES(MOVE3DVIEWER REQUIRED libmove3d)
  INCLUDE_DIRECTORIES(${MOVE3DVIEWER_INCLUDE_DIRS})
  LINK_DIRECTORIES( ${MOVE3DVIEWER_LIBRARY_DIRS} )
  SET(LIBS ${LIBS} ${MOVE3DVIEWER_LIBRARIES})
  ADD_DEFINITIONS(${MOVE3DVIEWER_CFLAGS})
  #MESSAGE(${MOVE3DVIEWER_CFLAGS})
  MESSAGE("MOVE3DVIEWER_INCLUDE_DIRS = " ${MOVE3DVIEWER_INCLUDE_DIRS})
  MESSAGE("MOVE3DVIEWER_LIBRARIES = " ${MOVE3DVIEWER_LIBRARIES})
#ENDIF()

### libhri dependency
IF(HRI_PLANNER)
  PKG_CHECK_MODULES(LIBHRI REQUIRED libmove3d-hri)
  INCLUDE_DIRECTORIES(${LIBHRI_INCLUDE_DIRS})
  LINK_DIRECTORIES( ${LIBHRI_LIBRARY_DIRS} )
  SET(LIBS ${LIBS} ${LIBHRI_LIBRARIES})
ENDIF()

#-------------- Ioc ------------------------------------------------------------

#SET(IOC_DIR /home/jmainpri/workspace/move3d/install) #/home/jmainpri/openrobots
#INCLUDE_directories(includes)
#SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${IOC_DIR}/includes)
#SET(LIBS ${LIBS} ${IOC_DIR}/lib/libowlqn.so)

#-------------- Eigen ----------------------------------------------------------

message("CMAKE_MODULE_PATH : " ${CMAKE_MODULE_PATH})

FIND_PACKAGE(Eigen3 REQUIRED)
INCLUDE_directories(${EIGEN3_INCLUDE_DIR})
SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
MESSAGE(Eigen3_INCLUDE_DIR : ${EIGEN3_INCLUDE_DIR})

#-------------- Boost ----------------------------------------------------------

# boost is used e.g. for managing the cost functions (T-RRT).
FIND_PACKAGE(Boost REQUIRED COMPONENTS thread)
SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${BOOST_INCLUDE_DIRS})
SET(LIBS ${LIBS} ${Boost_LIBRARIES})
ADD_DEFINITIONS(-pthread)
MESSAGE("Boost_VERSION : ${Boost_VERSION}")
MESSAGE("Boost_INCLUDE_DIRS : ${Boost_INCLUDE_DIRS}")
MESSAGE("Boost_LIBRARIES : ${Boost_LIBRARIES}")

#-------------- GBM ------------------------------------------------------------
#IF(USE_GBM)
#FIND_PACKAGE(GBM REQUIRED)
#INCLUDE_directories(${GBM_INCLUDE_DIR})
#  SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${GBM_INCLUDE_DIR})
#SET(LIBS ${LIBS} ${GBM_LIBRARIES})
#ENDIF(USE_GBM)

#-------------- SOFTMOTION -----------------------------------------------------
IF(MULTILOCALPATH)
  FIND_PACKAGE(SOFTMOTION REQUIRED)
  INCLUDE_directories(${SOFTMOTION_INCLUDE_DIR})
  SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${SOFTMOTION_INCLUDE_DIR})
  SET(LIBS ${LIBS} ${SOFTMOTION_LIBRARIES})
ENDIF(MULTILOCALPATH)

#-------------- PolVox ----------------------------------------------------------

#FIND_PACKAGE(PolyVox REQUIRED)
#INCLUDE_directories(${PolyVox_INCLUDE_DIRS})
#SET(INCLUDE_DIRS ${PolyVox_INCLUDE_DIRS})
#SET(LIBS ${LIBS} ${PolyVox_LIBRARIES})

#-------------- Matio -----------------------------------------------------------
#FIND_PACKAGE(MATIO REQUIRED matio)
#INCLUDE_directories(${MATIO_INCLUDE_DIRS})
#SET(INCLUDE_DIRS ${MATIO_INCLUDE_DIRS})
#SET(LIBS ${LIBS} ${MATIO_LIBRARIES})
#MESSAGE("${MATIO_INCLUDE_DIRS}")
#MESSAGE("${MATIO_LIBRARIES}")

#------------------- PKG_CONFIG ---------------------------
#----------------------------------------------------------

STRING(REPLACE ";" " -D" Move3DMPLIB_CompilationFlags "-D${Move3DMPLIB_CompilationFlags}")
STRING(REPLACE ";" " -I" Move3DMPLIB_Compilation_includes "-I${INCLUDE_DIRS}")

STRING(REGEX REPLACE "(debug;[^;]*;)|optimized" "" Move3DMPLIB_Compilation_libs "${LIBS}")
STRING(REPLACE ";" " " Move3DMPLIB_Compilation_libs "${Move3DMPLIB_Compilation_libs}")
add_subdirectory(pkgconfig)

#-------------------------------------------------------------------------------
# SUBDIR PROCESS 
#-------------------------------------------------------------------------------
SET(Move3DMPLIB_src "")
SET(Move3DMPLIB_inc "")

SET(PROJECT_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})

SET(Move3DMPLIB_SOURCE_DIRECTORIES src)

MACRO(BM3D_SRC_SUBDIR_PROCESS)
  FOREACH(srcfile ${ARGV})
    LIST(APPEND Move3DMPLIB_src "${BM3D_MODULE_NAME}/${srcfile}")
    SOURCE_GROUP(${BM3D_MODULE_NAME} FILES ${BM3D_MODULE_NAME}/${srcfile})
  ENDFOREACH(srcfile)
ENDMACRO(BM3D_SRC_SUBDIR_PROCESS)

MACRO(BM3D_INC_DIR_PROCESS directory)
  include_directories (${directory})
  file (GLOB incfiles "${directory}/*.h")
  LIST(APPEND Move3DMPLIB_inc "${incfiles}")
  SOURCE_GROUP(${BM3D_MODULE_NAME} FILES ${incfiles})
  file (GLOB incfiles "${directory}/*.hpp")
  LIST(APPEND Move3DMPLIB_inc "${incfiles}")
  SOURCE_GROUP(${BM3D_MODULE_NAME} FILES ${incfiles})
ENDMACRO(BM3D_INC_DIR_PROCESS)

#-------------------------------------------------------------------------------
# QT4 SUBDIR PROCESS (same but for preprocessing qt4 files)
#-------------------------------------------------------------------------------
SET(Move3DMPLIB_QT_FILES_FOR_MOC "")

MACRO(BM3D_QT_GENERATE_MOC)
  FOREACH(file ${ARGV})
    LIST(APPEND BM3D_QT_FILES_FOR_MOC "${BM3D_MODULE_NAME}/${file}")
  ENDFOREACH(file ${ARGV})
ENDMACRO(BM3D_QT_GENERATE_MOC)

SET(Move3DMPLIB_QT_UI_FILES "")

MACRO(BM3D_QT_GENERATE_UI_HEADERS)
  FOREACH(file ${ARGV})
    LIST(APPEND BM3D_QT_UI_FILES "${BM3D_MODULE_NAME}/${file}")
  ENDFOREACH(file ${ARGV})
ENDMACRO(BM3D_QT_GENERATE_UI_HEADERS)

FOREACH(srcDir ${Move3DMPLIB_SOURCE_DIRECTORIES})
  INCLUDE(${PROJECT_SOURCE_DIR}/${srcDir}/SourceList.cmake)
ENDFOREACH(srcDir)

#-------------------------------------------------------------------------------
# Directories to include 
#-------------------------------------------------------------------------------
BM3D_INC_DIR_PROCESS(${PROJECT_SOURCE_DIR})

#MESSAGE("Move3DMPLIB_src : ${Move3DMPLIB_src}") #Uncomment to print the included file by Cmake
#MESSAGE("Move3DMPLIB_inc : ${Move3DMPLIB_inc}") #Uncomment to print the included file by Cmake

IF(QT_LIBRARY)
  # necessary for the UI headers
  BM3D_INC_DIR_PROCESS(${CMAKE_CURRENT_BINARY_DIR})
  # generate the UI headers
  QT4_WRAP_UI(Move3DMPLIB_QT_UI_HEADERS ${BM3D_QT_UI_FILES})
  # Qt4 files that need to be preprocessed
  QT4_WRAP_CPP(Move3DMPLIB_QT_MOC ${BM3D_QT_FILES_FOR_MOC})
ENDIF(QT_LIBRARY)

# -------------------------------------------------------------------------------
# Doc
# -------------------------------------------------------------------------------
MACRO(GENERATE_DOCUMENTATION DOXYGEN_CONFIG_FILE)
FIND_PACKAGE(Doxygen)
SET(DOXYFILE_FOUND false)
IF(EXISTS ${PROJECT_SOURCE_DIR}/${DOXYGEN_CONFIG_FILE})
    SET(DOXYFILE_FOUND true)
ENDIF(EXISTS ${PROJECT_SOURCE_DIR}/${DOXYGEN_CONFIG_FILE})

IF( DOXYGEN_FOUND )
    IF( DOXYFILE_FOUND )
        # Add target
	ADD_CUSTOM_TARGET( doc ALL ${DOXYGEN_EXECUTABLE} "${PROJECT_SOURCE_DIR}/${DOXYGEN_CONFIG_FILE}" )
    ELSE( DOXYFILE_FOUND )
	MESSAGE( STATUS "Doxygen configuration file not found - Documentation will not be generated" )
    ENDIF( DOXYFILE_FOUND )
ELSE(DOXYGEN_FOUND)
    MESSAGE(STATUS "Doxygen not found - Documentation will not be generated")
ENDIF(DOXYGEN_FOUND)
ENDMACRO(GENERATE_DOCUMENTATION)


# GENERATE_DOCUMENTATION(Doxyfile)

# -------------------------------------------------------------------------------
# Compiler and Compilation flags
# -------------------------------------------------------------------------------
SET(CMAKE_CXX_COMPILER g++)
SET(CMAKE_C_COMPILER g++)

IF(UNIX)
  ADD_DEFINITIONS(-DUNIX)
  IF(APPLE)
    ADD_DEFINITIONS(-DMACOSX -x c++)
    SET (CMAKE_SHARED_LINKER_FLAGS "-dynamiclib -Wl,-headerpad_max_install_names,-undefined,dynamic_lookup")
  ELSE(APPLE)
    #ADD_DEFINITIONS(-DLINUX -std=gnu++0x)
    ADD_DEFINITIONS(-DLINUX)
  ENDIF(APPLE)
ELSE(UNIX)
 MESSAGE (FATAL_ERROR "Not supported Platform")
ENDIF(UNIX)

#Default mode is Debug
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Debug CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
ENDIF(NOT CMAKE_BUILD_TYPE)

SET(LIBRARY_OUTPUT_PATH "${PROJECT_SOURCE_DIR}/lib/$ENV{HOSTTYPE}")

#Default Flags
ADD_DEFINITIONS(-DKCD_MOVE3D -DPRINT_ERROR -DPRINT_INFO)

IF(${CMAKE_BUILD_TYPE} MATCHES "Debug")
  ADD_DEFINITIONS(-g -Wall -Wno-unknown-pragmas -fstack-protector-all)
ELSEIF(${CMAKE_BUILD_TYPE} MATCHES "Release")
  ADD_DEFINITIONS(-O3 -Wall -Wno-unknown-pragmas)
ENDIF(${CMAKE_BUILD_TYPE} MATCHES "Debug")

IF(QT_LIBRARY)
	ADD_LIBRARY(move3d-planners SHARED ${Move3DMPLIB_src} ${Move3DMPLIB_QT_MOC} ${Move3DMPLIB_QT_UI_HEADERS} ${Move3DMPLIB_inc})
ELSE()
	ADD_LIBRARY(move3d-planners SHARED ${Move3DMPLIB_src} ${Move3DMPLIB_inc})
ENDIF()

TARGET_LINK_LIBRARIES(move3d-planners ${LIBS})

# ------------------------------------------------------------------------------
# Intallation
# ------------------------------------------------------------------------------
IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
#  SET(CMAKE_INSTALL_PREFIX "${Move3DMPLIB_BINARY_DIR}/install" CACHE PATH "Move3DMPLIB install prefix" FORCE)
   SET(CMAKE_INSTALL_PREFIX "$ENV{ROBOTPKG_BASE}" CACHE PATH "Move3DMPLIB install prefix" FORCE)
ENDIF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)

#ADD_EXECUTABLE(move3dplannerstest ${CMAKE_SOURCE_DIR}/src/test/main.cpp)
#SET_TARGET_PROPERTIES(move3dplannerstest PROPERTIES LINKER_LANGUAGE CXX)
#TARGET_LINK_LIBRARIES(move3dplannerstest move3d-planners)


INSTALL(TARGETS move3d-planners
        RUNTIME DESTINATION bin CONFIGURATIONS ${CMAKE_BUILD_TYPE}
        LIBRARY DESTINATION lib CONFIGURATIONS ${CMAKE_BUILD_TYPE})
	
INSTALL(
	DIRECTORY ${Move3DMPLIB_SOURCE_DIRECTORIES}/ 
	DESTINATION include/libmove3d/planners 
	FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")
# -------------------------------------------------------------
# --- Testing -------------------------------------------------
# -------------------------------------------------------------
ENABLE_TESTING()

MESSAGE("CMAKE_BINARY_DIR = " ${CMAKE_BINARY_DIR})

ADD_TEST(NAME basic COMMAND move3dplannerstest)

