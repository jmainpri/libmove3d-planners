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

CMAKE_MINIMUM_REQUIRED (VERSION 2.6.4)

PROJECT (Move3DMPLIB C CXX)

#Default mode is Debug ---------------------------------------------------------
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Debug CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
ENDIF(NOT CMAKE_BUILD_TYPE)

IF(APPLE)
    ADD_DEFINITIONS(-DMACOSX)
ENDIF(APPLE)

	
# Set the project version ------------------------------------------------------

if( NOT PROJECT_VERSION)

if( NOT PROJECT_MAJOR_VERSION)
set( PROJECT_MAJOR_VERSION 1)
endif( NOT PROJECT_MAJOR_VERSION)

if( NOT PROJECT_MINOR_VERSION)
set( PROJECT_MINOR_VERSION 0)
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
  OPTION(${flagName} ${description} ${status})
  LIST(APPEND Move3DMPLIB_CompilationFlags ${flagName})
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

Move3DMPLIB_flag(QT_LIBRARY "Sets QtLibrary compiling mode" ON)

Move3DMPLIB_flag(P3D_CONSTRAINTS "Compilation with module Constraints" ON)
Move3DMPLIB_flag(P3D_COLLISION_CHECKING "Compilation with module Collision Checking" ON)
Move3DMPLIB_flag(P3D_LOCALPATH "Compilation with module Localpath" ON)
Move3DMPLIB_flag(P3D_PLANNER "Compilation with module Planner" ON)
Move3DMPLIB_flag(LIGHT_PLANNER "Compilation with module Light Planner" ON)
Move3DMPLIB_flag(GRASP_PLANNING "Compilation with module Manipulation Planner" ON)
Move3DMPLIB_flag(MULTILOCALPATH "Compilation with module Multilocalpath" ON)
Move3DMPLIB_flag(MULTIGRAPH "Compilation with module Multi-Graph" ON)

Move3DMPLIB_flag(USE_GBM "Compilation with module Multilocalpath" ON)
Move3DMPLIB_flag(USE_GSL "Compilation with module Hir-planner" ON)

Move3DMPLIB_flag(HRI_COSTSPACE "With the hri work" ON)
Move3DMPLIB_flag(HRI_PLANNER "With the hri work done in BioMove3D" ON USE_GSL)

Move3DMPLIB_CheckCompilationFlags()

# ------------------------------------------------------------------------------
# --- Default compilation option : ---------------------------------------------
# --- DO NOT MODIFY!!! use ccmake ----------------------------------------------
#-------------------------------------------------------------------------------
#-------------- Modules --------------------------------------------------------
#-------------------------------------------------------------------------------
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/script/CMake")

#-------------- Qt Library -----------------------------------------------------
IF(QT_LIBRARY)
  FIND_PACKAGE(Qt4 REQUIRED)
  INCLUDE_DIRECTORIES(${QT_INCLUDE_DIR})
  SET(Move3DMPLIB_COMPILATION_INCLUDES ${Move3DMPLIB_COMPILATION_INCLUDES} ${QT_INCLUDE_DIR})
  IF(QT_GL)
    SET(QT_USE_QTOPENGL TRUE)
  ENDIF(QT_GL)
  INCLUDE(${QT_USE_FILE})
  SET(LIBS ${LIBS} ${QT_LIBRARIES})
ENDIF(QT_LIBRARY)
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

FIND_PACKAGE(MOVE3D-CORE REQUIRED)
INCLUDE_directories(${MOVE3D-CORE_INCLUDE_DIR} ${p3d_INCLUDE_DIR})
SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${MOVE3D-CORE_INCLUDE_DIR})
SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${p3d_INCLUDE_DIR})
SET(LIBS ${LIBS} ${MOVE3D-CORE_LIBRARIES})

#-------------- Eigen ----------------------------------------------------------

FIND_PACKAGE(EIGEN REQUIRED)
INCLUDE_directories(${Eigen_INCLUDE_DIR})
SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${Eigen_INCLUDE_DIR})

#-------------- Robotpim -------------------------------------------------------

#  FIND_PACKAGE(ROBOPTIM REQUIRED)
#  INCLUDE_directories(${ROBOPTIM_INCLUDE_DIR})
#  SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${ROBOPTIM_INCLUDE_DIR})
#  SET(LIBS ${LIBS} ${ROBOPTIM_LIBRARIES})

#-------------- Boost ----------------------------------------------------------

# boost is used e.g. for managing the cost functions (T-RRT).
#  FIND_PACKAGE(Boost REQUIRED)
#  INCLUDE_directories(${BOOST_INCLUDE_DIR})
#  SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${BOOST_INCLUDE_DIR})

# boost is used e.g. for managing the cost functions (T-RRT).
FIND_PACKAGE(Boost REQUIRED)
INCLUDE_directories(/opt/local/include/)        
SET(INCLUDE_DIRS ${INCLUDE_DIRS} /opt/local/include/ ${BOOST_INCLUDE_DIRS})

#-------------- GBM ------------------------------------------------------------
IF(MULTILOCALPATH)
FIND_PACKAGE(GBM REQUIRED)
INCLUDE_directories(${GBM_INCLUDE_DIR})
  SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${GBM_INCLUDE_DIR})
SET(LIBS ${LIBS} ${GBM_LIBRARIES})
ENDIF(MULTILOCALPATH)

#-------------- Grasp Planner --------------------------------------------------

#IF(GRASP_PLANNING)
#  FIND_PACKAGE(GLPK REQUIRED)
#  INCLUDE_directories(${GLPK_INCLUDE_DIR})
#  SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${GLPK_INCLUDE_DIR})
#  SET(LIBS ${LIBS} ${GLPK_LIBRARIES})
#  
#  FIND_PACKAGE(QHULL REQUIRED)
#  INCLUDE_directories(${QHULL_INCLUDE_DIR})
#  SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${QHULL_INCLUDE_DIR})
#  SET(LIBS ${LIBS} ${QHULL_LIBRARIES})
#  
#  FIND_PACKAGE(GLIBCONFIG REQUIRED)
#  INCLUDE_directories(${GLIBCONFIG_INCLUDE_DIR})
#  SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${GLIBCONFIG_INCLUDE_DIR})
#  
#  FIND_PACKAGE(GLIB REQUIRED)
#  INCLUDE_directories(${GLIB_INCLUDE_DIR})
#  SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${GLIB_INCLUDE_DIR})
#  SET(LIBS ${LIBS} ${GLIB_LIBRARIES})
#
#  FIND_PACKAGE(GTS REQUIRED)
#  INCLUDE_directories(${GTS_INCLUDE_DIR})
#  SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${GTS_INCLUDE_DIR})
#  SET(LIBS ${LIBS} ${GTS_LIBRARIES})
#ENDIF(GRASP_PLANNING)


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
  INCLUDE(${Move3DMPLIB_SOURCE_DIR}/${srcDir}/SourceList.cmake)
ENDFOREACH(srcDir)

#-------------------------------------------------------------------------------
# Directories to include 
#-------------------------------------------------------------------------------
BM3D_INC_DIR_PROCESS(${Move3DMPLIB_SOURCE_DIR})
BM3D_INC_DIR_PROCESS(${Move3DMPLIB_SOURCE_DIR})

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

# SET(EXECUTABLE_OUTPUT_PATH "${PROJECT_SOURCE_DIR}/bin/$ENV{HOSTTYPE}")
# ADD_EXECUTABLE(move3d ${Move3DMPLIB_src} ${Move3DMPLIB_QT_MOC} ${Move3DMPLIB_QT_UI_HEADERS} ${Move3DMPLIB_inc})
# TARGET_LINK_LIBRARIES(move3d ${LIBS} m)

#Default Flags
ADD_DEFINITIONS(-DKCD_MOVE3D -DPRINT_ERROR -DPRINT_INFO)

IF(${CMAKE_BUILD_TYPE} MATCHES "Debug")
  ADD_DEFINITIONS(-g -Wall -Wno-unknown-pragmas -fstack-protector-all)
ELSEIF(${CMAKE_BUILD_TYPE} MATCHES "Release")
  ADD_DEFINITIONS(-O2 -Wall -Wno-unknown-pragmas)
ENDIF(${CMAKE_BUILD_TYPE} MATCHES "Debug")

ADD_LIBRARY(Move3D-motionPlanner SHARED ${Move3DMPLIB_src} ${Move3DMPLIB_QT_MOC} ${Move3DMPLIB_QT_UI_HEADERS} ${Move3DMPLIB_inc})
TARGET_LINK_LIBRARIES(Move3D-motionPlanner ${LIBS} m)

#ADD_EXECUTABLE(move3d ${BioMove3D_SOURCE_DIR}/move3d/move3d.c ${Move3DMPLIB_inc})
#TARGET_LINK_LIBRARIES(move3d Move3DMPLIB ${LIBS} m)

# ------------------------------------------------------------------------------
# Intallation
# ------------------------------------------------------------------------------
IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  SET(CMAKE_INSTALL_PREFIX "${Move3DMPLIB_BINARY_DIR}/install" CACHE PATH "Move3DMPLIB install prefix" FORCE)
ENDIF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)

INSTALL(TARGETS Move3D-motionPlanner
        RUNTIME DESTINATION bin PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_WRITE GROUP_EXECUTE WORLD_READ WORLD_EXECUTE CONFIGURATIONS ${CMAKE_BUILD_TYPE}
        LIBRARY DESTINATION lib PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ GROUP_WRITE WORLD_READ CONFIGURATIONS ${CMAKE_BUILD_TYPE})
	
INSTALL(DIRECTORY ${Move3DMPLIB_SOURCE_DIRECTORIES} DESTINATION include/Move3D-motionPlanner FILES_MATCHING REGEX "\\.[h|hpp]")
 
