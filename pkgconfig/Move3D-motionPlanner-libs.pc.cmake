prefix=@CMAKE_INSTALL_PREFIX@
exec_prefix=@CMAKE_INSTALL_PREFIX@
libdir=@CMAKE_INSTALL_PREFIX@/lib
includedir=@CMAKE_INSTALL_PREFIX@/include

MPLibsFlags= @Move3DMPLIB_CompilationFlags@
MPLibsIncludes=@Move3DMPLIB_Compilation_includes@
MPLibsLibs=@Move3DMPLIB_Compilation_libs@
 
Name: Move3D-motionPlanner-libs
Description: Motion Planning Platform - Headless library
Version: @BIOMOVE3D_VERSION@
Libs: ${MPLibsFlags} -L${libdir} -lMove3D-motionPlanner-libs
Cflags: -I${includedir}/Move3D-motionPlanner-libs/src ${MPLibsIncludes} ${MPLibsFlags} 
