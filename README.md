libmove3d-planners
==================

Motion planning and geometric reasoning C++ library based on libmove3d

### Full install

See move3d-launch: https://github.com/jmainpri/move3d-launch

### Stand alone install

Depends on libmove3d and libmove3d-hri

    mkdir build && cd build
    cmake ..
    make install

### Install with custom boost

    export TARGET=$MOVE3D_INSTALL_DIR
    cmake -DCMAKE_INSTALL_PREFIX=$TARGET -DBoost_NO_BOOST_CMAKE=TRUE -DBoost_NO_SYSTEM_PATHS=TRUE -DBOOST_ROOT:PATHNAME=$TARGET -DBoost_LIBRARY_DIRS:FILEPATH=${TARGET}/lib ..
