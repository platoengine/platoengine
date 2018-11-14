option( PLATOMAIN     "Flag to turn on compilation of PlatoMain"           ON )
option( PLATOSTATICS  "Flag to turn on compilation of Statics performer"   ON )
option( REGRESSION    "Flag to create regression test suite"               ON )
option( SALINAS       "Flag to turn on testing of Salinas performer"      OFF )
option( ALBANY        "Flag to turn on testing of Albany performer"       OFF )
option( ENABLE_CGAL   "Flag to turn on CGAL geometry library"             OFF )
option( PARALLEL      "Flag to compile for parallel computing"             ON )
option( UNIT_TESTING  "Flag to turn on unit testing"                       ON )
option( ENABLE_PBCS   "Flag to turn on Periodic Boundary Conditions"      OFF )
option( EXECUTION_SUITE "Flag to enable execution suite"                  OFF )
option( PLATOPROXY    "Flag to turn on regression test with proxy apps"    ON )
option( STK_ENABLED   "Flag to indicate STK is available"                  ON )
option( GEOMETRY      "Flag to turn on Plato Geometry"                    OFF )
option( EXPY          "Build exodus python API"                           OFF )
option( SEACAS        "Seacas tools"                                       ON )

option(PLATO_ENABLE_SERVICES_PYTHON "Enable the Plato Python interface"   OFF )

set( TRILINOS_INSTALL_DIR   /projects/plato/dev/opt/trilinos/Latest/gcc-4.9.3-dbg
  CACHE STRING "path to Trilinos library")
set( CMAKE_CXX_COMPILER     mpiCC            
  CACHE STRING "c++ compiler")
set( CMAKE_C_COMPILER       mpicc            
  CACHE STRING "c compiler")
set( GTEST_HOME /projects/plato/dev/opt/gtest/1.8/gcc-4.9.3-openmpi-1.10.2-dbg
  CACHE STRING "path to GTest unit testing library")
set( NLOPT_HOME /projects/plato/dev/opt/nlopt/2.4.2/gcc-4.9.3 
  CACHE STRING "path to NLopt optimization library")
set( CMAKE_BUILD_TYPE Debug
  CACHE STRING "Debug or Release")
set( CMAKE_INSTALL_PREFIX 
  ../install_${CMAKE_BUILD_TYPE}
  CACHE STRING "Location to install")
set( SEACAS_PATH /projects/seacas/linux_rhel6/current
  CACHE STRING "Path to seacas tools")

