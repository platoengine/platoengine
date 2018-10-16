############ Project setup ##################################################
PROJECT(tests CXX)

CMAKE_MINIMUM_REQUIRED(VERSION 2.8.12)
#############################################################################
#### Required paths ####
set( TRILINOS_INSTALL_DIR
/home/jrobbin/Projects/ATO/VolInt/Trilinos/install-static-dbg CACHE STRING "Trilinos path")

set( COGENT_SEARCH_PATH 
/home/jrobbin/Projects/ATO/VolInt/CubeQuadrature/Checkin/install_dbg CACHE STRING "Cogent path")

set (CMAKE_CXX_COMPILER mpic++ CACHE STRING "C++ compiler")
set (CMAKE_C_COMPILER mpicc CACHE STRING "C compiler")
option (ENABLE_GPROF "(ON|OFF) Enable GPROF" OFF)

option (ENABLE_BOOST_TIMER "(ON|OFF) Enable BOOST timer" OFF )

set( BOOST_INCLUDE_DIR /home/jrobbin/include CACHE STRING "Boost includes")
set( BOOST_LIBRARY_DIR /home/jrobbin/lib CACHE STRING "Boost libraries")

set( BLAS_LIB_NAME BLAS     CACHE STRING "Blas library base name" )
set( LAPACK_LIB_NAME LAPACK CACHE STRING "Lapack library base name" )
set( BLAS_SEARCH_PATH /System/Library/Frameworks/Accelerate.framework/Versions/Current/Frameworks/vecLib.framework/Versions/Current
     CACHE STRING "Blas search directory" )
set( LAPACK_SEARCH_PATH /System/Library/Frameworks/Accelerate.framework/Versions/Current/Frameworks/vecLib.framework/Versions/Current
     CACHE STRING "Lapack search directory" )

set( CMAKE_CXX_FLAGS "-Wall -fno-strict-aliasing -fPIC -std=c++11" )
set( CMAKE_CXX_FLAGS_DEBUG "-g" )
set( CMAKE_CXX_FLAGS_RELEASE "-O3" )

############################  Trilinos Setup  #################################
###############################################################################
IF (DEFINED TRILINOS_INSTALL_DIR)
  SET(CMAKE_PREFIX_PATH ${TRILINOS_INSTALL_DIR} ${CMAKE_PREFIX_PATH})
ENDIF()
FIND_PACKAGE(Trilinos REQUIRED)

MESSAGE("\nFound Trilinos!  Here are the details: ")
MESSAGE("   Trilinos_DIR = ${Trilinos_DIR}")
MESSAGE("   Trilinos_VERSION = ${Trilinos_VERSION}")
MESSAGE("End of Trilinos details\n")

include_directories( ${Trilinos_INCLUDE_DIRS} ${Trilinos_TPL_INCLUDE_DIRS})
###############################################################################
###############################################################################


if( ENABLE_GPROF )
  if( NOT(CMAKE_BUILD_TYPE MATCHES DEBUG) AND
      NOT(CMAKE_BUILD_TYPE MATCHES Debug) AND
      NOT(CMAKE_BUILD_TYPE MATCHES debug) )
    message("CMAKE_BUILD_TYPE set to ${CMAKE_BUILD_TYPE}")
    message(FATAL_ERROR "gprof requires debug build.  set CMAKE_BUILD_TYPE to DEBUG")
  endif()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg")
else()
  message(STATUS "GPROF not enabled")
endif()

if( ENABLE_BOOST_TIMER )
  message(STATUS "BOOST timers enabled")
  INCLUDE_DIRECTORIES(${BOOST_INCLUDE_DIR})
  LINK_DIRECTORIES(${BOOST_LIBRARY_DIR})
  set(BOOST_TIMER_LIB boost_timer)
  ADD_DEFINITIONS(-DENABLE_BOOST_TIMER)
else()
  message(STATUS "BOOST timers disabled")
endif()



find_library( BLAS_LIBRARY        NAMES ${BLAS_LIB_NAME}      NO_DEFAULT_PATH PATHS ${BLAS_SEARCH_PATH} )
find_library( LAPACK_LIBRARY      NAMES ${LAPACK_LIB_NAME}    NO_DEFAULT_PATH PATHS ${LAPACK_SEARCH_PATH} )

set(BLAS_LAPACK_LIBRARY ${BLAS_LIBRARY} ${LAPACK_LIBRARY} ${BLAS_LIBRARY} ${LAPACK_LIBRARY})

message( "-- Info: looking for Cogent in: ${COGENT_SEARCH_PATH}" )
find_library( COGENT_LIBRARY      NAMES Cogent    NO_DEFAULT_PATH PATHS ${COGENT_SEARCH_PATH}/lib )
find_path   ( COGENT_INCLUDE      NAMES Cogent_Integrator.hpp        PATHS ${COGENT_SEARCH_PATH}/include )

if( COGENT_LIBRARY )
  message( "-- Info: found COGENT" )
else( COGENT_LIBRARY )
  message( "-- Error: COGENT not found." )
endif( COGENT_LIBRARY )

find_path( COGENT_INCLUDE NAMES Trilinos_version.h
           PATHS ${COGENT_SEARCH_PATH}/include 
         )

if( COGENT_INCLUDE )
  message( "-- Info: Using COGENT headers in ${COGENT_INCLUDE}" )
  include_directories( ${COGENT_INCLUDE} )
else( COGENT_INCLUDE )
  message( "-- Error: COGENT headers not found." )
endif( COGENT_INCLUDE )

set(LINK_LIBRARIES ${COGENT_LIBRARY} ${Trilinos_LIBRARIES} ${Trilinos_TPL_LIBRARIES} -lgfortran)

add_subdirectory(Measure)
add_subdirectory(Conformal)
add_subdirectory(Reduced)
