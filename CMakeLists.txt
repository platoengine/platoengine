############ Project setup ##################################################
CMAKE_MINIMUM_REQUIRED(VERSION 3.0.0)
#############################################################################

include( ${CMAKE_SOURCE_DIR}/base/config/tools.cmake )
Plato_no_src_build()

option(PLATO_ENABLE_SERVICES_PYTHON "Enable the Plato Python interface" OFF)

option( GEOMETRY      "Enable Geometry subcomponent"   OFF)
option( EXPY          "Build exodus python API"        OFF)
option( SEACAS        "Seacas tools"                    ON)
option( ENABLE_STK    "Flag to turn on use of STK"     OFF)


##################################################################
## Load config.cmake                                            ##
##################################################################
if( DEFINED CONFIG )

  message(STATUS "-DCONFIG specified: " ${CONFIG})
  if( EXISTS ${CMAKE_BINARY_DIR}/${CONFIG} )
   include( ${CMAKE_BINARY_DIR}/${CONFIG} )
  else()
    message(FATAL_ERROR "The configuration file '${CONFIG}' specified with -DCONFIG doesn't exist")
  endif()

else()

  if( EXISTS ${CMAKE_BINARY_DIR}/config.cmake )
   include( ${CMAKE_BINARY_DIR}/config.cmake )
  else()
    message(STATUS "A configuration file named 'config.cmake' was not provided in the build directory.")
    message(STATUS "Using command line arguments only to configure build.")
    include( ${CMAKE_SOURCE_DIR}/base/config/defaults.cmake )
  endif()

endif()

if( DEFINED PLATO_SYSTEM_CONFIG )
  if( EXISTS ${PLATO_SYSTEM_CONFIG} )
   include( ${PLATO_SYSTEM_CONFIG} )
  else()
    message(STATUS "Specified system configuration file ($PLATO_SYSTEM_CONFIG} not found")
    message(FATAL_ERROR " ")
  endif()
endif()


project(PlatoEngine) 

################################################################################
################################################################################

############################## gtest Setup ####################################
if( UNIT_TESTING )
###############################################################################
find_library( GTEST_LIBRARY NAMES gtest
              PATHS ${GTEST_HOME}/lib NO_DEFAULT_PATH)

if( GTEST_LIBRARY )
  message( "-- Info: Using GTest library: ${GTEST_LIBRARY}" )
else( GTEST_LIBRARY )
  message( "-- Error: GTest not found." )
endif( GTEST_LIBRARY )

find_path( GTEST_INCLUDE NAMES gtest.h
           PATHS ${GTEST_HOME}/include/gtest NO_DEFAULT_PATH )

if( GTEST_INCLUDE )
  message( "-- Info: Using GTest headers in ${GTEST_INCLUDE}" )
  include_directories( ${GTEST_INCLUDE} )
else( GTEST_INCLUDE )
  message( "-- Error: GTest headers not found." )
endif( GTEST_INCLUDE )
###############################################################################
endif()
###############################################################################


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

if( ENABLE_ROL )
  add_definitions(-DENABLE_ROL)
endif()

include_directories( ${Trilinos_INCLUDE_DIRS} ${Trilinos_TPL_INCLUDE_DIRS})

###############################################################################
###############################################################################


###############################################################################
###############################################################################
message("Trilinos_CXX_COMPILER_FLAGS: ${Trilinos_CXX_COMPILER_FLAGS}")
set(CMAKE_C_FLAGS "${Trilinos_C_COMPILER_FLAGS} -fPIC")
set(CMAKE_CXX_FLAGS "${Trilinos_CXX_COMPILER_FLAGS} -fPIC")
set(CMAKE_CXX_FLAGS_DEBUG "${Trilinos_CXX_COMPILER_FLAGS} -fPIC")
set(CMAKE_CXX_FLAGS_RELEASE "${Trilinos_CXX_COMPILER_FLAGS} -fPIC")

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # Clang specific flags
elseif(${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU")
  # gnu specific flags
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lgfortran")
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -lgfortran")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -lgfortran")
elseif(${CMAKE_CXX_COMPILER_ID} STREQUAL "Intel")
  # intel specific flags
else()
  message(WARNING "Unknown compiler type: ${CMAKE_CXX_COMPILER_ID}. Using default flags.")
endif()

if("${CMAKE_BUILD_TYPE}" STREQUAL Release)
  add_definitions( -DNDEBUG)
endif()

if( GEOMETRY )
  add_definitions( -DGEOMETRY )
endif()
###############################################################################
###############################################################################

if( REGRESSION OR UNIT_TESTING )
    enable_testing()
endif()

set(PLATO_LIBRARIES ${GTEST_LIBRARY})
set(PLATO_INCLUDES)

add_subdirectory(base/src)
add_subdirectory(base/src/analyze)
add_subdirectory(base/src/data_layer)
add_subdirectory(base/src/Filters)
add_subdirectory(base/src/interface)
add_subdirectory(base/src/models)
add_subdirectory(base/src/optimize)
add_subdirectory(base/src/proxy)
add_subdirectory(base/src/tools)
if(GEOMETRY)
  add_subdirectory(base/src/geometry/MovingLeastSquares)
endif()
add_subdirectory(base/src/PlatoSubproblemLibrary/Agent)
add_subdirectory(base/src/PlatoSubproblemLibraryInterface)
add_subdirectory(base/src/PlatoSubproblemLibrary/SpatialSearching)
add_subdirectory(base/src/PlatoSubproblemLibrary/AbstractInterface)
add_subdirectory(base/src/PlatoSubproblemLibrary/ActivationFunction)
add_subdirectory(base/src/PlatoSubproblemLibrary/BoundedSupportFunction)
add_subdirectory(base/src/PlatoSubproblemLibrary/DataLossFunction)
add_subdirectory(base/src/PlatoSubproblemLibrary/DataOrganizer)
add_subdirectory(base/src/PlatoSubproblemLibrary/DecisionMetric)
add_subdirectory(base/src/PlatoSubproblemLibrary/DiscreteGlobalOptimization)
add_subdirectory(base/src/PlatoSubproblemLibrary/DistanceVoting)
add_subdirectory(base/src/PlatoSubproblemLibrary/Eigen)
add_subdirectory(base/src/PlatoSubproblemLibrary/Example)
add_subdirectory(base/src/PlatoSubproblemLibrary/Filter)
add_subdirectory(base/src/PlatoSubproblemLibrary/Geometry)
add_subdirectory(base/src/PlatoSubproblemLibrary/Graph)
add_subdirectory(base/src/PlatoSubproblemLibrary/Helper)
add_subdirectory(base/src/PlatoSubproblemLibrary/InputOutput)
add_subdirectory(base/src/PlatoSubproblemLibrary/InvertibleUnitRange)
add_subdirectory(base/src/PlatoSubproblemLibrary/LearningPreprocessor)
add_subdirectory(base/src/PlatoSubproblemLibrary/NeighborsSearching)
add_subdirectory(base/src/PlatoSubproblemLibrary/NodeSetMatching)
add_subdirectory(base/src/PlatoSubproblemLibrary/ParameterData)
add_subdirectory(base/src/PlatoSubproblemLibrary/SupervisedLearning)
add_subdirectory(base/src/PlatoSubproblemLibrary/VectorKernel)
add_subdirectory(base/src/input_checkers)
add_subdirectory(base/src/input_generator)

if( ENABLE_ISO )
  message( "-- Compiling PlatoIso " )
  add_definitions(-DENABLE_ISO)
  add_subdirectory(base/src/iso/main)
endif()

if( ENABLE_STK )
  message( "-- Compiling with STK " )
  add_definitions(-DSTK_ENABLED)
endif()

if( PLATOMAIN )
  message( "-- Compiling PlatoMain " )
  add_subdirectory(apps/services)
endif()

if( PLATOPROXY )
  message( "-- Compiling PlatoProxy " )
  add_subdirectory(apps/proxy)
  add_subdirectory(apps/rosenbrock)
endif()

if( PLATOSTATICS )
  message( "-- Compiling Statics " )
  add_subdirectory(apps/statics)
endif()

if(EXPY)
  message( "-- Compiling Expy " )
  add_subdirectory(apps/services/python)
endif()

if( REGRESSION )
  if( PLATOMAIN )
    if(SEACAS)
      Plato_find_exe(SEACAS_EPU     epu      ${SEACAS_PATH}/bin)
      Plato_find_exe(SEACAS_EXODIFF exodiff  ${SEACAS_PATH}/bin)
      Plato_find_exe(SEACAS_DECOMP  decomp   ${SEACAS_PATH}/bin)
    endif()
    add_subdirectory(examples)
  else()
    message("-- !! -- PlatoMain is required for regression testing.")
    message("-- !! -- Regression has not been enabled.")
  endif()
endif()

