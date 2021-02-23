set(LIB_NAMES)

set(LIB_NAME Filters)
set(LIB_NAMES ${LIB_NAMES} ${LIB_NAME})
set(${LIB_NAME}_SOURCES
    PlatoEngine_KernelFilter.cpp
    PlatoEngine_KernelThenHeavisideFilter.cpp
    PlatoEngine_KernelThenTANHFilter.cpp
    PlatoEngine_AbstractKernelThenFilter.cpp
    PlatoEngine_AbstractFilter.cpp
    PlatoEngine_FilterFactory.cpp
    PlatoEngine_IdentityFilter.cpp
    )
set(${LIB_NAME}_HEADERS
    PlatoEngine_KernelFilter.hpp
    PlatoEngine_KernelThenHeavisideFilter.hpp
    PlatoEngine_KernelThenTANHFilter.hpp
    PlatoEngine_AbstractKernelThenFilter.hpp
    PlatoEngine_AbstractFilter.hpp
    PlatoEngine_FilterFactory.hpp
    PlatoEngine_IdentityFilter.hpp
    )

if( AMFILTER_ENABLED )
    set(${LIB_NAME}_SOURCES ${${LIB_NAME}_SOURCES} PlatoEngine_KernelThenStructuredAMFilter.cpp)
    set(${LIB_NAME}_HEADERS ${${LIB_NAME}_HEADERS} PlatoEngine_KernelThenStructuredAMFilter.hpp)
endif()

INCLUDE_DIRECTORIES(${PLATO_INCLUDES})

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/base/src/PlatoSubproblemLibraryInterface)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/base/src/PlatoSubproblemLibrary/AbstractInterface)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/base/src/PlatoSubproblemLibrary/Agent)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/base/src/PlatoSubproblemLibrary/BoundedSupportFunction)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/base/src/PlatoSubproblemLibrary/Example)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/base/src/PlatoSubproblemLibrary/Filter)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/base/src/PlatoSubproblemLibrary/Geometry)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/base/src/PlatoSubproblemLibrary/Helper)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/base/src/PlatoSubproblemLibrary/InputData)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/base/src/PlatoSubproblemLibrary/NodeSetMatching)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/base/src/PlatoSubproblemLibrary/SpatialSearching)

FOREACH(PLATO_LIB ${LIB_NAMES})
  add_library(Plato${PLATO_LIB}     ${${PLATO_LIB}_SOURCES}     ${${PLATO_LIB}_HEADERS}     )
  set(ADD_PLATO_LIBRARIES ${ADD_PLATO_LIBRARIES} Plato${PLATO_LIB})
ENDFOREACH()
set(PLATO_LIBRARIES ${PLATO_LIBRARIES} ${ADD_PLATO_LIBRARIES} PARENT_SCOPE)

if( CMAKE_INSTALL_PREFIX )
 FOREACH(PLATO_LIB ${LIB_NAMES})
  install( TARGETS Plato${PLATO_LIB} EXPORT Plato${PLATO_LIB}
           LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
           ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
           PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_PREFIX}/include)
  install( FILES ${${PLATO_LIB}_HEADERS} DESTINATION ${CMAKE_INSTALL_PREFIX}/include)
 ENDFOREACH()
endif()
