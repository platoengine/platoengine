set(PLATO_INCLUDES ${PLATO_INCLUDES} 
  ${CMAKE_SOURCE_DIR}/base/src/support_structure/main
  PARENT_SCOPE )


set(SupportStructureLib_SOURCES MeshWrapper.cpp
                        SupportStructure.cpp)
set(SupportStructureLib_HEADERS MeshWrapper.hpp
                       SupportStructure.hpp)
INCLUDE_DIRECTORIES(${PLATO_INCLUDES})
add_library(SupportStructureLib ${SupportStructureLib_SOURCES} ${SupportStructureLib_HEADERS})
set(PLATO_LIBRARIES ${PLATO_LIBRARIES} SupportStructureLib PARENT_SCOPE)

target_link_libraries(SupportStructureLib ${Trilinos_LIBRARIES})

install( TARGETS SupportStructureLib EXPORT SupportStructureLib
         LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
         ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
         PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_PREFIX}/include)
