set(LIB_NAMES)

set(LIB_NAME Analyze)
set(LIB_NAMES ${LIB_NAMES} ${LIB_NAME})
SET(${LIB_NAME}_SOURCES data_mesh.cpp
                        data_container.cpp
                        mesh_io.cpp
                        mesh_services.cpp
                        communicator.cpp
                        topological_element.cpp
                        exception_handling.cpp
                        exodus_io.cpp
                        matrix_container.cpp
                        material_container.cpp
                        math_library.cpp
                        linear_elastic.cpp
                        lightmp.cpp
                        nemesis_io.cpp
                        solid_statics.cpp)

SET(${LIB_NAME}_HEADERS data_mesh.hpp
                        types.hpp
                        data_container.hpp
                        mesh_io.hpp
                        mesh_services.hpp
                        communicator.hpp
                        topological_element.hpp
                        exception_handling.hpp
                        matrix_container.hpp
                        material_container.hpp
                        math_library.hpp
                        linear_elastic.hpp
                        lightmp.hpp
                        solid_statics.hpp
                        bc.hpp)

INCLUDE_DIRECTORIES(${PLATO_INCLUDES})

FOREACH(PLATO_LIB ${LIB_NAMES})
  add_library(Plato${PLATO_LIB} ${${PLATO_LIB}_SOURCES} ${${PLATO_LIB}_HEADERS})
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
