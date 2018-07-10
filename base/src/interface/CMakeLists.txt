set(LIB_NAMES)

set(LIB_NAME Interface)
set(LIB_NAMES ${LIB_NAMES} ${LIB_NAME})
set(${LIB_NAME}_SOURCES Plato_Interface.cpp
                        Plato_Operation.cpp
                        Plato_MultiOperation.cpp
                        Plato_SingleOperation.cpp
                        Plato_OperationFactory.cpp
                        Plato_Stage.cpp
                        Plato_Performer.cpp)
set(${LIB_NAME}_HEADERS Plato_Application.hpp
                        Plato_Interface.hpp
                        Plato_Operation.hpp
                        Plato_MultiOperation.hpp
                        Plato_SingleOperation.hpp
                        Plato_OperationFactory.hpp
                        Plato_Stage.hpp
                        Plato_Performer.hpp)

INCLUDE_DIRECTORIES(${PLATO_INCLUDES})

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
