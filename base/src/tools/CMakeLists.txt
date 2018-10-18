set(LIB_NAMES)

set(LIB_NAME Parser)
set(LIB_NAMES ${LIB_NAMES} ${LIB_NAME})
set(${LIB_NAME}_SOURCES Plato_Parser.cpp
                        Plato_InputData.cpp
                        Plato_OptimizerEngineStageData.cpp
                        Plato_OperationInputDataMng.cpp
                        Plato_StageInputDataMng.cpp
                        math_parser/tinyexpr.c
                        pugixml.cpp
                        Plato_UniqueCounter.cpp
                        Plato_FreeFunctions.cpp
                        Plato_TimersTree.cpp
                        Plato_Vector3DVariations.cpp
                        Plato_AppErrorChecks.cpp
                        )
set(${LIB_NAME}_HEADERS Plato_Parser.hpp
                        Plato_InputData.hpp
                        Plato_OptimizerEngineStageData.hpp
                        Plato_OperationInputDataMng.hpp
                        Plato_StageInputDataMng.hpp
                        pugixml.hpp
                        pugiconfig.hpp
                        Plato_UniqueCounter.hpp
                        Plato_FreeFunctions.hpp
                        Plato_TimersTree.hpp
                        Plato_AppErrorChecks.hpp
                        Plato_Vector3DVariations.hpp
                        )

set(LIB_NAME Exceptions)
set(LIB_NAMES ${LIB_NAMES} ${LIB_NAME})
set(${LIB_NAME}_SOURCES Plato_Exceptions.cpp)
set(${LIB_NAME}_HEADERS Plato_Exceptions.hpp)

set(LIBLESS_HEADERS
  Plato_Communication.hpp
)

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
 install( FILES ${LIBLESS_HEADERS} DESTINATION ${CMAKE_INSTALL_PREFIX}/include)
 install( FILES math_parser/tinyexpr.h DESTINATION ${CMAKE_INSTALL_PREFIX}/include/math_parser)
endif()

if( UNIT_TESTING )
  add_subdirectory(./unittest)
endif()
