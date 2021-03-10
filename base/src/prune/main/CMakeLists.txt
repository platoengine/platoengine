set(PLATO_INCLUDES ${PLATO_INCLUDES} 
  ${CMAKE_SOURCE_DIR}/base/src/prune/main
  PARENT_SCOPE )


set(Prune_SOURCES MeshBalance.cpp
                        MeshManager.cpp
                        MeshPrune.cpp
                        MeshTransfer.cpp
                        PerceptPrune.cpp
                        PerceptPruneTool.cpp
                        PruneMeshAPISTK.cpp
                        PruneTool.cpp
                        STKPrune.cpp
                        MeshRefine.cpp)
set(Prune_HEADERS MeshBalance.hpp
                        MeshManager.hpp
                        MeshPrune.hpp
                        MeshTransfer.hpp
                        PerceptPrune.hpp
                        PerceptPruneTool.hpp
                        PruneMeshAPISTK.hpp
                        PruneMeshAPI.hpp
                        PruneTool.hpp
                        STKPrune.hpp
                        MeshRefine.hpp)
INCLUDE_DIRECTORIES(${PLATO_INCLUDES})
add_library(Prune ${Prune_SOURCES} ${Prune_HEADERS})
set(PLATO_LIBRARIES ${PLATO_LIBRARIES} Prune PARENT_SCOPE)

target_link_libraries(Prune ${Trilinos_LIBRARIES})

install( TARGETS Prune EXPORT Prune
         LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
         ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
         PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_PREFIX}/include)
