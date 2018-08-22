set(PLATO_INCLUDES ${PLATO_INCLUDES} 
  ${CMAKE_SOURCE_DIR}/base/src/iso/main
  PARENT_SCOPE )


set(IsoExtract_SOURCES IsoVolumeExtractionTool.cpp
                        IVEMeshAPISTK.cpp
                        STKExtract.cpp
                        IsoVector.cpp)
set(IsoExtract_HEADERS IsoVolumeExtractionTool.hpp
                       IVEMeshAPI.hpp
                       IVEMeshAPISTK.hpp
                       STKExtract.hpp
                       IsoVector.hpp)
INCLUDE_DIRECTORIES(${PLATO_INCLUDES})
add_library(IsoExtract ${IsoExtract_SOURCES} ${IsoExtract_HEADERS})
set(PLATO_LIBRARIES ${PLATO_LIBRARIES} IsoExtract PARENT_SCOPE)

target_link_libraries(IsoExtract ${Trilinos_LIBRARIES})

install( TARGETS IsoExtract EXPORT IsoExtract
         LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
         ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
         PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_PREFIX}/include)
