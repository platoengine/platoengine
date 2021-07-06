set(PLATO_INCLUDES ${PLATO_INCLUDES} 
  ${CMAKE_SOURCE_DIR}/base/src/iso/main
  PARENT_SCOPE )

add_subdirectory(main)

set(extract_iso_sources ExtractMain.cpp)
set(extract_iso_headers main/IsoVolumeExtractionTool.hpp
                       main/IVEMeshAPI.hpp
                       main/IVEMeshAPISTK.hpp
                       main/STKExtract.hpp
                       main/IsoVector.hpp)
INCLUDE_DIRECTORIES(${PLATO_INCLUDES})
add_executable(extract_iso ${extract_iso_sources} ${extract_iso_headers})

target_link_libraries(extract_iso ${Trilinos_LIBRARIES} IsoExtract)

install( TARGETS extract_iso EXPORT extract_iso
         RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
