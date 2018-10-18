###############################################################################
# Sources:
###############################################################################
SET(SupportStructure_SRCS SupportStructureMain.cpp)
#SET(SupportStructure_HDRS )

INCLUDE_DIRECTORIES(${PLATO_INCLUDES} ./main)

# actual target:
set(PLATOMAIN_LIBS ${PLATO_LIBRARIES} ${Plato_EXTRA_LINK_FLAGS})
add_executable(create_support_structure ${SupportStructure_SRCS})
target_link_libraries(create_support_structure SupportStructureLib ${PLATOMAIN_LIBS} ${Trilinos_LIBRARIES} ${Trilinos_TPL_LIBRARIES})

if( CMAKE_INSTALL_PREFIX )
  install( TARGETS create_support_structure DESTINATION ${CMAKE_INSTALL_PREFIX}/bin )
endif()
###############################################################################
###############################################################################

if( UNIT_TESTING )
#  add_subdirectory(./unittest)
endif()

