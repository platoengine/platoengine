###############################################################################
# Sources:
###############################################################################
SET(Su2ToExodus_SRC Su2ToExodus_Main.cpp Su2ToExodus.cpp)
SET(Su2ToExodus_HDRS Su2ToExodus.hpp)

STRING(FIND ${MPI_C_COMPILER} "openmpi" FIND_POS) 
#message("FIND_POS: " ${FIND_POS})

if( FIND_POS EQUAL -1 )
#   set(USING_OPEN_MPI FALSE)
#   message("Couldn't find openmpi in c compiler")
else()
  add_definitions(-DUSING_OPEN_MPI)
#   message("Found openmpi in c compiler")
endif()

INCLUDE_DIRECTORIES(${PLATO_INCLUDES})

# actual target:
set(PLATOMAIN_LIBS ${PLATO_LIBRARIES} ${Plato_EXTRA_LINK_FLAGS})
add_executable(Su2ToExodus ${Su2ToExodus_SRC})
target_link_libraries(Su2ToExodus ${PLATOMAIN_LIBS} ${Trilinos_LIBRARIES})
#target_link_libraries(Su2ToExodus ${PLATOMAIN_LIBS} ${Trilinos_LIBRARIES} ${Trilinos_TPL_LIBRARIES})
#set(INTERFACEXMLCHECKER_BINARY ${CMAKE_BINARY_DIR}/base/src/input_checkers/XMLGenerator PARENT_SCOPE)

if( CMAKE_INSTALL_PREFIX )
  install( TARGETS Su2ToExodus DESTINATION ${CMAKE_INSTALL_PREFIX}/bin )
endif()
###############################################################################
###############################################################################

if( UNIT_TESTING )
  add_subdirectory(./unittest)
endif()

