###############################################################################
# Sources:
###############################################################################
SET(prune_and_refine_SRCS PruneAndRefineMain.cpp)

find_package(MPI)

# try to determine if Open MPI is being used
STRING(FIND ${MPI_C_COMPILER} "openmpi" FIND_POS) 

if( FIND_POS EQUAL -1 )
  execute_process(COMMAND mpirun --version OUTPUT_VARIABLE OUT)
  STRING(FIND ${OUT} "Open MPI" FIND_POS) 
  if( FIND_POS EQUAL -1 )
  else()
    add_definitions(-DUSING_OPEN_MPI)
  endif()
 else()
   add_definitions(-DUSING_OPEN_MPI)
endif()

#MESSAGE("******************************")
#FOREACH(tril_tpl ${Trilinos_TPL_LIBRARIES})
#MESSAGE(${tril_tpl})
#ENDFOREACH()
#MESSAGE("******************************")

INCLUDE_DIRECTORIES(${PLATO_INCLUDES})

# actual target:
set(PLATOMAIN_LIBS ${PLATO_LIBRARIES} ${Plato_EXTRA_LINK_FLAGS})
add_executable(prune_and_refine ${prune_and_refine_SRCS})
#target_link_libraries(prune_and_refine ${PLATO_LIBRARIES} ${PLATOMAIN_LIBS} ${Trilinos_LIBRARIES} ${Trilinos_TPL_LIBRARIES} ${PLATO_LIBRARIES} ${Trilinos_LIBRARIES} ${Trilinos_TPL_LIBRARIES})

target_link_libraries(prune_and_refine ${PLATO_LIBRARIES} ${PLATOMAIN_LIBS} ${Trilinos_LIBRARIES}



)
#set(INTERFACEXMLCHECKER_BINARY ${CMAKE_BINARY_DIR}/base/src/input_checkers/XMLGenerator PARENT_SCOPE)

if( CMAKE_INSTALL_PREFIX )
  install( TARGETS prune_and_refine DESTINATION ${CMAKE_INSTALL_PREFIX}/bin )
endif()
###############################################################################
###############################################################################


