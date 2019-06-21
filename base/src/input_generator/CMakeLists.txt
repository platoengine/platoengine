###############################################################################
# Sources:
###############################################################################
SET(XMLGenerator_SRCS XMLGenerator_Main.cpp XMLGenerator.cpp)
SET(XMLGenerator_HDRS XMLGenerator.hpp XMLGeneratorDataStruct.hpp XMLGeneratorDataStruct.hpp)

find_package(MPI)

#message("MPI_FOUND" ${MPI_FOUND} )
#message("MPI_VERSION" ${MPI_VERSION} )
#message("MPI_C_VERSION" ${MPI_C_VERSION} )
#message("MPI_C_COMPILER" ${MPI_C_COMPILER} )
#message("MPI_C_LIBRARIES" ${MPI_C_LIBRARIES} )
#message("MPI_CXX_COMPILER" ${MPI_CXX_COMPILER} )

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
add_executable(XMLGenerator ${XMLGenerator_SRCS})
target_link_libraries(XMLGenerator ${PLATOMAIN_LIBS} ${Trilinos_LIBRARIES} ${Trilinos_TPL_LIBRARIES})
#set(INTERFACEXMLCHECKER_BINARY ${CMAKE_BINARY_DIR}/base/src/input_checkers/XMLGenerator PARENT_SCOPE)

if( CMAKE_INSTALL_PREFIX )
  install( TARGETS XMLGenerator DESTINATION ${CMAKE_INSTALL_PREFIX}/bin )
endif()
###############################################################################
###############################################################################

if( UNIT_TESTING )
  add_subdirectory(./unittest)
endif()

