################################# test setup ##################################
###############################################################################
# Sources:
SET(reduced_SRCS  reduced_boundary.cpp)
SET(reduced_parameterized_SRCS  reduced_boundary_parameterized.cpp)
SET(reduced_nonparameterized_SRCS  reduced_boundary_nonparameterized.cpp)
SET(reduced_2LS_SRCS  reduced_boundary_2LS.cpp)
SET(reduced_physical_SRCS  reduced_boundary_physical.cpp)

# actual target:
ADD_EXECUTABLE(reduced_boundary ${reduced_SRCS})
ADD_EXECUTABLE(reduced_boundary_parameterized ${reduced_parameterized_SRCS})
ADD_EXECUTABLE(reduced_boundary_nonparameterized ${reduced_nonparameterized_SRCS})
ADD_EXECUTABLE(reduced_boundary_2LS ${reduced_2LS_SRCS})
ADD_EXECUTABLE(reduced_boundary_physical ${reduced_physical_SRCS})

# add libraries:
TARGET_LINK_LIBRARIES(reduced_boundary ${LINK_LIBRARIES})
TARGET_LINK_LIBRARIES(reduced_boundary_parameterized ${LINK_LIBRARIES})
TARGET_LINK_LIBRARIES(reduced_boundary_nonparameterized ${LINK_LIBRARIES})
TARGET_LINK_LIBRARIES(reduced_boundary_2LS ${LINK_LIBRARIES})
TARGET_LINK_LIBRARIES(reduced_boundary_physical ${LINK_LIBRARIES})

# copy geometry file:
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/geom_file.xml ${CMAKE_CURRENT_BINARY_DIR}/geom_file.xml COPYONLY)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/parameterized.xml ${CMAKE_CURRENT_BINARY_DIR}/parameterized.xml COPYONLY)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/nonparameterized.xml ${CMAKE_CURRENT_BINARY_DIR}/nonparameterized.xml COPYONLY)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/geom_file_2LS.xml ${CMAKE_CURRENT_BINARY_DIR}/geom_file_2LS.xml COPYONLY)
###############################################################################
###############################################################################

