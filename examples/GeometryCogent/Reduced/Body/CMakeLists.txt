################################# test setup ##################################
###############################################################################
# Sources:
SET(reduced_SRCS  reduced.cpp)
SET(harness_SRCS  harness.cpp)
SET(reduced_sweep_SRCS  reduced_sweep.cpp)
SET(reduced_sweep_2LS_SRCS  reduced_sweep_2LS.cpp)
SET(reduced_sweep_3LS_SRCS  reduced_sweep_3LS.cpp)
SET(reduced_parameterized_SRCS  reduced_parameterized.cpp)
SET(reduced_nonparameterized_SRCS  reduced_nonparameterized.cpp)

# actual target:
ADD_EXECUTABLE(reduced ${reduced_SRCS})
ADD_EXECUTABLE(harness ${harness_SRCS})
ADD_EXECUTABLE(reduced_sweep ${reduced_sweep_SRCS})
ADD_EXECUTABLE(reduced_sweep_2LS ${reduced_sweep_2LS_SRCS})
ADD_EXECUTABLE(reduced_sweep_3LS ${reduced_sweep_3LS_SRCS})
ADD_EXECUTABLE(reduced_parameterized ${reduced_parameterized_SRCS})
ADD_EXECUTABLE(reduced_nonparameterized ${reduced_nonparameterized_SRCS})

# add libraries:
TARGET_LINK_LIBRARIES(reduced ${LINK_LIBRARIES})
TARGET_LINK_LIBRARIES(harness ${LINK_LIBRARIES})
TARGET_LINK_LIBRARIES(reduced_sweep ${LINK_LIBRARIES})
TARGET_LINK_LIBRARIES(reduced_sweep_2LS ${LINK_LIBRARIES})
TARGET_LINK_LIBRARIES(reduced_sweep_3LS ${LINK_LIBRARIES})
TARGET_LINK_LIBRARIES(reduced_parameterized ${LINK_LIBRARIES})
TARGET_LINK_LIBRARIES(reduced_nonparameterized ${LINK_LIBRARIES})

# copy geometry file:
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/geom_file.xml ${CMAKE_CURRENT_BINARY_DIR}/geom_file.xml COPYONLY)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/harness_file.xml ${CMAKE_CURRENT_BINARY_DIR}/harness_file.xml COPYONLY)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/geom_file_2ls.xml ${CMAKE_CURRENT_BINARY_DIR}/geom_file_2ls.xml COPYONLY)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/geom_file_3ls.xml ${CMAKE_CURRENT_BINARY_DIR}/geom_file_3ls.xml COPYONLY)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/parameterized.xml ${CMAKE_CURRENT_BINARY_DIR}/parameterized.xml COPYONLY)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/nonparameterized.xml ${CMAKE_CURRENT_BINARY_DIR}/nonparameterized.xml COPYONLY)
###############################################################################
###############################################################################

