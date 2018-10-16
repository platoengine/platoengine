################################# test setup ##################################
###############################################################################
# Sources:
SET(measure_SRCS  measure_body.cpp)
SET(measure_3LS_SRCS  measure_body_3LS.cpp)

# actual target:
ADD_EXECUTABLE(measure_body ${measure_SRCS})
ADD_EXECUTABLE(measure_body_3LS ${measure_3LS_SRCS})

# add libraries:
TARGET_LINK_LIBRARIES(measure_body ${LINK_LIBRARIES})
TARGET_LINK_LIBRARIES(measure_body_3LS ${LINK_LIBRARIES})

# copy geometry file:
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/geom_file.xml ${CMAKE_CURRENT_BINARY_DIR}/geom_file.xml COPYONLY)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/geom_file_3LS.xml ${CMAKE_CURRENT_BINARY_DIR}/geom_file_3LS.xml COPYONLY)
###############################################################################
###############################################################################

