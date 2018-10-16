################################# test setup ##################################
###############################################################################
# Sources:
SET(measure_SRCS  measure_boundary.cpp)

# actual target:
ADD_EXECUTABLE(measure_boundary ${measure_SRCS})

# add libraries:
TARGET_LINK_LIBRARIES(measure_boundary ${LINK_LIBRARIES})

# copy geometry file:
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/geom_file.xml ${CMAKE_CURRENT_BINARY_DIR}/geom_file.xml COPYONLY)
###############################################################################
###############################################################################

