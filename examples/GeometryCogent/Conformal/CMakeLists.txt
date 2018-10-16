################################# test setup ##################################
###############################################################################
# Sources:
SET(conformal_SRCS  conformal.cpp)
SET(conformal_parameterized_SRCS  conformal_parameterized.cpp)

# actual target:
ADD_EXECUTABLE(conformal ${conformal_SRCS})
ADD_EXECUTABLE(conformal_parameterized ${conformal_parameterized_SRCS})

# add libraries:
TARGET_LINK_LIBRARIES(conformal ${LINK_LIBRARIES})
TARGET_LINK_LIBRARIES(conformal_parameterized ${LINK_LIBRARIES})

# copy geometry file:
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/geom_file.xml ${CMAKE_CURRENT_BINARY_DIR}/geom_file.xml COPYONLY)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/parameterized.xml ${CMAKE_CURRENT_BINARY_DIR}/parameterized.xml COPYONLY)
###############################################################################
###############################################################################
