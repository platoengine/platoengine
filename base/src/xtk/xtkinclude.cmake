# -------------------------------------------------------------------------
# Eigen libraries ---------------------------------------------------------
# -------------------------------------------------------------------------
if(NOT EIGEN_FOUND_ONCE)
    set(EIGEN_ENV_VARS
        $ENV{EIGENDIR}
        $ENV{EIGEN_DIR}
        $ENV{Eigen_DIR}
        $ENV{Eigen3_DIR}
        $ENV{EIGEN_ROOT}
        $ENV{Eigen_ROOT}
        $ENV{Eigen3_ROOT}
        $ENV{EIGEN_PATH}
        $ENV{Eigen_PATH}
        $ENV{Eigen3_PATH})

    find_package(Eigen
        REQUIRED
        NAMES Eigen3
        HINTS ${EIGEN_ENV_VARS})
    
          
    set(MORIS_EIGEN_INCLUDE_DIRS "${EIGEN3_ROOT_DIR}/include/eigen3"
        CACHE PATH "Eigen include directories." )
    set(MORIS_EIGEN_TARGETS "${EIGEN3_ROOT_DIR}/share/eigen3/cmake/Eigen3Targets.cmake"
        CACHE PATH "Eigen targets file.")
    
    mark_as_advanced(MORIS_EIGEN_INCLUDE_DIRS
        MORIS_EIGEN_TARGETS
        Eigen_DIR
        )
    
    message(STATUS "EIGEN3_FOUND: ${EIGEN3_FOUND}")
    
    if(EIGEN3_FOUND)
        set(EIGEN_FOUND_ONCE TRUE CACHE INTERNAL "Eigen was found.")
    endif()
else()
    include(${MORIS_EIGEN_TARGETS})
endif()

add_definitions("-DMORIS_USE_EIGEN")
add_definitions("-DMORIS_USE_32BIT")

include_directories("${MORIS_EIGEN_INCLUDE_DIRS}")
list(APPEND MORIS_ARMADILLO_EIGEN_LIBS "Eigen3::Eigen")

include_directories( $ENV{XTK_INCLUDE_DIR}/ALG)
include_directories( $ENV{XTK_INCLUDE_DIR}/COM)
include_directories( $ENV{XTK_INCLUDE_DIR}/IOS)
include_directories( $ENV{XTK_INCLUDE_DIR}/LINALG)
include_directories( $ENV{XTK_INCLUDE_DIR}/MRS/ASR)
include_directories( $ENV{XTK_INCLUDE_DIR}/MRS/CHR)
include_directories( $ENV{XTK_INCLUDE_DIR}/MRS/CON)
include_directories( $ENV{XTK_INCLUDE_DIR}/MRS/COR)
include_directories( $ENV{XTK_INCLUDE_DIR}/MRS/EXC)
include_directories( $ENV{XTK_INCLUDE_DIR}/MTK)
include_directories( $ENV{XTK_INCLUDE_DIR}/XTK)