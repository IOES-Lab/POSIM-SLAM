cmake_minimum_required(VERSION 3.24)
project(posim_slam_basalt_core LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/modules)
if(NOT SOURCE_ROOT)
 message(FATAL_ERROR "SOURCE_ROOT containing verified dependency checkouts is required")
endif()
find_package(Eigen3 5.0.1 EXACT CONFIG REQUIRED)
find_package(TBB CONFIG REQUIRED)
find_package(OpenCV 4.13.0 EXACT CONFIG REQUIRED COMPONENTS core imgproc calib3d features2d imgcodecs)
find_package(fmt CONFIG REQUIRED)
set(BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(BUILD_PYTHON OFF CACHE BOOL "" FORCE)
add_compile_definitions(EIGEN_MPL2_ONLY EIGEN_DONT_PARALLELIZE BASALT_INSTANTIATIONS_DOUBLE)
add_subdirectory(${SOURCE_ROOT}/opengv opengv EXCLUDE_FROM_ALL)
set_target_properties(opengv PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON)
target_compile_options(opengv PRIVATE -include cassert)
set(B ${SOURCE_ROOT}/source)
find_package(Python3 COMPONENTS Interpreter REQUIRED)
set(JOINT_HEADERS ${CMAKE_CURRENT_BINARY_DIR}/joint_include)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
 ${CMAKE_CURRENT_SOURCE_DIR}/prepare_joint_headers.py)
execute_process(COMMAND ${Python3_EXECUTABLE}
 ${CMAKE_CURRENT_SOURCE_DIR}/prepare_joint_headers.py ${B} ${JOINT_HEADERS}
 RESULT_VARIABLE JOINT_HEADERS_STATUS)
if(NOT JOINT_HEADERS_STATUS EQUAL 0)
 message(FATAL_ERROR "Pinned Basalt joint landmark interface preparation failed")
endif()
# Replace only the pinned visualization export function in a build-local copy.
# The upstream checkout stays clean and retains its original license header.
file(READ ${B}/src/vi_estimator/ba_base.cpp BA_SOURCE)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
 ${CMAKE_CURRENT_SOURCE_DIR}/map_points.inc ${B}/src/vi_estimator/ba_base.cpp)
set(MAP_BEGIN "template <class Scalar_>\ntemplate <class Scalar2>\nvoid BundleAdjustmentBase<Scalar_>::get_current_points(")
set(MAP_END "template <class Scalar_>\nvoid BundleAdjustmentBase<Scalar_>::filterOutliers(")
string(FIND "${BA_SOURCE}" "${MAP_BEGIN}" MAP_START)
string(FIND "${BA_SOURCE}" "${MAP_END}" MAP_STOP)
if(MAP_START LESS 0 OR MAP_STOP LESS MAP_START)
 message(FATAL_ERROR "Pinned Basalt map export function not found")
endif()
math(EXPR MAP_LENGTH "${MAP_STOP}-${MAP_START}")
string(SUBSTRING "${BA_SOURCE}" ${MAP_START} ${MAP_LENGTH} MAP_ORIGINAL)
file(READ ${CMAKE_CURRENT_SOURCE_DIR}/map_points.inc MAP_REPLACEMENT)
string(REPLACE "${MAP_ORIGINAL}" "${MAP_REPLACEMENT}\n" BA_PATCHED "${BA_SOURCE}")
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/ba_base.cpp "${BA_PATCHED}")
add_library(basalt_core STATIC
 ${B}/src/linearization/landmark_block.cpp
 ${B}/src/linearization/linearization_abs_qr.cpp
 ${B}/src/linearization/linearization_abs_sc.cpp
 ${B}/src/linearization/linearization_base.cpp
 ${B}/src/linearization/linearization_rel_sc.cpp
 ${B}/src/optical_flow/optical_flow.cpp
 ${B}/src/utils/keypoints.cpp
 ${B}/src/utils/system_utils.cpp
 ${B}/src/utils/time_utils.cpp
 ${B}/src/utils/vio_config.cpp
 ${CMAKE_CURRENT_BINARY_DIR}/ba_base.cpp
 ${B}/src/vi_estimator/landmark_database.cpp
 ${B}/src/vi_estimator/marg_helper.cpp
 ${B}/src/vi_estimator/sc_ba_base.cpp
 ${B}/src/vi_estimator/sqrt_ba_base.cpp
 ${B}/src/vi_estimator/sqrt_keypoint_vio.cpp
 ${B}/src/vi_estimator/sqrt_keypoint_vo.cpp
 ${B}/src/vi_estimator/vio_estimator.cpp)
target_include_directories(basalt_core PUBLIC
 ${JOINT_HEADERS} ${B}/include ${SOURCE_ROOT}/headers/include ${SOURCE_ROOT}/sophus
 ${SOURCE_ROOT}/cereal/include ${SOURCE_ROOT}/magic_enum/include
 ${SOURCE_ROOT}/opengv/include ${SOURCE_ROOT}/json/single_include ${OpenCV_INCLUDE_DIRS})
target_compile_options(basalt_core PRIVATE -Wno-error=missing-template-arg-list-after-template-kw)
target_link_libraries(basalt_core PUBLIC Eigen3::Eigen TBB::tbb fmt::fmt opengv ${OpenCV_LIBS})
add_executable(basalt_synthetic_probe synthetic_probe.cc)
target_link_libraries(basalt_synthetic_probe PRIVATE basalt_core)
add_executable(basalt_replay replay.cc)
target_link_libraries(basalt_replay PRIVATE basalt_core)
add_executable(basalt_stream stream.cc)
target_link_libraries(basalt_stream PRIVATE basalt_core)

add_executable(basalt_input_probe input_probe.cc)
target_link_libraries(basalt_input_probe PRIVATE basalt_core)
add_executable(basalt_joint_landmark_probe joint_landmark_probe.cc)
target_link_libraries(basalt_joint_landmark_probe PRIVATE basalt_core)
