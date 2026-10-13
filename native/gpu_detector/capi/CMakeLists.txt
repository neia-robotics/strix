# Detector-only shared library with a plain C API around the FRC 971 CUDA
# AprilTag detector.  See README.md in this directory.
#
# Works both as a standalone project (cmake -S capi -B build-capi) and as a
# subdirectory of the top-level build (-DBUILD_GPU_CAPI=ON).
#
# Compared with the top-level build this needs only:
#   * the CUDA toolkit (nvcc 12.x, or clang-17 as the CUDA compiler),
#   * glog (+ gflags, which the core sources use for two debug flags),
#   * the CCCL headers (CUB/Thrust/libcudacxx), download-only,
#   * the cgpadwick/apriltag fork sources, compiled straight into this
#     library with hidden visibility.
# No OpenCV, no WPILib (ntcore/wpiutil), no seasocks, no nlohmann json.
cmake_minimum_required(VERSION 3.18)
if(POLICY CMP0135)
  cmake_policy(SET CMP0135 NEW)  # FetchContent URL downloads: use extraction time stamps
endif()

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  set(ATGPU_STANDALONE ON)
else()
  set(ATGPU_STANDALONE OFF)
endif()

set(ATGPU_BACKEND "cuda" CACHE STRING
    "Detector backend: 'cuda' (the real GPU detector) or 'cpu' (host-only \
reference backend using the AprilTag C library, for testing the C API \
without a GPU)")
set_property(CACHE ATGPU_BACKEND PROPERTY STRINGS cuda cpu)
if(NOT ATGPU_BACKEND STREQUAL "cuda" AND NOT ATGPU_BACKEND STREQUAL "cpu")
  message(FATAL_ERROR "ATGPU_BACKEND must be 'cuda' or 'cpu', got '${ATGPU_BACKEND}'")
endif()

option(ATGPU_BUILD_TESTS "Build the C API test program and register ctest tests" ON)
option(ATGPU_LINK_NO_UNDEFINED
       "Fail the link if the shared library has unresolved symbols (Linux)" ON)

if(ATGPU_STANDALONE)
  if(ATGPU_BACKEND STREQUAL "cuda")
    if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
      set(CMAKE_CUDA_ARCHITECTURES 87)  # Jetson Orin (Nano/NX/AGX)
    endif()
    project(ApriltagGpuCapi LANGUAGES C CXX CUDA)
  else()
    project(ApriltagGpuCapi LANGUAGES C CXX)
  endif()
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
  if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
  endif()
else()
  # Added from the top-level CMakeLists.txt, which enables CUDA and CXX only.
  enable_language(C)
endif()

# The detector sources need C++20. CMake learned nvcc's -std=c++20 flag in 3.26
# (and only for nvcc 12+); older CMake fails later with "requires the language
# dialect CUDA20". Ubuntu 22.04 / JetPack 6 apt ships 3.22: `pip install cmake`.
if(ATGPU_BACKEND STREQUAL "cuda" AND CMAKE_CUDA_COMPILER_ID STREQUAL "NVIDIA")
  if(CMAKE_VERSION VERSION_LESS 3.26)
    message(FATAL_ERROR "Building with nvcc needs CMake 3.26 or newer for C++20 "
                        "(this is ${CMAKE_VERSION}). Install one with `pip install cmake`, "
                        "or use clang-17 as the CUDA compiler.")
  endif()
  if(CMAKE_CUDA_COMPILER_VERSION VERSION_LESS 12.0)
    message(FATAL_ERROR "nvcc ${CMAKE_CUDA_COMPILER_VERSION} has no C++20 support; use "
                        "CUDA 12+ (JetPack 6) or clang-17 as the CUDA compiler (JetPack 5).")
  endif()
endif()

find_package(Threads REQUIRED)
find_package(glog REQUIRED)

# ---------------------------------------------------------------------------
# Dependencies that are source-only: the apriltag fork and (cuda only) CCCL.
# Resolution order: explicit cache variable, pre-downloaded copy in
# ${ATGPU_DEPS_DIR} (what fetch_deps.sh produces, for offline builds), then a
# download at configure time.
# ---------------------------------------------------------------------------
include(FetchContent)
set(ATGPU_DEPS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/deps" CACHE PATH
    "Directory holding pre-downloaded dependency sources (see fetch_deps.sh)")
set(ATGPU_APRILTAG_SOURCE_DIR "" CACHE PATH
    "Source tree of the cgpadwick/apriltag fork, tag 3.3.0 (empty: deps/ or download)")
set(ATGPU_CCCL_SOURCE_DIR "" CACHE PATH
    "Source tree of NVIDIA CCCL v2.3.2 (empty: deps/ or download)")

set(ATGPU_APRILTAG_VERSION 3.3.0)
set(ATGPU_APRILTAG_URL
    "https://github.com/cgpadwick/apriltag/archive/refs/tags/${ATGPU_APRILTAG_VERSION}.tar.gz")
set(ATGPU_CCCL_VERSION 2.3.2)
set(ATGPU_CCCL_URL
    "https://github.com/NVIDIA/cccl/archive/refs/tags/v${ATGPU_CCCL_VERSION}.tar.gz")

# atgpu_resolve_source(<cache var> <name> <deps subdir> <marker file> <url>)
function(atgpu_resolve_source var name subdir marker url)
  if(${var})
    if(NOT EXISTS "${${var}}/${marker}")
      message(FATAL_ERROR "${var}='${${var}}' does not contain ${marker}")
    endif()
    message(STATUS "${name}: using ${${var}}")
    return()
  endif()
  if(EXISTS "${ATGPU_DEPS_DIR}/${subdir}/${marker}")
    set(${var} "${ATGPU_DEPS_DIR}/${subdir}" PARENT_SCOPE)
    message(STATUS "${name}: using pre-downloaded ${ATGPU_DEPS_DIR}/${subdir}")
    return()
  endif()
  message(STATUS "${name}: downloading ${url} (run fetch_deps.sh first for offline builds)")
  # SOURCE_SUBDIR points at a directory without a CMakeLists.txt so that
  # FetchContent_MakeAvailable only populates the sources and never calls
  # add_subdirectory() on the dependency's own build.
  FetchContent_Declare(${name} URL "${url}" SOURCE_SUBDIR atgpu-no-cmake-here)
  FetchContent_MakeAvailable(${name})
  set(${var} "${${name}_SOURCE_DIR}" PARENT_SCOPE)
endfunction()

atgpu_resolve_source(ATGPU_APRILTAG_SOURCE_DIR apriltag_fork
                     "apriltag-${ATGPU_APRILTAG_VERSION}" apriltag.h
                     "${ATGPU_APRILTAG_URL}")
if(ATGPU_BACKEND STREQUAL "cuda")
  atgpu_resolve_source(ATGPU_CCCL_SOURCE_DIR cccl "cccl-${ATGPU_CCCL_VERSION}"
                       cub/cub/cub.cuh "${ATGPU_CCCL_URL}")
endif()

# The pieces of the apriltag fork the detector needs (everything except the
# Python wrapper and the examples).  Explicit so a missing file is a clear
# configure error instead of a link error.
set(ATGPU_APRILTAG_SOURCES
    apriltag.c apriltag_pose.c apriltag_quad_thresh.c
    tag16h5.c tag25h9.c tag36h11.c tagCircle21h7.c tagCircle49h12.c
    tagCustom48h12.c tagStandard41h12.c tagStandard52h13.c
    common/g2d.c common/getopt.c common/homography.c common/image_u8.c
    common/image_u8x3.c common/image_u8x4.c common/matd.c common/pam.c
    common/pjpeg-idct.c common/pjpeg.c common/pnm.c common/string_util.c
    common/svd22.c common/time_util.c common/unionfind.c common/workerpool.c
    common/zarray.c common/zhash.c common/zmaxheap.c)
list(TRANSFORM ATGPU_APRILTAG_SOURCES PREPEND "${ATGPU_APRILTAG_SOURCE_DIR}/")
foreach(src ${ATGPU_APRILTAG_SOURCES})
  if(NOT EXISTS "${src}")
    message(FATAL_ERROR "apriltag fork source missing: ${src}")
  endif()
endforeach()

# Compiled once, shared by the library and the test program.
add_library(atgpu_apriltag_objs OBJECT ${ATGPU_APRILTAG_SOURCES})
target_include_directories(atgpu_apriltag_objs PUBLIC
    "${ATGPU_APRILTAG_SOURCE_DIR}" "${ATGPU_APRILTAG_SOURCE_DIR}/common")
set_target_properties(atgpu_apriltag_objs PROPERTIES
    POSITION_INDEPENDENT_CODE ON
    C_VISIBILITY_PRESET hidden)
target_compile_options(atgpu_apriltag_objs PRIVATE -w)  # third-party code
if(APPLE)
  # The fork's common/workerpool.h uses pthread types without including
  # <pthread.h>; glibc's <sys/types.h> provides them, macOS's does not.
  target_compile_options(atgpu_apriltag_objs PRIVATE -include pthread.h)
endif()
target_link_libraries(atgpu_apriltag_objs PUBLIC Threads::Threads m)

# ---------------------------------------------------------------------------
# The shared library.
# ---------------------------------------------------------------------------
set(ATGPU_CORE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../detector")
set(ATGPU_CORE_SOURCES
    ${ATGPU_CORE_DIR}/apriltag_detect.cu
    ${ATGPU_CORE_DIR}/apriltag_gpu.cu
    ${ATGPU_CORE_DIR}/cuda_frc971.cu
    ${ATGPU_CORE_DIR}/labeling_allegretti_2019_BKE.cu
    ${ATGPU_CORE_DIR}/line_fit_filter.cu
    ${ATGPU_CORE_DIR}/points.cu
    ${ATGPU_CORE_DIR}/threshold.cu)

set(ATGPU_API_SOURCES
    src/atgpu_capi.cpp
    src/atgpu_apriltag_common.cpp
)

if(ATGPU_BACKEND STREQUAL "cuda")
  set(ATGPU_BACKEND_SOURCES src/atgpu_backend_cuda.cu ${ATGPU_CORE_SOURCES})
else()
  set(ATGPU_BACKEND_SOURCES src/atgpu_backend_cpu.cpp)
endif()

add_library(apriltag_gpu_capi SHARED
    ${ATGPU_API_SOURCES}
    ${ATGPU_BACKEND_SOURCES}
    $<TARGET_OBJECTS:atgpu_apriltag_objs>)
add_library(apriltag_gpu_capi::apriltag_gpu_capi ALIAS apriltag_gpu_capi)

target_include_directories(apriltag_gpu_capi
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    PRIVATE
      "${CMAKE_CURRENT_SOURCE_DIR}/src"
      "${ATGPU_APRILTAG_SOURCE_DIR}"
      "${ATGPU_APRILTAG_SOURCE_DIR}/common")
target_compile_features(apriltag_gpu_capi PRIVATE cxx_std_20)
target_link_libraries(apriltag_gpu_capi PRIVATE glog::glog Threads::Threads m)
set_target_properties(apriltag_gpu_capi PROPERTIES
    OUTPUT_NAME apriltag_gpu_capi
    VERSION 1.0.0
    SOVERSION 1
    POSITION_INDEPENDENT_CODE ON
    C_VISIBILITY_PRESET hidden
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN ON)

if(ATGPU_BACKEND STREQUAL "cuda")
  target_include_directories(apriltag_gpu_capi PRIVATE
      "${ATGPU_CORE_DIR}"
      "${ATGPU_CCCL_SOURCE_DIR}/cub"
      "${ATGPU_CCCL_SOURCE_DIR}/libcudacxx/include"
      "${ATGPU_CCCL_SOURCE_DIR}/thrust")
  set_target_properties(apriltag_gpu_capi PROPERTIES
      CUDA_VISIBILITY_PRESET hidden
      CUDA_RUNTIME_LIBRARY Static)   # no libcudart.so dependency at load time
  if(ATGPU_STANDALONE)
    set_target_properties(apriltag_gpu_capi PROPERTIES
        CUDA_STANDARD 20 CUDA_STANDARD_REQUIRED ON)
  endif()
  # Same flags the top-level build uses for nvcc; clang needs nothing extra.
  target_compile_options(apriltag_gpu_capi PRIVATE
      $<$<AND:$<COMPILE_LANGUAGE:CUDA>,$<CUDA_COMPILER_ID:NVIDIA>>:--expt-relaxed-constexpr>)
  # libcudart_static needs these; CMake adds them for nvcc but be explicit so
  # the clang-as-CUDA-compiler path and --no-undefined are covered too.
  target_link_libraries(apriltag_gpu_capi PRIVATE ${CMAKE_DL_LIBS})
  if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_link_libraries(apriltag_gpu_capi PRIVATE rt)
  endif()
endif()

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  # Belt and braces on top of hidden visibility: the version script makes
  # every symbol except atgpu_* local, and --exclude-libs keeps symbols of
  # static archives (cudart_static, ...) from being re-exported.
  target_link_options(apriltag_gpu_capi PRIVATE
      "-Wl,--version-script=${CMAKE_CURRENT_SOURCE_DIR}/src/atgpu_exports.map"
      "-Wl,--exclude-libs,ALL")
  set_target_properties(apriltag_gpu_capi PROPERTIES
      LINK_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/atgpu_exports.map")
  if(ATGPU_LINK_NO_UNDEFINED)
    target_link_options(apriltag_gpu_capi PRIVATE "-Wl,--no-undefined")
  endif()
endif()

include(GNUInstallDirs)
install(TARGETS apriltag_gpu_capi
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(FILES include/apriltag_gpu_capi.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# ---------------------------------------------------------------------------
# Tests.
# ---------------------------------------------------------------------------
if(ATGPU_BUILD_TESTS)
  add_executable(atgpu_capi_test test/atgpu_capi_test.cpp
                 $<TARGET_OBJECTS:atgpu_apriltag_objs>)
  target_include_directories(atgpu_capi_test PRIVATE
      "${ATGPU_APRILTAG_SOURCE_DIR}" "${ATGPU_APRILTAG_SOURCE_DIR}/common")
  target_compile_features(atgpu_capi_test PRIVATE cxx_std_17)
  target_link_libraries(atgpu_capi_test PRIVATE apriltag_gpu_capi Threads::Threads m)

  if(ATGPU_STANDALONE OR NOT CMAKE_TESTING_ENABLED)
    enable_testing()
  endif()
  set(ATGPU_TEST_DATA_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../testdata")
  add_test(NAME atgpu_capi_test
           COMMAND atgpu_capi_test "${ATGPU_TEST_DATA_DIR}")
  add_test(NAME atgpu_exports
           COMMAND bash "${CMAKE_CURRENT_SOURCE_DIR}/test/check_exports.sh"
                   $<TARGET_FILE:apriltag_gpu_capi>)
  find_package(Python3 COMPONENTS Interpreter QUIET)
  if(Python3_Interpreter_FOUND)
    # The smoke test reads the JPEG test images with OpenCV and needs numpy.
    execute_process(COMMAND ${Python3_EXECUTABLE} -c "import numpy, cv2"
                    RESULT_VARIABLE ATGPU_PY_DEPS_RESULT OUTPUT_QUIET ERROR_QUIET)
    if(NOT ATGPU_PY_DEPS_RESULT EQUAL 0)
      message(STATUS "Skipping atgpu_ctypes_smoke: ${Python3_EXECUTABLE} lacks numpy or cv2 "
                     "(pass -DPython3_EXECUTABLE=<a python with both> to run it)")
      set(Python3_Interpreter_FOUND FALSE)
    endif()
  endif()
  if(Python3_Interpreter_FOUND)
    add_test(NAME atgpu_ctypes_smoke
             COMMAND ${Python3_EXECUTABLE}
                     "${CMAKE_CURRENT_SOURCE_DIR}/test/smoke_ctypes.py"
                     $<TARGET_FILE:apriltag_gpu_capi>
                     --data "${ATGPU_TEST_DATA_DIR}")
  endif()
endif()
