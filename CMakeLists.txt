cmake_minimum_required(VERSION 3.16)
project(ompl_ply_planner LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include(FetchContent)

# Provide a minimal FindPython.cmake to avoid distutils issues (Python bindings are off anyway).
list(PREPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

# Avoid pulling optional dependencies to keep OMPL small.
set(CMAKE_DISABLE_FIND_PACKAGE_ODE ON CACHE BOOL "" FORCE)
set(CMAKE_DISABLE_FIND_PACKAGE_flann ON CACHE BOOL "" FORCE)
set(CMAKE_DISABLE_FIND_PACKAGE_spot ON CACHE BOOL "" FORCE)
set(CMAKE_DISABLE_FIND_PACKAGE_Doxygen ON CACHE BOOL "" FORCE)
set(CMAKE_DISABLE_FIND_PACKAGE_castxml ON CACHE BOOL "" FORCE)
set(CMAKE_DISABLE_FIND_PACKAGE_pypy ON CACHE BOOL "" FORCE)

# Prefer a locally built Boost if present (third_party/boost/install)
set(BOOST_ROOT "${PROJECT_SOURCE_DIR}/third_party/boost/install" CACHE PATH "Local Boost root")
if(EXISTS "${BOOST_ROOT}/include/boost")
  set(Boost_NO_SYSTEM_PATHS ON CACHE BOOL "" FORCE)
  set(Boost_USE_STATIC_LIBS ON CACHE BOOL "" FORCE)
  set(Boost_USE_MULTITHREADED ON CACHE BOOL "" FORCE)
  set(BOOST_INCLUDEDIR "${BOOST_ROOT}/include" CACHE PATH "" FORCE)
  set(BOOST_LIBRARYDIR "${BOOST_ROOT}/lib" CACHE PATH "" FORCE)
  list(PREPEND CMAKE_PREFIX_PATH "${BOOST_ROOT}")
endif()

# Eigen3 (fetch header-only to keep this self-contained)
FetchContent_Declare(
  eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG 3.4.0
)
FetchContent_GetProperties(eigen)
if(NOT eigen_POPULATED)
  FetchContent_Populate(eigen)
endif()
set(EIGEN3_INCLUDE_DIR "${eigen_SOURCE_DIR}" CACHE PATH "" FORCE)
set(EIGEN3_INCLUDE_DIRS "${eigen_SOURCE_DIR}" CACHE PATH "" FORCE)
set(Eigen3_FOUND TRUE CACHE BOOL "" FORCE)
include_directories("${eigen_SOURCE_DIR}")

# OMPL (build as static library in this build tree)
FetchContent_Declare(
  ompl
  GIT_REPOSITORY https://github.com/ompl/ompl.git
  GIT_TAG 1.6.0
)
set(OMPL_BUILD_DEMOS OFF CACHE BOOL "" FORCE)
set(OMPL_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(OMPL_BUILD_PYBINDINGS OFF CACHE BOOL "" FORCE)
set(OMPL_BUILD_APP OFF CACHE BOOL "" FORCE)
set(OMPL_BUILD_PLATFORMS OFF CACHE BOOL "" FORCE)
set(OMPL_REGRESSION_TESTS OFF CACHE BOOL "" FORCE)
set(OMPL_EIGEN3_EMBEDDED OFF CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(ompl)

add_library(ply STATIC
  src/ply.cpp
  src/ply.h
)

add_executable(generate_ply src/generate_ply.cpp)
target_link_libraries(generate_ply PRIVATE ply)

add_executable(plan_path src/main.cpp)
target_link_libraries(plan_path PRIVATE ompl ply)
if(EXISTS "${BOOST_ROOT}/include")
  target_include_directories(plan_path PRIVATE "${BOOST_ROOT}/include")
endif()

# OMPL includes
if(TARGET ompl)
  target_include_directories(plan_path PRIVATE ${ompl_SOURCE_DIR}/src)
  target_include_directories(plan_path PRIVATE ${ompl_BINARY_DIR}/src)
endif()

# Windows math defines if needed
if (MSVC)
  target_compile_definitions(plan_path PRIVATE _USE_MATH_DEFINES)
endif()
