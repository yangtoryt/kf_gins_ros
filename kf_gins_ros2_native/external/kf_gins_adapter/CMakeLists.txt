
cmake_minimum_required(VERSION 3.16)
project(kf_gins_adapter)

# This adapter directly compiles KF-GINS core sources and exposes a small factory used by kf_gins_ros2_native.

# Find Eigen3 - required by KF-GINS
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(yaml-cpp REQUIRED)

add_library(kf_gins_core_adapter STATIC
  adapter.cpp
  # Pull in KF-GINS core sources (only two .cpp files are needed)
  ${KF_GINS_DIR}/src/kf-gins/gi_engine.cpp
  ${KF_GINS_DIR}/src/kf-gins/insmech.cpp
)

# Expect KF_GINS_DIR to point to the cloned KF-GINS repo folder (the one that has src/kf-gins).
# By default, assume it's checked out at: external/KF-GINS next to this adapter folder.
if(NOT DEFINED KF_GINS_DIR)
  get_filename_component(ADAPTER_DIR "${CMAKE_CURRENT_LIST_DIR}" ABSOLUTE)
  set(KF_GINS_DIR "${ADAPTER_DIR}/../KF-GINS")
endif()

# Headers
target_include_directories(kf_gins_core_adapter PUBLIC
  ${KF_GINS_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}
)

# Link Eigen3 library
target_link_libraries(kf_gins_core_adapter PUBLIC Eigen3::Eigen yaml-cpp)

# fPIC to link into shared nodes
set_target_properties(kf_gins_core_adapter PROPERTIES POSITION_INDEPENDENT_CODE ON)

# No external deps required; yaml/abseil are only used by the standalone demo, not by gi_engine/insmech.
