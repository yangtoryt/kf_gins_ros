find_package(Eigen3 REQUIRED)
add_library(kf_gins_core_stub STATIC stub_core.cpp)
# Ensure static lib is PIC
set_target_properties(kf_gins_core_stub PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(kf_gins_core_stub PUBLIC
  ${PROJECT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(kf_gins_core_stub Eigen3::Eigen)
target_compile_definitions(kf_gins_core_stub PUBLIC KF_GINS_STUB_CORE=1)
